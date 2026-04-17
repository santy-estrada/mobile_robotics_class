import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import math
from std_msgs.msg import Bool


class OrientationControlNode(Node):
    """
    Orientation Control Node
    
    Controls the angular orientation (alpha) of the robot to minimize angular error.
    Works in tandem with dist_finder.py by subscribing to the /error topic.
    Uses PD controller to generate steering commands based on angular error only.
    """

    def __init__(self):
        super().__init__('orientation_control_node')

        # -- PARAMETERS --
        self.declare_parameter('kp_orientation', 0.8)           # Proportional gain for angular PD controller
        self.declare_parameter('kd_orientation', 1.0)           # Derivative gain for angular PD controller
        self.declare_parameter('max_angular_vel', 1.0)  # Max angular velocity saturation (radians/s)
        self.declare_parameter('min_angular_vel', -1.0) # Min angular velocity saturation (radians/s)
        self.declare_parameter('orientation_tolerance', math.radians(3.0))  # Tolerance for angular error (radians)
        self.declare_parameter('filter_alpha', 0.9)          # First-order filter coefficient in [0, 1)
        self.declare_parameter('forward_velocity', 2.0*1.18)         # Constant forward velocity (m/s)
        self.declare_parameter('start_flag', False)             # Flag to signal that the controller is active
        self.declare_parameter('pub_logger', True)              # Flag to publish logger info

        # Get parameters
        self.kp = self.get_parameter('kp_orientation').value
        self.kd = self.get_parameter('kd_orientation').value
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.min_angular_vel = float(self.get_parameter('min_angular_vel').value)
        self.orientation_tolerance = float(self.get_parameter('orientation_tolerance').value)
        self.filter_alpha = float(self.get_parameter('filter_alpha').value)
        self.forward_vel = self.get_parameter('forward_velocity').value
        self.start_flag = bool(self.get_parameter('start_flag').value)
        self.pub_logger = bool(self.get_parameter('pub_logger').value)

        # Clamp filter alpha to a stable range.
        self.filter_alpha = max(0.0, min(0.999, self.filter_alpha))

        # State variables for derivative calculation
        self.prev_angular_error = 0.0
        self.prev_time = None
        self.prev_filtered_angular_vel = 0.0

        # Subscription to error topic from dist_finder
        self.error_sub = self.create_subscription(
            Twist,
            '/error',
            self.error_callback,
            10
        )

        # Subscription to start topic
        # self.start_sub = self.create_subscription(
        #     Bool,
        #     '/start',
        #     self.start_callback,
        #     10
        # )

        # Subscription to activation topic
        self.activation_sub = self.create_subscription(
            Twist,
            '/cmd_ang_tcc',
            self.activation_callback,
            10
        )

        # Publisher for control commands
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel_nav',
            10
        )

        self.get_logger().info('Orientation Control Node initialized')

    def start_callback(self, msg: Bool):
        """Handle start signal to activate the controller."""
        if msg.data:
            self.get_logger().info("Received start signal. Starting orientation control.")
            self.start_flag = True

    def activation_callback(self, msg: Twist):
        """Handle activation signal to activate the controller."""
        if msg.angular.y == 3.0:  # Assuming angular.y is used as activation flag
            if self.pub_logger:
                self.get_logger().info("Received activation signal. Starting orientation control.")
            self.start_flag = True
        else:
            if self.pub_logger:
                self.get_logger().info("Received deactivation signal. Stopping orientation control.")
            self.start_flag = False

    def error_callback(self, msg: Twist):
        """
        Callback for error messages from dist_finder.
        
        Extracts angular error (alpha) and computes PD control law.
        
        msg.linear.x: distance error from wall (not used in orientation control)
        msg.linear.y: distance travelled since last callback (not used in orientation control)
        msg.angular.z: angular error / orientation angle (alpha) - USED FOR CONTROL
        msg.linear.z: wall lost indicator (1.0 if wall lost, 0.0 otherwise)
        """

        if not self.start_flag:
            self.get_logger().info("Controller not started or stopped. Ignoring error messages.")
            self.prev_filtered_angular_vel = 0.0
            return

        current_time = self.get_clock().now()

        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_angular_error = 0.0
            return

        # Calculate time step
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        # Extract angular error from error message
        alpha = msg.angular.z  # Angular error (alpha)
        wall_lost = msg.linear.z == 1.0

        # Angular error to control: minimize alpha (make it zero)
        angular_error = alpha

        # Calculate error rate (derivative)
        if dt > 0:
            angular_error_rate = (angular_error - self.prev_angular_error) / dt
        else:
            angular_error_rate = 0.0

        self.prev_angular_error = angular_error

        # Check if within tolerance
        within_tolerance = abs(angular_error) < self.orientation_tolerance

        # PD Control Law: angular_vel = Kp * alpha + Kd * d(alpha)/dt
        angular_vel = self.kp * angular_error + self.kd * angular_error_rate

        # Apply saturation (min and max limits)
        angular_vel = max(
            self.min_angular_vel,
            min(self.max_angular_vel, -angular_vel)
        )

        # First-order low-pass filter on control action:
        # u_f[k] = alpha * u_f[k-1] + (1 - alpha) * u[k]
        filtered_angular_vel = (
            self.filter_alpha * self.prev_filtered_angular_vel
            + (1.0 - self.filter_alpha) * angular_vel
        )
        self.prev_filtered_angular_vel = filtered_angular_vel

        # Create and publish command
        cmd = TwistStamped()

        if wall_lost:
            self.prev_filtered_angular_vel = 0.0
            if self.pub_logger:
                self.get_logger().warn("Wall lost detected. Stopping orientation control commands.")
            return  # Do not publish any command if wall is lost, dist_finder will handle recovery
        else:
            # Normal operation
            if within_tolerance:
                # If within tolerance, move forward at full speed
                cmd.twist.linear.x = self.forward_vel
                cmd.twist.angular.z = 0.0  # No angular command needed
                self.prev_filtered_angular_vel = 0.0
            else:
                # If not within tolerance, prioritize orientation correction
                cmd.twist.linear.x = self.forward_vel * 0.99
                cmd.twist.angular.z = filtered_angular_vel

        self.cmd_pub.publish(cmd)

        # Logging
        if self.pub_logger:
            status = "ALIGNED" if within_tolerance else "CORRECTING"
            self.get_logger().info(
                f"[{status}] Alpha={math.degrees(alpha):.2f}° | "
                f"Error={math.degrees(angular_error):.2f}° | "
                f"ErrorRate={math.degrees(angular_error_rate):.2f}°/s | "
                f"AngVelRaw={math.degrees(angular_vel):.2f}°/s | "
                f"AngVelFiltered={math.degrees(filtered_angular_vel):.2f}°/s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = OrientationControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
