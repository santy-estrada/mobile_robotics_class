import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import math
from std_msgs.msg import Bool


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        # -- PARAMETERS --
        self.declare_parameter('kp', 0.5)                    # Proportional gain for PD controller
        self.declare_parameter('kd', 0.9)                    # Derivative gain for PD controller
        self.declare_parameter('max_steering', math.radians(60))          # Max steering angle saturation (radians)
        self.declare_parameter('min_steering', math.radians(-60))         # Min steering angle saturation (radians)
        self.declare_parameter('forward_velocity', 2.5)      # Constant forward velocity (m/s)
        self.declare_parameter('brake_turn_angle', 1.3) #Puede ser 1.0 si fv = 2.0


        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.forward_vel = self.get_parameter('forward_velocity').value
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.min_steering = float(self.get_parameter('min_steering').value)
        self.brake_turn_angle= float(self.get_parameter('brake_turn_angle').value)
        # State variables for derivative calculation
        self.prev_error = 0.0
        self.prev_time = None
        # Brake state
        self.brake_active = False

        # Subscription to brake state
        self.brake_sub = self.create_subscription(
            Bool,
            '/brake_active',
            self.brake_callback,
            10
        )

        # Subscription to error topic
        self.error_sub = self.create_subscription(
            Twist,
            '/error',
            self.error_callback,
            10
        )

        # Publisher for control commands
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel_nav',
            10
        )

        self.get_logger().info('PD Control node initialized')

    def brake_callback(self, msg: Bool):
        self.brake_active = msg.data

    def error_callback(self, msg: Twist):
        """
        Callback for error messages from dist_finder.
        
        msg.linear.x: distance error from wall (y)
        msg.linear.y: distance travelled since last callback (L or AC)
        msg.angular.z: angular error / orientation angle (theta or alpha)
        """
        current_time = self.get_clock().now()

        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = 0.0
            return

        # Calculate time step
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        # Extract error components
        y = msg.linear.x           # Distance error from wall
        L = msg.linear.y           # Distance travelled (AC)
        theta = msg.angular.z      # Angular error (alpha)

        wall_lost = msg.linear.z == 1.0        

        # Composite error: e(t) = -(y + L*sin(theta))
        # This represents the predicted lateral deviation at the next step
        error = -(y)

        # Calculate error rate (derivative)
        if dt > 0:
            error_rate = (error - self.prev_error) / dt
        else:
            error_rate = 0.0

        self.prev_error = error

        # PD Control Law: theta_desired = Kp*e(t) + Kd*e'(t)
        theta_desired = self.kp * error + self.kd * error_rate

        # Calculate steering angle: steering = theta - theta_desired
        # (theta is the measured angular error alpha)
        # steering_angle = theta - theta_desired
        steering_angle = -theta_desired
        # Apply saturation (min and max limits)
        steering_angle = max(
            self.min_steering,
            min(self.max_steering, steering_angle)
        )

        # Create and publish command
        cmd = TwistStamped()

        # Set constant forward velocity and steering angle
        
        # Forward velocity blocked if brake is active
        if self.brake_active:
            cmd.twist.linear.x = self.forward_vel
            cmd.twist.angular.z = self.brake_turn_angle  # Turn in place to the left when braking
        else:
            if wall_lost:
                cmd.twist.linear.x = self.forward_vel * 0.7  # Move forward at half speed when wall is lost
                cmd.twist.angular.z = -self.brake_turn_angle *0.7 # Turn in place to the left when wall is lost
            else:
                cmd.twist.linear.x = self.forward_vel * math.exp(-self.kp * abs(error))
                cmd.twist.angular.z = steering_angle
        

        self.cmd_pub.publish(cmd)

        # Logging
        self.get_logger().info(
            f"y={y:.3f}m | L={L:.3f}m | alpha={math.degrees(theta):.1f}° | "
            f"e={error:.3f} | e'={error_rate:.3f} | "
            f"theta_desired={math.degrees(theta_desired):.1f}° | "
            f"Steering={math.degrees(steering_angle):.1f}°"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()