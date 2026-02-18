import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import math
import statistics
from std_msgs.msg import Bool, Float32MultiArray


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        # -- PARAMETERS --
        self.declare_parameter('kp', 0.6)                    # Proportional gain for PD controller
        self.declare_parameter('kd', 0.9)                    # Derivative gain for PD controller
        self.declare_parameter('k_fwd', 0.25)                  # Forward velocity gain based on error magnitude
        self.declare_parameter('max_steering', math.radians(60))          # Max steering angle saturation (radians)
        self.declare_parameter('min_steering', math.radians(-60))         # Min steering angle saturation (radians)
        self.declare_parameter('forward_velocity', 1.4)      # Constant forward velocity (m/s)
        self.declare_parameter('brake_turn_angle', 1.3) #Puede ser 1.0 si fv = 2.0
        self.declare_parameter('start_flag', False)               #Flag to signal that the car has received its first forward input from Joystickz
        self.declare_parameter('pub_logger', True)               #Flag to signal whether to publish error values for logging in ttc_gap_logger_node
        self.declare_parameter('front_wall_gain', 0.8)           # Gain for predictive turn correction when front wall detected
        self.declare_parameter('front_wall_ttc_threshold', 2.8)  # TTC threshold below which to start turning (seconds)
        self.declare_parameter('wall_lost_angular_vel_gain', 0.9)    # Angular velocity multiplier when wall is lost

        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.k_fwd = self.get_parameter('k_fwd').value
        self.forward_vel = self.get_parameter('forward_velocity').value
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.min_steering = float(self.get_parameter('min_steering').value)
        self.brake_turn_angle= float(self.get_parameter('brake_turn_angle').value)
        self.start_flag = bool(self.get_parameter('start_flag').value)
        self.pub_logger = bool(self.get_parameter('pub_logger').value)
        self.front_wall_gain = float(self.get_parameter('front_wall_gain').value)
        self.front_wall_ttc_threshold = float(self.get_parameter('front_wall_ttc_threshold').value)
        self.wall_lost_angular_vel_gain = float(self.get_parameter('wall_lost_angular_vel_gain').value)
        # State variables for derivative calculation
        self.prev_error = 0.0
        self.prev_time = None
        # Brake state
        self.brake_active = False
        # Predictive turn bias based on front wall proximity
        self.front_wall_turn_bias = 0.0

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

        # Subscription to start topic
        self.start_sub = self.create_subscription(
            Bool,
            '/start',
            self.start_callback,
            10
        )

        self.ttc_arr_sub = self.create_subscription(
            Float32MultiArray,
            '/ttc_values',
            self.ttc_array_callback,
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

    def start_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Received start signal. Starting control.")
            self.start_flag = True

    def ttc_array_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        data.pop()
        if len(data) < 720:
            return

        # Analyze front cone (centered 20 rays, ~10° forward)
        start_idx = (len(data) - 20) // 2
        ttc_front = data[start_idx-10:start_idx + 20]
        
        # Filter out inf values for more accurate mean
        finite_ttc = [t for t in ttc_front if math.isfinite(t)]
        
        if len(finite_ttc) > 0:
            m_ttc = statistics.mean(finite_ttc)
            
            # Calculate predictive turn bias using inverse proportionality
            # As TTC decreases, turn bias increases inversely (more aggressively)
            if m_ttc < self.front_wall_ttc_threshold:
                # Inverse proportional: bias ∝ 1/TTC
                # turn_intensity = (threshold / TTC) - 1
                # When TTC = threshold: intensity = 0 (no turn)
                # When TTC = threshold/2: intensity = 1 (moderate turn)
                # When TTC → 0: intensity → large (aggressive turn)
                turn_intensity = (self.front_wall_ttc_threshold / m_ttc) - 1.0
                
                # Cap intensity to prevent extreme values when very close
                turn_intensity = min(turn_intensity, 10.0)
                
                self.front_wall_turn_bias = self.front_wall_gain * turn_intensity * self.max_steering
            else:
                self.front_wall_turn_bias = 0.0
        else:
            # All inf means no obstacles ahead
            self.front_wall_turn_bias = 0.0
            m_ttc = float('inf')

        # Log TTC values for debugging
        if self.pub_logger:
            self.get_logger().info(
                f"Front TTC: {m_ttc:.2f}s | Turn bias: {math.degrees(self.front_wall_turn_bias):.1f}°"
            )

    def error_callback(self, msg: Twist):
        """
        Callback for error messages from dist_finder.
        
        msg.linear.x: distance error from wall (y)
        msg.linear.y: distance travelled since last callback (L or AC)
        msg.angular.z: angular error / orientation angle (theta or alpha)
        """

        if not self.start_flag:
            self.get_logger().info("Controller not started yet. Ignoring error messages.")
            return
        
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

        # Calculate base steering angle from PD controller
        steering_angle = -theta_desired
        
        # Add predictive turn bias when front wall detected
        # This provides feedforward control to start turning before the corner
        steering_angle += self.front_wall_turn_bias
        
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
            cmd.twist.angular.z = -self.brake_turn_angle  # Turn in place to the right when braking
        else:
            if wall_lost:
                cmd.twist.linear.x = self.forward_vel * 0.25  # Move forward at half speed when wall is lost
                cmd.twist.angular.z = -self.brake_turn_angle *self.wall_lost_angular_vel_gain # Turn in place to the left when wall is lost
            else:
                if self.front_wall_turn_bias == 0.0 and error < 0.1:
                    # If no front wall detected and error is small, increase forward velocity for efficiency
                    cmd.twist.linear.x = self.forward_vel * 2.0
                else:
                    cmd.twist.linear.x = self.forward_vel * math.exp(-self.k_fwd * abs(error))
                cmd.twist.angular.z = steering_angle
        

        self.cmd_pub.publish(cmd)

        # Logging
        if self.pub_logger:
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