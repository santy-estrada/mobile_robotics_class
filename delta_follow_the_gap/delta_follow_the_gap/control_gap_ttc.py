import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import math
from std_msgs.msg import Bool


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_gap_ttc')

        # -- PARAMETERS --
        self.declare_parameter('kp', 1.0)                    # Proportional gain for PD controller
        self.declare_parameter('max_steering', math.radians(60))          # Max steering angle saturation (radians)
        self.declare_parameter('min_steering', math.radians(-60))         # Min steering angle saturation (radians)
        self.declare_parameter('forward_velocity', 2.0)      # Constant forward velocity (m/s)
        self.declare_parameter('brake_turn_angle', 1.3) #Puede ser 1.0 si fv = 2.0


        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.forward_vel = self.get_parameter('forward_velocity').value
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.min_steering = float(self.get_parameter('min_steering').value)
        self.brake_turn_angle= float(self.get_parameter('brake_turn_angle').value)

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
            '/cmd_ang_tcc',
            self.error_callback,
            10
        )

        # Publisher for control commands
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel_ttc_gap',
            10
        )

        self.get_logger().info('P Control node initialized')

    def brake_callback(self, msg: Bool):
        self.brake_active = msg.data

    def error_callback(self, msg: Twist):
        """
        Callback for error messages from dist_finder.
        msg.angular.z: angular error
        """
        theta =msg.angular.z     # Angular error (alpha)
        # Create and publish command
        cmd = TwistStamped()

        if self.brake_active:
            # If brake is active, set forward velocity to 0 and turn sharply
            cmd.twist.linear.x = -0.7*self.forward_vel
            cmd.twist.angular.z = self.brake_turn_angle if theta >= 0 else -self.brake_turn_angle
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f"Brake active! crr_env={theta} | cmd=0.0 linear, {cmd.twist.angular.z} angular"
            )
            return

        u = min(self.max_steering, max(self.min_steering, self.kp * theta))

        cmd.twist.linear.x = self.forward_vel
        cmd.twist.angular.z = u
  

        self.cmd_pub.publish(cmd)

        # Logging
        self.get_logger().info(
            f"crr_env={theta} | cmd={u}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()