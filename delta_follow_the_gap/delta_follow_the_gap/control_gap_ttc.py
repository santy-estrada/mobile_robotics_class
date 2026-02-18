import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_gap_ttc')

        # -- PARAMETERS --
        self.declare_parameter('forward_velocity', 2.0)      # Constant forward velocity (m/s)
        self.declare_parameter('start_flag', False)          # Flag to signal that the car has received its first forward input from Joystick
        self.declare_parameter('pub_logger', True)           # Flag to signal whether to publish error values for logging

        # Get parameters
        self.forward_vel = self.get_parameter('forward_velocity').value
        self.start_flag = bool(self.get_parameter('start_flag').value)
        self.pub_logger = bool(self.get_parameter('pub_logger').value)

        # Subscription to brake state
        self.start_sub = self.create_subscription(
            Bool,
            '/start',
            self.start_callback,
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

        self.get_logger().info('PD Control node initialized')

    def start_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Received start signal. Starting control.")
            self.start_flag = True
            cmd = TwistStamped()
            cmd.twist.linear.x = self.forward_vel
            cmd.twist.angular.z = -5.14  # Initial turn to start following the gap
            self.cmd_pub.publish(cmd)

    def error_callback(self, msg: Twist):
        """
        Callback for error messages from dist_finder.
        
        msg.angular.y: alert flag
        msg.angular.z: angular error / orientation angle (theta or alpha)
        """
        if not self.start_flag:
            self.get_logger().info("Controller not started yet. Ignoring error messages.")
            return

        # Extract error components
        theta = msg.angular.z      # Angular error (alpha)
        alertado = msg.angular.y   # Alert flag
        
        # Create and publish command
        cmd = TwistStamped()
        if alertado == 1.0:
            cmd.twist.linear.x = 0.8 * self.forward_vel
            cmd.twist.angular.z = 1.7 * theta
        else:
            cmd.twist.linear.x = 1.1 * self.forward_vel
            cmd.twist.angular.z = 1.7 * theta

        self.cmd_pub.publish(cmd)

        # Logging
        if self.pub_logger:
            self.get_logger().info(f"crr_env={theta} | ")


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()