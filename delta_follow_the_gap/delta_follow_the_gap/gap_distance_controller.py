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
        self.declare_parameter('forward_vel', 1.4)      # Constant forward velocity (m/s)
        self.declare_parameter('brake_turn_angle', 0.9) #Puede ser 1.0 si fv = 2.0
        self.declare_parameter('start_flag', False)               #Flag to signal that the car has received its first forward input from Joystickz
        self.declare_parameter('pub_logger', True)      #Flag to pub logger info

        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.forward_vel = self.get_parameter('forward_vel').value
        self.brake_turn_angle= float(self.get_parameter('brake_turn_angle').value)
        self.start_flag = bool(self.get_parameter('start_flag').value)
        self.pub_logger = bool(self.get_parameter('pub_logger').value)

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

        #Subscription to start topic
        self.start_sub = self.create_subscription(
            Bool,
            '/start',
            self.start_callback,
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

    def start_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Received start signal. Starting control.")
            self.start_flag = True
        


    def error_callback(self, msg: Twist):
        """
        Callback for error messages from dist_finder.
        msg.angular.z: angular error
        """

        if not self.start_flag:
            self.get_logger().info("Controller not started yet. Ignoring error messages.")
            return
        
        theta =msg.angular.z     # Angular error (alpha)
        # Create and publish command
        cmd = TwistStamped()

        if self.brake_active:
            # If brake is active, turn
            cmd.twist.linear.x = 0.1
            cmd.twist.angular.z = self.brake_turn_angle if theta >= 0 else -self.brake_turn_angle
            self.cmd_pub.publish(cmd)

            if self.pub_logger:
                self.get_logger().info(
                    f"Brake active! crr_env={theta} | cmd=0.0 linear, {cmd.twist.angular.z} angular"
                )
            return

        u = self.kp * theta

        cmd.twist.linear.x = self.forward_vel
        cmd.twist.angular.z = u


        self.cmd_pub.publish(cmd)

        # Logging
        if self.pub_logger:
            self.get_logger().info(
                f"crr_env={math.degrees(theta)} | cmd={u}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()