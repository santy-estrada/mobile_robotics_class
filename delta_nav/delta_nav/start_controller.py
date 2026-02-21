import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
import threading




class StartController(Node):
    def __init__(self):
        super().__init__('start_controller')
        
        # ---- Parameters ----
        self.declare_parameter('start_flag', False)               # Flag to signal that the car has received its first forward input
        self.declare_parameter('use_joystick_start', False)        # If true, use joystick forward command; if false, use 's' key
        
        
        self.start_flag = bool(self.get_parameter('start_flag').value)
        self.use_joystick_start = bool(self.get_parameter('use_joystick_start').value)
              
        
        # ---- Pub/Sub ----
        # QoS profile compatible with sensor publishers (RELIABLE, VOLATILE)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to input based on parameter
        if self.use_joystick_start:
            self.create_subscription(TwistStamped, '/cmd_vel_joy', self._joystick_cmd_vel_callback, qos)
            self.get_logger().info("Using joystick forward command to start controller")
        else:
            # Start keyboard listener thread
            self.keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
            self.keyboard_thread.start()
            self.get_logger().info("Using 's' key to start controller. Press 's' to start.")
        
        # Publisher to start controllers
        self.brake_pub = self.create_publisher(Bool, '/start', 10)


    def _joystick_cmd_vel_callback(self, msg: TwistStamped):
        """Set start flag to True when first joystick command is received."""
        fwd_vel = msg.twist.linear.x
        if fwd_vel > 0.0:
            if not self.start_flag:
                self.start_flag = True
                self.get_logger().info("Received first joystick command. Starting TTC monitoring.")
            # Publish start signal to other nodes
            start_msg = Bool()
            start_msg.data = self.start_flag
            self.brake_pub.publish(start_msg)

    def _keyboard_listener(self):
        """Listen for keyboard input in a separate thread."""
        try:
            while rclpy.ok():
                key = input().strip().lower()
                if 's' in key:
                    if not self.start_flag:
                        self.start_flag = True
                        self.get_logger().info("Received 's' key press. Starting TTC monitoring.")
                    # Publish start signal to other nodes
                    start_msg = Bool()
                    start_msg.data = self.start_flag
                    self.brake_pub.publish(start_msg)
        except EOFError:
            self.get_logger().warn("Keyboard input stream closed.")
        except Exception as e:
            self.get_logger().error(f"Keyboard listener error: {e}")


    
def main(args=None):
    rclpy.init(args=args)
    node = StartController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()