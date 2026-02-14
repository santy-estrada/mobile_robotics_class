import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped
import math
import numpy as np

def getRange(ranges, th, angle_min, angle_increment):
    # ángulo a 90 grados del auto por la derecha
    theta_B = -math.pi / 2                 # -90°
    
    idx_B = int((theta_B - angle_min) / angle_increment)
    theta_A = theta_B + math.radians(th)
    idx_A = int((theta_A - angle_min) / angle_increment)
    r_B = ranges[idx_B]
    r_A = ranges[idx_A]
    return r_B, r_A

class dist_finder(Node):
    def __init__(self):
        super().__init__('dist_finder')
        # -- PARAMETERS --
        self.declare_parameter('wall_distance', 2.0)
        self.declare_parameter('body_velocity', 1.0)
        self.declare_parameter('angle_th', 60.0)
        
        self.body_vel = float(self.get_parameter('body_velocity').value)
        self.desired_r = float(self.get_parameter('wall_distance').value)
        self.angle_th = float(self.get_parameter('angle_th').value)
        
        self.current_cmd_vel = TwistStamped()
        
        # Estado de búsqueda del muro
        self.wall_lost = False
        self.accumulated_search_error = 0.0
        self.last_valid_error = 0.0
        self.last_valid_alpha = 0.0
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.subscription = self.create_subscription(
            TwistStamped,
            '/diffdrive_controller/cmd_vel',
            self._cmd_vel_callback,
            10
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            '/error',
            10
        )
        self.get_logger().info('Nodo de seguimiento de muro iniciado')
        self.past_time = None
    
    def _cmd_vel_callback(self, msg: TwistStamped):
        """Store the latest commanded velocity from the controller."""
        self.current_cmd_vel = msg
    
    def scan_callback(self, msg: LaserScan):
        current_time = self.get_clock().now()
        if self.past_time is None:
            self.past_time = current_time
            return
        
        elapsed = (current_time - self.past_time).nanoseconds * 1e-9
        self.past_time = current_time
        
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max)
        
        angulo = self.angle_th
        r_B, r_A = getRange(ranges, angulo, msg.angle_min, msg.angle_increment)

        
        cmd = Twist()
        

        alpha = math.atan((r_A * math.cos(math.radians(angulo)) - r_B) / 
                            (r_A * math.sin(math.radians(angulo))))
        AB = r_B * math.cos(alpha)
        AC = self.current_cmd_vel.twist.linear.x * elapsed
        CD = AB + AC * math.sin(alpha)
        error = self.desired_r - CD
        
        # Guardar valores válidos
        self.last_valid_error = error
        self.last_valid_alpha = alpha
        
        cmd.linear.x = error
        cmd.linear.y = AC
        cmd.angular.z = alpha
        
        self.get_logger().info(
            f"NORMAL | Error: {error:.3f}m | Alpha: {math.degrees(alpha):.1f}° | "
            f"CD: {CD:.3f}m | r_A: {r_A:.2f}m | r_B: {r_B:.2f}m"
        )
        

        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = dist_finder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()