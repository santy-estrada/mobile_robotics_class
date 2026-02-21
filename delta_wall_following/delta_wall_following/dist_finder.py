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
        self.declare_parameter('wall_distance', 0.6)
        self.declare_parameter('body_velocity', 1.0)
        self.declare_parameter('angle_th', 50.0)
        self.declare_parameter('max_discontinuity', 2.5)
        self.declare_parameter('pub_logger', True)      #Flag to pub logger info
        
        self.body_vel = float(self.get_parameter('body_velocity').value)
        self.desired_r = float(self.get_parameter('wall_distance').value)
        self.max_discontinuity = float(self.get_parameter('max_discontinuity').value)
        self.angle_th = float(self.get_parameter('angle_th').value)
        self.pub_logger = bool(self.get_parameter('pub_logger').value)
        
        self.current_cmd_vel = TwistStamped()
        
        
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

    def is_valid_wall_reading(self, r_A, r_B):
        """
        Verifica si las lecturas de los rayos son válidas para seguimiento de muro.
        Retorna (es_valido, tipo_problema)
        """

        if abs(r_A - r_B/math.cos(math.radians(self.angle_th))) > self.max_discontinuity:
            return False, "discontinuidad"
    
        return True, None
    
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

        # Validar lecturas
        is_valid, problem_type = self.is_valid_wall_reading(r_A, r_B)

        cmd = Twist()
        
        if is_valid:
            alpha = math.atan((r_A * math.cos(math.radians(angulo)) - r_B) / 
                                (r_A * math.sin(math.radians(angulo))))
            AB = r_B * math.cos(alpha)
            AC = self.current_cmd_vel.twist.linear.x * elapsed
            CD = AB + AC * math.sin(alpha)
            error = self.desired_r - CD
            
            cmd.linear.x = error
            cmd.linear.y = AC
            cmd.angular.z = alpha
            cmd.linear.z = 0.0  # Indicador de seguimiento normal

            if self.pub_logger:
                self.get_logger().info(
                f"NORMAL | Error: {error:.3f}m | Alpha: {math.degrees(alpha):.1f}° | "
                f"CD: {CD:.3f}m | r_A: {r_A:.2f}m | r_B: {r_B:.2f}m"
        )

        else:
            cmd.linear.z = 1.0  # Indicador de búsqueda de muro
            if self.pub_logger:
                self.get_logger().info(
                    f"WALL LOST | Tipo de problema: {problem_type} | r_A - r_B/cos(th): {abs(r_A - r_B/math.cos(math.radians(self.angle_th))):.2f}m | "
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