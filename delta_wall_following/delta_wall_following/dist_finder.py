import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped
import math
import numpy as np

def getRange(ranges,th,angle_min,angle_increment):
    # andulo a 90 grados del auto por la derecha
    theta_B = -math.pi / 2                 # -90°
    
    idx_B = int((theta_B - angle_min) / angle_increment)
    theta_A = theta_B + math.radians(th)
    idx_A = int((theta_A - angle_min) / angle_increment)
    r_B=ranges[idx_B]
    r_A=ranges[idx_A]
    return r_B,r_A


class dist_finder(Node):

    def __init__(self):
        super().__init__('dist_finder')

        # -- PARAMETERS --
        self.declare_parameter('wall_distance', 3.0) # Desired distance to the wall in meters
        self.declare_parameter('body_velocity', 6.0)   # Forward velocity of the robot in m/s
        self.declare_parameter('angle_th', 15.0)        # Angle between the two laser rays in degrees

        self.body_vel = float(self.get_parameter('body_velocity').value)
        self.desired_r = float(self.get_parameter('wall_distance').value)
        self.angle_th = float(self.get_parameter('angle_th').value)

        self.current_cmd_vel = TwistStamped()    # Last received command velocity


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

        self.get_logger().info('Nodo de segmentación láser iniciado')
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
        # clean the data a bit
        ranges = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max)
        
        angulo=self.angle_th # angulo del segundo rayo respecto al primero, en grados
        r_B, r_A = getRange(
        ranges,
        angulo,
        msg.angle_min,
        msg.angle_increment
        )

        alpha= math.atan((r_A* math.cos(math.radians(angulo)) - r_B)/(r_A * math.sin(math.radians(angulo))))

        AB=r_B*math.cos(alpha)
        AC=self.current_cmd_vel.twist.linear.x*elapsed #asumiendo que la velocidad entra en m/s, dado que elapsed está en metros por segundo, crear una sub a donde se tenag la velocidad del body y meter el valor en la variable self.body_vel
        CD=AB+AC*math.sin(alpha)
        error=self.desired_r-CD

        cmd = Twist()
        cmd.linear.x = error      # Distance error from wall (y)
        cmd.linear.y = AC          # Distance travelled (L)
        cmd.angular.z = alpha      # Angular error (theta)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"Error: {error:.3f}m | Alpha: {alpha:.3f} rad = {math.degrees(alpha):.1f} | CD = {CD:.3f}m | AB = {AB:.3f}m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = dist_finder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()