import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist 


class TTCGapLoggerNode(Node):
    def __init__(self):
        super().__init__('ttc_gap_logger_node')
        self.declare_parameter('pub_logger', True)               #Flag to signal whether to publish error values for logging in ttc_gap_logger_node

        self.pub_logger = bool(self.get_parameter('pub_logger').value)
        self.last_scan = None
        self.clearance_angle = 20
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        self.create_subscription(Float32MultiArray, '/ttc_values', self._ttc_callback, 10)
        

        self.marker_pub = self.create_publisher(Marker, '/gap_marker', 10)

        self.ang_err_pub = self.create_publisher(
            Twist,
            '/cmd_ang_tcc',
            10
        )

        self.get_logger().info("Nodo TTC Gap Logger iniciado.")

    def _scan_callback(self, msg):
        self.last_scan = msg

    def _ttc_callback(self, msg):
        if self.last_scan is None:
            return

        data = list(msg.data)
        if len(data) < 721:
            return

        ttc = data[:720]
        direction_flag = data[720]
        # -------- Clearance measure indices --------
        offset = int(self.clearance_angle * 2)

        idx_1 = offset
        idx_2 = 360 - offset
        idx_3 = 360 + offset
        idx_4 = 720 - offset

        # Asegurar que estén dentro de rango [0, 719]
        indices_clearance = [
            idx_1 % 720,
            idx_2 % 720,
            idx_3 % 720,
            idx_4 % 720
        ]

        clearance_measure = [ttc[i] for i in indices_clearance]
        perpe_measure=ttc[181]
        N = len(ttc)
        angle_min = self.last_scan.angle_min
        angle_inc = self.last_scan.angle_increment

        # ---------- Build index lists based on angles ----------
        # Forward: angles in [-pi/2, +pi/2]
        # Find i_start and i_end such that angle(i) is within that interval
        # angle(i) = angle_min + i*angle_inc
        i_start = math.ceil(((-math.pi/2) - angle_min) / angle_inc)
        i_end   = math.floor(((+math.pi/2) - angle_min) / angle_inc)

        # Clamp to [0, N-1]
        i_start = max(0, min(N-1, i_start))
        i_end   = max(0, min(N-1, i_end))

        # Forward indices are contiguous if scan spans [-pi,pi]
        forward_indices = list(range(i_start, i_end + 1))

        # Backward indices are the complement (two segments). We make them continuous by concatenation:
        # segment_high: (i_end+1 .. N-1)  corresponds to angles (+90..+180)
        # segment_low : (0 .. i_start-1)  corresponds to angles (-180..-90)
        segment_high = list(range(i_end + 1, N))
        segment_low  = list(range(0, i_start))
        backward_indices_ordered = segment_high + segment_low  # continuous rear sweep

        # Decide which set to use based on flag
        if abs(direction_flag - 0.0) < 1e-6:
            indices = forward_indices
        else:
            indices = backward_indices_ordered

        if len(indices) == 0:
            return

        # ---------- Find largest gap of inf in that ordered index list ----------
        # A "gap" is consecutive indices IN THIS ORDER where ttc[idx] is inf
        largest_len = 0
        largest_start_k = None
        largest_end_k = None

        in_gap = False
        gap_start_k = 0

        for k, idx in enumerate(indices):
            is_inf = math.isinf(ttc[idx])

            if is_inf and not in_gap:
                in_gap = True
                gap_start_k = k

            elif (not is_inf) and in_gap:
                in_gap = False
                gap_end_k = k - 1
                gap_len = gap_end_k - gap_start_k + 1

                if gap_len > largest_len:
                    largest_len = gap_len
                    largest_start_k = gap_start_k
                    largest_end_k = gap_end_k

        # Close gap if ended with inf
        if in_gap:
            gap_end_k = len(indices) - 1
            gap_len = gap_end_k - gap_start_k + 1
            if gap_len > largest_len:
                largest_len = gap_len
                largest_start_k = gap_start_k
                largest_end_k = gap_end_k

        if largest_len == 0:
            return  # no inf gaps

        # Center index of the largest gap (in the ordered list)
        center_k = (largest_start_k + largest_end_k) // 2
        center_index = indices[center_k]  # this is the real scan ray index 0..N-1

        # ---------- Convert to angle ----------
        angle_rad = angle_min + center_index * angle_inc
        # Normalize to [-pi, pi] (optional but nice)
        while angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        while angle_rad < -math.pi:
            angle_rad += 2 * math.pi

        # Publish marker
        self._publish_gap_marker(angle_rad)
        
        cmd_ang = Twist()
        mark_alert = 0.0
        if angle_rad > 0 and clearance_measure[2] < 1.7:
            angle_rad = -1.4 * angle_rad
            mark_alert = 1.0
        if angle_rad < 0 and clearance_measure[2] < 0.8:
            angle_rad = 1.5 * angle_rad
            mark_alert = 1.0
        if angle_rad < 0 and clearance_measure[1] < 1.0:
            angle_rad = -2.0 * angle_rad
            mark_alert = 1.0

        

        cmd_ang.angular.z = angle_rad
        cmd_ang.angular.y = mark_alert
        self.ang_err_pub.publish(cmd_ang)
        
        # Logging
        if self.pub_logger:
            self.get_logger().info(f"clearances: {perpe_measure} ")

    def _publish_gap_marker(self, angle_rad):
        marker = Marker()
        marker.header.frame_id = self.last_scan.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "gap_direction"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # yaw -> quaternion (z,w)
        qz = math.sin(angle_rad / 2.0)
        qw = math.cos(angle_rad / 2.0)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        marker.scale.x = 1.0
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TTCGapLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()