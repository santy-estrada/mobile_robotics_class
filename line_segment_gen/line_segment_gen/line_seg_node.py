
import rclpy
from rclpy.node import Node
import math
import numpy as np

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class line_seg_node(Node):

    def __init__(self):
        super().__init__('line_seg_node')
        self.declare_parameter('epsilon', 0.05)
        self.declare_parameter('delta', 0.2)
        self.declare_parameter('seed_size', 6)


        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.marker_pub = self.create_publisher(
        Marker,
        '/line_segments',
        10
        )

        # PARAMETROS
        self.epsilon = self.get_parameter('epsilon').value
        self.delta = self.get_parameter('delta').value
        self.seed_size = self.get_parameter('seed_size').value

        self.get_logger().info("Nodo line segmentation iniciado.")


    def scan_callback(self, scan):

        self.points = []

        angle_min = scan.angle_min
        angle_inc = scan.angle_increment

        for i, r in enumerate(scan.ranges):

            if math.isinf(r) or math.isnan(r):
                continue

            theta = angle_min + i * angle_inc

            x = r * math.cos(theta)
            y = r * math.sin(theta)

            self.points.append((x, y, i))

        # Detectar seeds
        seeds = self.detect_seed_segments()
        self.publish_segments(seeds)
        self.get_logger().info(f"Seeds detectadas: {len(seeds)}")

    def fit_line_orthogonal(self, pts):

        X = np.array([[p[0], p[1]] for p in pts])

        centroid = np.mean(X, axis=0)

        cov = np.cov(X.T)

        eigvals, eigvecs = np.linalg.eig(cov)

        normal = eigvecs[:, np.argmin(eigvals)]

        a, b = normal
        c = -(a * centroid[0] + b * centroid[1])

        return a, b, c


    def point_line_distance(self, a, b, c, x, y):
        return abs(a * x + b * y + c)


    def detect_seed_segments(self):

        seeds = []
        n = len(self.points)
        i = 0

        while i <= n - self.seed_size:

            window = self.points[i:i + self.seed_size]
            # Ajustar recta inicial
            a, b, c = self.fit_line_orthogonal(window)

            valid = True
            # Condición epsilon
            for x, y, _ in window:
                d = self.point_line_distance(a, b, c, x, y)
                if d > self.epsilon:
                    valid = False
                    break

            if not valid:
                i += 1
                continue

            # Condición delta
            for j in range(len(window) - 1):
                x1, y1, _ = window[j]
                x2, y2, _ = window[j + 1]

                dist = math.hypot(x2 - x1, y2 - y1)
                if dist > self.delta:
                    valid = False
                    break

            if not valid:
                i += 1
                continue

            segment, a, b, c, next_index = self.comprobar(
                a, b, c, i, self.seed_size
            )

            seeds.append((segment, a, b, c))

            i = next_index

        return seeds

    def comprobar(self, a, b, c, ini, leng):

        segment_points = self.points[ini:ini + leng]
        current_length = leng
        n = len(self.points)

        while True:

            next_index = ini + current_length
            if next_index >= n:
                break

            new_point = self.points[next_index]

            # Condición epsilon
            d = self.point_line_distance(a, b, c, new_point[0], new_point[1])
            if d > self.epsilon:
                break

            # Condición delta
            x1, y1, _ = segment_points[-1]
            x2, y2, _ = new_point

            dist = math.hypot(x2 - x1, y2 - y1)
            if dist > self.delta:
                break

            # Agregar punto
            segment_points.append(new_point)
            current_length += 1

            # Recalcular recta
            a, b, c = self.fit_line_orthogonal(segment_points)

        end_index = ini + current_length
        return segment_points, a, b, c, end_index

    def publish_segments(self, seeds):

        marker = Marker()

        marker.header.frame_id = "lidar_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "line_segments"
        marker.id = 0

        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.03  # grosor de la línea

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.points = []
        #debe convertir segmentoss en red de puntos
        for segment, _, _, _ in seeds:

            if len(segment) < 2:
                continue

            # Tomamos primer y último punto del segmento
            x1, y1, _ = segment[0]
            x2, y2, _ = segment[-1]

            p1 = Point()
            p1.x = x1
            p1.y = y1
            p1.z = 0.0

            p2 = Point()
            p2.x = x2
            p2.y = y2
            p2.z = 0.0

            marker.points.append(p1)
            marker.points.append(p2)

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = line_seg_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()