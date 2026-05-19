#!/usr/bin/env python3
"""
odometry_node.py
Recibe la pose desde /robot1/pose y publica el TF odom → base_link.
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Pose
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # ── Parámetros ──────────────────────────────────────────────────
        self.declare_parameter('pose_topic',     '/robot1/pose')
        self.declare_parameter('odom_topic',     '/odom')
        self.declare_parameter('parent_frame',   'odom')
        self.declare_parameter('child_frame',    'base_link')

        pose_topic = self.get_parameter('pose_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.parent       = self.get_parameter('parent_frame').value
        self.child        = self.get_parameter('child_frame').value

        # ── Estado ──────────────────────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.orientation = None

        # ── Publicadores ────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_br    = TransformBroadcaster(self)

        # ── Suscriptores ────────────────────────────────────────────────
        self.create_subscription(Pose, pose_topic, self.pose_cb, 10)

        self.get_logger().info(
            f"Pose ← '{pose_topic}'\n"
            f"Odom → '{odom_topic}' | TF: '{self.parent}'→'{self.child}'"
        )

    # ── Pose Callback: lee la posición y orientación y publica ───────────
    def pose_cb(self, msg: Pose):
        # Actualizamos la posición y orientación desde el tópico externo
        self.x = msg.position.x
        self.y = msg.position.y
        self.orientation = msg.orientation

        now = self.get_clock().now().to_msg()

        # ── Publicar Odometry ───────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.parent
        odom.child_frame_id  = self.child

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation   = self.orientation

        # Covarianza diagonal
        odom.pose.covariance[0]  = 0.05   # σ² x
        odom.pose.covariance[7]  = 0.05   # σ² y
        odom.pose.covariance[35] = 0.02   # σ² yaw

        # Twist básico
        odom.twist.twist.linear.x  = 0.0 
        odom.twist.twist.angular.z = 0.0
        odom.twist.covariance[0]   = 0.01
        odom.twist.covariance[35]  = 0.05

        self.odom_pub.publish(odom)

        # ── Publicar TF ─────────────────────────────────────────────────
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = self.parent
        tf.child_frame_id  = self.child

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation      = self.orientation

        self.tf_br.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()