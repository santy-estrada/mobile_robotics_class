#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy

class OdometryFusionNode(Node):
    def __init__(self):
        super().__init__('odometry_fusion_node')

        # ── Estado ─────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.back_button_pressed = False

        self.prev_time = self.get_clock().now()

        # ── Parámetro del botón de retroceso ───
        self.declare_parameter('back_button_index', 5)  # RB en Logitech F710
        self.back_button_index = self.get_parameter('back_button_index').value

        # ── Subscripciones ─────────────────────
        self.create_subscription(Float64, '/encoder/vel', self.vel_cb, 10)
        self.create_subscription(Pose, '/robot1/orientation', self.ori_cb, 10)
        self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        # ── Publicador ─────────────────────────
        self.pub = self.create_publisher(Pose, '/robot1/pose', 10)

    def joy_cb(self, msg: Joy):
        if len(msg.buttons) > self.back_button_index:
            self.back_button_pressed = (msg.buttons[self.back_button_index] == 1)

    def vel_cb(self, msg):
        self.v = msg.data
        self.update()

    def ori_cb(self, msg):
        q = msg.orientation
        self.theta = np.arctan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y**2 + q.z**2)
        )

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        if dt <= 0:
            return

        direccion = -1.0 if self.back_button_pressed else 1.0

        self.x += self.v * np.cos(self.theta) * dt * direccion
        self.y += self.v * np.sin(self.theta) * dt * direccion

        # ── Publicar pose completa ─────────────
        self.get_logger().info(
                f"[X: {self.x:.2f} | Y:{self.y:2f} "
         )  
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = 0.0

        pose.orientation.w = np.cos(self.theta/2)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = np.sin(self.theta/2)

        self.pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()