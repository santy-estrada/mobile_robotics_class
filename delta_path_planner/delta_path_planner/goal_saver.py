#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

import json
import os


class GoalPoseSaver(Node):

    def __init__(self):
        super().__init__('goal_pose_saver')

        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.waypoints = []

        # Archivo de salida
        self.output_file = 'waypoints.json'

        self.get_logger().info('Nodo GoalPoseSaver iniciado.')

    def goal_callback(self, msg):

        x = round(msg.pose.position.x, 2)
        y = round(msg.pose.position.y, 2)

        waypoint = {
            "x": x,
            "y": y
        }

        self.waypoints.append(waypoint)

        data = {
            "waypoints": self.waypoints
        }

        with open(self.output_file, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(
            f'Waypoint guardado: x={x}, y={y}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GoalPoseSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()