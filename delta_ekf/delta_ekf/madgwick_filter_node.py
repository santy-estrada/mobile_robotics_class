#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Pose

from delta_ekf.madgwick_filter import MadgwickAHRS

import tf2_ros


SAMPLE_PERIOD = 1.0 / 10.0
BETA = 0.02
FRAME_ID = "base_link"


def quat_to_rot(qx, qy, qz, qw):
    """Quaternion → rotation matrix (3x3)"""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx*qx + qy*qy)]
    ])


class MadgwickNode(Node):
    def __init__(self):
        super().__init__("madgwick_ahrs_node")

        self._ahrs = MadgwickAHRS(sample_period=SAMPLE_PERIOD, beta=BETA)
        self._latest_mag = np.array([1.0, 0.0, 0.0])

        self._pub = self.create_publisher(Pose, "/robot1/orientation", 10)

        self.create_subscription(MagneticField, "/imu/mag", self._mag_callback, 10)
        self.create_subscription(Imu, "/imu/data_raw", self._imu_callback, 10)

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._R = None

        self.get_logger().info("Madgwick + TF iniciado")

    def _mag_callback(self, msg):
        self._latest_mag = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
        ])

    def _get_rotation(self):
        """Obtiene R (imu_link → base_link)"""
        if self._R is not None:
            return self._R

        try:
            tf_msg = self._tf_buffer.lookup_transform(
                "base_link",
                "imu_link",
                rclpy.time.Time()
            )

            q = tf_msg.transform.rotation
            self._R = quat_to_rot(q.x, q.y, q.z, q.w)

            self.get_logger().info(f"R cargada:\n{np.round(self._R, 3)}")

        except Exception:
            return None

        return self._R

    def _imu_callback(self, msg):
        raw_gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])

        raw_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])

        mag = self._latest_mag

        R = self._get_rotation()
        if R is None:
            return
        # corrección giroscopio y acelerometro
        gyro = R @ raw_gyro
        accel = R @ raw_accel   

        self._ahrs.update(gyro, accel, mag)

        q = self._ahrs.quaternion

        pose_msg = Pose()
        pose_msg.orientation.w = float(q[0])
        pose_msg.orientation.x = float(q[1])
        pose_msg.orientation.y = float(q[2])
        pose_msg.orientation.z = float(q[3])

        self.get_logger().info(f"accel corregido: {np.round(accel,2)}")

        self._pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MadgwickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()