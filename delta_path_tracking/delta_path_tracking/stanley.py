#!/usr/bin/env python3
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path

import tf2_ros
from tf2_ros import TransformException

# Requires: ros-<distro>-tf2-geometry-msgs
from tf2_geometry_msgs import do_transform_pose_stamped


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class StanleyNode(Node):
    """
    Stanley path tracking controller for cmd_vel (diff-drive style):
      - subscribes: nav_msgs/Path
      - publishes: geometry_msgs/TwistStamped
      - uses TF: path.header.frame_id -> base_link
    """

    def __init__(self):
        super().__init__("stanley_node")

        # ---- Parameters
        self.declare_parameter("path_topic", "planned_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("base_frame", "base_link")

        # Control
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("v_nominal", 1.0)      # m/s (kept constant)
        self.declare_parameter("max_omega", 1.5)      # rad/s
        self.declare_parameter("goal_tolerance", 0.25)

        # Stanley control parameters
        self.declare_parameter("stanley_k", 1.0)
        self.declare_parameter("velocity_softening", 0.1)

        # Small eps to avoid division by zero
        self.declare_parameter("eps", 1e-6)

        # TF timeout
        self.declare_parameter("tf_timeout_sec", 0.2)

        # ---- Read parameters
        self.path_topic = self.get_parameter("path_topic").value
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.base_frame = self.get_parameter("base_frame").value

        self.rate_hz = max(float(self.get_parameter("control_rate_hz").value), 1.0)
        self.v_nominal = max(float(self.get_parameter("v_nominal").value), 0.0)
        self.max_omega = float(self.get_parameter("max_omega").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance").value)

        self.k_stanley = float(self.get_parameter("stanley_k").value)
        self.v_soft = max(float(self.get_parameter("velocity_softening").value), 0.0)

        self.eps = float(self.get_parameter("eps").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)

        # ---- ROS interfaces
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, 10)

        # ---- TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- State
        self.path: List[PoseStamped] = []
        self.path_frame: Optional[str] = None
        self.has_path = False
        self.last_closest_index = 0

        # ---- Timer
        dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(dt, self.on_timer)

        self.get_logger().info(
            f"Stanley controller listening on {self.path_topic}, publishing {self.cmd_topic}, "
            f"v={self.v_nominal:.2f} m/s, rate={self.rate_hz:.1f} Hz"
        )

    def on_path(self, msg: Path) -> None:
        self.path = list(msg.poses)
        self.path_frame = msg.header.frame_id if msg.header.frame_id else None
        self.has_path = len(self.path) > 0 and self.path_frame is not None
        self.last_closest_index = 0

        if not self.has_path:
            self.get_logger().warn("Received empty path or missing frame_id.")
        else:
            self.get_logger().info(
                f"Received path with {len(self.path)} poses in frame '{self.path_frame}'."
            )

    def on_timer(self) -> None:
        # Safety condition: no path -> stop
        if not self.has_path:
            self.publish_stop()
            return

        # 1) Get transform base_link <- path_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.path_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed ({self.path_frame} -> {self.base_frame}): {ex}")
            self.publish_stop()
            return

        # 2) Goal condition: if close enough to final pose -> stop
        goal_pose_b = self.transform_pose_to_base(self.path[-1], tf)
        if goal_pose_b is None:
            self.publish_stop()
            return

        goal_dist = math.hypot(goal_pose_b.pose.position.x, goal_pose_b.pose.position.y)
        if goal_dist <= self.goal_tol:
            self.publish_stop()
            return

        # 3) Compute Stanley heading and cross-track errors
        errors = self.compute_stanley_errors(tf)
        if errors is None:
            self.publish_stop()
            return

        heading_error, cross_track_error, idx = errors
        self.last_closest_index = idx

        # 4) Stanley control law: delta = heading_error + atan2(k * e_ct, v)
        v_cmd = self.v_nominal
        denom = max(abs(v_cmd), self.v_soft, self.eps)
        delta = wrap_to_pi(
            heading_error + math.atan2(self.k_stanley * cross_track_error, denom)
        )
        omega = clamp(delta, -self.max_omega, self.max_omega)

        # 5) Publish cmd_vel
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(v_cmd)
        cmd.twist.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def transform_pose_to_base(self, pose_st: PoseStamped, tf) -> Optional[PoseStamped]:
        """Transform PoseStamped from path_frame into base_frame using TF2."""
        try:
            return do_transform_pose_stamped(pose_st, tf)
        except Exception as ex:
            self.get_logger().warn(f"Pose transform failed: {ex}")
            return None

    def _get_transformed_xy(
        self,
        idx: int,
        tf,
        cache: Dict[int, Tuple[float, float]],
    ) -> Optional[Tuple[float, float]]:
        if idx < 0 or idx >= len(self.path):
            return None

        if idx in cache:
            return cache[idx]

        pose_b = self.transform_pose_to_base(self.path[idx], tf)
        if pose_b is None:
            return None

        xy = (pose_b.pose.position.x, pose_b.pose.position.y)
        cache[idx] = xy
        return xy

    def _segment_errors(
        self,
        p0: Tuple[float, float],
        p1: Tuple[float, float],
    ) -> Optional[Tuple[float, float]]:
        x0, y0 = p0
        x1, y1 = p1

        dx = x1 - x0
        dy = y1 - y0
        seg_norm = math.hypot(dx, dy)
        if seg_norm <= self.eps:
            return None

        # Robot heading is zero in base_link, so heading error is path heading directly.
        heading_error = wrap_to_pi(math.atan2(dy, dx))

        # Signed distance from robot origin to path segment line (left is positive).
        cross_track_error = (dx * y0 - dy * x0) / seg_norm
        return heading_error, cross_track_error

    def compute_stanley_errors(self, tf) -> Optional[Tuple[float, float, int]]:
        """
        Compute Stanley terms from the closest path segment in base frame.
        Returns (heading_error, cross_track_error, closest_index).
        """
        n = len(self.path)
        start = min(max(self.last_closest_index, 0), n - 1)

        cache: Dict[int, Tuple[float, float]] = {}
        closest_idx: Optional[int] = None
        closest_dist_sq = float("inf")

        for i in range(start, n):
            xy = self._get_transformed_xy(i, tf, cache)
            if xy is None:
                continue

            x, y = xy
            dist_sq = x * x + y * y
            if dist_sq < closest_dist_sq:
                closest_dist_sq = dist_sq
                closest_idx = i

        if closest_idx is None:
            return None

        if closest_idx < n - 1:
            p0 = self._get_transformed_xy(closest_idx, tf, cache)
            p1 = self._get_transformed_xy(closest_idx + 1, tf, cache)
            if p0 is not None and p1 is not None:
                errors = self._segment_errors(p0, p1)
                if errors is not None:
                    heading_error, cross_track_error = errors
                    return heading_error, cross_track_error, closest_idx

        if closest_idx > 0:
            p0 = self._get_transformed_xy(closest_idx - 1, tf, cache)
            p1 = self._get_transformed_xy(closest_idx, tf, cache)
            if p0 is not None and p1 is not None:
                errors = self._segment_errors(p0, p1)
                if errors is not None:
                    heading_error, cross_track_error = errors
                    return heading_error, cross_track_error, closest_idx

        p_closest = self._get_transformed_xy(closest_idx, tf, cache)
        if p_closest is None:
            return None

        _, y = p_closest
        return 0.0, y, closest_idx

    def publish_stop(self) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = StanleyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()