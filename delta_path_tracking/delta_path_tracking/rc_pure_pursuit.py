#!/usr/bin/env python3
import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64, Int32

import tf2_ros
from tf2_ros import TransformException

# Requires: ros-<distro>-tf2-geometry-msgs
from tf2_geometry_msgs import do_transform_pose_stamped


# Identified steering regression model coefficients (signal <-> angle in degrees).
STEERING_MODEL_A = 10.422533
STEERING_MODEL_B = 1.026880


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def signal_to_angle_deg(signal: float) -> float:
    signal = clamp(signal, -3.0, 3.0)
    return math.copysign(STEERING_MODEL_A * abs(signal) ** STEERING_MODEL_B, signal)


def angle_deg_to_signal(target_angle_deg: float) -> float:
    # Keep requested limit at +/-33 deg and always enforce signal hardware limits.
    target_angle_deg = clamp(target_angle_deg, -33.0, 33.0)
    if target_angle_deg == 0.0:
        return 0.0

    signal = math.copysign(
        (abs(target_angle_deg) / STEERING_MODEL_A) ** (1.0 / STEERING_MODEL_B),
        target_angle_deg,
    )
    return clamp(signal, -3.0, 3.0)


class PurePursuitNode(Node):
    """
    Minimal but robust Pure Pursuit for cmd_vel (diff-drive style):
      - subscribes: nav_msgs/Path
      - publishes: geometry_msgs/Twist
      - uses TF: path.header.frame_id -> base_link
    """

    def __init__(self):
        super().__init__("pure_pursuit_node")
        # ---- Parameters
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("velocity_topic", "/encoder/vel")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")

        self.declare_parameter("base_frame", "base_link")

        # Control
        self.declare_parameter("control_rate_hz", 25.0)
        self.declare_parameter("max_cmd_velocity", 3.0)           # m/s - hard cap on output velocity command
        self.declare_parameter("use_speed_adaptive", False)       # Use heading-error-based speed adjustment
        self.declare_parameter("speed_nominal", 2.2)             # m/s - nominal speed when adaptive is disabled
        self.declare_parameter("speed_adaptive_min", 0.5)         # m/s - minimum speed in adaptive mode
        self.declare_parameter("speed_heading_coupling", 1.0)     # How much heading error affects speed
        self.declare_parameter("cmd_smoothing_factor", 0.3)       # Smoothing alpha for speed command
        self.declare_parameter("max_omega", 1.5)            # rad/s
        self.declare_parameter("goal_tolerance", 0.25)      # m
        self.declare_parameter("goal_index_window", 100)    # points from end required to accept goal
        self.declare_parameter("use_StartFlag", True)
        self.declare_parameter("use_ttc", True)
        self.declare_parameter("use_ttc_brake_speed_reduction", False)
        self.declare_parameter("ttc_brake_speed", 0.9)
        self.declare_parameter("ttc_brake_topic", "/brake_active")
        self.declare_parameter("ttc_brake_warn_topic", "/brake_warn")
        self.declare_parameter("ttc_dir_topic", "/dir_brake")
        self.declare_parameter("ttc_turn_boost", 0.7)
        self.declare_parameter("max_omega_ttc", 2.5)
        self.declare_parameter("pub_debug", True)
        self.declare_parameter("pub_errs", True)
        self.declare_parameter("cross_track_error_topic", "/cross_track_error")
        self.declare_parameter("heading_error_topic", "/heading_error")
        self.declare_parameter("delta_topic", "/delta")
        self.declare_parameter("use_ackermann_steering_cmd", False)
        self.declare_parameter("wheelbase", 0.30)
        self.declare_parameter("steering_angle_limit_deg", 33.0)

        # Lookahead: Ld = clamp(L0 + k*v, Lmin, Lmax)
        self.declare_parameter("lookahead_L0", 0.6)         # m
        self.declare_parameter("lookahead_kv", 0.5)         # s  (0 disables adaptation)
        self.declare_parameter("lookahead_min", 0.4)        # m
        self.declare_parameter("lookahead_max", 2.0)        # m

        # Small eps to avoid division by zero
        self.declare_parameter("eps", 1e-6)

        # TF timeout
        self.declare_parameter("tf_timeout_sec", 0.2)

        # ---- Read parameters
        self.path_topic = self.get_parameter("path_topic").value
        self.velocity_topic = self.get_parameter("velocity_topic").value
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.base_frame = self.get_parameter("base_frame").value

        self.rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.max_cmd_velocity = float(self.get_parameter("max_cmd_velocity").value)
        self.use_speed_adaptive = bool(self.get_parameter("use_speed_adaptive").value)
        self.speed_nominal = clamp(float(self.get_parameter("speed_nominal").value), 0.0, self.max_cmd_velocity)
        self.speed_adaptive_min = max(float(self.get_parameter("speed_adaptive_min").value), 0.0)
        self.speed_heading_coupling = max(float(self.get_parameter("speed_heading_coupling").value), 0.0)
        self.cmd_smoothing_factor = clamp(float(self.get_parameter("cmd_smoothing_factor").value), 0.0, 1.0)
        self.max_omega = float(self.get_parameter("max_omega").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance").value)
        self.goal_index_window = int(self.get_parameter("goal_index_window").value)
        self.use_start_flag = bool(self.get_parameter("use_StartFlag").value)
        self.use_ttc = bool(self.get_parameter("use_ttc").value)
        self.use_ttc_brake_speed_reduction = bool(
            self.get_parameter("use_ttc_brake_speed_reduction").value
        )
        self.ttc_brake_speed = max(float(self.get_parameter("ttc_brake_speed").value), 0.0)
        self.ttc_brake_topic = str(self.get_parameter("ttc_brake_topic").value)
        self.ttc_brake_warn_topic = str(self.get_parameter("ttc_brake_warn_topic").value)
        self.ttc_dir_topic = str(self.get_parameter("ttc_dir_topic").value)
        self.ttc_turn_boost = abs(float(self.get_parameter("ttc_turn_boost").value))
        self.pub_debug = bool(self.get_parameter("pub_debug").value)
        self.pub_errs = bool(self.get_parameter("pub_errs").value)
        self.cross_track_error_topic = str(self.get_parameter("cross_track_error_topic").value)
        self.heading_error_topic = str(self.get_parameter("heading_error_topic").value)
        self.delta_topic = str(self.get_parameter("delta_topic").value)
        self.use_ackermann_steering_cmd = bool(
            self.get_parameter("use_ackermann_steering_cmd").value
        )
        self.wheelbase = max(float(self.get_parameter("wheelbase").value), 1e-6)
        self.steering_angle_limit_deg = max(
            float(self.get_parameter("steering_angle_limit_deg").value),
            0.0,
        )
        self.steering_angle_limit_rad = math.radians(self.steering_angle_limit_deg)

        self.L0 = float(self.get_parameter("lookahead_L0").value)
        self.kv = float(self.get_parameter("lookahead_kv").value)
        self.Lmin = float(self.get_parameter("lookahead_min").value)
        self.Lmax = float(self.get_parameter("lookahead_max").value)

        self.eps = float(self.get_parameter("eps").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.max_omega_ttc = max(float(self.get_parameter("max_omega_ttc").value), self.max_omega)

        # ---- ROS interfaces
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, 10)
        self.vel_sub = self.create_subscription(Float64, self.velocity_topic, self.on_velocity, 10)

        # Optional error publishers
        self.cross_track_error_pub = None
        self.heading_error_pub = None
        self.delta_pub = None
        if self.pub_errs:
            self.cross_track_error_pub = self.create_publisher(
                Float64,
                self.cross_track_error_topic,
                10,
            )
            self.heading_error_pub = self.create_publisher(
                Float64,
                self.heading_error_topic,
                10,
            )
            self.delta_pub = self.create_publisher(
                Float64,
                self.delta_topic,
                10,
            )
            self.get_logger().info(
                "Error publishers enabled "
                f"(cte: {self.cross_track_error_topic}, "
                f"heading: {self.heading_error_topic}, "
                f"compound: {self.delta_topic})."
            )
        else:
            self.get_logger().info("Error publishers disabled.")

        # TTC-assisted turning state
        self.ttc_brake_active = False
        self.ttc_brake_warn_active = False
        self.ttc_obstacle_dir = 3  # 0:right obstacle, 1:left obstacle, 3:indeterminate
        if self.use_ttc:
            self.ttc_brake_sub = self.create_subscription(
                Bool,
                self.ttc_brake_topic,
                self.ttc_brake_callback,
                10,
            )
            self.ttc_dir_sub = self.create_subscription(
                Int32,
                self.ttc_dir_topic,
                self.ttc_dir_callback,
                10,
            )
            self.ttc_brake_warn_sub = self.create_subscription(
                Bool,
                self.ttc_brake_warn_topic,
                self.ttc_brake_warn_callback,
                10,
            )
            self.get_logger().info(
                f"TTC assist enabled (brake: {self.ttc_brake_topic}, dir: {self.ttc_dir_topic})."
            )
        else:
            self.get_logger().info("TTC assist disabled.")

        self.start_flag = not self.use_start_flag
        if self.use_start_flag:
            self.get_logger().info("Start flag mode enabled. Waiting for /start signal to begin.")
            self.start_sub = self.create_subscription(
                Bool,
                "/start",
                self.start_callback,
                10,
            )
        else:
            self.get_logger().info("Start flag mode disabled.")

        # ---- TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- State
        self.path: List[PoseStamped] = []
        self.path_frame: Optional[str] = None
        self.has_path = False
        self.last_target_index = 0  # progressing index
        self.first_run = True
        self.real_vel = 0.0  # Actual longitudinal velocity from encoder/odometry
        self.prev_v_cmd = self.speed_nominal if not self.use_speed_adaptive else self.max_cmd_velocity

        # ---- Timer
        dt = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(dt, self.on_timer)

        self.get_logger().info(f"Pure Pursuit listening on {self.path_topic}, publishing {self.cmd_topic}")
        self.get_logger().info(
            f"Speed control: max_cmd={self.max_cmd_velocity:.2f} m/s, "
            f"adaptive_mode={self.use_speed_adaptive}"
        )
        if self.use_speed_adaptive:
            self.get_logger().info(
                f"Adaptive speed: min={self.speed_adaptive_min:.2f} m/s, "
                f"heading_coupling={self.speed_heading_coupling:.2f}, "
                f"smoothing={self.cmd_smoothing_factor:.2f}"
            )
        else:
            self.get_logger().info(f"Fixed speed mode: nominal={self.speed_nominal:.2f} m/s")
        self.get_logger().info(
            "Goal gating enabled "
            f"(goal_tolerance={self.goal_tol:.2f} m, index_window={self.goal_index_window} points, "
            "<=0 disables gate)."
        )
        self.get_logger().info(
            "TTC brake speed cap "
            f"({'enabled' if self.use_ttc_brake_speed_reduction else 'disabled'}, "
            f"speed={self.ttc_brake_speed:.2f} m/s when brake is active)."
        )
        self.get_logger().info(
            "Ackermann steering command mode "
            f"({'enabled' if self.use_ackermann_steering_cmd else 'disabled'}, "
            f"wheelbase={self.wheelbase:.3f} m, "
            f"steer_limit={self.steering_angle_limit_deg:.2f} deg)."
        )

    def start_callback(self, msg: Bool) -> None:
        if msg.data:
            if not self.start_flag:
                self.get_logger().info("Received start signal. Starting control.")
            self.start_flag = True

    def on_velocity(self, msg: Float64) -> None:
        """Update real longitudinal velocity from encoder/odometry."""
        self.real_vel = max(msg.data, 0.0)  # Clamp to non-negative

    def ttc_brake_callback(self, msg: Bool) -> None:
        if not self.first_run:
            self.ttc_brake_active = bool(msg.data)
        if self.pub_debug:
            state = "ACTIVE" if self.ttc_brake_active else "INACTIVE"
            self.get_logger().info(f"TTC brake state changed: {state}")

    def ttc_brake_warn_callback(self, msg: Bool) -> None:
        if not self.first_run:
            self.ttc_brake_warn_active = bool(msg.data)
        if self.pub_debug:
            state = "ACTIVE" if self.ttc_brake_warn_active else "INACTIVE"
            self.get_logger().info(f"TTC brake WARNING state changed: {state}")

    def ttc_dir_callback(self, msg: Int32) -> None:
        direction = int(msg.data)
        if direction in (0, 1, 3):
            self.ttc_obstacle_dir = direction
        else:
            self.ttc_obstacle_dir = 3

        if self.pub_debug:
            dir_str = {0: "RIGHT", 1: "LEFT", 3: "INDETERMINATE"}.get(self.ttc_obstacle_dir, "UNKNOWN")
            self.get_logger().info(f"TTC obstacle direction updated: {dir_str} ({self.ttc_obstacle_dir})")

    def apply_ttc_brake_speed_reduction(self, speed_cmd: float, upper_bound: float) -> float:
        speed_cmd = clamp(speed_cmd, 0.0, upper_bound)

        if not self.use_ttc_brake_speed_reduction:
            return speed_cmd
        if not self.use_ttc or not self.ttc_brake_active:
            return speed_cmd

        brake_speed_cap = min(self.ttc_brake_speed, upper_bound)
        return clamp(min(speed_cmd, brake_speed_cap), 0.0, upper_bound)

    def publish_error_signals(
        self,
        cross_track_error: float,
        heading_error: float,
        delta: float,
    ) -> None:
        if not self.pub_errs:
            return

        if self.cross_track_error_pub is not None:
            msg = Float64()
            msg.data = float(cross_track_error)
            self.cross_track_error_pub.publish(msg)

        if self.heading_error_pub is not None:
            msg = Float64()
            msg.data = float(heading_error)
            self.heading_error_pub.publish(msg)

        if self.delta_pub is not None:
            msg = Float64()
            msg.data = float(delta)
            self.delta_pub.publish(msg)

    def apply_ttc_turn_assist(self, omega: float) -> float:
        omega_base = clamp(omega, -self.max_omega, self.max_omega)
        ttc_emergency = self.use_ttc and self.ttc_brake_active
        ttc_warning = self.use_ttc and self.ttc_brake_warn_active and not ttc_emergency

        # Apply assist for both emergency brake and warning levels.
        if not (ttc_emergency or ttc_warning):
            return omega_base

        # Build an avoidance steering target, then blend with the Pure Pursuit command.
        if self.ttc_obstacle_dir == 0:
            # Obstacle on right -> steer left.
            omega_avoid = abs(omega_base) + self.ttc_turn_boost
            if self.pub_debug:
                state = "brake" if ttc_emergency else "warning"
                self.get_logger().info(
                    f"TTC {state} active with right obstacle. Applying left turn assist."
                )
        elif self.ttc_obstacle_dir == 1:
            # Obstacle on left -> steer right.
            omega_avoid = -(abs(omega_base) + self.ttc_turn_boost)
            if self.pub_debug:
                state = "brake" if ttc_emergency else "warning"
                self.get_logger().info(
                    f"TTC {state} active with left obstacle. Applying right turn assist."
                )
        else:
            # Unknown side: keep Pure Pursuit steering to avoid spinning on stale direction info.
            return omega_base

        if ttc_warning:
            # Warning mode should still steer away, but less aggressively than full brake mode.
            omega_avoid *= 0.5

        omega_avoid = clamp(omega_avoid, -self.max_omega_ttc, self.max_omega_ttc)
        return omega_avoid

    def on_path(self, msg: Path) -> None:
        self.path = list(msg.poses)
        self.path_frame = msg.header.frame_id if msg.header.frame_id else None
        self.has_path = len(self.path) > 0 and self.path_frame is not None

        if not self.has_path:
            self.get_logger().warn("Received empty path or missing frame_id.")
        else:
            self.get_logger().info(f"Received path with {len(self.path)} poses in frame '{self.path_frame}'.")

    def on_timer(self) -> None:
        if not self.has_path:
            self.publish_stop()
            return

        if self.use_start_flag and not self.start_flag:
            self.publish_stop()
            return

        # 1) Get transform base_link <- path_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.path_frame,
                rclpy.time.Time(),  # latest
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed ({self.path_frame} -> {self.base_frame}): {ex}")
            self.publish_stop()
            return

        # 2) Check goal distance (transform last pose)
        goal_pose_b = self.transform_pose_to_base(self.path[-1], tf)
        if goal_pose_b is None:
            self.publish_stop()
            return

        goal_dx = goal_pose_b.pose.position.x
        goal_dy = goal_pose_b.pose.position.y
        goal_dist = math.hypot(goal_dx, goal_dy)
        n = len(self.path)

        # Mirror Stanley-style "near-end" gating to avoid immediate goal on closed loops.
        # If window <= 0, disable this gate and keep original tolerance-only behavior.
        if self.goal_index_window <= 0:
            goal_window_reached = True
        else:
            # Clamp the effective window so it remains meaningful on short paths.
            effective_goal_window = min(self.goal_index_window, max(n - 1, 1))
            goal_window_reached = (n <= 1) or (n - self.last_target_index <= effective_goal_window)

        if goal_dist <= self.goal_tol and goal_window_reached:
            if self.use_start_flag and self.start_flag:
                self.get_logger().info(
                    "Goal reached. Stopping and waiting for next /start signal."
                )
                self.start_flag = False
            self.publish_stop()
            self.first_run = True
            return

        # 3) Compute lookahead using real velocity (actual vehicle speed, not commanded)
        v_for_lookahead = clamp(self.real_vel, 0.0, self.max_cmd_velocity)
        Ld = clamp(self.L0 + self.kv * abs(v_for_lookahead), self.Lmin, self.Lmax)

        # 4) Select target point in base_link frame
        target = self.find_lookahead_target(tf, Ld)
        if target is None:
            # No valid "ahead" target; safest is stop
            self.publish_stop()
            return

        bx, by, idx = target
        self.last_target_index = idx

        # 5) Pure Pursuit curvature and angular velocity
        # kappa = 2*by / Ld^2
        heading_error = math.atan2(by, bx)
        if self.use_speed_adaptive:
            v_target = self.speed_adaptive_min + (self.max_cmd_velocity - self.speed_adaptive_min) * math.exp(
                -self.speed_heading_coupling * abs(heading_error)
            )
            v_target = clamp(v_target, self.speed_adaptive_min, self.max_cmd_velocity)
            v_cmd = (1.0 - self.cmd_smoothing_factor) * self.prev_v_cmd + self.cmd_smoothing_factor * v_target
            v_cmd = clamp(v_cmd, self.speed_adaptive_min, self.max_cmd_velocity)
            v_cmd = self.apply_ttc_brake_speed_reduction(v_cmd, self.max_cmd_velocity)
        else:
            v_cmd = self.speed_nominal
            v_cmd = self.apply_ttc_brake_speed_reduction(v_cmd, self.max_cmd_velocity)
        v_cmd = clamp(v_cmd, 0.0, self.max_cmd_velocity)

        cross_track_error = by
        kappa = (2.0 * by) / (Ld * Ld + self.eps)
        self.publish_error_signals(cross_track_error, heading_error, kappa)
        omega = v_cmd * kappa

        if self.use_ackermann_steering_cmd:
            steering_angle_rad = math.atan(self.wheelbase * kappa)
            steering_angle_rad = clamp(
                steering_angle_rad,
                -self.steering_angle_limit_rad,
                self.steering_angle_limit_rad,
            )
            steering_angle_deg = math.degrees(steering_angle_rad)
            steering_signal = angle_deg_to_signal(steering_angle_deg)
            omega_out = steering_signal
        else:
            omega = self.apply_ttc_turn_assist(omega)

            # Clamp omega
            omega_limit = self.max_omega
            if self.use_ttc and (self.ttc_brake_active or self.ttc_brake_warn_active):
                omega_limit = self.max_omega_ttc
            omega_out = clamp(omega, -omega_limit, omega_limit)

        # 6) Publish cmd_vel
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(v_cmd)
        cmd.twist.angular.z = float(omega_out)
        self.cmd_pub.publish(cmd)
        self.prev_v_cmd = float(v_cmd)
        self.first_run = False


    def transform_pose_to_base(self, pose_st, tf):
        """
        Transform PoseStamped from path_frame into base_frame using TF2.
        """
        try:
            return do_transform_pose_stamped(pose_st, tf)
        except Exception as ex:
            self.get_logger().warn(f"Pose transform failed: {ex}")
            return None

    def find_lookahead_target(self, tf, Ld: float) -> Optional[Tuple[float, float, int]]:
        """
        Scan forward from last_target_index, find first point that:
          - is in front of robot (bx > 0)
          - has distance >= Ld
        Returns (bx, by, index) in base_link frame.
        """
        n = len(self.path)
        start = clamp(self.last_target_index, 0, n - 1)

        best_fallback = None  # last pose ahead, in case none reaches Ld

        for i in range(int(start), min(int(start) + 100, n)):  # Limit the search
            pose_b = self.transform_pose_to_base(self.path[i], tf)
            if pose_b is None:
                continue

            bx = pose_b.pose.position.x
            by = pose_b.pose.position.y

            if bx <= 0.0:
                continue  # behind or exactly sideways

            d = math.hypot(bx, by)

            # Keep a fallback "ahead" point
            best_fallback = (bx, by, i)

            if d >= Ld:
                return (bx, by, i)

        # If we never reached Ld, use the farthest ahead point we saw
        return best_fallback

    def publish_stop(self) -> None:
        self.prev_v_cmd = 0.0

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PurePursuitNode()
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