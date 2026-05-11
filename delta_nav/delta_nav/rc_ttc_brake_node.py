import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
from std_msgs.msg import Bool, Int32, Float64
from std_msgs.msg import Float32MultiArray



class TTCBreakNode(Node):
    def __init__(self):
        super().__init__('ttc_break_node')
        
        # ---- Parameters ----
        self.declare_parameter('publish_rate', 100.0)           # Hz - rate to check TTC and publish commands
        self.declare_parameter('ttc_threshold', 0.5)            # seconds - TTC threshold for emergency braking
        self.declare_parameter('min_distance_threshold', 0.6)   # meters - minimum distance to obstacle for braking
        self.declare_parameter('forward_angle_range', 8.0)      # degrees - angle range in front of robot to consider
        self.declare_parameter('min_range', 0.01)                # meters - ignore measurements closer than this
        self.declare_parameter('max_range', 12.0)               # meters - ignore measurements farther than this
        self.declare_parameter("safety_bubble", True)           # If true, consider all measurements within min_distance_threshold as braking threats regardless of TTC
        self.declare_parameter("heartbeat_rate_hz", 1.0)


        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.ttc_threshold = float(self.get_parameter('ttc_threshold').value)
        self.min_distance_threshold = float(self.get_parameter('min_distance_threshold').value)
        self.forward_angle_range = float(self.get_parameter('forward_angle_range').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.safety_bubble = bool(self.get_parameter('safety_bubble').value)
        self.heartbeat_rate_hz = float(self.get_parameter('heartbeat_rate_hz').value)

        # ---- State ----
        self.current_longitudinal_velocity = 0.0  # Last received longitudinal velocity from /encoder/vel
        self.last_laser_scan = None              # Last received laser scan
        self.min_ttc = float('inf')              # Minimum TTC from all forward objects
        self.min_distance = float('inf')         # Minimum distance from all obstacles in relevant zone
        self.should_brake = False                # Flag indicating if emergency brake is needed
        self.brake_trigger_angle = None          # Angle of the measurement that triggered brake
        self.is_stopped = None                   # Last published /brake_active state
        self.is_warned = None                    # Last published /brake_warn state
        
        # ---- Pub/Sub ----
        # QoS profile compatible with sensor publishers (RELIABLE, VOLATILE)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to laser scan data
        self.create_subscription(LaserScan, '/scan', self._laser_scan_callback, qos)
        
        # Subscribe to measured longitudinal velocity from encoder pipeline
        self.create_subscription(Float64, '/encoder/vel', self._encoder_vel_callback, 10)
        
        # Publisher for safety-override velocity commands
        self.cmd_ttc_publisher = self.create_publisher(TwistStamped, '/cmd_ttc', qos)
         # Publisher for safety-override velocity commands
        self.brake_pub = self.create_publisher(Bool, '/brake_active', 10)
        # Publisher for brake direction (0=right, 1=left, 3=indeterminate)
        self.dir_brake_pub = self.create_publisher(Int32, '/dir_brake', 10)

        if self.safety_bubble:
            self.brake_warn_pub = self.create_publisher(Bool, '/brake_warn', 10)

        self.ttc_array_pub = self.create_publisher(Float32MultiArray, '/ttc_values', 10)
        
        # ---- Timer ----
        # Timer to periodically check TTC and issue commands if needed
        self.dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.dt, self._check_ttc_and_publish)

        # heartbeat timer to log that node is alive
        # self.heartbeat_timer = self.create_timer(1.0 / self.heartbeat_rate_hz, self._heartbeat_callback)

    def _heartbeat_callback(self):
        # Publish the current state of only the brake state
        if self.is_stopped is not None:
            brake_msg = Bool()
            brake_msg.data = self.is_stopped
            self.brake_pub.publish(brake_msg)


    def _laser_scan_callback(self, msg: LaserScan):
        """Store the latest laser scan message."""
        self.last_laser_scan = msg

    def _encoder_vel_callback(self, msg: Float64):
        """Store the latest measured longitudinal velocity."""
        self.current_longitudinal_velocity = max(float(msg.data), 0.0)

    def _publish_brake_state_if_changed(self, brake_state: bool):
        """Publish /brake_active only when state changes."""
        if self.is_stopped == brake_state:
            return

        brake_msg = Bool()
        brake_msg.data = brake_state
        self.brake_pub.publish(brake_msg)
        self.is_stopped = brake_state

    def _publish_warn_state_if_changed(self, warn_state: bool):
        """Publish /brake_warn only when state changes."""
        if not self.safety_bubble:
            return
        if self.is_warned == warn_state:
            return

        if warn_state:
            self.get_logger().warn(
                "SAFETY BUBBLE ALERT: Obstacle within safety bubble "
                f"(threshold: {self.min_distance_threshold}m). "
                f"Issuing brake warning!"
            )
        else:
            self.get_logger().info(
                "SAFETY BUBBLE CLEAR: No obstacles within safety bubble. Clearing brake warning!"
            )

        brake_msg = Bool()
        brake_msg.data = warn_state
        self.brake_warn_pub.publish(brake_msg)
        self.is_warned = warn_state
    
    def _calculate_ttc_for_measurement(self, range_val: float, angle: float,
                                       longitudinal_speed: float) -> float:
        """
        Calculate Time-To-Collision (TTC) for a single laser measurement.
        
        Args:
            range_val: Distance to object (meters)
            angle: Angle of measurement relative to robot forward (radians)
            longitudinal_speed: Measured longitudinal velocity in x direction (m/s)
        
        Returns:
            TTC in seconds, or infinity if no collision threat
        """
        # Filter out invalid measurements
        if range_val < self.min_range or range_val > self.max_range:
            return float('inf')
        
        # Calculate ri_dot: projection of relative velocity onto line-of-sight
        # ri_dot = V_x * cos(theta)
        ri_dot = longitudinal_speed * math.cos(angle)
        
        # TTC = ri / ri_dot (only approaching rays with positive closing speed)
        if ri_dot <= 1e-5:
            return float('inf')
        ttc = range_val / ri_dot
        
        # Only consider positive TTC values
        if ttc > 0.0:
            return ttc
        else:
            return float('inf')
    
    def _determine_brake_direction(self) -> int:
        """
        Determine the direction of the obstacle that triggered braking.
        
        Returns:
            0 if obstacle is on the right side (angle < 0)
            1 if obstacle is on the left side (angle > 0)
            3 if cannot be determined or indeterminate (angle ≈ 0)
        """
        if self.brake_trigger_angle is None:
            return 3
        
        # Threshold to classify as "straight ahead" (indeterminate)
        straight_threshold = math.radians(self.forward_angle_range*0.03) # 3% of forward angle range
        
        if abs(self.brake_trigger_angle) < straight_threshold:
            return 3  # Straight ahead, indeterminate
        elif self.brake_trigger_angle < 0:
            return 0  # Right side
        else:
            return 1  # Left side
        
    def _determine_brake_direction_ang(self, angle) -> int:
        """
        Determine the direction of the obstacle that triggered braking.
        
        Returns:
            0 if obstacle is on the right side (angle < 0)
            1 if obstacle is on the left side (angle > 0)
            3 if cannot be determined or indeterminate (angle ≈ 0)
        """
        if angle is None:
            return 3
        
        # Threshold to classify as "straight ahead" (indeterminate)
        straight_threshold = math.radians(self.forward_angle_range*0.03) # 3% of forward angle range
        
        if abs(angle) < straight_threshold:
            return 3  # Straight ahead, indeterminate
        elif angle < 0:
            return 0  # Right side
        else:
            return 1  # Left side
    
    def _compute_directional_ttc_array(self, scan_msg, longitudinal_speed):
        """
        Compute TTC for the front 180° ([-90°, +90°]).
        All other rays are set to inf.

        Note: this node is forward-only by design.
        Returns:
            List of TTC values (same size as scan.ranges)
        """

        ttc_array = []

        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment

            # Half plane = 180 degrees
        half_plane_rad = math.pi / 2.0

        for i, range_val in enumerate(scan_msg.ranges):

                # Default value
            ttc = float('inf')

                # Skip invalid ranges
            if not math.isfinite(range_val):
                    ttc_array.append(ttc)
                    continue

            angle = angle_min + i * angle_increment

                # Normalize angle to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # Front 180° only.
            in_direction = abs(angle) <= half_plane_rad

            if in_direction:
                ttc = self._calculate_ttc_for_measurement(
                    range_val, angle, longitudinal_speed
                )

            ttc_array.append(float(ttc))

        return ttc_array

    
    def _check_ttc_and_publish(self):
        """
        Periodically check TTC for forward obstacles and publish safety override if needed.
        """
        scan_msg = self.last_laser_scan
        # Early exit if no recent laser scan data
        if self.last_laser_scan is None:
            return
        
        # Reset minimum TTC for this cycle
        self.min_ttc = float('inf')
        self.min_distance = float('inf')
        self.brake_trigger_angle = None
        
        # Get the measured longitudinal speed from /encoder/vel
        longitudinal_speed = self.current_longitudinal_velocity
        
        # Forward-only mode: treat non-forward commands as stopped.
        if longitudinal_speed <= 0.0:
            # Create array of inf values for all laser rays
            directional_ttc_array = [float('inf')] * len(scan_msg.ranges)
            # Add direction flag (0.0 for forward by default)
            directional_ttc_array.append(0.0)
            
            ttc_msg = Float32MultiArray()
            ttc_msg.data = directional_ttc_array
            self.ttc_array_pub.publish(ttc_msg)
            
            # No motion -> no active braking condition.
            self._publish_brake_state_if_changed(False)
            return
        
        # Process laser scan measurements
        scan_msg = self.last_laser_scan
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        
        # TTC is computed only for the front half-plane [-90 deg, +90 deg].
        front_half_plane_rad = math.pi / 2.0
        bubble_warn_active = False
        bubble_warn_distance = float('inf')
        bubble_warn_angle = None
        
        # Iterate through measurements
        for i, range_val in enumerate(scan_msg.ranges):
            # Skip invalid measurements (inf or nan)
            ttc = float('inf')
            if not math.isfinite(range_val):
                continue
            
            # Calculate the angle of this measurement
            angle = angle_min + i * angle_increment
            
            # Normalize angle to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            in_front_zone = abs(angle) <= front_half_plane_rad
            if not in_front_zone:
                continue

            # If safety bubble is enabled, consider front measurements inside threshold as warning threats.
            if self.safety_bubble and range_val < self.min_distance_threshold:
                bubble_warn_active = True
                if range_val < bubble_warn_distance:
                    bubble_warn_distance = range_val
                    bubble_warn_angle = angle

                if range_val < self.min_distance:
                    self.min_distance = range_val
                    self.brake_trigger_angle = angle
                continue

            ttc = self._calculate_ttc_for_measurement(range_val, angle, longitudinal_speed)

            # Track minimum TTC
            if ttc < self.min_ttc:
                self.min_ttc = ttc
                self.brake_trigger_angle = angle

            # Track minimum distance
            if range_val < self.min_distance:
                self.min_distance = range_val
                if self.min_distance < self.min_distance_threshold:
                    self.brake_trigger_angle = angle
            
        # Compute full directional TTC array (180° depending on motion)
        directional_ttc_array = self._compute_directional_ttc_array(
            scan_msg, longitudinal_speed
        )

        # Publish safety-bubble warning only on state transitions.
        if self.safety_bubble:
            self._publish_warn_state_if_changed(bubble_warn_active)
            if bubble_warn_active and not self.should_brake:
                dir_msg = Int32()
                dir_msg.data = self._determine_brake_direction_ang(bubble_warn_angle)
                self.dir_brake_pub.publish(dir_msg)
                self.get_logger().warn(
                    f"(threshold: {self.min_distance_threshold}m). Direction {dir_msg.data}."
                )

        # Append direction flag as forward-only mode marker.
        directional_ttc_array.append(0.0)

        ttc_msg = Float32MultiArray()
        ttc_msg.data = directional_ttc_array
        self.ttc_array_pub.publish(ttc_msg)



        # Determine if we should brake
        was_braking = self.should_brake
        # Brake if TTC is below threshold OR if any obstacle is too close (within min_distance_threshold)
        self.should_brake = ((self.min_ttc < self.ttc_threshold and self.min_ttc != float('inf')) or 
                             (self.min_distance < self.min_distance_threshold))
        
        # Publish brake state only on state transitions.
        self._publish_brake_state_if_changed(self.should_brake)
        
        # Publish brake direction
        if self.should_brake:
            dir_msg = Int32()
            dir_msg.data = self._determine_brake_direction()
            self.dir_brake_pub.publish(dir_msg)

        # Log state changes
        if self.should_brake != was_braking:
            if self.should_brake:
                self.get_logger().warn(
                    f"COLLISION ALERT: TTC = {self.min_ttc:.2f}s (threshold: {self.ttc_threshold}s), "
                    f"Distance = {self.min_distance:.2f}m (threshold: {self.min_distance_threshold}m). "
                    f"Issuing emergency brake!"
                )
            else:
                self.get_logger().info(
                    f"Collision cleared. TTC = {self.min_ttc:.2f}s, Distance = {self.min_distance:.2f}m"
                )
        
        # Publish safety override command if braking is needed
        if self.should_brake:
            # Ackermann braking override: fully stop (no steering command while braking).
            safety_cmd = TwistStamped()
            safety_cmd.twist.linear.x = 0.0      # Zero forward velocity
            safety_cmd.twist.linear.y = 0.0      # No lateral motion
            safety_cmd.twist.linear.z = 0.0      # No vertical motion
            safety_cmd.twist.angular.x = 0.0    # No pitch
            safety_cmd.twist.angular.y = 0.0      # No roll
            safety_cmd.twist.angular.z = 0.0      # No yaw steering while braking
            
            self.cmd_ttc_publisher.publish(safety_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TTCBreakNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()