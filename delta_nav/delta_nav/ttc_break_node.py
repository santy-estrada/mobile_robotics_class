import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
from std_msgs.msg import Bool



class TTCBreakNode(Node):
    def __init__(self):
        super().__init__('ttc_break_node')
        
        # ---- Parameters ----
        self.declare_parameter('publish_rate', 100.0)           # Hz - rate to check TTC and publish commands
        self.declare_parameter('ttc_threshold', 1.0)            # seconds - TTC threshold for emergency braking
        self.declare_parameter('min_distance_threshold', 0.5)   # meters - minimum distance to obstacle for braking
        self.declare_parameter('forward_angle_range', 10.0)     # degrees - angle range in front of robot to consider
        self.declare_parameter('rear_angle_range', 10.0)        # degrees - angle range at rear of robot to consider
        self.declare_parameter('min_range', 0.1)                # meters - ignore measurements closer than this
        self.declare_parameter('max_range', 10.0)               # meters - ignore measurements farther than this
        
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.ttc_threshold = float(self.get_parameter('ttc_threshold').value)
        self.min_distance_threshold = float(self.get_parameter('min_distance_threshold').value)
        self.forward_angle_range = float(self.get_parameter('forward_angle_range').value)
        self.rear_angle_range = float(self.get_parameter('rear_angle_range').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        
        # ---- State ----
        self.current_cmd_vel = TwistStamped()    # Last received command velocity
        self.last_laser_scan = None              # Last received laser scan
        self.min_ttc = float('inf')              # Minimum TTC from all forward objects
        self.min_distance = float('inf')         # Minimum distance from all obstacles in relevant zone
        self.should_brake = False                # Flag indicating if emergency brake is needed
        
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
        
        # Subscribe to commanded velocity (so we can intercept forward commands)
        self.create_subscription(TwistStamped, '/diffdrive_controller/cmd_vel', self._cmd_vel_callback, qos)
        
        # Publisher for safety-override velocity commands
        self.cmd_ttc_publisher = self.create_publisher(TwistStamped, '/cmd_ttc', qos)
         # Publisher for safety-override velocity commands
        self.brake_pub = self.create_publisher(Bool, '/brake_active', 10)

        
        # ---- Timer ----
        # Timer to periodically check TTC and issue commands if needed
        self.dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.dt, self._check_ttc_and_publish)


    def _laser_scan_callback(self, msg: LaserScan):
        """Store the latest laser scan message."""
        self.last_laser_scan = msg
    
    def _cmd_vel_callback(self, msg: TwistStamped):
        """Store the latest commanded velocity from the controller."""
        self.current_cmd_vel = msg
    
    def _calculate_ttc_for_measurement(self, range_val: float, angle: float, 
                                        cmd_linear_x: float) -> float:
        """
        Calculate Time-To-Collision (TTC) for a single laser measurement.
        
        Args:
            range_val: Distance to object (meters)
            angle: Angle of measurement relative to robot forward (radians)
            cmd_linear_x: Commanded linear velocity in x direction (m/s)
        
        Returns:
            TTC in seconds, or infinity if no collision threat
        """
        # Filter out invalid measurements
        if range_val < self.min_range or range_val > self.max_range:
            return float('inf')
        
        # Calculate ri_dot: projection of relative velocity onto line-of-sight
        # ri_dot = V_x * cos(theta)
        ri_dot = cmd_linear_x * math.cos(angle)
        
        # TTC = ri / ri_dot
        ri_dot = max(ri_dot, 1e-5)  # Avoid division by zero
        ttc = range_val / ri_dot
        
        # Only consider positive TTC values
        if ttc > 0.0:
            return ttc
        else:
            return float('inf')
    
    def _check_ttc_and_publish(self):
        """
        Periodically check TTC for forward and rear obstacles and publish safety override if needed.
        """
        # Early exit if no recent laser scan data
        if self.last_laser_scan is None:
            return
        
        # Reset minimum TTC for this cycle
        self.min_ttc = float('inf')
        self.min_distance = float('inf')
        
        # Get the current forward velocity command
        cmd_linear_x = self.current_cmd_vel.twist.linear.x

        
        # If robot is not commanded to move, no need to brake
        if cmd_linear_x == 0.0:
            return
        
        # Process laser scan measurements
        scan_msg = self.last_laser_scan
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        
        # Determine if moving forward or backward
        is_moving_forward = cmd_linear_x > 0.0
        
        # Convert angle ranges to radians
        forward_range_rad = math.radians(self.forward_angle_range)
        rear_range_rad = math.radians(self.rear_angle_range)
        
        # Iterate through measurements
        for i, range_val in enumerate(scan_msg.ranges):
            # Skip invalid measurements (inf or nan)
            if not math.isfinite(range_val):
                continue
            
            # Calculate the angle of this measurement
            angle = angle_min + i * angle_increment
            
            # Normalize angle to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            
            # Check if measurement is in relevant cone based on direction of motion
            in_relevant_zone = False
            if is_moving_forward:
                # For forward motion, check front cone (around angle 0)
                in_relevant_zone = abs(angle) <= forward_range_rad
            else:
                # For reverse motion, check rear cone (around angle ±pi)
                in_relevant_zone = abs(angle) >= (math.pi - rear_range_rad)
            
            if in_relevant_zone:
                ttc = self._calculate_ttc_for_measurement(range_val, angle, cmd_linear_x)
                
                # Track minimum TTC
                if ttc < self.min_ttc:
                    self.min_ttc = ttc
                
                # Track minimum distance
                if range_val < self.min_distance:
                    self.min_distance = range_val
        
        # Determine if we should brake
        was_braking = self.should_brake
        # Brake if TTC is below threshold OR if any obstacle is too close (within min_distance_threshold)
        self.should_brake = ((self.min_ttc < self.ttc_threshold and self.min_ttc != float('inf')) or 
                             (self.min_distance < self.min_distance_threshold))
        
        # Publish brake state ALWAYS
        brake_msg = Bool()
        brake_msg.data = self.should_brake
        self.brake_pub.publish(brake_msg)

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
            # Create a zero-velocity command to override forward motion
            '''For a differential drive robot, we typically want to zero out the forward 
            linear velocity (x) but allow rotation and lateral motion if needed.'''
            safety_cmd = TwistStamped()
            safety_cmd.twist.linear.x = 0.0      # Zero forward velocity
            safety_cmd.twist.linear.y = 0.0      # No lateral motion
            safety_cmd.twist.linear.z = 0.0      # No vertical motion
            # Allow rotation and backward motion (don't constrain angular or other linear components)
            safety_cmd.twist.angular.x = 0.0    # No pitch
            safety_cmd.twist.angular.y = 0.0      # No roll
            safety_cmd.twist.angular.z = self.current_cmd_vel.twist.angular.z  # Allow rotation if needed
            
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