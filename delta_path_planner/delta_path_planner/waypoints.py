import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from rclpy.qos import QoSProfile, DurabilityPolicy
import json
from nav_msgs.msg import Path



class WaypointsNode(Node):
    def __init__(self):
        super().__init__('waypoints_node')
        
        # Parameters
        self.manual = self.declare_parameter('manual', True).value
        self.num_points = self.declare_parameter('num_points', 5).value
        self.closed_loop = self.declare_parameter('closed_loop', True).value
        self.waypoints_file = self.declare_parameter('waypoints_file', '/home/santy-estrada/mrad_ws_2601_delta/src/delta_path_planner/waypoints_json/sample_waypoints.json').value

        qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=10)

        # Subscribers
        if self.manual:
            self.create_subscription(
                PoseStamped,
                '/goal_pose',
                self.goal_pose_callback,
                10,
            )
            self.get_logger().info(
                f'Waypoints node in MANUAL mode. Collecting {self.num_points} waypoints.'
            )
        else:
            self.get_logger().info('Waypoints node in AUTO mode. Loading waypoints from file.')

        # Publishers (using standard Path for now as a temporary solution)
        # In production, this would publish delta_path_planner/WaypointPair
        self.waypoints_pub = self.create_publisher(Path, '/waypoints_topic', qos)

        # tf2 listener for robot position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Storage
        self.collected_waypoints = []
        self.robot_home_position = None
        self.publishing_active = False

        if not self.manual:
            # In auto mode, delay loading until TF is available (system is ready)
            self.create_timer(1.0, self.auto_mode_loader)
            self.auto_mode_loaded = False
        else:
            self.auto_mode_loaded = True

    def goal_pose_callback(self, msg):
        """Collect waypoint from goal_pose topic in manual mode."""
        self.collected_waypoints.append(msg)
        self.get_logger().info(
            f'Waypoint collected ({len(self.collected_waypoints)}/{self.num_points}): '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

        if len(self.collected_waypoints) == self.num_points:
            self.get_logger().info(f'All {self.num_points} waypoints collected. '
                                 'Publishing waypoint pairs...')
            self.process_and_publish_waypoints()
            self.collected_waypoints = []

    def auto_mode_loader(self):
        """Timer callback for auto mode: waits for TF to be available before loading waypoints."""
        if self.auto_mode_loaded:
            return  # Already loaded, disable timer
        
        try:
            # Test if TF is available
            self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            # If we get here, TF is available
            self.get_logger().info('TF is available. Loading waypoints from file...')
            self.load_waypoints_from_file()
            self.auto_mode_loaded = True
        except Exception:
            # TF not ready yet, will retry on next timer tick
            pass

    def load_waypoints_from_file(self):
        """Load waypoints from JSON file in auto mode."""
        if not self.waypoints_file:
            self.get_logger().warn(
                'AUTO mode selected but no waypoints_file parameter provided. '
                'No waypoints will be loaded.'
            )
            return

        try:
            with open(self.waypoints_file, 'r') as f:
                data = json.load(f)
                for point in data.get('waypoints', []):
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = float(point['x'])
                    pose.pose.position.y = float(point['y'])
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0
                    self.collected_waypoints.append(pose)
                self.get_logger().info(f'Loaded {len(self.collected_waypoints)} waypoints from file')
                self.process_and_publish_waypoints()
        except FileNotFoundError:
            self.get_logger().error(f'Waypoints file not found: {self.waypoints_file}')
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in waypoints file: {self.waypoints_file}')

    def process_and_publish_waypoints(self):
        """Convert collected waypoints to (start, goal) pairs and publish."""
        if not self.collected_waypoints:
            self.get_logger().warn('No waypoints to process')
            return

        try:
            # Get robot current position
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_pos = PoseStamped()
            robot_pos.header.frame_id = 'map'
            robot_pos.pose.position.x = tf.transform.translation.x
            robot_pos.pose.position.y = tf.transform.translation.y
            robot_pos.pose.position.z = tf.transform.translation.z
            robot_pos.pose.orientation.x = tf.transform.rotation.x
            robot_pos.pose.orientation.y = tf.transform.rotation.y
            robot_pos.pose.orientation.z = tf.transform.rotation.z
            robot_pos.pose.orientation.w = tf.transform.rotation.w
            
            self.robot_home_position = robot_pos
        except Exception as e:
            self.get_logger().warn(f"TF error getting robot position: {e}")
            return

        # Generate waypoint pairs
        waypoint_pairs = []
        
        # First pair: robot_pos -> first waypoint
        waypoint_pairs.append({
            'start': self.robot_home_position,
            'goal': self.collected_waypoints[0],
            'pair_id': 0,
            'is_final': False,
        })

        # Intermediate pairs: waypoint[i] -> waypoint[i+1]
        for i in range(len(self.collected_waypoints) - 1):
            waypoint_pairs.append({
                'start': self.collected_waypoints[i],
                'goal': self.collected_waypoints[i + 1],
                'pair_id': i + 1,
                'is_final': False,
            })

        # Final pair (if closed_loop): last_waypoint -> robot_pos
        if self.closed_loop:
            waypoint_pairs.append({
                'start': self.collected_waypoints[-1],
                'goal': self.robot_home_position,
                'pair_id': len(waypoint_pairs),
                'is_final': True,
            })

        # Publish waypoint pairs
        self.publish_waypoint_pairs(waypoint_pairs)

    def publish_waypoint_pairs(self, waypoint_pairs):
        """Publish waypoint pairs to waypoints_topic."""
        
        # Publish all pairs as a composite Path message
        # Structure: path.poses = [start_0, goal_0, start_1, goal_1, ...]
        path = Path()
        path.header.frame_id = 'map'

        for pair in waypoint_pairs:
            path.poses.append(pair['start'])
            path.poses.append(pair['goal'])

        self.waypoints_pub.publish(path)
        
        self.get_logger().info(f'Published {len(waypoint_pairs)} waypoint pairs to /waypoints_topic')
        for i, pair in enumerate(waypoint_pairs):
            start_pos = pair['start'].pose.position
            goal_pos = pair['goal'].pose.position
            self.get_logger().info(
                f'  Pair {i}: ({start_pos.x:.2f}, {start_pos.y:.2f}) -> '
                f'({goal_pos.x:.2f}, {goal_pos.y:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
