import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import tf2_ros
from rclpy.qos import QoSProfile, DurabilityPolicy
import tf_transformations
import numpy as np
import heapq


class BestFirst(Node):
    def __init__(self):
        super().__init__('best_first_node')
        
        # --- add these params at __init__ ---
        self.occ_thresh = self.declare_parameter('occ_thresh', 100).value
        self.robot_radius_m = self.declare_parameter('robot_radius_m', 0.5).value
        self.safety_margin_m = self.declare_parameter('safety_margin_m', 0.05).value
        self.inbounds_thresh = self.declare_parameter('inbounds_thresh', 50).value
        self.heuristic = self.declare_parameter('heuristic', 'manhattan').value
        self.center = self.declare_parameter('center', False).value
        self.avoid = self.declare_parameter('avoid', False).value
        self.use_waypoints = self.declare_parameter('waypoints', True).value

        self.map_topic = self.declare_parameter('topics.map_topic', '/map').value
        self.goal_topic = self.declare_parameter('topics.goal_topic', '/goal_pose').value
        self.waypoints_topic = self.declare_parameter('topics.waypoints_topic', '/waypoints_topic').value
        self.path_topic = self.declare_parameter('topics.path_topic', '/best_first_path').value

        qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=10)

        # Subscribers
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, qos)
        
        if self.use_waypoints:
            self.create_subscription(Path, self.waypoints_topic, self.waypoints_callback, qos)
            self.get_logger().info('Best First node in WAYPOINTS mode')
        else:
            self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
            self.get_logger().info('Best First node in GOAL_POSE mode')

        # Publishers
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        # tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Storage
        self.map = None
        self.goal = None
        self.waypoint_pairs = []
        self.get_logger().info('Best First node initialized')

    def map_callback(self, msg):
        self.map = msg
        
        # 2) compute how many cells to inflate
        res = self.map.info.resolution
        inflation_cells = max(0, int(np.floor((self.robot_radius_m + self.safety_margin_m) / res)))

        # 3) make mask of cells to inflate (True = will become occupied)
        inflated_mask = self.make_inflation_mask(self.map, inflation_cells, self.occ_thresh)

        # 4) write 100s into the map data (hard inflation)
        w, h = self.map.info.width, self.map.info.height
        data = np.array(self.map.data, dtype=np.int16).reshape((h, w))
        data[inflated_mask] = 100  # or 254, anything >= occ_thresh
        self.map.data = data.flatten().tolist()
        
        self.get_logger().info('Map received')
		
    def make_inflation_mask(self, occ_grid, inflation_cells: int, occ_thresh: int):
        """Return boolean mask (h,w): True where cells are within `inflation_cells` of any obstacle."""
        w, h = occ_grid.info.width, occ_grid.info.height
        data = np.array(occ_grid.data, dtype=np.int16).reshape((h, w))
        occ = (data >= occ_thresh).astype(np.uint8)

        # Multi-source BFS (brushfire) distance in cells
        INF = 1_000_000
        dist = np.full((h, w), INF, dtype=np.int32)

        from collections import deque
        q = deque()
        ys, xs = np.where(occ == 1)
        for y, x in zip(ys, xs):
            dist[y, x] = 0
            q.append((x, y))

        # 4-connected neighbors for distance
        for_yx = [(-1,0),(1,0),(0,-1),(0,1)]
        while q:
            x, y = q.popleft()
            d = dist[y, x]
            for dx, dy in for_yx:
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and dist[ny, nx] > d + 1:
                    dist[ny, nx] = d + 1
                    q.append((nx, ny))

        # Inflate: cells with distance <= inflation_cells become occupied
        return dist <= inflation_cells
        
    def goal_callback(self, msg):
        self.goal = msg
        self.get_logger().info('Goal received')
        self.plan_path()

    def waypoints_callback(self, msg):
        """Handle waypoint pairs from waypoints_topic.
        Path.poses are structured as: [start_0, goal_0, start_1, goal_1, ...]
        """
        self.waypoint_pairs = []
        # Extract pairs from alternating poses
        for i in range(0, len(msg.poses) - 1, 2):
            start = msg.poses[i]
            goal = msg.poses[i + 1]
            self.waypoint_pairs.append({'start': start, 'goal': goal})
        
        self.get_logger().info(f'Received {len(self.waypoint_pairs)} waypoint pairs')
        self.plan_waypoint_paths()

    def plan_waypoint_paths(self):
        """Plan paths for all waypoint pairs and publish as single combined path."""
        if not self.map or not self.waypoint_pairs:
            self.get_logger().warn('Map or waypoint pairs not available')
            return
        
        # Create a single Path message containing all paths
        combined_path = Path()
        combined_path.header.frame_id = 'map'
        
        for i, pair in enumerate(self.waypoint_pairs):
            start = self.world_to_grid(pair['start'].pose.position)
            goal = self.world_to_grid(pair['goal'].pose.position)
            path = self.best_first(start, goal)
            
            if self.center and len(path) > 0:
                path = [(x + 0.5 * self.map.info.resolution, y + 0.5 * self.map.info.resolution) for (x, y) in path]

            # Add all poses from this path to the combined path
            for (x, y) in path:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x * self.map.info.resolution + self.map.info.origin.position.x
                pose.pose.position.y = y * self.map.info.resolution + self.map.info.origin.position.y
                pose.pose.orientation.w = 1.0
                combined_path.poses.append(pose)
            
            self.get_logger().info(f'Waypoint pair {i} path calculated (length: {len(path)})')
        
        # Publish all paths as a single message
        self.path_pub.publish(combined_path)
        self.get_logger().info(f'Published combined path with {len(combined_path.poses)} total poses')
		
    def plan_path(self):
        self.get_logger().info('Planning path...')
        if not self.map or not self.goal:
            self.get_logger().info(
                f'Waiting for map {self.map is not None} and goal {self.goal is not None}...'
            )
            return

        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")
            return
        
        # print(f"Robot pose in map frame: {tf.transform.translation.x}, {tf.transform.translation.y}")

        # Convert start and goal to grid coordinates
        start = self.world_to_grid(tf.transform.translation)
        goal = self.world_to_grid(self.goal.pose.position)

        path = self.best_first(start, goal)

        if self.center and len(path) > 0:
            # Adjust path to center of cells
            path = [(x + 0.5* self.map.info.resolution, y + 0.5* self.map.info.resolution) for (x, y) in path]

        ros_path = Path()
        ros_path.header.frame_id = 'map'
        for (x, y) in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x * self.map.info.resolution + self.map.info.origin.position.x
            pose.pose.position.y = y * self.map.info.resolution + self.map.info.origin.position.y
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)
        

        self.path_pub.publish(ros_path)
        self.get_logger().info('Path published')
    
    def world_to_grid(self, pos):
        mx = int((pos.x - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((pos.y - self.map.info.origin.position.y) / self.map.info.resolution)
        return (mx, my)
    
    def best_first(self, start, goal):
        """Best First Search using heuristic to guide the search."""
        width = self.map.info.width
        height = self.map.info.height
        data = np.array(self.map.data).reshape((height, width))

        def in_bounds(x, y):
            return 0 <= x < width and 0 <= y < height and data[y][x] < self.inbounds_thresh and data[y][x] >= 0
        
        def heuristic(pos):
            """Manhattan distance or Euclidean distance based on parameter."""
            if self.heuristic == 'manhattan':
                return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])
            else:
                return np.hypot(pos[0] - goal[0], pos[1] - goal[1])
            
        def avoid_cost(pos):
            """Additional cost for cells near obstacles if avoid parameter is True."""
            if not self.avoid:
                return 0
            x, y = pos
            cost = 0
            for dx in range(-5, 6):  # Check a 6x6 area around the cell
                for dy in range(-5, 6):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height and data[ny][nx] >= self.inbounds_thresh:
                        cost += 1  # Arbitrary penalty for proximity to obstacles
            return 1 if cost > 0 else 0  # Add a small cost if near obstacles

        visited = set()
        g = {start: 0}  # Track actual cost to reach each node
        g[goal] = float('inf')
        
        # Priority queue: (heuristic_value, node)
        queue = [(heuristic(start), start)]
        came_from = {start: None}

        while queue:
            _, current = heapq.heappop(queue)

            if current in visited:
                continue
            
            visited.add(current)

            if current == goal:
                break

            x, y = current
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:  # 8-connected neighbors
                nx, ny = x + dx, y + dy
                neighbor = (nx, ny)
                
                if in_bounds(nx, ny) and neighbor not in visited:
                    new_cost = g[current] + 1  # Cost from current to neighbor
                    
                    # Initialize g value if not seen before
                    if neighbor not in g:
                        g[neighbor] = float('inf')
                    
                    # Update if we found a cheaper path
                    if new_cost < g[neighbor]:
                        g[neighbor] = new_cost
                        came_from[neighbor] = current
                        # Push with heuristic as priority (not cost!)
                        heapq.heappush(queue, (heuristic(neighbor) + avoid_cost(neighbor), neighbor))

        # Reconstruct path
        path = []
        node = goal
        while node:
            path.append(node)
            node = came_from.get(node)
        path.reverse()

        if not path or path[0] != start or path[-1] != goal:
            self.get_logger().warn('No valid path found!')
            return []
        
        return path
	

def main(args=None):
    rclpy.init(args=args)
    node = BestFirst()
    rclpy.spin(node)
    rclpy.shutdown()
	

if __name__ == '__main__':    main()