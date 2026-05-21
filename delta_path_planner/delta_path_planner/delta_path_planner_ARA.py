#!/usr/bin/env python3
"""
ARA* (Anytime Repairing A*) grid planner (ROS 2).
Basado en la estructura de Dijkstra proporcionada en clase.
"""
import math
import heapq
import time
from collections import deque
from typing import List, Tuple, Optional, Dict

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion
import tf2_ros
from dataclasses import dataclass
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Bool

@dataclass
class ARANode:
    x: int
    y: int
    g: float = float('inf')
    v: float = float('inf')
    parent: Optional[Tuple[int, int]] = None
    
    def __lt__(self, other):
        return False

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q

class ARAPlannerNode(Node):
    def __init__(self):
        super().__init__('ara_planner_node')

        self.declare_parameter('topics.map_topic', '/map')
        self.declare_parameter('topics.goal_topic', '/goal_pose')
        self.declare_parameter('topics.path_topic', '/ara_path')
        self.use_waypoints = self.declare_parameter('waypoints', False).value
        self.use_start = self.declare_parameter('use_start', self.use_waypoints).value

        self.declare_parameter('frames.base_frame', 'base_link')
        self.declare_parameter('frames.global_frame', 'map')
        self.declare_parameter('topics.debug_paths', '/ara_debug_paths')
        
        self.declare_parameter('geometry.occupied_threshold', 65)
        self.declare_parameter('geometry.use_8_connected', True)
        self.declare_parameter('geometry.treat_unknown_as_obstacle', True)

        self.occ_thresh = self.declare_parameter('occ_thresh', 100).value
        self.robot_radius_m = self.declare_parameter('robot_radius_m', 0.6).value
        self.safety_margin_m = self.declare_parameter('safety_margin_m', 0.05).value
        self.declare_parameter('geometry.inflate_radius', 0.7)

        self.declare_parameter('ara_core.epsilon_start', 2.5)
        self.declare_parameter('ara_core.epsilon_decrease', 0.5)
        self.declare_parameter('ara_core.time_limit_sec', 1.5)
        self.declare_parameter('ara_core.heuristic_type', 'euclidean')

        self.declare_parameter('debug.publish_all_paths', False)
        self._debug_mode = self.get_parameter('debug.publish_all_paths').get_parameter_value().bool_value      

        self._heuristic_type = self.get_parameter('ara_core.heuristic_type').get_parameter_value().string_value.lower()
        self._use_8_conn = self.get_parameter('geometry.use_8_connected').get_parameter_value().bool_value

        straight_moves = [(0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0)]
        diagonal_moves = [(1, 1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (-1, -1, 1.4142)]
        self._allowed_moves = straight_moves + diagonal_moves if self._use_8_conn else straight_moves

        map_topic = self.get_parameter('topics.map_topic').get_parameter_value().string_value
        goal_topic = self.get_parameter('topics.goal_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('topics.path_topic').get_parameter_value().string_value
        debug_topic = self.get_parameter('topics.debug_paths').get_parameter_value().string_value

        qos_waypoints = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=10)

        if self.use_waypoints:
            self.create_subscription(Path, '/waypoints_topic', self.waypoints_callback, qos_waypoints)
            self.get_logger().info('ARA node in WAYPOINTS mode')
        else:
            self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)
            self.get_logger().info('ARA node in GOAL_POSE mode')

        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.debug_paths_pub = self.create_publisher(MarkerArray, debug_topic, 10)

        # --- Publisher para visualización de waypoints en RViz ---
        self.waypoints_markers_pub = self.create_publisher(MarkerArray, '/ara_waypoints_markers', 10)

        self.start_flag = (not self.use_waypoints) or (not self.use_start)
        self.pending_paths = []

        if self.use_waypoints and self.use_start:
            self.create_subscription(Bool, '/start', self.start_callback, 10)
            self.get_logger().info(
                'ARA start gate enabled. Planning runs immediately; path publication waits for /start.'
            )
        elif self.use_waypoints:
            self.get_logger().info('ARA start gate disabled. Planned paths publish immediately.')

        qos_map = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_cb, qos_map)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._map: Optional[OccupancyGrid] = None
        self._obstacles: Optional[np.ndarray] = None
        self._dist_cells: Optional[np.ndarray] = None
        self.waypoint_pairs = []
        self.waypoints_pending_plan = False
        if self._debug_mode:
            self.get_logger().info("ARA* Planner Node Iniciado con los siguientes parámetros:" \
            f"- Epsilon Start: {self.get_parameter('ara_core.epsilon_start').get_parameter_value().double_value}" \
            f"- Epsilon Decrease: {self.get_parameter('ara_core.epsilon_decrease').get_parameter_value().double_value}" \
            f"- Time Limit (sec): {self.get_parameter('ara_core.time_limit_sec').get_parameter_value().double_value}" \
            f"- Heuristic Type: {self.get_parameter('ara_core.heuristic_type').get_parameter_value().string_value}" \
            f"- Use 8-Connected: {self.get_parameter('geometry.use_8_connected').get_parameter_value().bool_value}" \
            f"- Inflation Occ Thresh: {self.occ_thresh}" \
            f"- Robot Radius (m): {self.robot_radius_m}" \
            f"- Safety Margin (m): {self.safety_margin_m}")

        self.get_logger().info("ARA* Planner Node Iniciado y esperando el mapa...")

    # =================================================================
    # CALLBACKS DE ROS 2
    # =================================================================
    def start_callback(self, msg: Bool):
        if not msg.data:
            return
        if not self.start_flag:
            self.get_logger().info('Received /start signal. Releasing queued planned paths.')
        self.start_flag = True
        self._publish_pending_paths()

    def _publish_pending_paths(self):
        if not self.pending_paths:
            return

        queued_paths = self.pending_paths
        self.pending_paths = []
        self.get_logger().info(f'Publishing {len(queued_paths)} queued path message(s).')

        for path_msg, log_msg in queued_paths:
            self.path_pub.publish(path_msg)
            self.get_logger().info(log_msg)

    def _publish_or_queue_path(self, path_msg: Path, log_msg: str):
        if self.start_flag:
            self.path_pub.publish(path_msg)
            self.get_logger().info(log_msg)
        else:
            self.pending_paths.append((path_msg, log_msg))
            self.get_logger().info(
                'Start signal not received yet. Planned path queued for publication.'
            )

    def map_cb(self, msg: OccupancyGrid):
        self._map = msg
        W = msg.info.width
        H = msg.info.height
        res = msg.info.resolution

        grid = np.array(msg.data, dtype=np.int16).reshape((H, W))

        inflation_cells = max(
            0,
            int(np.floor((float(self.robot_radius_m) + float(self.safety_margin_m)) / res))
        )
        inflated_mask = self.make_inflation_mask(grid, inflation_cells, int(self.occ_thresh))

        inflated_grid = grid.copy()
        inflated_grid[inflated_mask] = 80
        self._grid = inflated_grid

        occ_th = self.get_parameter('geometry.occupied_threshold').get_parameter_value().integer_value
        unknown_as_obs = self.get_parameter('geometry.treat_unknown_as_obstacle').get_parameter_value().bool_value

        obstacles = (inflated_grid >= occ_th)
        if unknown_as_obs:
            obstacles = np.logical_or(obstacles, inflated_grid == -1)

        self._obstacles = obstacles
        self.get_logger().info(
            f'Map received: {W}x{H}, res={res:.3f} m/px, inflation_cells={inflation_cells}'
        )

        if self.use_waypoints and self.waypoints_pending_plan and self.waypoint_pairs:
            self.get_logger().info('Map ready with pending waypoint plan. Planning waypoint paths now.')
            self.plan_waypoint_paths()

    def make_inflation_mask(self, grid: np.ndarray, inflation_cells: int, occ_thresh: int) -> np.ndarray:
        occ = (grid >= occ_thresh)
        dist_cells = self.compute_distance_to_obstacles(occ)
        self._dist_cells = dist_cells
        return dist_cells <= inflation_cells

    def compute_distance_to_obstacles(self, obstacles: np.ndarray) -> np.ndarray:
        H, W = obstacles.shape
        INF = np.iinfo(np.int32).max
        dist = np.full((H, W), INF, dtype=np.int32)

        q = deque()
        ys, xs = np.nonzero(obstacles)
        for y, x in zip(ys, xs):
            dist[y, x] = 0
            q.append((x, y))

        if not q:
            return dist

        nbr4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        while q:
            x, y = q.popleft()
            d = dist[y, x]
            nd = d + 1
            for dx, dy in nbr4:
                nx, ny = x + dx, y + dy
                if 0 <= nx < W and 0 <= ny < H and dist[ny, nx] > nd:
                    dist[ny, nx] = nd
                    q.append((nx, ny))
        return dist

    def goal_cb(self, msg: PoseStamped):
        if self._map is None or self._obstacles is None:
            self.get_logger().warn("No hay mapa todavía.")
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('frames.global_frame').value,
                self.get_parameter('frames.base_frame').value,
                rclpy.time.Time()
            )
            start_pose = PoseStamped()
            start_pose.header.frame_id = self.get_parameter('frames.global_frame').value
            start_pose.pose.position.x = transform.transform.translation.x
            start_pose.pose.position.y = transform.transform.translation.y
            
        except Exception as e:
            self.get_logger().error(f"Error obteniendo TF: {e}")
            return

        path_msg = self.plan_ara_star(start_pose, msg)

        if path_msg is not None:
            self._publish_or_queue_path(path_msg, "¡Ruta ARA* publicada!")
        else:
            self.get_logger().error("ARA* falló al encontrar una ruta.")

    def waypoints_callback(self, msg: Path):
        """Handle waypoint pairs from waypoints_topic.

        Path.poses are structured as: [start_0, goal_0, start_1, goal_1, ...]
        """
        self.waypoint_pairs = []

        for i in range(0, len(msg.poses) - 1, 2):
            start = msg.poses[i]
            goal = msg.poses[i + 1]
            self.waypoint_pairs.append({'start': start, 'goal': goal})

        self.get_logger().info(f'Received {len(self.waypoint_pairs)} waypoint pairs')
        self.waypoints_pending_plan = True

        # --- Publicar marcadores de visualización de waypoints ---
        self._publish_waypoint_markers(msg)

        self.plan_waypoint_paths()

    # =================================================================
    # VISUALIZACIÓN DE WAYPOINTS
    # =================================================================
    def _publish_waypoint_markers(self, msg: Path):
        """
        Publica un MarkerArray en /ara_waypoints_markers mostrando
        únicamente una vuelta única de waypoints (sin repetidos).

          - Una esfera por waypoint único.
          - Un texto encima con índice y coordenadas.
        """
        frame_id = self.get_parameter('frames.global_frame').value
        stamp = self.get_clock().now().to_msg()

        marker_array = MarkerArray()

        # Borrar marcadores anteriores
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.ns = 'ara_waypoints'
        marker_array.markers.append(delete_all)

        # ============================================================
        # FILTRAR WAYPOINTS REPETIDOS
        # ============================================================
        unique_poses = []
        seen = set()

        for pose_stamped in msg.poses:
            x = round(pose_stamped.pose.position.x, 3)
            y = round(pose_stamped.pose.position.y, 3)

            key = (x, y)

            if key not in seen:
                seen.add(key)
                unique_poses.append(pose_stamped)

        # ============================================================
        # PUBLICAR SOLO LOS WAYPOINTS ÚNICOS
        # ============================================================
        for idx, pose_stamped in enumerate(unique_poses):
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y

            # ---- Esfera ----
            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = stamp
            sphere.ns = 'ara_waypoints'
            sphere.id = idx * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.0
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.15
            sphere.scale.y = 0.15
            sphere.scale.z = 0.15
            sphere.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)

            marker_array.markers.append(sphere)

            # ---- Texto ----
            label = Marker()
            label.header.frame_id = frame_id
            label.header.stamp = stamp
            label.ns = 'ara_waypoints'
            label.id = idx * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.30
            label.pose.orientation.w = 1.0
            label.scale.z = 0.18
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f'#{idx}  ({x:.2f}, {y:.2f})'

            marker_array.markers.append(label)

        self.waypoints_markers_pub.publish(marker_array)

        self.get_logger().info(
            f'Published {len(unique_poses)} unique waypoint markers on /ara_waypoints_markers'
        )

    def plan_waypoint_paths(self):
        """Plan paths for all waypoint pairs and publish as single combined path."""
        if self._map is None or self._obstacles is None or not self.waypoint_pairs:
            self.get_logger().warn('Map or waypoint pairs not available')
            return

        combined_path = Path()
        combined_path.header.frame_id = self.get_parameter('frames.global_frame').value

        for i, pair in enumerate(self.waypoint_pairs):
            path_msg = self.plan_ara_star(pair['start'], pair['goal'])
            if path_msg is None or not path_msg.poses:
                self.get_logger().warn(f'No valid path found for waypoint pair {i}')
                continue

            combined_path.poses.extend(path_msg.poses)
            self.get_logger().info(
                f'Waypoint pair {i} path calculated (length: {len(path_msg.poses)})'
            )

        if not combined_path.poses:
            self.get_logger().warn('No combined path could be generated from waypoint pairs')
            return

        self._publish_or_queue_path(
            combined_path,
            f'Published combined path with {len(combined_path.poses)} total poses'
        )
        self.waypoints_pending_plan = False

    # =================================================================
    # 4. FUNCIONES AUXILIARES 
    # =================================================================
    def world_to_map(self, x, y, x0, y0, res, W, H) -> Optional[Tuple[int, int]]:
        ix = int(math.floor((x - x0) / res))
        iy = int(math.floor((y - y0) / res))
        if 0 <= ix < W and 0 <= iy < H:
            return ix, iy
        return None

    def map_to_world(self, ix, iy, x0, y0, res) -> Tuple[float, float]:
        x = x0 + (ix + 0.5) * res
        y = y0 + (iy + 0.5) * res
        return x, y

    def get_neighbors(self, ix: int, iy: int, W: int, H: int) -> List[Tuple[int, int, float]]:
        neighbors = []
        
        for dx, dy, cost in self._allowed_moves:
            nx = ix + dx
            ny = iy + dy

            if 0 <= nx < W and 0 <= ny < H:
                if not self._obstacles[ny, nx]:
                    if abs(dx) == 1 and abs(dy) == 1:
                        if self._obstacles[iy, nx] or self._obstacles[ny, ix]:
                            continue
                    
                    neighbors.append((nx, ny, cost))
        return neighbors

    # =================================================================
    # 5. EL NÚCLEO MATEMÁTICO ARA* 
    # =================================================================
    def calculate_heuristic(self, curr_idx: Tuple[int, int], goal_idx: Tuple[int, int]) -> float:
        dx = abs(curr_idx[0] - goal_idx[0])
        dy = abs(curr_idx[1] - goal_idx[1])

        if self._heuristic_type == 'manhattan':
            return float(dx + dy)
        else:
            return math.hypot(dx, dy)

    def f_value(self, g: float, h: float, epsilon: float) -> float:
        return g + (epsilon * h)

    def improve_path(self, goal_idx: Tuple[int, int], epsilon: float, 
                     state_space: Dict[Tuple[int, int], 'ARANode'], 
                     OPEN: list, CLOSED: set, INCONS: set):
        W = self._map.info.width
        H = self._map.info.height

        if goal_idx not in state_space:
            state_space[goal_idx] = ARANode(goal_idx[0], goal_idx[1])

        while OPEN and state_space[goal_idx].g > OPEN[0][0]:
            
            current_f, current_idx = heapq.heappop(OPEN)

            if current_idx in CLOSED:
                continue

            current_node = state_space[current_idx]
            current_node.v = current_node.g
            CLOSED.add(current_idx)

            neighbors = self.get_neighbors(current_idx[0], current_idx[1], W, H)
            
            for nx, ny, transition_cost in neighbors:
                neighbor_idx = (nx, ny)
                
                if neighbor_idx not in state_space:
                    state_space[neighbor_idx] = ARANode(nx, ny)
                
                neighbor_node = state_space[neighbor_idx]
                new_g = current_node.g + transition_cost
                
                if neighbor_node.g > new_g:
                    neighbor_node.g = new_g
                    neighbor_node.parent = current_idx 

                    if neighbor_idx not in CLOSED:
                        h_val = self.calculate_heuristic(neighbor_idx, goal_idx)
                        new_f = self.f_value(new_g, h_val, epsilon)
                        heapq.heappush(OPEN, (new_f, neighbor_idx))
                    else:
                        INCONS.add(neighbor_idx)

    def reconstruct_path(self, start_idx, goal_idx, state_space, path_msg_header, x0, y0, res) -> Path:
        path_msg = Path()
        path_msg.header = path_msg_header

        current_idx = goal_idx
        path_cells = []

        while current_idx is not None:
            path_cells.append(current_idx)
            if current_idx == start_idx:
                break
            current_idx = state_space[current_idx].parent

        path_cells.reverse()

        last_yaw = 0.0
        
        for i, (ix, iy) in enumerate(path_cells):
            x, y = self.map_to_world(ix, iy, x0, y0, res)
            
            pose = PoseStamped()
            pose.header = path_msg_header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            if i + 1 < len(path_cells):
                next_ix, next_iy = path_cells[i + 1]
                nx_world, ny_world = self.map_to_world(next_ix, next_iy, x0, y0, res)
                last_yaw = math.atan2(ny_world - y, nx_world - x)
            
            pose.pose.orientation = yaw_to_quaternion(last_yaw)
            path_msg.poses.append(pose)

        return path_msg

    def plan_ara_star(self, start: PoseStamped, goal: PoseStamped) -> Optional[Path]:
        info = self._map.info
        res = info.resolution
        x0, y0 = info.origin.position.x, info.origin.position.y
        W, H = info.width, info.height

        s_idx = self.world_to_map(start.pose.position.x, start.pose.position.y, x0, y0, res, W, H)
        g_idx = self.world_to_map(goal.pose.position.x, goal.pose.position.y, x0, y0, res, W, H)

        if s_idx is None or g_idx is None:
            self.get_logger().error("Start o Goal fuera del mapa.")
            return None

        epsilon = self.get_parameter('ara_core.epsilon_start').value
        eps_dec = self.get_parameter('ara_core.epsilon_decrease').value
        time_limit = self.get_parameter('ara_core.time_limit_sec').value

        OPEN = []
        CLOSED = set()
        INCONS = set()
        state_space: Dict[Tuple[int, int], ARANode] = {} 

        start_node = ARANode(s_idx[0], s_idx[1])
        start_node.g = 0.0
        state_space[s_idx] = start_node
        
        h_start = self.calculate_heuristic(s_idx, g_idx)
        f_start = self.f_value(start_node.g, h_start, epsilon)
        heapq.heappush(OPEN, (f_start, s_idx))

        start_time = time.time()
        best_path_found = False

        debug_markers = MarkerArray()
        iteration_count = 0

        while epsilon >= 1.0:
            self.get_logger().info(f"Buscando ruta con epsilon={epsilon:.2f}...")
            
            self.improve_path(g_idx, epsilon, state_space, OPEN, CLOSED, INCONS)

            if g_idx in state_space and state_space[g_idx].g < float('inf'):
                best_path_found = True
                self.get_logger().info(f"¡Ruta subóptima encontrada para eps={epsilon:.2f}!")
                if self._debug_mode:
                    curr = g_idx
                    cells = []
                    while curr is not None:
                        cells.append(curr)
                        if curr == s_idx: break
                        curr = state_space[curr].parent
                    
                    marker = self.create_path_marker(
                        cells, epsilon, iteration_count, 
                        start.header.frame_id, x0, y0, res
                    )
                    debug_markers.markers.append(marker)
                    iteration_count += 1
            
            if (time.time() - start_time) > time_limit:
                self.get_logger().warn("Tiempo de cálculo agotado.")
                break
            
            if epsilon == 1.0:
                break
                
            epsilon -= eps_dec
            if epsilon < 1.0:
                epsilon = 1.0

            new_open_list = []
            
            for _, idx in OPEN:
                if idx not in CLOSED:
                    h_val = self.calculate_heuristic(idx, g_idx)
                    new_f = self.f_value(state_space[idx].g, h_val, epsilon)
                    new_open_list.append((new_f, idx))
            
            for idx in INCONS:
                h_val = self.calculate_heuristic(idx, g_idx)
                new_f = self.f_value(state_space[idx].g, h_val, epsilon)
                new_open_list.append((new_f, idx))

            heapq.heapify(new_open_list)
            OPEN = new_open_list
            
            INCONS.clear()
            CLOSED.clear()

        if self._debug_mode and debug_markers.markers:
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            debug_markers.markers.insert(0, delete_marker)
            self.debug_paths_pub.publish(debug_markers)

        if best_path_found:
            return self.reconstruct_path(s_idx, g_idx, state_space, start.header, x0, y0, res)
        else:
            return None

    # =================================================================
    # 6. FUNCIONES DE DEPURACIÓN
    # =================================================================
    def create_path_marker(self, path_cells: List[Tuple[int, int]], epsilon: float, 
                           marker_id: int, frame_id: str, x0: float, y0: float, res: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ara_eps_paths"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01 
        
        eps_start = self.get_parameter('ara_core.epsilon_start').value
        ratio = (epsilon - 1.0) / max((eps_start - 1.0), 0.01)
        
        marker.color = ColorRGBA()
        marker.color.r = max(0.0, min(1.0, float(ratio)))
        marker.color.g = max(0.0, min(1.0, float(1.0 - ratio)))
        marker.color.b = 0.0
        marker.color.a = 0.8

        for ix, iy in path_cells:
            x, y = self.map_to_world(ix, iy, x0, y0, res)
            p = Point()
            p.x, p.y = x, y
            p.z = marker_id * 0.02
            marker.points.append(p)
            
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = ARAPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()