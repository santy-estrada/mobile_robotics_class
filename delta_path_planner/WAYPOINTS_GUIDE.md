# Waypoints Trajectory Planning System - User Guide

## System Architecture

Your path planner now supports two modes:

### Mode 1: Single Goal (DEFAULT)
- Path planners listen to `/goal_pose` topic
- Each goal generates one trajectory
- Original behavior preserved
- **Parameter**: `waypoints:=false` (path planners)

### Mode 2: Waypoints (Multi-Point Trajectories)
- Waypoints node collects multiple points and creates trajectory pairs
- Path planners listen to `/waypoints_topic` for waypoint pairs
- All trajectories calculated and visible simultaneously in RViz
- **Parameter**: `waypoints:=true` (path planners)

---

## Waypoints Node

The `waypoints_node` is the orchestrator that:
1. Collects waypoints from user input or file
2. Computes start-goal pairs
3. Publishes pairs to path planners

### Waypoints Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `manual` | bool | `true` | If true, collect points from RViz; if false, load from file |
| `num_points` | int | `5` | Number of waypoints to collect in manual mode |
| `closed_loop` | bool | `true` | If true, last pair goes from final waypoint back to robot home |
| `waypoints_file` | string | `""` | Path to JSON file with waypoints (used when `manual:=false`) |

### Manual Mode (manual=true)

**Workflow:**
1. Start waypoints node
2. Click points in RViz using "Publish Point" tool
3. When num_points is reached, pairs are auto-generated and sent to planners
4. Process repeats (ready for new set of waypoints)

**Generated Pairs:**
- Pair 0: robot_position → waypoint_1
- Pair 1: waypoint_1 → waypoint_2
- Pair 2: waypoint_2 → waypoint_3
- ...
- Pair N (if closed_loop=true): waypoint_n → robot_position

**Example:**
```bash
ros2 run delta_path_planner waypoints_node
# Click 5 points in RViz → 5 trajectories generated
```

### Auto Mode (manual=false)

**Workflow:**
1. Prepare waypoints JSON file
2. Start waypoints node with file path
3. Node immediately processes waypoints and publishes pairs
4. Planners generate all trajectories at once

**JSON File Format:**
```json
{
  "waypoints": [
    {"x": 1.5, "y": 2.3},
    {"x": 3.2, "y": 4.1},
    {"x": 5.0, "y": 1.8},
    {"x": 2.5, "y": 0.5}
  ]
}
```

**Example:**
```bash
ros2 run delta_path_planner waypoints_node --ros-args \
  -p manual:=false \
  -p waypoints_file:=/home/user/my_waypoints.json \
  -p closed_loop:=true
```

---

## Path Planner Configuration

Both Dijkstra and Best First planners support the waypoints parameter:

### Path Planner Parameters (New)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `waypoints` | bool | `false` | If true, listen to `/waypoints_topic`; if false, listen to `/goal_pose` |

### Existing Parameters (Unchanged)
- `occ_thresh`: Occupancy threshold
- `robot_radius_m`: Robot radius in meters
- `safety_margin_m`: Safety margin around obstacles

### Best First Only
- `heuristic`: `'manhattan'` or `'euclidean'`
- `center`: Center paths on cell centers
- `avoid`: Increase cost near obstacles
- `inbounds_thresh`: Obstacle detection threshold

---

## Complete Usage Examples

### Example 1: Launch Full System (Gazebo + Both Planners + Waypoints)

**Terminal 1 - Main Launch:**
```bash
cd ~/mrad_ws_2601_delta
colcon build --packages-select delta_path_planner
source install/setup.bash
ros2 launch delta_path_planner delta_paths.launch.py
```

**Terminal 2 - Waypoints Node (Manual):**
```bash
source install/setup.bash
ros2 run delta_path_planner waypoints_node \
  --ros-args \
  -p manual:=true \
  -p num_points:=4 \
  -p closed_loop:=true
```

**Terminal 3 - Enable Planners to Use Waypoints:**
```bash
# NOTE: If your launch file doesn't already include planner nodes, run:
source install/setup.bash
ros2 run delta_path_planner dijkstra_node --ros-args -p waypoints:=true &
ros2 run delta_path_planner best_first_node --ros-args -p waypoints:=true &
```

**In RViz:**
1. Add tool "2D Nav Goal" or "Publish Point"
2. Click 4 points on the map
3. Waypoints node processes them and generates trajectories
4. All 4 paths appear in RViz on respective topics

---

### Example 2: Auto Mode with Saved Waypoints

**Create waypoints file** (`/tmp/mission.json`):
```json
{
  "waypoints": [
    {"x": 0.0, "y": 0.0},
    {"x": 5.0, "y": 0.0},
    {"x": 5.0, "y": 5.0},
    {"x": 0.0, "y": 5.0}
  ]
}
```

**Terminal 1 - Main Launch:**
```bash
ros2 launch delta_path_planner delta_paths.launch.py
```

**Terminal 2 - Waypoints from File:**
```bash
ros2 run delta_path_planner waypoints_node \
  --ros-args \
  -p manual:=false \
  -p waypoints_file:=/tmp/mission.json \
  -p closed_loop:=true
```

**Result:** Square mission path (4 waypoints, return to start)

---

### Example 3: Single Waypoint Only (Old Behavior)

**No waypoints node needed:**
```bash
ros2 launch delta_path_planner delta_paths.launch.py
```

**OR with explicit parameter:**
```bash
ros2 run delta_path_planner dijkstra_node --ros-args -p waypoints:=false
ros2 run delta_path_planner best_first_node --ros-args -p waypoints:=false
```

Then publish goals to `/goal_pose` as before.

---

## RViz Visualization Setup

### Display Multiple Paths

1. **Add Path displays** (one for each topic):
   - `/dijkstra_path`
   - `/best_first_path`

2. **Configure colors** to distinguish paths:
   - Dijkstra: Green
   - Best First: Red/Blue

3. **Set frame** to `map`

4. **Set "Style" to** `Arrows` or `Tubes` for better visibility

---

## Troubleshooting

### Waypoints Not Being Collected
- Check that waypoints_node is running: `ros2 node list | grep waypoint`
- Verify RViz "Publish Point" is active
- Check console output for TF errors (need map → base_link transform)

### Paths Not Generating
- Ensure planner nodes have `waypoints:=true`
- Check that `/waypoints_topic` is being published: `ros2 topic list | grep waypoint`
- Verify map is available on `/map` topic
- Check planner node logs for obstacles blocking paths

### closed_loop Issues
- If `closed_loop:=true`, ensure robot can return to start (no obstacles)
- If paths are not closing properly, check robot_radius and safety_margin

### Performance
- For many waypoints, planning may take time
- Consider using Best First with `heuristic:=euclidean` for speed
- Monitor CPU usage if waypoints are dense

---

## Implementation Details

### Message Flow
```
RViz Goal Input
    ↓
Waypoints Node (manual mode)
    ↓ (collects num_points)
    ↓
Waypoint Pair Generation
    ↓
/waypoints_topic (nav_msgs/Path)
    ↓
Dijkstra & Best First Nodes (waypoints:=true)
    ↓
Path Planning (per pair)
    ↓
/dijkstra_path & /best_first_path
    ↓
RViz Visualization
```

### Data Structure
Path.poses = [start_pair0, goal_pair0, start_pair1, goal_pair1, ...]

Each pose is a PoseStamped with:
- header.frame_id = "map"
- pose.position (x, y, z)
- pose.orientation (w=1.0, others=0)

---

## Parameter Cheat Sheet

**Quick Copy-Paste Commands:**

```bash
# Manual 5-point loop trajectory
ros2 run delta_path_planner waypoints_node -p manual:=true -p num_points:=5 -p closed_loop:=true

# Auto mode with file
ros2 run delta_path_planner waypoints_node -p manual:=false -p waypoints_file:=/path/to/file.json

# Enable waypoints in Dijkstra
ros2 run delta_path_planner dijkstra_node -p waypoints:=true

# Enable waypoints in Best First
ros2 run delta_path_planner best_first_node -p waypoints:=true -p heuristic:=euclidean

# Single goal mode (default)
ros2 run delta_path_planner dijkstra_node -p waypoints:=false
```

---

## Next Steps

1. **Build the package:**
   ```bash
   cd ~/mrad_ws_2601_delta
   colcon build --packages-select delta_path_planner
   ```

2. **Test manual mode** with 3-5 waypoints in RViz

3. **Create a waypoints JSON file** for your mission

4. **Test closed_loop** with both true and false

5. **Monitor topics** in RViz to visualize path generation

Enjoy your waypoint trajectories!
