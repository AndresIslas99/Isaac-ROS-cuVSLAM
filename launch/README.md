# Launch Files

## agv_slam.launch.py — Main Pipeline

Orchestrates the full SLAM pipeline with timed startup sequence.

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `localization` | `false` | `true` = localize on existing map, `false` = build new map |
| `enable_rviz` | `false` | Launch RViz2 with pre-configured layout |
| `database_path` | `/home/orza/slam_maps/greenhouse.db` | RTAB-Map SQLite database path |
| `use_sim_time` | `false` | Use simulation clock |

### Environment Variables (set automatically)

| Variable | Value | Purpose |
|----------|-------|---------|
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | Use CycloneDDS instead of FastDDS |
| `CYCLONEDDS_URI` | `file://.../config/cyclonedds.xml` | DDS transport configuration |

### Node Launch Order

```
t=0s   ┌── Static TF (base_link → zed2i_base_link)
       ├── ZED 2i camera driver
t=3s   ├── Depth Filter Node
t=5s   ├── cuVSLAM (GPU VIO)
t=8s   ├── RTAB-Map (mapping OR localization, conditional)
t=10s  ├── SLAM Monitor
t=0s   └── RViz (if enable_rviz:=true)
```

Delays are implemented via `TimerAction` to ensure upstream nodes are publishing before downstream consumers start.

### Mapping vs Localization Mode

**Mapping** (`localization:=false`, default):
- RTAB-Map starts with `--delete_db_on_start` (fresh map)
- `Mem/IncrementalMemory: true` (adds new nodes)

**Localization** (`localization:=true`):
- RTAB-Map loads existing database (no `--delete_db_on_start`)
- `Mem/IncrementalMemory: false` (read-only map)
- `Mem/InitWMWithAllNodes: true` (load entire map to RAM)

### Topic Remappings

**cuVSLAM** receives stereo + IMU:
```
visual_slam/image_0      ← /zed/zed_node/left/image_rect_gray
visual_slam/camera_info_0 ← /zed/zed_node/left/camera_info
visual_slam/image_1      ← /zed/zed_node/right/image_rect_gray
visual_slam/camera_info_1 ← /zed/zed_node/right/camera_info
visual_slam/imu          ← /zed/zed_node/imu/data
```

**RTAB-Map** receives filtered RGB-D + external odometry + IMU:
```
rgb/image        ← /filtered/rgb
depth/image      ← /filtered/depth
rgb/camera_info  ← /filtered/camera_info
odom             ← /visual_slam/tracking/odometry
imu              ← /zed/zed_node/imu/data
```

### Static Transform

`base_link → zed2i_base_link`:
- x=0.1m (forward), y=0.0m, z=0.3m (up)
- pitch=0.087 rad (~5 degrees downward tilt)
- Adjust these values to match your physical camera mount

## monitor.launch.py — Visualization

Launches two nodes:
1. **RViz2** with `rviz/slam.rviz` — shows TF, occupancy grid, point cloud, images, odometry, path
2. **rqt_topic** — real-time topic browser for debugging rates and data

No arguments. Connects to already-running pipeline topics.

## Adding a Node to the Launch

1. Import `Node` and `TimerAction` (already imported)
2. Define the node with config file, remappings, and output:
   ```python
   new_node = TimerAction(
       period=X.0,  # seconds delay (after dependencies are publishing)
       actions=[
           Node(
               package='agv_slam',
               executable='new_node_name',
               name='new_node',
               parameters=[os.path.join(pkg_agv_slam, 'config', 'new_node.yaml')],
               remappings=[...],
               output='screen',
           ),
       ]
   )
   ```
3. Add to the `LaunchDescription` list at the appropriate position
4. Choose delay period based on what topics the node needs (see launch order above)
