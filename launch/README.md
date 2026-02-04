# Launch Files

## agv_slam.launch.py — Main Pipeline

Orchestrates the full SLAM pipeline with timed startup sequence using `TimerAction` delays.

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_foxglove` | `false` | Launch Foxglove WebSocket bridge on port 8765 |
| `camera_x` | `0.1` | Camera forward offset from base_link (meters) |
| `camera_y` | `0.0` | Camera lateral offset (meters) |
| `camera_z` | `0.3` | Camera height above base_link (meters) |
| `camera_pitch` | `0.087` | Camera downward tilt in radians (~5 degrees) |

### Environment Variables (set automatically)

| Variable | Value | Purpose |
|----------|-------|---------|
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | Use CycloneDDS |
| `CYCLONEDDS_URI` | `file://.../config/cyclonedds.xml` | DDS config path |

### Node Launch Order

```
t=0s   ┌── Static TF publisher (base_link → zed_camera_link)
       ├── component_container_mt (hosts ZED + cuVSLAM + nvblox)
t=3s   ├── depth_filter_node (C++ standalone)
t=5s   ├── cuVSLAM (composable, loaded into container)
t=8s   ├── nvblox (composable, loaded into container)
t=10s  ├── pipeline_watchdog_node (C++ standalone)
       ├── slam_monitor_node (C++ standalone)
t=12s  ├── coverage_monitor.py (Python)
       └── session_recorder.py (Python)
(if enabled) foxglove_bridge
```

### Composable Container

ZED, cuVSLAM, and nvblox run as composable nodes in `component_container_mt` (multi-threaded).

**IPC is disabled**:
- ZED launch args include `'enable_ipc': 'false'`
- cuVSLAM and nvblox use `extra_arguments=[{'use_intra_process_comms': False}]`

This is required because ZED wrapper creates transient_local QoS publishers that conflict with IPC's volatile-only requirement.

### Topic Remappings

**cuVSLAM** receives stereo + IMU:
```
visual_slam/image_0       ← /zed/zed_node/left/image_rect_gray
visual_slam/camera_info_0 ← /zed/zed_node/left/camera_info
visual_slam/image_1       ← /zed/zed_node/right/image_rect_gray
visual_slam/camera_info_1 ← /zed/zed_node/right/camera_info
visual_slam/imu           ← /zed/zed_node/imu/data
```

**nvblox** receives depth + RGB + odometry:
```
depth/image     ← /zed/zed_node/depth/depth_registered
color/image     ← /zed/zed_node/rgb/image_rect_color
color/camera_info ← /zed/zed_node/rgb/color/rect/camera_info
```

### Static Transform

`base_link → zed_camera_link`:
- x=0.1m (forward), y=0.0m, z=0.3m (up)
- pitch=0.087 rad (~5 degrees downward tilt)
- Override via launch arguments: `camera_x`, `camera_y`, `camera_z`, `camera_pitch`

### Foxglove Bridge

When `enable_foxglove:=true`, launches `foxglove_bridge` with a whitelist of relevant topics for web visualization at `ws://<jetson_ip>:8765`.
