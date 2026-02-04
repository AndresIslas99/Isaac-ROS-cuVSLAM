# Jetson SLAM Pipeline Architecture -- v3.0.0

## Overview

The v3.0.0 stack runs on a Jetson Orin AGX 64GB (R36.4.7) with ROS2 Humble and CycloneDDS. The pipeline combines a ZED 2i stereo camera with NVIDIA cuVSLAM for visual-inertial odometry and nvblox for mesh/voxel reconstruction. RTAB-Map has been fully removed as of v3.0.0.

## Data Flow

```
ZED 2i (HD1080@15fps, IMU@400Hz)
  |
  |--- /zed/left/image_rect_color -----+
  |--- /zed/right/image_rect_color ----+---> cuVSLAM (stereo VIO)
  |--- /zed/imu/data -----------------+        |
  |                                             |---> /visual_slam/tracking/odometry
  |                                             |---> /tf (odom->base_link)
  |
  |--- /zed/depth/depth_registered ---> depth_filter_node (4-stage)
  |                                        |
  |                                        |---> /depth_filter/filtered
  |                                        |---> /depth_filter/latency (Float32)
  |                                        |---> /depth_filter/quality (String JSON)
  |                                        |
  |                                        +---> nvblox
  |                                                |
  |                                                |---> /nvblox_node/mesh
  |                                                |---> /nvblox_node/map_slice
  |
  +---> slam_monitor_node
  |        |---> /slam/quality (String JSON, ~50 fields)
  |
  +---> pipeline_watchdog_node
  |        |---> /watchdog/heartbeat
  |
  +---> coverage_monitor.py
  |        |---> /coverage/grid
  |        |---> /coverage/status
  |
  +---> session_recorder.py
           |---> /session/info
           |---> /session/start_recording (std_srvs/Trigger)
           |---> /session/stop_recording  (std_srvs/Trigger)
```

## Composable Container Design

All GPU-intensive C++ nodes are loaded into a single `component_container_mt` (multi-threaded composable container) named `slam_container`. This avoids GPU context switching overhead.

Nodes loaded via `LoadComposableNodes`:
- ZED camera node
- cuVSLAM visual SLAM node
- nvblox mapping node

### IPC Decision

Intra-process communication (IPC) is **disabled**:
- `enable_ipc=false` on the container
- `use_intra_process_comms=False` on each `ComposableNode`

**Rationale:** Enabling IPC with the current cuVSLAM + nvblox combination causes intermittent crashes in the composable container due to shared pointer lifetime issues across the GPU pipeline. CycloneDDS zero-copy transport provides sufficient performance for the image topics at 15fps HD1080. The latency penalty (~2ms per topic hop) is acceptable.

### Standalone Nodes

The following run as separate processes outside the composable container:
- `depth_filter_node` -- standalone lifecycle node
- `slam_monitor_node` -- standalone node
- `pipeline_watchdog_node` -- standalone node
- `coverage_monitor.py` -- Python node
- `session_recorder.py` -- Python node

## Node Descriptions

### ZED Camera Node (composable)

Publishes stereo images, depth, and IMU data from the ZED 2i camera.

- Resolution: HD1080 (1920x1080)
- `grab_frame_rate`: 30 (SDK parameter; actual output is 15fps at 1080p due to hardware limits)
- Depth mode: PERFORMANCE
- IMU rate: 400Hz (BMI085 sensor)
- `publish_tf`: false (cuVSLAM handles the TF tree)

### cuVSLAM Node (composable)

NVIDIA CUDA Visual SLAM providing stereo visual-inertial odometry.

- Mode: Stereo VIO with IMU fusion (BMI085)
- Ground constraint enabled
- `image_jitter_threshold_ms`: 70 (calibrated for 15fps jitter)
- Pure VIO mode -- no internal loop closure or SLAM map. This node provides odometry only.
- Publishes: `/visual_slam/tracking/odometry`, TF `odom -> base_link`

### nvblox Node (composable)

Voxel-based 3D reconstruction replacing the former RTAB-Map pipeline.

- Consumes filtered depth from `depth_filter_node`
- Publishes mesh and 2D map slice for navigation
- Topics: `/nvblox_node/mesh`, `/nvblox_node/map_slice`

### depth_filter_node (standalone)

Four-stage depth image filter optimized for single-pass execution at ~40ms per frame on HD1080.

Pipeline stages:
1. Range clamp
2. Temporal smoothing
3. Bilateral filter
4. Hole filling

Publishes:
- `/depth_filter/filtered` (sensor_msgs/Image)
- `/depth_filter/latency` (std_msgs/Float32) -- per-frame processing time in ms
- `/depth_filter/quality` (std_msgs/String) -- JSON with per-stage metrics

### slam_monitor_node (standalone)

Comprehensive diagnostics node with 7 monitoring groups:

| Group | What it monitors |
|-------|-----------------|
| Sensors | Camera frame rates, IMU rates, image topic health |
| TF | Transform freshness, TF age threshold (150ms) |
| Jetson | GPU/CPU temps, power, clocks via tegrastats pipe thread |
| Latency | End-to-end pipeline latency, depth filter latency |
| DataQuality | Depth image statistics, NaN percentage, coverage |
| DiskIO | Disk usage via `statvfs`, I/O throughput via `/proc/diskstats` |
| Pipeline | Node liveness, topic publication rates, restart counts |

Publishes `/slam/quality` as a `std_msgs/String` containing a ~50-field JSON document.

Tegrastats data is read via a persistent pipe thread (not a subprocess per sample). Disk monitoring uses `statvfs()` for free space and parses `/proc/diskstats` for I/O counters.

### pipeline_watchdog_node (standalone)

Monitors critical nodes and performs crash recovery (restart via launch actions). Publishes `/watchdog/heartbeat` at 1Hz.

### coverage_monitor.py (standalone, Python)

Tracks spatial coverage of the SLAM session using camera FOV projection.

- Message types: `std_msgs/String` (JSON) -- does NOT use `agv_greenhouse_msgs`
- Publishes: `/coverage/grid`, `/coverage/status`

### session_recorder.py (standalone, Python)

Manages recording sessions (rosbag, SVO2 files, TUM-format trajectories).

- Services: `/session/start_recording`, `/session/stop_recording` (both `std_srvs/Trigger`)
- Publisher: `/session/info` (`std_msgs/String` JSON)
- Does NOT use `agv_greenhouse_msgs`

## Topic List

| Topic | Type | Hz | Producer |
|-------|------|-----|----------|
| `/zed/left/image_rect_color` | sensor_msgs/Image | 15 | ZED node |
| `/zed/right/image_rect_color` | sensor_msgs/Image | 15 | ZED node |
| `/zed/depth/depth_registered` | sensor_msgs/Image | 15 | ZED node |
| `/zed/imu/data` | sensor_msgs/Imu | 400 | ZED node |
| `/visual_slam/tracking/odometry` | nav_msgs/Odometry | 15 | cuVSLAM |
| `/tf` | tf2_msgs/TFMessage | 15 | cuVSLAM |
| `/depth_filter/filtered` | sensor_msgs/Image | 15 | depth_filter_node |
| `/depth_filter/latency` | std_msgs/Float32 | 15 | depth_filter_node |
| `/depth_filter/quality` | std_msgs/String | 15 | depth_filter_node |
| `/nvblox_node/mesh` | nvblox_msgs/Mesh | ~2 | nvblox |
| `/nvblox_node/map_slice` | nvblox_msgs/DistanceMapSlice | ~2 | nvblox |
| `/slam/quality` | std_msgs/String | 1 | slam_monitor_node |
| `/watchdog/heartbeat` | std_msgs/String | 1 | pipeline_watchdog_node |
| `/coverage/grid` | std_msgs/String | ~1 | coverage_monitor.py |
| `/coverage/status` | std_msgs/String | ~1 | coverage_monitor.py |
| `/session/info` | std_msgs/String | ~0.2 | session_recorder.py |

## TF Tree

```
odom (cuVSLAM)
  +--- base_link (cuVSLAM publishes this transform)
         +--- zed_camera_link (static TF from launch)
                +--- zed_left_camera_frame (static, from URDF/static_tf)
                +--- zed_right_camera_frame (static, from URDF/static_tf)
                +--- zed_imu_link (static, from URDF/static_tf)
```

cuVSLAM publishes the dynamic `odom -> base_link` transform. The ZED node has `publish_tf: false` to avoid conflicting transforms. All camera-to-base transforms are published as static TFs from the launch file.

## Platform

- Hardware: Jetson Orin AGX 64GB
- JetPack / L4T: R36.4.7
- ROS2: Humble
- DDS: CycloneDDS
- CUDA: via JetPack (used by cuVSLAM, nvblox, depth_filter_node)
