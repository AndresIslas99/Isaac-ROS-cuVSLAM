# CLAUDE.md — AI Assistant Guide for agv_slam

## Project Overview

Industrial SLAM pipeline for an AGV (Automated Guided Vehicle) operating in greenhouse environments. Runs on **Jetson Orin AGX 64GB** with a **ZED 2i** stereo camera. Combines NVIDIA Isaac ROS cuVSLAM (GPU-accelerated visual-inertial odometry) with RTAB-Map (mapping + loop closure) for real-time localization and mapping.

**Key architectural decision**: cuVSLAM handles odometry at ~1.8ms/frame on GPU, while RTAB-Map focuses exclusively on mapping and loop closure (its built-in VO at 435ms/frame was too slow). This 9x speedup enables 30Hz operation.

## Build & Run

```bash
# Build (from workspace root, one level above this package)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Launch full pipeline (mapping mode)
ros2 launch agv_slam agv_slam.launch.py

# Launch in localization mode (uses existing map)
ros2 launch agv_slam agv_slam.launch.py localization:=true

# Launch monitoring/visualization
ros2 launch agv_slam monitor.launch.py

# Validate pipeline health
bash scripts/validate_slam.sh [duration_seconds]

# Jetson performance tuning
sudo bash scripts/jetson_setup.sh
```

**No test suite exists.** Validation is done at runtime via `validate_slam.sh` which checks node status, topic rates, TF chain, and system resources.

## Repository Structure

```
agv_slam/
├── CMakeLists.txt          # ament_cmake build, two C++ executables
├── package.xml             # ROS2 package manifest (format 3)
├── config/
│   ├── cuvslam.yaml        # cuVSLAM GPU odometry parameters
│   ├── rtabmap.yaml        # RTAB-Map mapping/loop closure parameters
│   ├── depth_filter.yaml   # Depth preprocessing pipeline parameters
│   ├── zed2i.yaml          # ZED 2i camera driver configuration
│   └── cyclonedds.xml      # DDS shared-memory transport config
├── include/agv_slam/
│   ├── depth_filter_node.hpp
│   └── slam_monitor_node.hpp
├── src/
│   ├── depth_filter_node.cpp   # Depth preprocessing (confidence, temporal, bilateral)
│   └── slam_monitor_node.cpp   # 1Hz diagnostics publisher
├── launch/
│   ├── agv_slam.launch.py      # Main pipeline orchestration
│   └── monitor.launch.py       # RViz + rqt visualization
├── scripts/
│   ├── jetson_setup.sh         # Orin power/clock tuning
│   └── validate_slam.sh       # Runtime health checks
├── rviz/
│   └── slam.rviz              # Pre-configured visualization
└── urdf/
    └── zed2i_mount.urdf.xacro # Camera mount TF geometry
```

## Architecture & Data Flow

```
ZED 2i Driver ──► Depth Filter Node ──► cuVSLAM (GPU odometry)
     │                  │                      │
     │                  └──► RTAB-Map (mapping + loop closure)
     │                              │
     └──► SLAM Monitor Node ◄──────┘
```

**TF tree**: `map → odom → base_link → zed2i_base_link → camera frames`
- `map → odom`: RTAB-Map (loop closure corrections)
- `odom → base_link`: cuVSLAM (real-time odometry)
- `base_link → zed2i_base_link`: Static transform in launch file

**Launch sequence uses TimerAction delays**: ZED (0s) → Depth Filter (3s) → cuVSLAM (5s) → RTAB-Map (8s) → Monitor (10s).

## Code Conventions

- **Language**: C++17 with ROS2 Humble, launch files in Python
- **Namespace**: All custom code in `namespace agv_slam`
- **Classes**: PascalCase (`DepthFilterNode`, `SlamMonitorNode`)
- **Methods/variables**: snake_case (`apply_confidence_filter()`, `min_depth_`)
- **Member variables**: trailing underscore (`filtered_depth_pub_`)
- **Headers**: `.hpp`, implementations: `.cpp`
- **Include guards**: `#pragma once`
- **Indentation**: 2 spaces
- **Compiler flags**: `-Wall -Wextra -Wpedantic -O3`
- **Config files**: YAML for ROS2 parameters, XML for DDS only
- **QoS**: `SensorDataQoS` (best effort) for sensor topics, reliable for diagnostics

## Custom Nodes (this package provides two)

### depth_filter_node
- Subscribes to ZED RGB + Depth with ApproximateTime sync
- Applies: PassThrough → Temporal averaging → Bilateral smoothing → optional hole filling
- Publishes filtered images on `/filtered/rgb`, `/filtered/depth`, `/filtered/camera_info`
- Links: sensor_msgs, image_transport, cv_bridge, message_filters, OpenCV

### slam_monitor_node
- Subscribes to RGB, Depth, IMU, and Odometry topics
- Publishes `diagnostic_msgs/DiagnosticArray` on `/slam/diagnostics` at 1Hz
- Links: diagnostic_msgs, nav_msgs, tf2_ros

## External Dependencies (not built here)

| Package | Role |
|---------|------|
| `zed_wrapper` / `zed_ros2` | ZED 2i camera driver |
| `isaac_ros_visual_slam` | NVIDIA cuVSLAM GPU odometry |
| `rtabmap_ros` / `rtabmap_slam` | Mapping + loop closure |
| `rmw_cyclonedds_cpp` | DDS middleware with shared memory |

## Configuration Notes

- **cuvslam.yaml**: IMU fusion enabled, ground constraint on, mapping/landmarks disabled (RTAB-Map handles those)
- **rtabmap.yaml**: Tuned for greenhouse rows — aggressive loop closure (threshold 0.09), spatial proximity detection, GFTT+ORB features, GTSAM optimizer, 3DoF constraint (X,Y,Yaw)
- **zed2i.yaml**: NEURAL depth mode, HD720@30fps, strict confidence filtering, internal VIO for rolling shutter correction only (`publish_tf: false`)
- **cyclonedds.xml**: Shared memory enabled, 10MB buffers, localhost only
- **Map database**: Stored at `/home/orza/slam_maps/greenhouse.db`

## Target Platform

- **Hardware**: NVIDIA Jetson Orin AGX 64GB + ZED 2i stereo camera
- **Software**: JetPack 6.2+, Isaac ROS 3.2+, ZED SDK 5.1+, ROS2 Humble
- **No Docker** — runs directly on Jetson with system ROS2
- **DDS**: CycloneDDS (set via `RMW_IMPLEMENTATION` env var in launch)

## Common Tasks

**Adding a new parameter**: Declare in the relevant `config/*.yaml`, load via `get_parameter()` in the node constructor.

**Adding a new node**: Create `.hpp` in `include/agv_slam/`, `.cpp` in `src/`, add `add_executable()` + `ament_target_dependencies()` + `install(TARGETS)` in `CMakeLists.txt`, include in launch file with appropriate TimerAction delay.

**Modifying the pipeline**: Edit `launch/agv_slam.launch.py`. Nodes are launched with TimerAction delays to ensure proper initialization order. Respect the dependency chain: ZED → Depth Filter → cuVSLAM → RTAB-Map.

**Tuning SLAM parameters**: Edit `config/rtabmap.yaml` for mapping behavior, `config/cuvslam.yaml` for odometry. The greenhouse environment requires spatial proximity detection and aggressive loop closure for repetitive rows.
