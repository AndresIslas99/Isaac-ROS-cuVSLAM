# CLAUDE.md — AI Assistant Guide for agv_slam

> **For local Claude Code agents on Jetson**: This file is the root entry point. Read this first, then drill into the linked docs for detail on specific areas. Each directory has its own README.md with implementation-level context.

## Documentation Map

| Document | What it covers |
|----------|---------------|
| **CLAUDE.md** (this file) | Quick-reference overview, build/run, structure, conventions |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Full system design, data flow, TF tree, QoS, compute budget, greenhouse tuning rationale |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Diagnostic commands, 9 common failure modes with fixes, performance baselines |
| [docs/JETSON_DEPLOY.md](docs/JETSON_DEPLOY.md) | First-time Jetson setup, prerequisites, systemd service, map management |
| [config/README.md](config/README.md) | Every parameter in every YAML/XML file with values, defaults, and rationale |
| [src/README.md](src/README.md) | Node class details, method-level docs with line numbers, code style, "add a node" guide |
| [launch/README.md](launch/README.md) | Launch arguments, node timing, remappings, mapping vs localization mode |
| [scripts/README.md](scripts/README.md) | What each script does step-by-step, reversibility, prerequisites |

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

# Jetson performance tuning (run once per boot, requires sudo)
sudo bash scripts/jetson_setup.sh
```

**No test suite exists.** Validation is done at runtime via `validate_slam.sh` which checks node status, topic rates, TF chain, and system resources.

## Repository Structure

```
agv_slam/
├── CLAUDE.md               # THIS FILE — AI assistant entry point
├── README.md               # Human-facing project README
├── CMakeLists.txt          # ament_cmake build, two C++ executables
├── package.xml             # ROS2 package manifest (format 3)
├── docs/
│   ├── ARCHITECTURE.md     # Full system design + data flow + tuning rationale
│   ├── TROUBLESHOOTING.md  # 9 common issues with diagnostic steps + fixes
│   └── JETSON_DEPLOY.md    # Hardware setup, first-time install, systemd, map mgmt
├── config/
│   ├── README.md           # Parameter-level docs for all config files
│   ├── cuvslam.yaml        # cuVSLAM GPU odometry (IMU noise, frame IDs, VIO mode)
│   ├── rtabmap.yaml        # RTAB-Map mapping (node creation, loop closure, GTSAM, grid)
│   ├── depth_filter.yaml   # Depth preprocessing (range, temporal, bilateral, holes)
│   ├── zed2i.yaml          # ZED 2i camera (NEURAL depth, confidence, resolution)
│   └── cyclonedds.xml      # DDS transport (shared memory, buffers, localhost)
├── include/agv_slam/
│   ├── depth_filter_node.hpp   # DepthFilterNode class declaration
│   └── slam_monitor_node.hpp   # SlamMonitorNode class + RateTracker struct
├── src/
│   ├── README.md               # Node implementation details, method docs, code style
│   ├── depth_filter_node.cpp   # 4-stage depth preprocessing (278 lines)
│   └── slam_monitor_node.cpp   # 1Hz diagnostics publisher (164 lines)
├── launch/
│   ├── README.md               # Launch args, timing, remappings, modes
│   ├── agv_slam.launch.py      # Main pipeline (5 nodes + static TF + env setup)
│   └── monitor.launch.py       # RViz + rqt visualization
├── scripts/
│   ├── README.md               # Script docs, steps, reversibility
│   ├── jetson_setup.sh         # 7-step Orin MAXN tuning (requires sudo)
│   └── validate_slam.sh        # 5-check runtime health validation
├── rviz/
│   └── slam.rviz              # Pre-configured: TF, grid map, point cloud, images, odom
└── urdf/
    └── zed2i_mount.urdf.xacro # Camera mount: base_link → camera_mount → zed2i_base_link
```

## Architecture Summary

```
ZED 2i (30Hz RGB-D + 400Hz IMU)
    │
    ├── Stereo grayscale + IMU ──► cuVSLAM (GPU, 1.8ms/frame)
    │                                   │
    │                                   ├── /visual_slam/tracking/odometry
    │                                   └── TF: odom → base_link
    │
    ├── RGB + Depth ──► Depth Filter Node (OpenCV, <1ms)
    │                        │
    │                        ├── /filtered/rgb
    │                        ├── /filtered/depth
    │                        └── /filtered/camera_info
    │                                │
    └────────────────────────────────┼──► RTAB-Map (mapping only, ~50ms)
                                     │        │
                                     │        ├── TF: map → odom
                                     │        ├── /rtabmap/grid_map
                                     │        └── /rtabmap/cloud_map
                                     │
                                     └──► SLAM Monitor (1Hz diagnostics)
                                              └── /slam/diagnostics
```

**TF tree**: `map → odom → base_link → zed2i_base_link → camera optical frames`

**Launch timing**: ZED (0s) → Depth Filter (3s) → cuVSLAM (5s) → RTAB-Map (8s) → Monitor (10s)

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for complete details including QoS policies, memory budget, and greenhouse-specific tuning rationale.

## Code Conventions

- **Language**: C++17 with ROS2 Humble, launch files in Python
- **Namespace**: All custom code in `namespace agv_slam`
- **Classes**: PascalCase (`DepthFilterNode`, `SlamMonitorNode`)
- **Methods/variables**: snake_case (`apply_confidence_filter()`, `min_depth_`)
- **Member variables**: trailing underscore (`filtered_depth_pub_`)
- **Headers**: `.hpp`, implementations: `.cpp`
- **Include guards**: `#pragma once`
- **Include order**: own header → STL → ROS → OpenCV
- **Indentation**: 2 spaces
- **Section markers**: `// ── Section Name ──`
- **Compiler flags**: `-Wall -Wextra -Wpedantic -O3`
- **Config files**: YAML for ROS2 parameters, XML for DDS only
- **QoS**: `SensorDataQoS` (best effort) for sensor topics, reliable for diagnostics
- **Error logging**: `RCLCPP_ERROR_THROTTLE` for recoverable, exception catch for cv_bridge

## Custom Nodes (this package builds two executables)

### depth_filter_node
- **Class**: `DepthFilterNode` — `src/depth_filter_node.cpp` + `include/agv_slam/depth_filter_node.hpp`
- Subscribes to ZED RGB + Depth with `ApproximateTime` sync (buffer=10)
- 4-stage pipeline: PassThrough → Temporal averaging (deque, mutex) → Bilateral smoothing → Hole filling (Telea inpaint)
- Publishes `/filtered/rgb`, `/filtered/depth`, `/filtered/camera_info`
- Executor: `MultiThreadedExecutor` (2 threads)
- Dependencies: sensor_msgs, image_transport, cv_bridge, message_filters, OpenCV

### slam_monitor_node
- **Class**: `SlamMonitorNode` — `src/slam_monitor_node.cpp` + `include/agv_slam/slam_monitor_node.hpp`
- Subscribes to RGB, Depth, IMU, Odometry — tracks rates via `RateTracker` (EMA, mutex)
- Publishes `diagnostic_msgs/DiagnosticArray` on `/slam/diagnostics` at 1Hz
- Checks TF chain `map → base_link`, tracks odometry distance
- Health thresholds: RGB/Depth/Odom < 10Hz = WARN, IMU < 100Hz = WARN
- Dependencies: diagnostic_msgs, nav_msgs, tf2_ros

See [src/README.md](src/README.md) for method-level documentation with line numbers.

## External Dependencies (runtime only, not built here)

| Package | Role | Config file |
|---------|------|-------------|
| `zed_wrapper` / `zed_ros2` | ZED 2i camera driver | `config/zed2i.yaml` |
| `isaac_ros_visual_slam` | NVIDIA cuVSLAM GPU odometry | `config/cuvslam.yaml` |
| `rtabmap_ros` / `rtabmap_slam` | Mapping + loop closure | `config/rtabmap.yaml` |
| `rmw_cyclonedds_cpp` | DDS middleware | `config/cyclonedds.xml` |

## Key Configuration Decisions

| Decision | Parameter | Value | Why |
|----------|-----------|-------|-----|
| Map node every 5cm | `RGBD/LinearUpdate` | `0.05` | Default 0.1m too sparse for slow AGV |
| Aggressive loop closure | `Rtabmap/LoopThr` | `0.09` | Greenhouse rows look similar, need lower threshold |
| Spatial proximity | `RGBD/ProximityBySpace` | `true` | Row revisits detected by position, not appearance |
| GFTT+ORB features | `Vis/FeatureType` | `8` | Uniform distribution across image (vs clustering on plants) |
| GTSAM optimizer | `Optimizer/Strategy` | `2` | Gravity-aware graph optimization |
| 3DoF only | `Reg/Force3DoF` | `true` | Ground robot: X, Y, Yaw only |
| Unlimited memory | `Rtabmap/MemoryThr` | `0` | Exploit 64GB RAM |
| NEURAL depth | `depth_mode` | `NEURAL` | Best accuracy on Orin AGX at 34fps |
| VIO-only cuVSLAM | `enable_localization_n_mapping` | `false` | RTAB-Map does mapping, cuVSLAM just tracks |

See [config/README.md](config/README.md) for complete parameter documentation.

## Target Platform

- **Hardware**: NVIDIA Jetson Orin AGX 64GB + ZED 2i (USB 3.0)
- **Software**: JetPack 6.2+, Isaac ROS 3.2+, ZED SDK 5.1+, ROS2 Humble
- **No Docker** — runs directly on Jetson with system ROS2
- **DDS**: CycloneDDS with shared memory (set automatically by launch file)
- **Workspace path**: `~/agv_slam_ws/` (source in `src/agv_slam/`)
- **Map storage**: `/home/orza/slam_maps/greenhouse.db`

See [docs/JETSON_DEPLOY.md](docs/JETSON_DEPLOY.md) for full deployment guide.

## Common Tasks

**Adding a new parameter**: Declare in `config/*.yaml`, add `declare_parameter()` + `get_parameter()` in node constructor. Rebuild only if C++ code changed; YAML changes take effect on next launch with `--symlink-install`.

**Adding a new node**: Create `.hpp` in `include/agv_slam/`, `.cpp` in `src/`, add `add_executable()` + `ament_target_dependencies()` + `install(TARGETS)` in `CMakeLists.txt`, add to launch file with appropriate `TimerAction` delay. See [src/README.md](src/README.md) for template.

**Modifying the pipeline**: Edit `launch/agv_slam.launch.py`. Respect the dependency chain: ZED → Depth Filter → cuVSLAM → RTAB-Map. See [launch/README.md](launch/README.md) for details.

**Tuning SLAM for a new environment**: Edit `config/rtabmap.yaml`. Key levers: `RGBD/LinearUpdate` (node spacing), `Rtabmap/LoopThr` (closure sensitivity), `Vis/MinInliers` (feature match strictness), `RGBD/LocalRadius` (proximity search range).

**Diagnosing issues**: Run `bash scripts/validate_slam.sh 15` while pipeline is active. See [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) for the 9 most common failure modes.

**Deploying to a new Jetson**: Follow [docs/JETSON_DEPLOY.md](docs/JETSON_DEPLOY.md) from the top. Critical steps: JetPack + ZED SDK + Isaac ROS install, workspace build, `jetson_setup.sh` on each boot, camera mount TF calibration.
