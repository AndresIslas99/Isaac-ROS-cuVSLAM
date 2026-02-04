# CLAUDE.md — AI Assistant Guide for agv_slam

> **For local Claude Code agents on Jetson**: This file is the root entry point. Read this first, then drill into the linked docs for detail on specific areas.

## Documentation Map

| Document | What it covers |
|----------|---------------|
| **CLAUDE.md** (this file) | Quick-reference overview, build/run, structure, conventions |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Full system design, data flow, TF tree, QoS, compute budget |
| [docs/CALIBRATION.md](docs/CALIBRATION.md) | Camera mount TF, IMU noise parameters, depth confidence |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Common failure modes with diagnostic steps and fixes |
| [docs/JETSON_DEPLOY.md](docs/JETSON_DEPLOY.md) | First-time Jetson setup, systemd service, networking |
| [docs/RECORDING_GUIDE.md](docs/RECORDING_GUIDE.md) | Session recording, SVO2, coverage monitoring |
| [config/README.md](config/README.md) | Every parameter in every YAML/XML file with values and rationale |
| [src/README.md](src/README.md) | Node class details, method-level docs, code style |
| [launch/README.md](launch/README.md) | Launch arguments, node timing, remappings |
| [scripts/README.md](scripts/README.md) | What each script does, prerequisites |

## Project Overview

Industrial SLAM pipeline for an AGV operating in greenhouse environments. Runs on **Jetson Orin AGX 64GB** with a **ZED 2i** stereo camera at HD1080@15fps. Combines NVIDIA Isaac ROS cuVSLAM (GPU visual-inertial odometry) with nvblox (GPU TSDF mesh reconstruction) for real-time localization and 3D mapping.

**v3.0.0**: cuVSLAM + nvblox. No RTAB-Map (removed).

## Build & Run

```bash
# Build
cd ~/ros2_ws
colcon build --symlink-install --packages-select agv_slam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Jetson performance tuning (once per boot)
sudo bash scripts/jetson_setup.sh

# Source
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch full pipeline
ros2 launch agv_slam agv_slam.launch.py enable_foxglove:=true

# Validate pipeline health
ros2 topic echo /slam/diagnostics
ros2 topic echo /slam/quality
```

## Repository Structure

```
agv_slam/
├── CLAUDE.md               # THIS FILE
├── README.md               # Human-facing project README
├── CMakeLists.txt          # ament_cmake build
├── package.xml             # ROS2 package manifest (format 3)
├── docs/
│   ├── ARCHITECTURE.md     # System design + data flow
│   ├── CALIBRATION.md      # Camera mount, IMU noise, depth confidence
│   ├── JETSON_DEPLOY.md    # Hardware setup, systemd, networking
│   ├── RECORDING_GUIDE.md  # Session recording, coverage monitoring
│   └── TROUBLESHOOTING.md  # Common issues and fixes
├── config/
│   ├── README.md           # Parameter docs
│   ├── cuvslam.yaml        # cuVSLAM GPU VIO config
│   ├── nvblox.yaml         # nvblox TSDF mesh config
│   ├── depth_filter.yaml   # Depth preprocessing pipeline
│   ├── zed2i.yaml          # ZED 2i camera config
│   └── cyclonedds.xml      # DDS transport config
├── include/agv_slam/
│   ├── depth_filter_node.hpp
│   ├── slam_monitor_node.hpp
│   ├── pipeline_watchdog_node.hpp
│   └── rate_tracker.hpp
├── src/
│   ├── README.md
│   ├── depth_filter_node.cpp    # 4-stage depth preprocessing
│   ├── slam_monitor_node.cpp    # 7-group diagnostics + tegrastats
│   └── pipeline_watchdog_node.cpp
├── launch/
│   ├── README.md
│   └── agv_slam.launch.py      # Main pipeline launch
├── scripts/
│   ├── README.md
│   ├── jetson_setup.sh          # Orin MAXN tuning
│   ├── coverage_monitor.py      # FOV-based coverage grid
│   └── session_recorder.py      # Bag + SVO2 recording
├── foxglove/
│   └── (dashboard layouts)
└── test/
    └── (unit tests)
```

## Architecture Summary

```
ZED 2i (HD1080@15fps, 400Hz IMU)
  ├── stereo gray + IMU → cuVSLAM (GPU VIO, ~1.8ms/frame)
  │                          └── TF: odom → base_link
  │                          └── /visual_slam/tracking/odometry
  ├── depth + RGB → depth_filter_node (single-pass, ~40ms)
  │                    └── /filtered/depth, /filtered/rgb
  ├── depth + RGB → nvblox (GPU TSDF mesh)
  │                    └── /nvblox_node/mesh_marker
  └── IMU @400Hz → cuVSLAM (inertial fusion)

Monitoring:
  slam_monitor_node    → /slam/diagnostics (7 groups), /slam/quality (JSON)
  pipeline_watchdog    → /watchdog/heartbeat, crash recovery
  coverage_monitor.py  → /coverage/grid, /coverage/status (JSON)
  session_recorder.py  → /session/info (JSON), recording services
  foxglove_bridge      → ws://192.168.55.1:8765
```

**TF tree**: `odom → base_link → zed_camera_link → optical frames`

**Launch timing**: Static TF + container (0s) → depth_filter (3s) → cuVSLAM (5s) → nvblox (8s) → watchdog+monitor (10s) → Python scripts (12s)

**Composable container**: `component_container_mt` with IPC **disabled** (`enable_ipc=false`, `use_intra_process_comms=False`). IPC is disabled because ZED wrapper creates transient_local QoS publishers that conflict with IPC's volatile-only requirement.

## Custom Nodes

### depth_filter_node (C++)
- 4-stage pipeline: range filter → temporal averaging → bilateral smoothing → hole filling
- Optimized single-pass for stage 1: range filter + NaN removal + quality metrics in one loop
- Publishes `/depth_filter/latency` (Float32, per-frame ms) and `/depth_filter/quality` (String JSON)
- ~40ms per frame at HD1080
- Executor: `MultiThreadedExecutor` (2 threads)

### slam_monitor_node (C++)
- 7 diagnostic groups: Sensors, TF, Jetson, Latency, DataQuality, DiskIO, Pipeline
- Tegrastats background pipe thread for GPU/CPU/thermal/RAM/swap/power monitoring
- Disk I/O via statvfs() + /proc/diskstats
- ~50-field JSON published on `/slam/quality` at 1 Hz
- Rate tracking with EMA + frame drop detection via `rate_tracker.hpp`
- Thresholds calibrated for HD1080@15fps baseline

### pipeline_watchdog_node (C++)
- Monitors node liveness, crash detection + recovery
- Publishes `/watchdog/heartbeat` at 1 Hz
- Logs to `/mnt/ssd/slam_logs/`

### coverage_monitor.py (Python)
- FOV projection onto 500x500 grid (0.1m resolution, 50m x 50m)
- Multi-angle coverage: cell upgraded from "seen" to "well covered" at 30-degree yaw difference
- Uses `std_msgs/String` JSON (not custom messages)

### session_recorder.py (Python)
- Bag recording, SVO2 recording, TUM trajectory logging
- Uses `std_srvs/Trigger` for start/stop services
- Uses `std_msgs/String` JSON for session info

## Key Thresholds (HD1080@15fps)

| Metric | Expected | WARN | ERROR |
|--------|----------|------|-------|
| Camera rates | 15 Hz | <13 Hz | <10 Hz |
| IMU rate | 400+ Hz | <360 Hz | <200 Hz |
| Depth filter latency | 40 ms | >30 ms | >60 ms |
| Tracking confidence | 1.00 | <0.5 | - |
| Depth valid ratio | 67% | <50% | <30% |
| GPU utilization | 15-27% | >80% | >90% |
| Tj temperature | 60C | >80C | >90C |
| Frame drops | <1% | >3% | >10% |

## Code Conventions

- **Language**: C++17 with ROS2 Humble, Python 3 for scripts, launch files in Python
- **Namespace**: `namespace agv_slam`
- **Classes**: PascalCase (`DepthFilterNode`)
- **Methods/variables**: snake_case (`apply_temporal_filter()`, `min_depth_`)
- **Member variables**: trailing underscore
- **Headers**: `.hpp`, implementations: `.cpp`
- **Include guards**: `#pragma once`
- **Indentation**: 2 spaces
- **Section markers**: `// ── Section Name ──`
- **Compiler flags**: `-Wall -Wextra -Wpedantic -O3`
- **QoS**: `SensorDataQoS` (best effort) for sensor topics, reliable for diagnostics
- **Message types**: Standard ROS types only (`std_msgs`, `std_srvs`, `sensor_msgs`). No custom message packages.

## Target Platform

- **Hardware**: NVIDIA Jetson Orin AGX 64GB + ZED 2i (USB 3.0)
- **Software**: JetPack 6.2 (L4T R36.4.7), ROS2 Humble, ZED SDK 4.x, Isaac ROS 3.2
- **DDS**: CycloneDDS with shared memory, localhost only, 10MB buffers
- **Workspace**: `~/ros2_ws/` (source in `src/Isaac-ROS-cuVSLAM/`)
- **Data storage**: `/mnt/ssd/sessions/` for recordings, `/mnt/ssd/slam_logs/` for watchdog logs
