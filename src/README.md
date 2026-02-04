# Source Code — Custom ROS2 Nodes

This package provides three C++ ROS2 nodes plus two Python scripts. All C++ code is in namespace `agv_slam`.

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select agv_slam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Compiler flags: `-Wall -Wextra -Wpedantic -O3`

## depth_filter_node

**Files**: `src/depth_filter_node.cpp`, `include/agv_slam/depth_filter_node.hpp`

Preprocesses ZED depth images before they reach nvblox. Runs a 4-stage pipeline to remove noise, fill holes, and smooth edges.

### Class: `DepthFilterNode` (inherits `rclcpp::Node`)

**Constructor**:
- Declares and reads 14 parameters from `config/depth_filter.yaml`
- Sets up `message_filters::ApproximateTime` synchronizer for RGB + Depth (buffer=10)
- Creates publishers: `/filtered/depth`, `/filtered/rgb`, `/filtered/camera_info`
- Creates monitoring publishers: `/depth_filter/latency` (Float32), `/depth_filter/quality` (String JSON)

**Callback** — `depth_callback()`:
- Converts depth to OpenCV `CV_32FC1` via `cv_bridge`
- **Stage 1 (optimized single-pass)**: Range filter + NaN removal + valid pixel count + mean depth — all in one loop iteration over every pixel
- Stage 2: `apply_temporal_filter()` — running average over N frames in `std::deque` (mutex-protected)
- Stage 3: `apply_bilateral_filter()` — `cv::bilateralFilter` with NaN→0 swap
- Stage 4: `apply_hole_filling()` — morphological closing + `cv::inpaint(TELEA)`
- Publishes filtered depth + passes through RGB
- Publishes latency (Float32 ms) and quality JSON (valid_ratio, mean_depth, latency_ms)
- Logs stats every 100 frames

**Performance**: ~40ms per frame at HD1080 (single-pass optimization reduced from 54ms)

**Executor**: `MultiThreadedExecutor` with 2 threads

### Dependencies
```
rclcpp, sensor_msgs, std_msgs, image_transport, cv_bridge, message_filters, OpenCV
```

## slam_monitor_node

**Files**: `src/slam_monitor_node.cpp`, `include/agv_slam/slam_monitor_node.hpp`

Comprehensive production monitoring with 7 diagnostic groups, tegrastats integration, and disk I/O tracking.

### Class: `SlamMonitorNode` (inherits `rclcpp::Node`)

**Subscriptions** (8 topics):
- `/filtered/rgb`, `/filtered/depth` — filtered camera rates
- `/zed/zed_node/left/image_rect_gray`, `/zed/zed_node/depth/depth_registered` — raw ZED rates
- `/zed/zed_node/imu/data` — IMU rate
- `/visual_slam/tracking/odometry` — cuVSLAM odom rate + distance tracking
- `/depth_filter/latency` — depth filter timing
- `/depth_filter/quality` — depth valid ratio, mean depth

**Rate tracking** — `rate_tracker.hpp`:
- `RateTracker` struct with EMA-based `get_hz()` and frame drop detection
- Drop detection: compares inter-frame dt against expected period, counts drops when dt > 1.5x expected
- Per-tracker `expected_hz` (set to 0.0 to disable drop detection, e.g., for bursty IMU)
- 8 rate trackers calibrated for HD1080@15fps baseline

**Tegrastats** — background thread:
- Opens pipe to `tegrastats --interval 2000`
- Parses: GPU%, per-core CPU%, RAM, SWAP, Tj temperature, power
- Stores in `TegraData` struct with mutex

**Disk I/O** — `DiskStats`:
- Reads `statvfs()` for free space
- Reads `/proc/diskstats` for read/write MB/s
- Estimates recording time remaining

**Data Quality** — `DataQuality` struct:
- cuVSLAM tracking confidence and vo_state
- Depth valid ratio and mean depth (from depth_filter/quality)
- cuVSLAM latency

### 7 Diagnostic Groups

Published on `/slam/diagnostics` (DiagnosticArray) at 1 Hz:

| Group | Content |
|-------|---------|
| SLAM/Sensors | 8 rate trackers + frame drop % |
| SLAM/TF | odom→base_link age, base_link→camera existence |
| SLAM/Jetson | GPU%, CPU%, thermal, RAM, SWAP, power, per-core |
| SLAM/Latency | depth_filter ms, cuVSLAM ms |
| SLAM/DataQuality | tracking confidence, vo_state, depth valid ratio |
| SLAM/DiskIO | free GB, write MB/s, estimated minutes remaining |
| SLAM/Pipeline | aggregated overall health status |

### JSON Output

Published on `/slam/quality` (String) at 1 Hz with ~50 fields covering all metrics.

### Threshold Configuration

Three-tier alerting: OK (green) → WARN (yellow) → ERROR (red)

| Metric | OK | WARN | ERROR |
|--------|-----|------|-------|
| Camera rates | >=15 Hz | <13 Hz | <10 Hz |
| IMU rate | >=400 Hz | <360 Hz | <200 Hz |
| Depth filter latency | <30 ms | >30 ms | >60 ms |
| Frame drops | <3% | >3% | >10% |
| TF age | <150 ms | - | >150 ms |
| GPU util | <80% | >80% | >90% |
| Tj temp | <80C | >80C | >90C |
| RAM | <48 GB | >48 GB | >56 GB |
| Disk free | >20 GB | <20 GB | <5 GB |

**Executor**: Default single-threaded (`rclcpp::spin`)

### Dependencies
```
rclcpp, sensor_msgs, std_msgs, diagnostic_msgs, nav_msgs, geometry_msgs,
visualization_msgs, tf2_ros, OpenCV
```

## pipeline_watchdog_node

**Files**: `src/pipeline_watchdog_node.cpp`, `include/agv_slam/pipeline_watchdog_node.hpp`

Monitors pipeline health, detects node crashes, and triggers recovery.

- Publishes `/watchdog/heartbeat` (Int32) at 1 Hz
- Logs events to `/mnt/ssd/slam_logs/`
- Crash detection with configurable recovery actions

## Code Style

- **Namespace**: `namespace agv_slam { ... }`
- **Classes**: PascalCase
- **Methods**: snake_case
- **Members**: snake_case with trailing underscore (`min_depth_`)
- **Include guard**: `#pragma once`
- **Include order**: own header → STL → ROS → OpenCV
- **Section comments**: `// ── Section Name ──`
- **Indentation**: 2 spaces
- **Error handling**: `RCLCPP_ERROR_THROTTLE` for recoverable, try-catch for time source mismatches

## Adding a New Node

1. Create `include/agv_slam/new_node.hpp` with class declaration
2. Create `src/new_node.cpp` with implementation + `main()`
3. Add to `CMakeLists.txt`:
   ```cmake
   add_executable(new_node src/new_node.cpp)
   target_include_directories(new_node PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>)
   ament_target_dependencies(new_node rclcpp ...)
   install(TARGETS new_node DESTINATION lib/${PROJECT_NAME})
   ```
4. Add to launch file with appropriate `TimerAction` delay
