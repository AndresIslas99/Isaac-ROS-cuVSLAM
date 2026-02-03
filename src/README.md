# Source Code — Custom ROS2 Nodes

This package provides two C++ ROS2 nodes. All code lives in the `agv_slam` namespace.

## Build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Compiler flags: `-Wall -Wextra -Wpedantic -O3`

## depth_filter_node

**Files**: `src/depth_filter_node.cpp`, `include/agv_slam/depth_filter_node.hpp`

**Purpose**: Preprocesses ZED depth images before they reach RTAB-Map. Runs a 4-stage pipeline to remove noise, fill holes, and smooth edges.

### Class: `DepthFilterNode` (inherits `rclcpp::Node`)

**Constructor** (`depth_filter_node.cpp:8`):
- Declares and reads 14 parameters from `config/depth_filter.yaml`
- Sets up `message_filters::ApproximateTime` synchronizer for RGB + Depth (buffer=10)
- Creates publishers on `/filtered/depth`, `/filtered/rgb`, `/filtered/camera_info`
- Camera info is a direct passthrough from ZED

**Callback** — `depth_callback()` (`depth_filter_node.cpp:73`):
- Converts depth to OpenCV `CV_32FC1` via `cv_bridge`
- Runs 4 stages sequentially (each guarded by enable flag):
  1. `apply_confidence_filter()` — sets out-of-range and NaN pixels to NaN
  2. `apply_temporal_filter()` — running average over N frames in `std::deque` (mutex-protected)
  3. `apply_bilateral_filter()` — `cv::bilateralFilter` with NaN→0 swap
  4. `apply_hole_filling()` — morphological closing + `cv::inpaint(TELEA)`
- Publishes filtered depth + passes through RGB with matching timestamp
- Logs stats every 100 frames

**Processing methods**:

| Method | Line | Key logic |
|--------|------|-----------|
| `apply_confidence_filter()` | :138 | NaN mask via `depth != depth`, range mask via threshold, combined → set to NaN |
| `apply_temporal_filter()` | :159 | Deque of N frames, NaN-aware accumulation, divide by valid count per pixel |
| `apply_bilateral_filter()` | :203 | Temp-replace NaN with 0, bilateral, restore NaN positions |
| `apply_hole_filling()` | :218 | Morph close to find fillable pixels, normalize to 8-bit, Telea inpaint, denormalize |

**Executor**: `MultiThreadedExecutor` with 2 threads (`depth_filter_node.cpp:271`)

**Member variables** (all with trailing `_` underscore):
- `temporal_buffer_` — `std::deque<cv::Mat>`, protected by `buffer_mutex_`
- `frames_processed_`, `points_removed_` — running counters for diagnostics

### Dependencies
```
rclcpp, sensor_msgs, image_transport, cv_bridge, message_filters, OpenCV
```

## slam_monitor_node

**Files**: `src/slam_monitor_node.cpp`, `include/agv_slam/slam_monitor_node.hpp`

**Purpose**: Publishes 1Hz diagnostics with sensor rates, TF health, and odometry distance.

### Class: `SlamMonitorNode` (inherits `rclcpp::Node`)

**Constructor** (`slam_monitor_node.cpp:7`):
- Subscribes to 4 topics: `/filtered/rgb`, `/filtered/depth`, `/zed/.../imu/data`, `/visual_slam/.../odometry`
- Creates diagnostics publisher on `/slam/diagnostics` (queue=10)
- Creates 1-second wall timer for `publish_diagnostics()`
- Initializes TF2 buffer + listener for `map → base_link` lookup

**Rate tracking** — `RateTracker` struct (`slam_monitor_node.hpp:44`):
- Lightweight inline struct with `tick()` and `get_hz()` methods
- Uses exponential moving average: `hz = 0.9 * hz + 0.1 * (1.0 / dt)`
- Mutex-protected for thread safety
- Four instances: `rgb_rate_`, `depth_rate_`, `imu_rate_`, `odom_rate_`

**Callbacks** (`slam_monitor_node.cpp:45-70`):
- `rgb_callback()`, `depth_callback()`, `imu_callback()` — call `rate_.tick()` only
- `odom_callback()` — ticks rate + accumulates Euclidean distance from odometry positions

**Diagnostics** — `publish_diagnostics()` (`slam_monitor_node.cpp:72`):
- Publishes `DiagnosticArray` with two status entries:
  1. **SLAM/Sensors**: Hz for all 4 topics + total distance. Level = WARN if any below threshold (RGB/Depth/Odom < 10Hz, IMU < 100Hz)
  2. **SLAM/TF**: `map → base_link` transform lookup. Level = ERROR if broken, includes x/y position when OK
- Console summary log every 10 seconds

**Executor**: Default single-threaded (`rclcpp::spin`)

### Dependencies
```
rclcpp, sensor_msgs, std_msgs, diagnostic_msgs, nav_msgs, tf2_ros
```

## Code Style

- **Namespace**: `namespace agv_slam { ... }`
- **Classes**: PascalCase
- **Methods**: snake_case
- **Members**: snake_case with trailing underscore (`min_depth_`)
- **Include guard**: `#pragma once`
- **Include order**: own header → STL → ROS → OpenCV
- **Section comments**: `// ── Section Name ──`
- **Indentation**: 2 spaces
- **Error handling**: `RCLCPP_ERROR_THROTTLE` for recoverable errors, exception catch for cv_bridge

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
   ```
4. Add to `install(TARGETS ...)` block in `CMakeLists.txt`
5. Add to launch file with appropriate `TimerAction` delay
6. Add config YAML if needed + install in `CMakeLists.txt` `install(DIRECTORY ...)`
