# AGV Greenhouse SLAM — Industrial Grade Pipeline

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        JETSON ORIN AGX 64GB                            │
│                                                                        │
│  ┌──────────────┐     ┌───────────────────┐     ┌──────────────────┐  │
│  │  ZED 2i Node │     │  Isaac ROS cuVSLAM│     │    RTAB-Map      │  │
│  │  (zed_ros2)  │     │  (GPU-accelerated)│     │  (Mapper Only)   │  │
│  │              │     │                   │     │                  │  │
│  │ • RGB 30fps  ├────►│ • left_rect_gray  │     │ • visual_odom=   │  │
│  │ • Depth 30fps│     │ • right_rect_gray │     │   FALSE          │  │
│  │ • IMU 400Hz  │     │ • IMU fusion      │     │ • Loop closure   │  │
│  │ • NEURAL mode│     │ • 1.8ms/frame     │     │ • Graph optim    │  │
│  │ • HD720      │     │                   │     │ • Dense map      │  │
│  └──┬───┬───┬───┘     └────────┬──────────┘     └──┬───────────────┘  │
│     │   │   │                  │                    │                  │
│     │   │   │         /visual_slam/odom             │                  │
│     │   │   │                  │                    │                  │
│     │   │   │                  ▼                    │                  │
│     │   │   │         ┌──────────────────┐          │                  │
│     │   │   └────────►│  Depth Filter    │──────────┘                  │
│     │   │             │  (C++ node)      │                             │
│     │   │             │  • Confidence     │                             │
│     │   │             │  • PassThrough    │                             │
│     │   │             │  • Temporal avg   │                             │
│     │   │             └──────────────────┘                             │
│     │   │                                                              │
│     │   └──► /zed/rgb/image_rect_color ──────► RTAB-Map               │
│     │                                                                  │
│     └──────► /zed/imu/data ──► cuVSLAM + RTAB-Map (gravity)          │
│                                                                        │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                     CycloneDDS (zero-copy)                       │  │
│  └──────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
```

## Why This Architecture

| Component | Role | Latency | Why |
|-----------|------|---------|-----|
| ZED 2i Node | Sensor driver only | N/A | NEURAL depth + IMU at 30fps HD720 |
| cuVSLAM | Visual-Inertial Odometry | **1.8ms** | GPU-native, 500x faster than RTAB-Map VO |
| Depth Filter | Point cloud cleanup | <1ms | Remove NaN, confidence filter, temporal smooth |
| RTAB-Map | Mapping + Loop Closure | ~50ms | No VO burden, just graph optimization |

**Key insight**: RTAB-Map at 435ms/frame was computing visual odometry, feature extraction,
loop closure, AND map building simultaneously. By offloading odometry to cuVSLAM (1.8ms GPU),
RTAB-Map only does mapping/loop closure at ~50ms — a 9x speedup.

## Hardware Requirements

- Jetson Orin AGX 64GB Developer Kit
- ZED 2i Stereo Camera (USB 3.0)
- JetPack 6.2+ (L4T 36.x)
- ZED SDK 5.1+
- Isaac ROS 3.2+

## Build Instructions

```bash
# 1. Setup workspace
mkdir -p ~/agv_slam_ws/src
cd ~/agv_slam_ws/src

# 2. Clone dependencies
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
git clone https://github.com/introlab/rtabmap_ros.git -b humble-devel
# Isaac ROS — follow NVIDIA's official install:
# https://nvidia-isaac-ros.github.io/getting_started/index.html

# 3. Copy this package
cp -r agv_slam ~/agv_slam_ws/src/

# 4. Install dependencies
cd ~/agv_slam_ws
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Launch Order

```bash
# Terminal 1: Setup Jetson performance + DDS
./scripts/jetson_setup.sh

# Terminal 2: Launch everything
ros2 launch agv_slam agv_slam.launch.py

# Terminal 3: (Optional) Monitor
ros2 launch agv_slam monitor.launch.py
```

## File Structure

```
agv_slam/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── zed2i.yaml              # ZED SDK parameters
│   ├── cuvslam.yaml            # cuVSLAM parameters
│   ├── rtabmap.yaml            # RTAB-Map mapper parameters
│   ├── depth_filter.yaml       # Depth filter parameters
│   └── cyclonedds.xml          # DDS configuration
├── launch/
│   ├── agv_slam.launch.py      # Main launch file (everything)
│   └── monitor.launch.py       # Diagnostics + RViz
├── src/
│   ├── depth_filter_node.cpp   # C++ depth preprocessing
│   └── slam_monitor_node.cpp   # C++ diagnostics publisher
├── include/agv_slam/
│   ├── depth_filter_node.hpp
│   └── slam_monitor_node.hpp
├── scripts/
│   ├── jetson_setup.sh         # Jetson performance tuning
│   └── validate_slam.sh        # Accuracy validation script
├── rviz/
│   └── slam.rviz               # Pre-configured RViz layout
└── urdf/
    └── zed2i_mount.urdf.xacro  # Camera mount TF
```
