# Jetson Orin AGX Deployment Guide

## Hardware Requirements

| Component | Specification | Notes |
|-----------|--------------|-------|
| SBC | NVIDIA Jetson Orin AGX 64GB | 64GB required for unlimited RTAB-Map memory |
| Camera | ZED 2i | Connected via USB 3.0 (USB 2.0 insufficient) |
| Storage | 128GB+ NVMe SSD | RTAB-Map databases grow with map size |
| Cooling | Active heatsink + fan | Continuous GPU+CPU workload at MAXN |

## Software Prerequisites

| Software | Version | Install method |
|----------|---------|---------------|
| JetPack | 6.2+ (L4T 36.x) | NVIDIA SDK Manager or SD card image |
| CUDA | 12.x (bundled with JetPack) | Part of JetPack |
| ZED SDK | 5.1+ | [stereolabs.com/developers](https://www.stereolabs.com/developers/) |
| ROS2 | Humble Hawksbill | `apt install ros-humble-desktop` |
| Isaac ROS | 3.2+ | NVIDIA Isaac ROS repository |
| CycloneDDS | Latest | `apt install ros-humble-rmw-cyclonedds-cpp` |

## First-Time Setup

### 1. Install ROS2 Humble
```bash
sudo apt update && sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install ZED SDK
```bash
# Download from stereolabs.com for JetPack 6.x/L4T 36.x
chmod +x ZED_SDK_*.run
./ZED_SDK_*.run
```

### 3. Install Isaac ROS cuVSLAM
Follow the official NVIDIA guide for your JetPack version:
- Set up Isaac ROS workspace per NVIDIA instructions
- Install `isaac_ros_visual_slam` package

### 4. Clone and Build Workspace
```bash
mkdir -p ~/agv_slam_ws/src && cd ~/agv_slam_ws/src

# Clone dependencies
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
git clone https://github.com/introlab/rtabmap_ros.git -b humble-devel
# Isaac ROS packages should already be in the workspace from step 3

# Clone this package
git clone <this-repo-url> agv_slam

# Install ROS dependencies
cd ~/agv_slam_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source ~/agv_slam_ws/install/setup.bash" >> ~/.bashrc
```

### 5. Configure Jetson Performance
```bash
# Run once after boot (or set up as systemd service)
sudo bash ~/agv_slam_ws/src/agv_slam/scripts/jetson_setup.sh
```

This script performs:
- Sets MAXN power mode (`nvpmodel -m 0`)
- Locks CPU/GPU clocks at maximum (`jetson_clocks`)
- Configures network buffers for large ROS2 messages (10MB)
- Disables ZRAM swap (unnecessary with 64GB)
- Sets RT scheduling limits for ROS2
- Creates map storage directory `/home/orza/slam_maps/`

### 6. Adjust Camera Mount

Edit the static transform in `launch/agv_slam.launch.py` to match your physical camera mounting:
```python
# Line 251-265: Adjust these values
'--x', '0.1',        # Camera forward offset from base_link center (meters)
'--y', '0.0',        # Camera lateral offset (0 = centered)
'--z', '0.3',        # Camera height above base_link origin (meters)
'--pitch', '0.087',  # Downward tilt in radians (~5 degrees)
```

Also update `urdf/zed2i_mount.urdf.xacro` to match (used for RViz visualization):
```xml
<origin xyz="0.1 0.0 0.3" rpy="0.0 0.087 0.0"/>
```

## Running the Pipeline

### Mapping Mode (build new map)
```bash
# Terminal 1: Launch pipeline
ros2 launch agv_slam agv_slam.launch.py

# Terminal 2 (optional): Monitoring
ros2 launch agv_slam monitor.launch.py

# Terminal 3 (optional): Validate health
bash ~/agv_slam_ws/src/agv_slam/scripts/validate_slam.sh
```

### Localization Mode (use existing map)
```bash
ros2 launch agv_slam agv_slam.launch.py localization:=true
```
Requires a valid map database at `/home/orza/slam_maps/greenhouse.db` (or custom `database_path`).

### With RViz
```bash
ros2 launch agv_slam agv_slam.launch.py enable_rviz:=true
```

## Systemd Service (Auto-Start on Boot)

Create `/etc/systemd/system/agv-slam.service`:
```ini
[Unit]
Description=AGV SLAM Pipeline
After=network.target
Wants=network.target

[Service]
Type=simple
User=orza
Environment="HOME=/home/orza"
ExecStartPre=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/orza/agv_slam_ws/install/setup.bash'
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/orza/agv_slam_ws/install/setup.bash && ros2 launch agv_slam agv_slam.launch.py'
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable agv-slam
sudo systemctl start agv-slam
```

## Jetson Performance Script Breakdown

`scripts/jetson_setup.sh` performs these 7 steps:

| Step | Action | Command | Why |
|------|--------|---------|-----|
| 1 | MAXN power mode | `nvpmodel -m 0` | All 12 cores active, max power budget |
| 2 | Lock clocks | `jetson_clocks` | Prevent dynamic freq scaling during SLAM |
| 3 | GPU check | `nvidia-smi` or `tegrastats` | Verify CUDA available |
| 4 | Network buffers | `sysctl net.core.rmem_max=2GB` | CycloneDDS needs large buffers for images |
| 5 | Disable ZRAM | `systemctl stop nvzramconfig` | 64GB RAM = no swap needed, avoid latency |
| 6 | RT scheduling | `/etc/security/limits.d/ros-realtime.conf` | ROS2 callbacks need RT priority |
| 7 | Environment | `export RMW_IMPLEMENTATION=...` | Set CycloneDDS + config path |

## File System Layout on Jetson

```
/home/orza/
├── agv_slam_ws/
│   ├── src/
│   │   ├── agv_slam/                 # This package
│   │   ├── zed-ros2-wrapper/         # ZED ROS2 driver
│   │   ├── rtabmap_ros/              # RTAB-Map ROS bindings
│   │   └── isaac_ros_visual_slam/    # NVIDIA cuVSLAM
│   ├── build/                        # Build artifacts
│   ├── install/                      # Installed packages
│   └── log/                          # Build logs
└── slam_maps/
    └── greenhouse.db                 # RTAB-Map database (created at runtime)
```

## Monitoring in Production

```bash
# Live diagnostics topic
ros2 topic echo /slam/diagnostics

# Per-sensor rates
ros2 topic hz /visual_slam/tracking/odometry

# System resources
tegrastats  # Jetson-specific: CPU, GPU, RAM, temps

# Thermal monitoring (stay under 85C)
cat /sys/devices/virtual/thermal/thermal_zone*/temp

# Full pipeline validation
bash scripts/validate_slam.sh 30
```

## Map Database Management

```bash
# List saved maps
ls -lh /home/orza/slam_maps/

# Backup a map
cp /home/orza/slam_maps/greenhouse.db /home/orza/slam_maps/greenhouse_backup_$(date +%Y%m%d).db

# Use specific map for localization
ros2 launch agv_slam agv_slam.launch.py localization:=true \
    database_path:=/home/orza/slam_maps/greenhouse_v2.db

# Export map for inspection (while RTAB-Map is running)
ros2 service call /rtabmap/publish_map std_srvs/srv/Empty
```
