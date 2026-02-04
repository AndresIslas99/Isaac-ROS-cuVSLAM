# Jetson Orin AGX Deployment Guide

## Hardware Requirements

| Component | Specification | Notes |
|-----------|--------------|-------|
| SBC | NVIDIA Jetson Orin AGX 64GB | R36.4.7 (JetPack 6.2) |
| Camera | ZED 2i | Connected via USB 3.0 |
| Storage | NVMe SSD mounted at `/mnt/ssd` | For session recordings and logs |
| Cooling | Active heatsink + fan | Continuous GPU workload at MAXN |

## Software Stack

| Software | Version | Purpose |
|----------|---------|---------|
| JetPack | 6.2 (L4T R36.4.7) | Base OS + CUDA |
| CUDA | 12.x (bundled) | GPU compute |
| ZED SDK | 4.x | Camera driver |
| ROS2 | Humble Hawksbill | Middleware |
| Isaac ROS | 3.2 | cuVSLAM, nvblox |
| CycloneDDS | (via apt) | DDS transport |

## Network Configuration

| Interface | IP | Purpose |
|-----------|-----|---------|
| Ethernet | 192.168.50.100 | SSH access, development |
| USB (Jetson) | 192.168.55.1 | Foxglove bridge |

SSH access: `ssh orza@192.168.50.100`

## First-Time Setup

### 1. Jetson Performance Tuning

```bash
sudo bash scripts/jetson_setup.sh
```

This script performs:
- Sets MAXN power mode (`nvpmodel -m 0`)
- Locks CPU/GPU clocks (`jetson_clocks`)
- Configures network socket buffers for DDS (10MB+)
- Creates storage directories (`/mnt/ssd/sessions/`, `/mnt/ssd/slam_logs/`)

Run once per boot, or configure as a systemd service.

### 2. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select agv_slam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 3. Source Environment

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Running the Pipeline

### Full Launch

```bash
ros2 launch agv_slam agv_slam.launch.py enable_foxglove:=true
```

This starts all nodes in sequence with timed delays:
- 0s: Static TF + composable container (ZED camera)
- 3s: Depth filter node
- 5s: cuVSLAM (GPU visual-inertial odometry)
- 8s: nvblox (GPU TSDF mesh reconstruction)
- 10s: Pipeline watchdog + SLAM monitor
- 12s: Coverage monitor + session recorder
- Foxglove bridge (if `enable_foxglove:=true`)

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_foxglove` | `false` | Enable Foxglove WebSocket bridge |
| `camera_x` | `0.1` | Camera forward offset from base_link (m) |
| `camera_y` | `0.0` | Camera lateral offset (m) |
| `camera_z` | `0.3` | Camera height above base_link (m) |
| `camera_pitch` | `0.087` | Camera downward tilt (rad, ~5 deg) |

### Monitoring

```bash
# Diagnostics (7 groups)
ros2 topic echo /slam/diagnostics

# Full JSON metrics (~50 fields)
ros2 topic echo /slam/quality

# Depth filter performance
ros2 topic echo /depth_filter/latency
ros2 topic echo /depth_filter/quality

# Coverage status
ros2 topic echo /coverage/status

# Recording status
ros2 topic echo /session/info

# System resources
tegrastats
```

### Recording a Session

```bash
# Start recording (bag + SVO2 + TUM trajectory)
ros2 service call /session/start_recording std_srvs/srv/Trigger

# Stop recording
ros2 service call /session/stop_recording std_srvs/srv/Trigger
```

Sessions saved to `/mnt/ssd/sessions/session_YYYYMMDD_HHMMSS/`.

### Foxglove Visualization

Connect Foxglove Studio to `ws://192.168.55.1:8765` (or `ws://192.168.50.100:8765` from the network).

## Systemd Service (Auto-Start)

Create `/etc/systemd/system/agv-slam.service`:

```ini
[Unit]
Description=AGV SLAM Pipeline
After=network.target

[Service]
Type=simple
User=orza
Environment="HOME=/home/orza"
ExecStartPre=/bin/bash -c 'sudo bash /home/orza/ros2_ws/src/Isaac-ROS-cuVSLAM/scripts/jetson_setup.sh'
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/orza/ros2_ws/install/setup.bash && ros2 launch agv_slam agv_slam.launch.py enable_foxglove:=true'
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

## File System Layout

```
/home/orza/ros2_ws/
├── src/Isaac-ROS-cuVSLAM/   # This package (agv_slam)
├── build/
├── install/
└── log/

/mnt/ssd/
├── sessions/                  # Recording output
│   └── session_YYYYMMDD_HHMMSS/
│       ├── bag/
│       ├── trajectory.tum
│       ├── *.svo2
│       └── manifest.yaml
└── slam_logs/                 # Watchdog crash logs
```

## Performance Baseline

With the pipeline running at HD1080@15fps:

| Resource | Typical | Notes |
|----------|---------|-------|
| GPU | 15-27% | cuVSLAM + nvblox |
| CPU | ~10% across 12 cores | Depth filter is main CPU consumer |
| RAM | 9/63 GB | Plenty of headroom |
| Tj temp | ~60C | Well within safe range |
| Disk write | Varies | Only during recording sessions |

## Troubleshooting

See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for common issues.

Quick checks:
```bash
# Verify all nodes are running
ros2 node list

# Check for errors in diagnostics
ros2 topic echo /slam/diagnostics --once

# Check TF tree
ros2 run tf2_ros tf2_echo odom base_link

# Check DDS
echo $RMW_IMPLEMENTATION  # should be rmw_cyclonedds_cpp
```
