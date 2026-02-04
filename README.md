# agv_slam — Greenhouse AGV SLAM Pipeline

Autonomous greenhouse inspection pipeline running on **Jetson Orin AGX 64GB**.

**Stack**: ZED 2i + cuVSLAM (GPU VIO) + nvblox (GPU TSDF mesh) + production monitoring

## Architecture

```
ZED 2i (HD1080@15fps)
  ├── stereo gray → cuVSLAM (GPU visual-inertial odometry)
  │                    └── TF: odom → base_link
  │                    └── /visual_slam/tracking/odometry
  ├── depth + RGB  → depth_filter_node (40ms/frame)
  │                    └── /filtered/depth, /filtered/rgb
  ├── depth + RGB  → nvblox (GPU TSDF mesh reconstruction)
  │                    └── /nvblox_node/mesh_marker
  └── IMU @400Hz   → cuVSLAM (inertial fusion)

Monitoring:
  slam_monitor_node    → /slam/diagnostics (7 groups), /slam/quality (JSON)
  pipeline_watchdog    → /watchdog/heartbeat, crash recovery
  coverage_monitor.py  → /coverage/grid, /coverage/status (JSON)
  session_recorder.py  → /session/info (JSON), recording services
  foxglove_bridge      → ws://192.168.55.1:8765
```

## Quick Start

```bash
# On Jetson (ssh orza@192.168.50.100)
sudo bash scripts/jetson_setup.sh           # Once per boot
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch agv_slam agv_slam.launch.py enable_foxglove:=true
```

Open Foxglove Studio at `ws://192.168.55.1:8765` for web visualization.

## Nodes

| Node | Type | Purpose |
|------|------|---------|
| `slam_container` | Composable (MT) | Hosts ZED + cuVSLAM + nvblox |
| `depth_filter_node` | C++ standalone | Depth preprocessing (range filter + temporal avg) |
| `slam_monitor_node` | C++ standalone | 7-group diagnostics, tegrastats, disk I/O |
| `pipeline_watchdog_node` | C++ standalone | Crash detection + recovery |
| `coverage_monitor.py` | Python | FOV-based coverage grid tracking |
| `session_recorder.py` | Python | Bag recording + SVO2 + TUM trajectory |

## Monitoring Topics

| Topic | Type | Rate | Content |
|-------|------|------|---------|
| `/slam/diagnostics` | `DiagnosticArray` | 1 Hz | 7 diagnostic groups (see below) |
| `/slam/quality` | `String` | 1 Hz | ~50-field JSON with all metrics |
| `/depth_filter/latency` | `Float32` | 15 Hz | Per-frame processing time (ms) |
| `/depth_filter/quality` | `String` | 15 Hz | JSON: valid_ratio, mean_depth, latency |
| `/watchdog/heartbeat` | `Int32` | 1 Hz | Pipeline health counter |
| `/coverage/grid` | `OccupancyGrid` | 0.5 Hz | 500x500 coverage grid |
| `/coverage/status` | `String` | 1 Hz | JSON: coverage %, cells, area |
| `/session/info` | `String` | 1 Hz | JSON: recording status, disk free |

### Diagnostic Groups

| Group | Level | Content |
|-------|-------|---------|
| SLAM/Sensors | OK/WARN/ERR | 8 rate trackers + frame drop % |
| SLAM/TF | OK/ERR | odom→base_link age, camera TF |
| SLAM/Jetson | OK/WARN/ERR | GPU/CPU/thermal/RAM/swap/power/12-core |
| SLAM/Latency | OK/WARN/ERR | depth_filter ms, cuVSLAM ms |
| SLAM/DataQuality | OK/WARN/ERR | Tracking confidence, depth valid ratio |
| SLAM/DiskIO | OK/WARN/ERR | Free GB, write MB/s, est. minutes |
| SLAM/Pipeline | OK/WARN/ERR | Aggregated overall health |

## Performance Baseline (HD1080@15fps)

| Metric | Typical | Threshold |
|--------|---------|-----------|
| RGB/Depth rate | 15 Hz | WARN <13, ERR <10 |
| IMU rate | 400+ Hz | WARN <360 |
| cuVSLAM Odom | 15 Hz | WARN <13, ERR <10 |
| Depth filter latency | 40 ms | WARN >30, ERR >60 |
| Tracking confidence | 1.00 | WARN <0.5 |
| Depth valid ratio | 67% | WARN <50%, ERR <30% |
| GPU utilization | 15-27% | WARN >80%, ERR >90% |
| Tj temperature | 60C | WARN >80C, ERR >90C |
| RAM usage | 9/63 GB | WARN >48GB, ERR >56GB |
| Disk free | 834 GB | WARN <20GB, ERR <5GB |
| Frame drops | <1% | WARN >3%, ERR >10% |

## Configuration

All config in `config/`:
- `zed2i.yaml` — Camera: HD1080, PERFORMANCE depth, 400Hz IMU
- `cuvslam.yaml` — GPU VIO: stereo + IMU fusion, ground constraint
- `nvblox.yaml` — TSDF mesh: voxel size, integrator params
- `depth_filter.yaml` — Stages: range, temporal(2), bilateral(off), hole_filling(off)
- `cyclonedds.xml` — DDS: localhost, shared memory, 10MB buffers

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select agv_slam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Documentation

- [Architecture](docs/ARCHITECTURE.md) — Pipeline design, data flow, composable nodes
- [Calibration](docs/CALIBRATION.md) — Camera mount TF, IMU noise parameters
- [Deployment](docs/JETSON_DEPLOY.md) — Jetson setup, systemd, networking
- [Recording](docs/RECORDING_GUIDE.md) — SVO2, bag recording, session management
- [Troubleshooting](docs/TROUBLESHOOTING.md) — Common issues and fixes

## License

MIT
