# Scripts

## jetson_setup.sh — Performance Tuning

**Run once after each boot** (requires `sudo`). Configures the Jetson Orin AGX for maximum SLAM performance.

```bash
sudo bash scripts/jetson_setup.sh
```

### Steps performed:

| # | Action | Command | Reversible? |
|---|--------|---------|-------------|
| 1 | MAXN power mode | `nvpmodel -m 0` | Yes: `nvpmodel -m 1` |
| 2 | Lock clocks at max | `jetson_clocks` | Yes: reboot resets |
| 3 | Verify GPU | `tegrastats` check | Read-only |
| 4 | Network buffers | `sysctl net.core.rmem_max=67108864` | Reboot resets |
| 5 | Create directories | `/mnt/ssd/sessions/`, `/mnt/ssd/slam_logs/` | Persistent |
| 6 | Set DDS environment | `export RMW_IMPLEMENTATION=...` | Session only |

Steps 1-2 and 4 reset on reboot — run this script each boot or configure as systemd ExecStartPre.

## coverage_monitor.py — FOV Coverage Tracking

**Runs as part of the pipeline** (launched automatically at t=12s).

Tracks which areas have been observed by the camera using FOV projection onto a 2D occupancy grid.

- Publishes `/coverage/grid` (OccupancyGrid) at 0.5 Hz — 500x500 grid, 0.1m resolution (50m x 50m)
- Publishes `/coverage/status` (String JSON) at 1 Hz — coverage %, cells, area
- Multi-angle: cell upgraded to "well covered" when seen from 30+ degree yaw difference
- Reset service: `/coverage/reset` (std_srvs/Trigger)

Uses standard ROS types: `std_msgs/String` for JSON, `nav_msgs/OccupancyGrid` for grid. No custom messages.

## session_recorder.py — Session Recording

**Runs as part of the pipeline** (launched automatically at t=12s).

Records bag files, SVO2 raw camera data, and TUM-format trajectories.

- Start: `ros2 service call /session/start_recording std_srvs/srv/Trigger`
- Stop: `ros2 service call /session/stop_recording std_srvs/srv/Trigger`
- Status: `/session/info` (String JSON) at 1 Hz

Output to `/mnt/ssd/sessions/session_YYYYMMDD_HHMMSS/`:
- `bag/` — ROS2 bag with configured topics
- `trajectory.tum` — TUM format (timestamp tx ty tz qx qy qz qw)
- `*.svo2` — ZED SVO2 raw recording (if available)
- `manifest.yaml` — Session metadata

Uses standard ROS types: `std_srvs/Trigger` for services, `std_msgs/String` for JSON status. No custom messages.
