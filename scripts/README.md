# Scripts

## jetson_setup.sh — Performance Tuning

**Run once after each boot** (requires `sudo`). Configures the Jetson Orin AGX for maximum SLAM performance.

```bash
sudo bash scripts/jetson_setup.sh
```

### Steps performed:

| # | Action | Command | Reversible? |
|---|--------|---------|-------------|
| 1 | MAXN power mode | `nvpmodel -m 0` | Yes: `nvpmodel -m 1` for 30W mode |
| 2 | Lock clocks at max | `jetson_clocks` | Yes: reboot resets |
| 3 | Verify GPU | `nvidia-smi` / `tegrastats` | Read-only check |
| 4 | Network buffers 2GB | `sysctl net.core.rmem_max=2147483647` | Yes: reboot resets |
| 5 | Disable ZRAM swap | `systemctl stop nvzramconfig` | Yes: `systemctl start nvzramconfig` |
| 6 | RT scheduling limits | Write to `/etc/security/limits.d/ros-realtime.conf` | Persistent (delete file to undo) |
| 7 | Set DDS environment | `export RMW_IMPLEMENTATION=...` | Session only |

Also creates `/home/orza/slam_maps/` directory for map storage.

**Note**: Step 6 requires logout/login to take effect. Steps 1-2 and 4-5 reset on reboot — run this script each boot or create a systemd service.

## validate_slam.sh — Pipeline Health Check

**Run while the pipeline is active** to verify all systems are healthy.

```bash
bash scripts/validate_slam.sh [duration_seconds]
# Default duration: 15 seconds
```

### Checks performed:

| # | Check | Expected result |
|---|-------|-----------------|
| 1 | Node list | All 5 nodes running: zed_node, depth_filter, visual_slam, rtabmap, slam_monitor |
| 2 | Topic rates | RGB ~30Hz, Depth ~30Hz, IMU ~400Hz, cuVSLAM Odom ~30Hz, RTAB-Map MapData >0Hz |
| 3 | TF chain | `map → base_link` and `odom → base_link` transforms resolve |
| 4 | RTAB-Map stats | map_size > 1, loop_closure_id tracking, processing_time reasonable |
| 5 | System resources | RAM, CPU usage, GPU usage (Tegra-specific) |

### Prerequisites
- ROS2 Humble sourced
- Workspace sourced (`~/agv_slam_ws/install/setup.bash`)
- Pipeline actively running

### Interpreting output

- `[check mark] node_name` = node detected in `ros2 node list`
- `[X] node_name (MISSING!)` = node not running — check launch logs
- `NO DATA` for topic rate = topic not publishing — check upstream node
- `map → base_link: BROKEN` = TF chain incomplete — see `docs/TROUBLESHOOTING.md`
