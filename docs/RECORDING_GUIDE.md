# Recording Guide

## Session Recorder

The `session_recorder.py` node provides bag recording, SVO2 recording, and TUM trajectory logging.

### Starting a Recording

```bash
ros2 service call /session/start_recording std_srvs/srv/Trigger
```

This creates a session directory at `/mnt/ssd/sessions/session_YYYYMMDD_HHMMSS/` containing:
- `bag/` — ROS2 bag with configured topics
- `trajectory.tum` — TUM-format trajectory (timestamp tx ty tz qx qy qz qw)
- `*.svo2` — ZED SVO2 raw recording (H265, if ZED SVO service available)
- `manifest.yaml` — Session metadata

### Stopping a Recording

```bash
ros2 service call /session/stop_recording std_srvs/srv/Trigger
```

### Monitoring Recording Status

```bash
ros2 topic echo /session/info
```

Returns JSON:
```json
{
  "session_id": "session_20260203_231200",
  "session_path": "/mnt/ssd/sessions/session_20260203_231200",
  "recording_active": true,
  "frame_count": 450,
  "duration_seconds": 30.0,
  "disk_usage_gb": 1.2,
  "disk_free_gb": 834.5,
  "status_message": "Recording"
}
```

### Storage

- Output directory: `/mnt/ssd/sessions/` (configured via `output_dir` parameter)
- Disk space warning at 10 GB free (configurable via `disk_warning_gb`)
- The slam_monitor also tracks disk free space in the SLAM/DiskIO diagnostic group

## Coverage Monitoring

The `coverage_monitor.py` node tracks which areas have been observed by the camera using FOV projection onto a 2D grid.

### Coverage Grid

- Published on `/coverage/grid` (OccupancyGrid) at 0.5 Hz
- 500x500 grid at 0.1m resolution (50m x 50m area)
- Cell states: -1 (unknown), 50 (seen once), 100 (well covered)

### Coverage Status

Published on `/coverage/status` (String JSON) at 1 Hz:
```json
{
  "total_cells": 250000,
  "seen_cells": 15000,
  "well_covered_cells": 8000,
  "coverage_percent": 6.0,
  "well_covered_percent": 3.2,
  "mapped_area_m2": 150.0,
  "grid_resolution": 0.1
}
```

### Multi-Angle Coverage

A cell is upgraded from "seen once" to "well covered" when observed from a yaw angle differing by more than 30 degrees from the first observation. This ensures multiple viewing angles for greenhouse row inspection.

### Resetting Coverage

```bash
ros2 service call /coverage/reset std_srvs/srv/Trigger
```

## Foxglove Visualization

Connect Foxglove Studio to `ws://192.168.55.1:8765` (or the Jetson's IP on your network).

Key panels to configure:
- 3D View: TF tree, mesh markers, point clouds
- Image: `/filtered/rgb`, `/filtered/depth`
- Diagnostics: `/slam/diagnostics`
- Plot: `/depth_filter/latency`, rates from `/slam/quality`
- Raw Messages: `/slam/quality` for full JSON

The Foxglove bridge topic whitelist is configured in the launch file to include all relevant topics.
