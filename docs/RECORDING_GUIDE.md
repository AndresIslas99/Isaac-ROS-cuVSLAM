# Greenhouse Recording Guide

Best practices for capturing high-quality data for 3DGS reconstruction.

## Before Recording

1. Run the setup script (once per boot):
   ```bash
   sudo bash scripts/jetson_setup.sh
   ```

2. Check storage:
   ```bash
   python3 scripts/check_storage.py
   ```
   Ensure at least 50 GB free for a 15-minute session.

3. Launch the pipeline:
   ```bash
   ros2 launch agv_slam agv_slam.launch.py
   ```

4. Verify all sensors are online:
   ```bash
   bash scripts/validate_slam.sh 10
   ```

## Recording a Session

### Start recording:
```bash
ros2 service call /session/start_recording agv_greenhouse_msgs/srv/StartRecording "{session_name: 'greenhouse_row_A'}"
```

### Stop recording:
```bash
ros2 service call /session/stop_recording agv_greenhouse_msgs/srv/StopRecording "{}"
```

### Monitor during recording:
- Check `/session/info` for frame count and disk usage
- Watch the coverage grid in RViz or Foxglove
- The coverage monitor shows which areas still need scanning

## Optimal Capture Parameters

| Parameter | Value | Why |
|-----------|-------|-----|
| Robot speed | < 0.3 m/s | Prevents motion blur at 30fps |
| Overlap between passes | > 60% | Required for 3DGS quality |
| Session duration | 15-30 min | Balance coverage vs file size |
| Camera height | 0.3-1.5m | Varies with elevator position |

## Scanning Pattern

For best coverage, use a **serpentine pattern** through greenhouse rows:

```
Row 1: =========>
                 |
Row 2: <=========
|
Row 3: =========>
                 |
Row 4: <=========
```

### Key tips:
- **Move slowly at row ends** — turns need extra overlap
- **Pause briefly** at interesting structures (plants, equipment)
- **Cover each area from 2+ angles** when possible (forward + return pass)
- **Avoid sudden movements** — cuVSLAM tracks better with smooth motion

## Lighting Conditions

| Condition | Impact | Mitigation |
|-----------|--------|------------|
| Direct sunlight through glass | Overexposure, depth artifacts | Use auto-exposure, avoid midday |
| Shadows from structures | Depth holes in shadows | Multiple passes from different angles |
| Artificial grow lights | Color cast on images | Auto white balance handles this |
| Very low light | Noisy depth, tracking loss | Minimum 50 lux recommended |

## After Recording

1. **Validate the session**:
   ```bash
   bash scripts/export_session.sh /mnt/ssd/sessions/your_session
   ```

2. **Transfer to RTX laptop** (follow the instructions printed by export script)

3. **Review coverage** — replay the bag through nvblox:
   ```bash
   ros2 launch agv_slam playback.launch.py bag_path:=/mnt/ssd/sessions/your_session/bag
   ```

## Data Sizes (approximate at HD1080 30fps)

| Component | Rate | 15 min session |
|-----------|------|----------------|
| ROS2 bag (all topics) | ~2.5 GB/min | ~37 GB |
| SVO2 (H265) | ~0.8 GB/min | ~12 GB |
| TUM trajectory | negligible | < 1 MB |
| Total | ~3.3 GB/min | ~50 GB |

## Troubleshooting

- **"LOW DISK SPACE" warning**: Stop recording, export/archive old sessions
- **Coverage grid not updating**: Check cuVSLAM odometry is running (`/visual_slam/tracking/odometry`)
- **Recording won't start**: Check if a previous recording is still active
- **SVO2 not saving**: Verify ZED node services: `ros2 service list | grep svo`
