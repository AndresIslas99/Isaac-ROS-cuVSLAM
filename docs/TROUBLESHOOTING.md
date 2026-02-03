# Troubleshooting Guide

## Quick Diagnostics

```bash
# Check all nodes are running
ros2 node list
# Expected: /zed/zed_node, /depth_filter, /visual_slam, /rtabmap, /slam_monitor

# Check topic rates
ros2 topic hz /filtered/rgb           # Expected: ~30 Hz
ros2 topic hz /filtered/depth         # Expected: ~30 Hz
ros2 topic hz /zed/zed_node/imu/data  # Expected: ~400 Hz
ros2 topic hz /visual_slam/tracking/odometry  # Expected: ~30 Hz

# Check TF chain
ros2 run tf2_ros tf2_echo map base_link

# Read diagnostics
ros2 topic echo /slam/diagnostics

# Full validation
bash scripts/validate_slam.sh 15
```

## Common Issues

### 1. "Only 1 map node" — RTAB-Map not building graph

**Symptom**: RTAB-Map running but `/rtabmap/info` shows `map_size: 1` and no loop closures.

**Root cause**: Default `RGBD/LinearUpdate: 0.1m` is too large for slow AGV movement — robot doesn't travel far enough between RTAB-Map detection cycles to trigger new node creation.

**Fix** (already applied in `config/rtabmap.yaml`):
```yaml
RGBD/LinearUpdate: '0.05'    # 5cm instead of 10cm
RGBD/AngularUpdate: '0.1'    # ~6deg instead of default
Rtabmap/DetectionRate: '5.0'  # 5Hz processing rate
```

**Verify**: After fix, `/rtabmap/info` should show map_size increasing as the robot moves.

### 2. cuVSLAM not publishing odometry

**Symptom**: `/visual_slam/tracking/odometry` has no data.

**Checks**:
1. ZED stereo topics publishing: `ros2 topic hz /zed/zed_node/left/image_rect_gray`
2. IMU publishing: `ros2 topic hz /zed/zed_node/imu/data`
3. cuVSLAM node running: `ros2 node list | grep visual_slam`
4. Check cuVSLAM logs for "Lost tracking" or initialization errors

**Common fixes**:
- Ensure camera has visible features (not pointing at blank wall during startup)
- Check USB 3.0 connection (USB 2.0 bandwidth insufficient for stereo)
- Verify `enable_imu_fusion: true` in `config/cuvslam.yaml`
- Increase `img_jitter_threshold_ms` if frames are arriving with jitter

### 3. TF chain broken (map → base_link fails)

**Symptom**: `tf2_echo map base_link` returns "Could not transform".

**Diagnosis** — check each link independently:
```bash
ros2 run tf2_ros tf2_echo odom base_link      # cuVSLAM → should work first
ros2 run tf2_ros tf2_echo map odom             # RTAB-Map → needs loop closure
ros2 run tf2_ros tf2_echo base_link zed2i_base_link  # Static TF → always works
```

**If `odom → base_link` broken**: cuVSLAM not publishing (see issue #2 above)
**If `map → odom` broken**: RTAB-Map hasn't initialized yet or has no odometry input. Check that cuVSLAM odom is flowing and `publish_tf: true` in `config/rtabmap.yaml`.

### 4. No loop closures detected in greenhouse

**Symptom**: RTAB-Map builds nodes but `loop_closure_id` stays 0.

**Checks**:
1. Verify map has enough nodes (need >10-20 for spatial proximity to work)
2. Check feature count: `ros2 topic echo /rtabmap/info` → look for words/features count
3. Ensure robot revisits previously-mapped areas

**Tuning** (in `config/rtabmap.yaml`):
```yaml
Rtabmap/LoopThr: '0.09'              # Lower = more permissive (try 0.07 if still failing)
RGBD/ProximityBySpace: 'true'        # Must be true for row-based environments
Vis/MinInliers: '15'                 # Lower if features are sparse (try 10)
RGBD/ProximityMaxGraphDepth: '50'    # Increase to search further back
```

### 5. Depth filter removing too many pixels

**Symptom**: `/filtered/depth` image is mostly NaN/black, depth_filter logs show >50% removal.

**Diagnosis**: Check raw depth first:
```bash
ros2 topic echo /zed/zed_node/depth/depth_registered --once | head -5
```

**Fixes** (in `config/depth_filter.yaml`):
- Increase `max_depth` if environment extends beyond 10m
- Decrease `min_depth` below 0.3m only if using close-range ZED
- Disable `enable_bilateral: false` temporarily to isolate cause
- Adjust ZED confidence: raise `depth_confidence` in `config/zed2i.yaml` (50→70 = less strict)

### 6. High latency / frame drops

**Symptom**: Topics running below expected Hz, or `validate_slam.sh` shows low rates.

**Checks**:
```bash
# Check Jetson power mode
sudo nvpmodel -q   # Should be: MAXN (mode 0)
sudo jetson_clocks --show  # Clocks should be at max

# Check thermal throttling
cat /sys/devices/virtual/thermal/thermal_zone*/temp  # Should be <85000 (85C)

# Check RAM
free -h   # Should have significant free memory
```

**Fixes**:
- Run `scripts/jetson_setup.sh` (sets MAXN mode + locks clocks)
- Ensure adequate ventilation / heatsink on Jetson
- Reduce `grab_resolution` to VGA if thermal throttling persists
- Disable `enable_hole_filling` (most expensive depth filter stage)

### 7. DDS communication issues

**Symptom**: Nodes running but topics not visible across nodes, or high latency.

**Checks**:
```bash
echo $RMW_IMPLEMENTATION   # Must be: rmw_cyclonedds_cpp
echo $CYCLONEDDS_URI       # Must point to config/cyclonedds.xml
cat /tmp/cyclonedds.log    # Check for errors
```

**Fixes**:
- Ensure environment is set BEFORE launching nodes (launch file does this automatically)
- Verify shared memory is enabled: check `cyclonedds.xml` has `<Enable>true</Enable>` under `<SharedMemory>`
- Check `/dev/shm` has free space: `df -h /dev/shm`

### 8. RTAB-Map database errors

**Symptom**: RTAB-Map crashes or fails to start with database errors.

**Checks**:
```bash
ls -la /home/orza/slam_maps/greenhouse.db
du -h /home/orza/slam_maps/greenhouse.db   # Check size
```

**Fixes**:
- Ensure directory exists: `mkdir -p /home/orza/slam_maps`
- For mapping mode, DB is deleted on start (`--delete_db_on_start`). If disk full, clear old DBs
- For localization mode, the DB must exist and be valid. Re-run mapping first if needed
- Check disk space: `df -h /home/orza/slam_maps/`

### 9. ZED camera not detected

**Symptom**: `zed_node` fails to start or logs "No ZED camera detected".

**Checks**:
```bash
lsusb | grep -i "2b03"    # ZED vendor ID
ls /dev/video*             # Camera devices
```

**Fixes**:
- Re-seat USB 3.0 cable (must be USB 3.0, not 2.0)
- Check ZED SDK installation: `ZED_Explorer` should detect the camera
- Ensure no other process has the camera open
- Check `serial_number: 0` in `config/zed2i.yaml` (0 = auto-detect)

## Performance Baseline (Healthy Pipeline)

| Metric | Expected | Warning threshold |
|--------|----------|-------------------|
| RGB rate | ~30 Hz | < 10 Hz |
| Depth rate | ~30 Hz | < 10 Hz |
| IMU rate | ~400 Hz | < 100 Hz |
| cuVSLAM odom rate | ~30 Hz | < 10 Hz |
| cuVSLAM latency | ~1.8 ms | > 10 ms |
| RTAB-Map processing | ~50 ms | > 200 ms |
| Depth filter latency | < 1 ms | > 5 ms |
| RAM usage (total) | 8-15 GB | > 50 GB |
| GPU usage | ~20% | > 80% |
| CPU (total) | 15-25% | > 80% |
