# Troubleshooting Guide -- Jetson SLAM Pipeline v3.0.0

## 1. slam_container Crashes (IPC-related)

**Symptom:** The composable container `slam_container` segfaults or exits unexpectedly shortly after startup, often with errors referencing shared pointer use-after-free or intra-process manager.

**Cause:** Intra-process communication (IPC) enabled on the composable container. cuVSLAM and nvblox have incompatible shared pointer lifetime expectations when sharing GPU buffers via IPC.

**Fix:**
- Ensure `enable_ipc=false` is set on the `ComposableNodeContainer`.
- Ensure every `ComposableNode` has `extra_arguments=[{'use_intra_process_comms': False}]`.
- Do NOT set `use_intra_process_comms: True` in any config file.

**Verification:**
```bash
ros2 param get /slam_container use_intra_process_comms
# Should return: Boolean value is: False
```

---

## 2. cuVSLAM Frame Delta Warnings

**Symptom:** Repeated warnings in the log:
```
[visual_slam_node] Frame delta exceeds threshold: XXms > 70ms
```

**Cause:** The ZED 2i at HD1080 delivers frames at approximately 15fps (66.7ms interval). Network jitter, USB contention, or Jetson thermal throttling can push inter-frame intervals beyond the configured `image_jitter_threshold_ms` of 70ms.

**Diagnosis:**
```bash
# Check actual frame rate
ros2 topic hz /zed/left/image_rect_color
# Should show ~15 Hz. If significantly lower, investigate thermal or USB issues.

# Check Jetson thermal state
cat /sys/devices/virtual/thermal/thermal_zone*/temp
```

**Fix:**
- If occasional (1-2 per minute), this is normal at 15fps. The 70ms threshold allows ~4ms of jitter.
- If frequent, check for thermal throttling: `sudo jetson_clocks` or improve cooling.
- If persistent, increase `image_jitter_threshold_ms` to 80-85ms (not recommended for production).
- Check that no other USB3 devices are competing for bandwidth.

---

## 3. tegrastats Not Available

**Symptom:** `slam_monitor_node` logs:
```
[slam_monitor_node] Failed to open tegrastats pipe
[slam_monitor_node] Jetson diagnostics group disabled
```

The Jetson monitoring group in `/slam/quality` shows null values for GPU temp, power, and clock fields.

**Cause:** The `tegrastats` binary requires root privileges or the user is not in the correct group.

**Fix:**
```bash
# Option 1: Run with sudo (not recommended for production)
sudo tegrastats --interval 1000

# Option 2: Add user to the appropriate group
sudo usermod -aG gpio $USER
# Log out and back in

# Option 3: Set capabilities on the binary
sudo setcap cap_sys_admin+ep $(which tegrastats)
```

**Verification:**
```bash
tegrastats --interval 1000
# Should output a line of stats every second
```

---

## 4. CycloneDDS Buffer / Message Loss Issues

**Symptom:** Topics drop messages, `ros2 topic hz` shows inconsistent rates, or subscribers report gaps in sequence numbers. May see DDS warnings about buffer overflow or "sample lost."

**Cause:** Default CycloneDDS buffer sizes are insufficient for HD1080 image topics (~6MB per frame, stereo + depth = ~18MB at 15fps).

**Fix:** Ensure the custom `cyclonedds.xml` is loaded and properly configured.

```bash
# Verify DDS config is loaded
echo $CYCLONEDDS_URI
# Should point to your cyclonedds.xml

# Check the config is active
ros2 doctor --report | grep -i cyclone
```

Key `cyclonedds.xml` parameters to verify:
- `MaxMessageSize`: should be at least 65500 (UDP max)
- `ReceiveBufferSize`: at least 33554432 (32MB)
- `NetworkInterfaceAddress`: set to the correct interface (often `lo` for local-only)
- `SharedMemory`: should be enabled for local communication

If messages are still lost, increase `ReceiveBufferSize` and check kernel UDP buffer limits:
```bash
sudo sysctl -w net.core.rmem_max=67108864
sudo sysctl -w net.core.rmem_default=33554432
```

---

## 5. depth_filter_node High Latency

**Symptom:** `/depth_filter/latency` reports values consistently above 60ms (WARN threshold) or above 100ms (ERROR threshold).

**Expected:** ~40ms per frame at HD1080 with default config (temporal=2, bilateral=off, hole_filling=off).

**Diagnosis:**
```bash
# Monitor latency in real-time
ros2 topic echo /depth_filter/latency

# Check quality JSON for per-stage breakdown
ros2 topic echo /depth_filter/quality
```

**Common causes and fixes:**

| Cause | Fix |
|-------|-----|
| Thermal throttling | Run `sudo jetson_clocks`, improve cooling |
| Bilateral filter enabled | Set `bilateral_filter.enabled: false` in depth_filter.yaml |
| Hole filling enabled | Set `hole_filling.enabled: false` in depth_filter.yaml |
| Temporal window too large | Reduce `temporal_filter.window_size` (default: 2) |
| GPU memory pressure from nvblox | Check `tegrastats` for GPU memory usage |
| Other CUDA workloads | Ensure no competing GPU processes |

**Latency threshold reference:**
- OK: < 30ms
- WARN: 30-60ms
- ERROR: 60-100ms
- CRITICAL: > 100ms

---

## 6. TF Stale / Transform Age Exceeded

**Symptom:** Downstream nodes (nvblox, coverage_monitor) report stale transforms. `slam_monitor_node` shows TF age exceeding the 150ms threshold in `/slam/quality`.

**Diagnosis:**
```bash
# Check TF publication rate
ros2 topic hz /tf

# Check latest transform age
ros2 run tf2_ros tf2_echo odom base_link
```

**Common causes:**

1. **cuVSLAM lost tracking:** The VIO pipeline lost visual features.
   - Check lighting conditions and camera exposure.
   - Verify images are being published: `ros2 topic hz /zed/left/image_rect_color`
   - Look for cuVSLAM tracking status messages.

2. **ZED camera disconnected or lagging:**
   - Check USB connection: `lsusb | grep -i stereolabs`
   - Restart the ZED node or the full pipeline.

3. **ZED node accidentally publishing TF:**
   - Verify `publish_tf: false` in `zed2i.yaml`. If both ZED and cuVSLAM publish TF, the tree becomes inconsistent.

**Fix:** If cuVSLAM loses tracking persistently, restart the pipeline via the watchdog or manually:
```bash
ros2 service call /watchdog/restart_pipeline std_srvs/srv/Trigger
```

---

## 7. nvblox No Mesh Output

**Symptom:** `/nvblox_node/mesh` and `/nvblox_node/map_slice` have zero subscribers or zero publication rate. The 3D reconstruction is empty.

**Diagnosis:**
```bash
ros2 topic hz /nvblox_node/mesh
ros2 topic info /nvblox_node/mesh
ros2 topic hz /depth_filter/filtered
```

**Common causes:**

1. **Filtered depth not being published:** Check that `depth_filter_node` is running and publishing to `/depth_filter/filtered`.

2. **TF not available:** nvblox requires valid transforms to project depth into the voxel grid. See "TF Stale" above.

3. **nvblox not receiving depth:** Topic remapping may be incorrect. Verify that nvblox subscribes to `/depth_filter/filtered`, not `/zed/depth/depth_registered`.

4. **Insufficient GPU memory:** nvblox requires significant GPU memory for the TSDF volume. Check `tegrastats` output.

5. **nvblox crashed silently inside composable container:** Check container logs:
   ```bash
   ros2 component list /slam_container
   ```

---

## 8. Python Scripts ImportError

**Symptom:**
```
ImportError: No module named 'agv_greenhouse_msgs'
```
or similar import errors when launching `coverage_monitor.py` or `session_recorder.py`.

**Cause:** The v3.0.0 scripts use only `std_msgs/String` and `std_srvs/Trigger`. If you see an `agv_greenhouse_msgs` import error, the script has not been updated to v3.0.0 or an old version is cached.

**Fix:**
```bash
# Verify the correct script is installed
grep -r "agv_greenhouse_msgs" install/
# Should return NO results

# Clean and rebuild
rm -rf build/ install/ log/
colcon build --packages-select jetson_deploy

# Source the workspace
source install/setup.bash
```

If the scripts import only `std_msgs` and `std_srvs`, verify these packages are available:
```bash
ros2 pkg list | grep std_msgs
ros2 pkg list | grep std_srvs
```

---

## 9. Launch Timing / Node Startup Ordering Issues

**Symptom:** Nodes start before their dependencies are ready. Examples:
- cuVSLAM starts before ZED camera is publishing
- nvblox starts before cuVSLAM TF is available
- Monitor/watchdog starts before pipeline is up

**Cause:** The launch file uses `TimerAction` delays to sequence startup:

| Delay | Node(s) |
|-------|---------|
| 0s | static_tf, slam_container (ZED) |
| 3s | depth_filter_node |
| 5s | cuVSLAM |
| 8s | nvblox |
| 10s | watchdog, slam_monitor |
| 12s | coverage_monitor.py, session_recorder.py |

**Fix:** If the Jetson is under heavy load (cold boot, other processes), these delays may be insufficient. Increase the TimerAction delays in the launch file. A safe multiplier is 1.5x-2x for cold boot scenarios.

---

## 10. IMU Drop Detection False Positives

**Symptom:** `slam_monitor_node` reports IMU drops even though the IMU is functioning normally.

**Cause:** The ZED 2i BMI085 IMU publishes data in bursts (multiple samples at once) rather than at a steady 400Hz. This burst behavior triggers naive drop detection.

**Fix:** IMU drop detection is disabled by default in the v3.0.0 thresholds for exactly this reason. If it has been re-enabled, disable it:
- Ensure the monitor config does not set IMU drop thresholds.
- The Sensors group should only monitor camera frame rates (15/13/10 fps thresholds).

---

## 11. Frame Drop Rate Alarms

**Symptom:** `/slam/quality` JSON reports high `drop_percent` values.

**Thresholds (calibrated for 15fps):**
- OK: 0% drops
- WARN: 3% drops
- ERROR: 10% drops

**Diagnosis:** Check which topic is dropping frames via the quality JSON. Common causes:
- USB bandwidth saturation
- Thermal throttling reducing camera grab rate
- DDS buffer overflow (see issue #4)

---

## 12. Foxglove Bridge Not Starting

**Symptom:** Cannot connect Foxglove Studio to the robot.

**Cause:** The Foxglove bridge is optional and controlled by the `enable_foxglove` launch argument.

**Fix:**
```bash
ros2 launch jetson_deploy slam.launch.py enable_foxglove:=true
```

Ensure the `foxglove_bridge` package is installed:
```bash
sudo apt install ros-humble-foxglove-bridge
```

---

## Quick Health Check

Run these commands to verify the full pipeline is healthy:

```bash
# 1. Check all expected nodes are running
ros2 node list | grep -E "zed|visual_slam|nvblox|depth_filter|monitor|watchdog|coverage|session"

# 2. Check critical topic rates
ros2 topic hz /zed/left/image_rect_color     # expect ~15Hz
ros2 topic hz /visual_slam/tracking/odometry  # expect ~15Hz
ros2 topic hz /depth_filter/filtered          # expect ~15Hz
ros2 topic hz /slam/quality                   # expect ~1Hz

# 3. Check TF is live
ros2 run tf2_ros tf2_echo odom base_link

# 4. Read diagnostic summary
ros2 topic echo /slam/quality --once | python3 -m json.tool

# 5. Check depth filter latency
ros2 topic echo /depth_filter/latency --once
# Should show data < 40ms typically
```
