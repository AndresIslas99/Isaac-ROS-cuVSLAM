# Configuration Files

All parameters are loaded at launch time from these YAML/XML files. Changes require a relaunch (no dynamic reconfiguration).

## File Overview

| File | Target node | Purpose |
|------|-------------|---------|
| `cuvslam.yaml` | `isaac_ros_visual_slam` | GPU visual-inertial odometry |
| `nvblox.yaml` | `nvblox_node` | GPU TSDF mesh reconstruction |
| `depth_filter.yaml` | `depth_filter_node` | Depth image preprocessing pipeline |
| `zed2i.yaml` | `zed_wrapper` | Camera driver + depth sensing |
| `cyclonedds.xml` | CycloneDDS middleware | DDS transport, buffers, shared memory |

## cuvslam.yaml — GPU Visual-Inertial Odometry

cuVSLAM runs stereo VIO on the Orin GPU at ~1.8ms/frame. Pure odometry mode — no internal SLAM/mapping.

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `enable_imu_fusion` | `true` | Fuses BMI085 IMU for robust tracking |
| `enable_ground_constraint_in_odometry` | `true` | Constrains to 2D plane (ground robot) |
| `enable_localization_n_mapping` | `false` | Pure VIO — nvblox handles mapping |
| `enable_landmarks` | `false` | No landmark loop closure |
| `enable_slam_visualization` | `false` | Save GPU cycles |
| `publish_odom_to_base_tf` | `true` | Publishes `odom → base_link` TF |
| `publish_map_to_odom_tf` | `false` | Not used (no map frame) |
| `image_jitter_threshold_ms` | `70.0` | Relaxed for HD1080@15fps (~67ms frame period) |
| `gyro_noise_density` | `0.000244` | BMI085 spec: 0.014 deg/s/sqrt(Hz) |
| `gyro_random_walk` | `0.000019393` | BMI085 typical |
| `accel_noise_density` | `0.001862` | BMI085 spec: 120 ug/sqrt(Hz) |
| `accel_random_walk` | `0.003` | BMI085 typical |
| `calibration_frequency` | `200.0` | IMU rate / 2 |

**Frame IDs**: `odom`, `base_link` — must match launch static TF config.

## nvblox.yaml — GPU TSDF Mesh Reconstruction

nvblox builds a real-time 3D mesh from depth data using TSDF (Truncated Signed Distance Function) on the GPU.

Key parameters include voxel size, integrator settings, and mesh update rate. Consult NVIDIA nvblox documentation for full parameter reference.

## depth_filter.yaml — Depth Preprocessing

Four-stage pipeline, each independently toggleable:

| Stage | Parameter | Value | Effect |
|-------|-----------|-------|--------|
| 1. Range filter | `min_depth` / `max_depth` | 0.3 / 10.0 m | Set out-of-range pixels to NaN |
| 2. Temporal avg | `enable_temporal` / `temporal_window` | true / 2 | Average 2 frames, reduces flicker |
| 3. Bilateral | `enable_bilateral` | false | Edge-preserving smoothing (disabled for perf) |
| 4. Hole filling | `enable_hole_filling` | false | Morphological close + inpaint (disabled) |

**Current production config**: Only stages 1 and 2 are active. Bilateral and hole filling are disabled to keep latency at ~40ms.

Topic remapping (configurable):
```yaml
input_depth_topic:  '/zed/zed_node/depth/depth_registered'
input_rgb_topic:    '/zed/zed_node/rgb/image_rect_color'
output_depth_topic: '/filtered/depth'
output_rgb_topic:   '/filtered/rgb'
```

## zed2i.yaml — Camera Driver

| Section | Parameter | Value | Notes |
|---------|-----------|-------|-------|
| General | `grab_resolution` | `HD1080` | 1920x1080 |
| General | `grab_frame_rate` | `30` | SDK caps at **15fps** for 1080p |
| Depth | `depth_mode` | `PERFORMANCE` | Fast depth on Orin |
| Depth | `confidence_threshold` | `50` | Balance coverage vs noise |
| Depth | `texture_confidence_threshold` | `100` | Strictest: removes flat surfaces |
| Pos tracking | `pos_tracking_enabled` | `true` | Required for IMU fusion internally |
| Pos tracking | `publish_tf` | `false` | cuVSLAM handles TF |
| Sensors | `sensors/publish_imu_tf` | `false` | cuVSLAM manages frames |

**Important**: `grab_frame_rate: 30` is configured but the ZED SDK caps HD1080 at 15fps. All monitoring thresholds are calibrated for this 15fps actual rate.

For 30fps operation, change to `grab_resolution: 'HD720'` (1280x720) and update all rate thresholds.

## cyclonedds.xml — DDS Middleware

| Setting | Value | Purpose |
|---------|-------|---------|
| Network interface | `lo` (localhost) | All nodes on same Jetson |
| Shared memory | Enabled | Zero-copy for image data |
| Receive buffer | 10 MB | HD1080 images are ~6 MB each |
| Lease duration | 30s | Reduce discovery overhead |

Requires kernel socket buffer tuning (done by `jetson_setup.sh`):
```bash
sudo sysctl -w net.core.rmem_max=67108864
sudo sysctl -w net.core.rmem_default=67108864
sudo sysctl -w net.core.wmem_max=67108864
```

## Modifying Parameters

1. Edit the relevant YAML file
2. With `--symlink-install`, YAML changes take effect on next launch (no rebuild)
3. For C++ parameter changes that add/remove `declare_parameter()` calls, rebuild the package
4. Verify with `ros2 topic echo /slam/diagnostics` after changes
