# Configuration Files

All parameters are loaded at launch time from these YAML/XML files. There is no dynamic reconfiguration — changes require a relaunch.

## File Overview

| File | Target node | Purpose |
|------|-------------|---------|
| `cuvslam.yaml` | `isaac_ros_visual_slam` | GPU visual-inertial odometry parameters |
| `rtabmap.yaml` | `rtabmap_slam` | Mapping, loop closure, graph optimization |
| `depth_filter.yaml` | `depth_filter_node` | Depth image preprocessing pipeline |
| `zed2i.yaml` | `zed_wrapper` | Camera driver + depth sensing config |
| `cyclonedds.xml` | CycloneDDS middleware | DDS transport, buffers, shared memory |

## cuvslam.yaml — GPU Visual-Inertial Odometry

cuVSLAM runs stereo VIO on the Orin GPU at ~1.8ms/frame. Key design: it does odometry ONLY. Mapping and loop closure are disabled because RTAB-Map handles those.

| Parameter | Value | Impact |
|-----------|-------|--------|
| `enable_imu_fusion` | `true` | Fuses BMI085 IMU for robust tracking through texture-poor areas |
| `enable_ground_constraint_in_odometry` | `true` | Constrains to 2D plane (ground robot) |
| `enable_localization_n_mapping` | `false` | Pure VIO mode — no internal SLAM |
| `enable_landmarks` | `false` | No landmark-based loop closure — RTAB-Map does this |
| `enable_slam_visualization` | `false` | Save GPU cycles for depth processing |
| `publish_odom_to_base_tf` | `true` | Publishes `odom → base_link` TF |
| `publish_map_to_odom_tf` | `false` | RTAB-Map publishes `map → odom` |
| `gyro_noise_density` | `0.000244` | BMI085 spec: 0.014 deg/s/sqrt(Hz) |
| `accel_noise_density` | `0.001862` | BMI085 spec: 120 ug/sqrt(Hz) |
| `img_jitter_threshold_ms` | `34.0` | ~1 frame at 30fps tolerance |

**Frame IDs**: `map`, `odom`, `base_link` — must match RTAB-Map and launch TF config.

## rtabmap.yaml — Mapping + Loop Closure

RTAB-Map receives odometry from cuVSLAM and filtered RGB-D from depth_filter_node. It focuses on building the pose graph, detecting loop closures, and generating navigation maps.

### Critical settings (fixes for greenhouse environment):

**Map node creation** (fix for "only 1 map node" bug):
| Parameter | Value | Default | Why changed |
|-----------|-------|---------|-------------|
| `RGBD/LinearUpdate` | `0.05` | `0.1` | Create node every 5cm (robot moves slowly) |
| `RGBD/AngularUpdate` | `0.1` | `0.2` | Create node every ~6deg turn |
| `Rtabmap/DetectionRate` | `5.0` | `1.0` | Process 5 frames/sec for mapping |

**Loop closure detection** (fix for zero closures in repetitive rows):
| Parameter | Value | Why |
|-----------|-------|-----|
| `Rtabmap/LoopThr` | `0.09` | Lower than default 0.11 — greenhouse scenes are repetitive |
| `RGBD/ProximityBySpace` | `true` | Detect closures by spatial proximity (critical for rows) |
| `RGBD/LocalRadius` | `5.0` | 5m search radius for local proximity detection |
| `RGBD/ProximityMaxGraphDepth` | `50` | Search 50 nodes deep in graph |
| `RGBD/LoopClosureReextractFeatures` | `true` | Re-extract features at closure time for better match |

**Feature detection**:
| Parameter | Value | Why |
|-----------|-------|-----|
| `Vis/FeatureType` | `8` | GFTT+ORB — uniform distribution prevents clustering on plants |
| `Vis/MaxFeatures` | `1000` | Enough for indoor matching |
| `Vis/MinInliers` | `15` | Reduced from default for repetitive textures |
| `Kp/DetectorStrategy` | `8` | Must match `Vis/FeatureType` |
| `Kp/NNStrategy` | `3` | BruteForce — required for binary descriptors (ORB) |

**Graph optimization**:
| Parameter | Value | Why |
|-----------|-------|-----|
| `Optimizer/Strategy` | `2` | GTSAM — supports gravity-aware optimization |
| `Optimizer/GravitySigma` | `0.3` | Weight for IMU gravity alignment |
| `Optimizer/Robust` | `true` | Huber cost function rejects outlier loop closures |
| `Reg/Force3DoF` | `true` | Ground robot: only X, Y, Yaw |

**Memory** (exploits 64GB RAM):
| Parameter | Value | Why |
|-----------|-------|-----|
| `Rtabmap/MemoryThr` | `0` | Unlimited — use all 64GB |
| `Mem/InitWMWithAllNodes` | `true` | Keep everything in working memory |
| `Db/Sqlite3JournalMode` | `3` | WAL mode for database performance |

**Occupancy grid** (for nav2):
- 5cm cell size, 0.3-8m range, 2D only, normals-based ground segmentation

## depth_filter.yaml — Depth Preprocessing

Four-stage pipeline, each independently toggleable:

| Stage | Parameter | Default | Effect |
|-------|-----------|---------|--------|
| 1. Range filter | `min_depth` / `max_depth` | 0.3 / 10.0 m | Set out-of-range pixels to NaN |
| 2. Temporal avg | `enable_temporal` / `temporal_window` | true / 3 | Average 3 frames (~100ms), reduces flicker |
| 3. Bilateral | `enable_bilateral` / `bilateral_d` | true / 5 | Edge-preserving smoothing, kernel=5 |
| 4. Hole filling | `enable_hole_filling` / `hole_kernel_size` | false / 3 | Morphological close + Telea inpaint |

**Topic remapping** (configurable):
```yaml
input_depth_topic:  '/zed/zed_node/depth/depth_registered'
input_rgb_topic:    '/zed/zed_node/rgb/image_rect_color'
output_depth_topic: '/filtered/depth'
output_rgb_topic:   '/filtered/rgb'
```

## zed2i.yaml — Camera Driver

| Section | Key parameter | Value | Why |
|---------|--------------|-------|-----|
| General | `grab_resolution` | `HD720` | 1280x720 — best compute/quality ratio on Orin |
| General | `grab_frame_rate` | `30` | Matches cuVSLAM expectation |
| Depth | `depth_mode` | `NEURAL` | Best accuracy at ~34fps on Orin (vs ULTRA at 15fps) |
| Depth | `depth_confidence` | `50` | Strict — removes edge artifacts (default 100 too permissive) |
| Depth | `depth_texture_conf` | `100` | Removes flat/uniform surfaces (greenhouse walls) |
| Depth | `remove_saturated_areas` | `true` | Removes reflections from glass/wet surfaces |
| Pos tracking | `pos_tracking_enabled` | `true` | Required internally for IMU fusion / rolling shutter |
| Pos tracking | `publish_tf` | `false` | cuVSLAM handles TF, not ZED |
| Pos tracking | `imu_fusion` | `true` | Rolling shutter compensation |
| Pos tracking | `two_d_mode` | `true` | Ground robot constraint |
| Sensors | `publish_imu_tf` | `false` | cuVSLAM manages camera frame TFs |

## cyclonedds.xml — DDS Middleware

Selected over FastDDS for lower latency with large messages (images, point clouds).

| Setting | Value | Why |
|---------|-------|-----|
| Network interface | `lo` (localhost) | All nodes on same Jetson |
| Shared memory | Enabled | Zero-copy for image/depth/pointcloud data |
| Receive buffer | 10 MB min | HD720 images are ~1.8 MB each |
| Write high-watermark | 500 KB | Prevent writer backpressure |
| Lease duration | 30s | Reduce discovery overhead (local only) |
| Log file | `/tmp/cyclonedds.log` | Check for DDS errors |

## Modifying Parameters

1. Edit the relevant YAML file
2. If using `--symlink-install`, changes take effect on next launch (no rebuild needed for config changes)
3. For C++ parameter changes that add/remove parameters, also update the node's `declare_parameter()` calls in the source and rebuild
4. Always verify with `scripts/validate_slam.sh` after parameter changes
