# System Architecture

## Design Philosophy

This pipeline separates visual-inertial odometry (VIO) from mapping/loop-closure. NVIDIA cuVSLAM runs VIO on the GPU at 1.8ms/frame, freeing RTAB-Map to focus exclusively on graph optimization and loop closure at ~50ms/frame. Without this split, RTAB-Map attempted all tasks simultaneously at 435ms/frame, making 30Hz operation impossible.

## Pipeline Data Flow

```
                          STEREO GRAYSCALE (30Hz)
    ZED 2i ──────────────────────────────────────────► cuVSLAM (GPU)
      │                                                    │
      │  RGB + Depth (30Hz)                                │ /visual_slam/tracking/odometry
      │                                                    │ TF: odom → base_link
      ▼                                                    │
  Depth Filter Node                                        │
      │                                                    │
      │  /filtered/rgb                                     │
      │  /filtered/depth                                   │
      │  /filtered/camera_info                             │
      ▼                                                    ▼
  RTAB-Map  ◄──────────────────────────────────────────────┘
      │         (receives odometry + filtered RGB-D + IMU)
      │
      │  TF: map → odom (loop closure corrections)
      │  /rtabmap/grid_map (2D nav grid)
      │  /rtabmap/cloud_map (3D point cloud)
      │  /rtabmap/mapData (full graph)
      │
      ▼
  SLAM Monitor Node
      │
      │  /slam/diagnostics (1Hz health report)
      ▼
  [Navigation stack / operator]
```

## Node Responsibilities

### ZED 2i Driver (external: `zed_wrapper`)
- Publishes rectified stereo grayscale, RGB, depth, IMU, camera_info
- Runs NEURAL depth inference on GPU (~34fps at HD720)
- Internal VIO enabled ONLY for rolling shutter compensation (`publish_tf: false`)
- IMU at 400Hz (BMI085), images at 30Hz

### Depth Filter Node (this package: `depth_filter_node`)
- **Input**: `/zed/.../depth_registered` + `/zed/.../rgb/image_rect_color` (ApproximateTime sync, 10-msg buffer)
- **Processing pipeline** (sequential, each stage optional via YAML):
  1. **PassThrough**: NaN removal + range clamp [0.3m, 10.0m] → sets invalid pixels to NaN
  2. **Temporal averaging**: Running average over N frames (default 3 = ~100ms) with per-pixel NaN-aware accumulation
  3. **Bilateral filter**: Edge-preserving smoothing (kernel=5, sigma=50) — NaN handled via temporary zero-fill
  4. **Hole filling**: Morphological closing + Telea inpainting for small gaps (disabled by default)
- **Output**: `/filtered/rgb`, `/filtered/depth`, `/filtered/camera_info` (camera_info is passthrough)
- **Executor**: `MultiThreadedExecutor` with 2 threads
- **Diagnostics**: Logs frame count, processing time, and invalid pixel percentage every 100 frames

### cuVSLAM (external: `isaac_ros_visual_slam`)
- **Input**: Rectified stereo grayscale + IMU (via topic remapping)
- **Output**: `/visual_slam/tracking/odometry` (nav_msgs/Odometry) + TF `odom → base_link`
- **Mode**: Pure VIO (`enable_localization_n_mapping: false`, `enable_landmarks: false`)
- **Ground constraint**: Enabled — constrains odometry to 2D plane
- **IMU noise model**: Tuned for ZED 2i's BMI085 sensor
- Does NOT publish `map → odom` (RTAB-Map does that)

### RTAB-Map (external: `rtabmap_slam`)
- **Input**: Filtered RGB-D + cuVSLAM odometry + IMU
- **Critical setting**: `visual_odometry: false` — uses external odometry only
- **Functions**:
  - Pose graph construction (new node every 5cm / 6deg)
  - Loop closure detection (threshold 0.09, spatial proximity for greenhouse rows)
  - Graph optimization (GTSAM with gravity, robust Huber cost, 100 iterations)
  - 2D occupancy grid generation (5cm cells)
  - 3D point cloud assembly (2cm voxels)
  - `map → odom` TF publication (corrects drift on loop closure)
- **Modes**: Mapping (default, deletes DB on start) or Localization (loads existing DB, `Mem/IncrementalMemory: false`)
- **3DoF constraint**: `Reg/Force3DoF: true` — only X, Y, Yaw (ground robot assumption)

### SLAM Monitor Node (this package: `slam_monitor_node`)
- **Input**: `/filtered/rgb`, `/filtered/depth`, `/zed/.../imu/data`, `/visual_slam/tracking/odometry`
- **Output**: `/slam/diagnostics` (diagnostic_msgs/DiagnosticArray at 1Hz)
- **Monitors**:
  - Topic rates via exponential moving average (RGB, Depth, IMU, Odom)
  - TF chain integrity (`map → base_link` lookup)
  - Total odometry distance traveled
- **Health thresholds**: RGB/Depth/Odom < 10Hz = WARN, IMU < 100Hz = WARN
- **Console log**: Summary every 10 seconds

## TF Tree

```
map                          ← RTAB-Map publishes (loop closure correction)
  └── odom                   ← cuVSLAM publishes (real-time VIO)
        └── base_link        ← Static TF from launch file
              └── zed2i_base_link  ← Static TF from launch file
                    ├── zed2i_left_camera_optical_frame    ← ZED wrapper
                    ├── zed2i_right_camera_optical_frame   ← ZED wrapper
                    └── zed2i_imu_link                     ← ZED wrapper
```

**Static transform** (`base_link → zed2i_base_link`):
- Translation: x=0.1m (forward), y=0.0m, z=0.3m (up)
- Rotation: roll=0, pitch=0.087 rad (~5deg down), yaw=0
- Defined in both `launch/agv_slam.launch.py` and `urdf/zed2i_mount.urdf.xacro`

## Launch Sequence & Timing

Nodes launch with `TimerAction` delays to ensure each dependency is publishing before its consumer starts:

| Delay | Node | Waits for |
|-------|------|-----------|
| 0s | ZED 2i + Static TF | - |
| 3s | Depth Filter | ZED publishing RGB + Depth |
| 5s | cuVSLAM | ZED publishing Stereo + IMU |
| 8s | RTAB-Map | cuVSLAM publishing odometry + Depth Filter publishing filtered images |
| 10s | SLAM Monitor | All upstream nodes |

## QoS Policies

| Topic type | QoS | Rationale |
|-----------|-----|-----------|
| Sensor images/depth/IMU | `SensorDataQoS` (best effort, volatile) | Drop old frames, never block |
| Odometry | `SensorDataQoS` | Real-time, tolerate drops |
| Diagnostics | Reliable, queue=10 | Must not lose health alerts |
| RTAB-Map grid/cloud | Reliable, depth=5 | Maps must arrive intact |
| Message filter sync | ApproximateTime, buffer=10 | Tolerate slight timestamp mismatch |

## DDS Configuration (CycloneDDS)

- **Transport**: Localhost only (`lo` interface) — all nodes on same Jetson
- **Shared memory**: Enabled — zero-copy for images/point clouds between nodes
- **Receive buffers**: 10MB minimum (images are ~1.8MB each at HD720)
- **Write high-watermark**: 500KB
- **Discovery**: 30s lease duration (reduce overhead, all nodes are local)

## Memory & Compute Budget (Orin AGX 64GB)

| Resource | Consumer | Typical |
|----------|----------|---------|
| GPU | cuVSLAM VIO | ~1.7% |
| GPU | ZED NEURAL depth | ~15-20% |
| CPU | RTAB-Map graph optimization | ~5-10% (1 core) |
| CPU | Depth Filter (OpenCV) | ~3% (2 threads) |
| CPU | ZED driver | ~5% |
| RAM | RTAB-Map working memory | 2-8 GB (grows with map) |
| RAM | Image buffers (DDS) | ~500 MB |
| RAM | Available headroom | 40-55 GB |

RTAB-Map memory is unlimited (`Rtabmap/MemoryThr: 0`) to exploit the 64GB available.

## Greenhouse-Specific Tuning

The greenhouse environment has specific challenges addressed in the config:

1. **Repetitive visual scenes** (identical plant rows):
   - `RGBD/ProximityBySpace: true` — detect loop closures by spatial proximity, not just visual similarity
   - `Mem/RehearsalSimilarity: 0.6` — higher threshold to avoid merging distinct-but-similar nodes
   - `Vis/FeatureType: 8` (GFTT+ORB) — uniform feature distribution across image, prevents clustering on high-texture plants

2. **Slow robot movement**:
   - `RGBD/LinearUpdate: 0.05m` — create map node every 5cm (default 0.1m was too sparse)
   - `RGBD/AngularUpdate: 0.1 rad` — create node every ~6deg turn

3. **Reflective/transparent surfaces** (glass walls, wet leaves):
   - `depth_confidence: 50` — aggressive filtering in ZED config
   - `depth_texture_conf: 100` — remove flat/uniform depth regions
   - `remove_saturated_areas: true` — remove blown-out reflections

4. **Long corridor-like rows**:
   - `RGBD/LocalRadius: 5.0m` — expanded local search radius
   - `RGBD/ProximityMaxGraphDepth: 50` — search deeper in graph for spatial matches
