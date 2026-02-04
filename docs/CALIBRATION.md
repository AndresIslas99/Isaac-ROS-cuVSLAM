# Camera Calibration and Mount Configuration

## Camera Mount TF

The static transform `base_link -> zed_camera_link` is published by a `static_transform_publisher` node in the launch file.

### Current Values

```
x: 0.1 m    (forward from base_link)
y: 0.0 m    (centered)
z: 0.3 m    (above base_link)
roll: 0.0
pitch: 0.087 rad  (~5 degrees downward)
yaw: 0.0
```

### Launch Arguments

These can be overridden at launch time:

```bash
ros2 launch agv_slam agv_slam.launch.py \
  camera_x:=0.1 camera_y:=0.0 camera_z:=0.3 camera_pitch:=0.087
```

### Measuring the Transform

1. Measure the physical offset from robot center (base_link) to the camera mounting point
2. X = forward distance in meters
3. Y = leftward distance in meters (0 if centered)
4. Z = upward distance in meters
5. Pitch = downward angle in radians (positive = looking down). Use `python3 -c "import math; print(math.radians(5))"`

### Verification

After launch, verify the TF:

```bash
ros2 run tf2_ros tf2_echo base_link zed_camera_link
```

The slam_monitor will report `base_link->camera: OK` in the SLAM/TF diagnostic group if this transform exists.

## IMU Noise Parameters

cuVSLAM uses the ZED 2i's built-in BMI085 IMU. The noise parameters in `config/cuvslam.yaml` are from the BMI085 datasheet:

| Parameter | Value | Source |
|-----------|-------|--------|
| `gyro_noise_density` | 0.000244 | BMI085: 0.014 deg/s/sqrt(Hz) |
| `gyro_random_walk` | 0.000019393 | BMI085 typical |
| `accel_noise_density` | 0.001862 | BMI085: 120 ug/sqrt(Hz) |
| `accel_random_walk` | 0.003 | BMI085 typical |
| `calibration_frequency` | 200.0 | IMU rate / 2 |

These should not need adjustment unless replacing the camera.

## ZED 2i Depth Confidence

The ZED depth confidence parameters in `config/zed2i.yaml` control how aggressively noisy depth pixels are rejected:

| Parameter | Value | Effect |
|-----------|-------|--------|
| `confidence_threshold` | 50 | Balance between coverage and noise (0=all, 100=none) |
| `texture_confidence_threshold` | 100 | Strictest: removes flat/textureless surfaces |

For greenhouse environments with glass and reflective surfaces, keeping `confidence_threshold` at 50 provides a good balance. The depth_filter_node further removes ~33% of remaining invalid pixels.

## Frame Rate Consideration

The ZED 2i at HD1080 resolution achieves a maximum of **15 fps** (not 30fps as configured). The `grab_frame_rate: 30` in `zed2i.yaml` requests 30fps but the SDK caps at 15fps for 1080p. All monitoring thresholds are calibrated for this 15fps baseline.

If higher frame rates are needed for faster robot motion:
- Change `grab_resolution: 'HD720'` for 30fps at 1280x720
- Update slam_monitor thresholds back to 30Hz
- Update cuVSLAM `image_jitter_threshold_ms` to 40.0
