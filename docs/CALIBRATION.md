# Camera Mount Calibration

How to measure and set the camera-to-robot transform.

## What You Need

- Tape measure or ruler
- The launch arguments: `camera_x`, `camera_y`, `camera_z`, `camera_pitch`

## Measurement Reference

All measurements are from `base_link` (center of the robot at ground level):

- **X** = forward distance (positive = camera in front of robot center)
- **Y** = left distance (positive = camera left of center, typically 0)
- **Z** = height above ground
- **Pitch** = downward tilt in radians (positive = tilted down)

## Steps

1. **Measure X**: Distance from robot center to camera, along the forward direction.
   Default: 0.1m (10cm forward)

2. **Measure Y**: Lateral offset. Should be 0 if camera is centered.
   Default: 0.0m

3. **Measure Z**: Height from ground to the center of the ZED 2i lens.
   Default: 0.3m (30cm)

4. **Estimate pitch**: If the camera points slightly downward, measure the angle.
   5 degrees ≈ 0.087 radians. Use `pitch_rad = degrees * 3.14159 / 180`
   Default: 0.087 (5° down)

## Setting Values

```bash
# Via launch arguments (no rebuild needed):
ros2 launch agv_slam agv_slam.launch.py camera_x:=0.15 camera_z:=0.45 camera_pitch:=0.12

# Or edit the defaults in the launch file
```

## Verification

After launching, verify with:

```bash
ros2 run tf2_ros tf2_echo base_link zed_camera_link
```

The output should show your configured translation and rotation values.

## Tips

- Small errors in X/Y/Z (< 2cm) are acceptable — cuVSLAM adapts
- Pitch accuracy matters more for nvblox ground plane alignment
- Re-measure if the camera mount or elevator position changes
