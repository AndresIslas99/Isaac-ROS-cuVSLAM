#!/usr/bin/env python3
# =============================================================================
# Coverage Monitor Node — FOV-Based Coverage Tracking
# =============================================================================
# Maintains a 2D occupancy grid (bird's eye view) tracking which areas
# have been observed by the camera. Uses camera FOV frustum projection
# onto the ground plane and tracks multi-angle observations.
#
# Cell states: UNKNOWN=0, SEEN_ONCE=1, WELL_COVERED=2
# Multi-angle: cell upgraded when observed from yaw differing by >30°
#
# Published:
#   /coverage/grid    (nav_msgs/OccupancyGrid) at 0.5 Hz
#   /coverage/status  (std_msgs/String JSON) at 1 Hz
# Service:
#   /coverage/reset   (std_srvs/Trigger)
# =============================================================================

import numpy as np
import math

# Cell states
UNKNOWN = 0
SEEN_ONCE = 1
WELL_COVERED = 2


# ── Standalone pure functions (testable without ROS) ──

def yaw_from_quaternion(q_x, q_y, q_z, q_w):
    """Extract yaw from quaternion (2D simplification)."""
    siny_cosp = 2.0 * (q_w * q_z + q_x * q_y)
    cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_grid(wx, wy, x_min, y_min, resolution):
    """Convert world coords to grid indices."""
    gx = int((wx - x_min) / resolution)
    gy = int((wy - y_min) / resolution)
    return gx, gy


class CoverageGrid:
    """Track which grid cells have been observed from multiple angles."""

    UNVISITED = 0
    PARTIAL = 1
    COVERED = 2

    def __init__(self, x_min, x_max, y_min, y_max, resolution, angles_required=4):
        self.x_min = x_min
        self.y_min = y_min
        self.resolution = resolution
        self.angles_required = angles_required
        self.width = int((x_max - x_min) / resolution)
        self.height = int((y_max - y_min) / resolution)
        self.visit_count = np.zeros((self.height, self.width), dtype=np.int32)
        self.angle_sets = [[set() for _ in range(self.width)] for _ in range(self.height)]

    def mark_observed(self, gx, gy, angle_bin):
        """Mark a cell as observed from a given angle bin."""
        if 0 <= gx < self.width and 0 <= gy < self.height:
            self.angle_sets[gy][gx].add(angle_bin)
            self.visit_count[gy, gx] += 1

    def get_state(self, gx, gy):
        """Return cell state: UNVISITED, PARTIAL, or COVERED."""
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return self.UNVISITED
        n_angles = len(self.angle_sets[gy][gx])
        if n_angles == 0:
            return self.UNVISITED
        elif n_angles >= self.angles_required:
            return self.COVERED
        else:
            return self.PARTIAL

    def coverage_percent(self):
        """Return percentage of cells fully covered."""
        total = self.width * self.height
        if total == 0:
            return 0.0
        covered = sum(
            1 for gy in range(self.height) for gx in range(self.width)
            if self.get_state(gx, gy) == self.COVERED
        )
        return (covered / total) * 100.0


# ── ROS2 Node (requires rclpy + agv_greenhouse_msgs) ──

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String as CoverageStatusStr
    import json
    from nav_msgs.msg import OccupancyGrid, Odometry
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import Pose

    class CoverageMonitorNode(Node):
        def __init__(self):
            super().__init__('coverage_monitor')

            # Parameters
            self.declare_parameter('x_min', -25.0)
            self.declare_parameter('x_max', 25.0)
            self.declare_parameter('y_min', -25.0)
            self.declare_parameter('y_max', 25.0)
            self.declare_parameter('resolution', 0.1)  # meters per cell
            self.declare_parameter('camera_hfov_deg', 110.0)  # ZED 2i ~110° HFOV
            self.declare_parameter('camera_range_m', 5.0)  # Max useful depth
            self.declare_parameter('multi_angle_threshold_deg', 30.0)
            self.declare_parameter('grid_publish_rate_hz', 0.5)
            self.declare_parameter('status_publish_rate_hz', 1.0)

            x_min = self.get_parameter('x_min').value
            x_max = self.get_parameter('x_max').value
            y_min = self.get_parameter('y_min').value
            y_max = self.get_parameter('y_max').value
            self.resolution = self.get_parameter('resolution').value
            self.hfov = math.radians(self.get_parameter('camera_hfov_deg').value)
            self.cam_range = self.get_parameter('camera_range_m').value
            self.angle_thresh = math.radians(
                self.get_parameter('multi_angle_threshold_deg').value)

            # Grid dimensions
            self.x_min = x_min
            self.y_min = y_min
            self.width = int((x_max - x_min) / self.resolution)
            self.height = int((y_max - y_min) / self.resolution)

            # Coverage grid (cell states)
            self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
            # Yaw at first observation (for multi-angle detection)
            self.first_yaw = np.full((self.height, self.width), np.nan, dtype=np.float32)

            # Subscribers
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE, depth=1)
            self.odom_sub = self.create_subscription(
                Odometry, '/visual_slam/tracking/odometry',
                self.odom_callback, sensor_qos)

            # Publishers
            self.grid_pub = self.create_publisher(OccupancyGrid, '/coverage/grid', 10)
            self.status_pub = self.create_publisher(CoverageStatusStr, '/coverage/status', 10)

            # Services
            self.reset_srv = self.create_service(
                Trigger, '/coverage/reset', self.reset_callback)

            # Timers
            grid_rate = self.get_parameter('grid_publish_rate_hz').value
            status_rate = self.get_parameter('status_publish_rate_hz').value
            self.grid_timer = self.create_timer(1.0 / grid_rate, self.publish_grid)
            self.status_timer = self.create_timer(1.0 / status_rate, self.publish_status)

            self.get_logger().info(
                f'Coverage monitor: {self.width}x{self.height} grid, '
                f'{self.resolution}m resolution, '
                f'HFOV={math.degrees(self.hfov):.0f}°, range={self.cam_range}m')

        def _yaw_from_quaternion(self, q):
            """Extract yaw from quaternion (2D simplification)."""
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)

        def _world_to_grid(self, wx, wy):
            """Convert world coords to grid indices."""
            gx = int((wx - self.x_min) / self.resolution)
            gy = int((wy - self.y_min) / self.resolution)
            return gx, gy

        def odom_callback(self, msg):
            """Project camera FOV onto ground plane and mark visible cells."""
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)

            # Camera FOV frustum — generate rays from -hfov/2 to +hfov/2
            half_fov = self.hfov / 2.0
            n_rays = 32  # Number of rays to sample across FOV

            for i in range(n_rays + 1):
                angle = yaw - half_fov + (self.hfov * i / n_rays)

                # March along ray at resolution steps
                for d in np.arange(0.3, self.cam_range, self.resolution):
                    wx = px + d * math.cos(angle)
                    wy = py + d * math.sin(angle)

                    gx, gy = self._world_to_grid(wx, wy)
                    if 0 <= gx < self.width and 0 <= gy < self.height:
                        current = self.grid[gy, gx]
                        if current == UNKNOWN:
                            self.grid[gy, gx] = SEEN_ONCE
                            self.first_yaw[gy, gx] = yaw
                        elif current == SEEN_ONCE:
                            # Check multi-angle
                            first = self.first_yaw[gy, gx]
                            if not np.isnan(first):
                                delta = abs(yaw - first)
                                delta = min(delta, 2 * math.pi - delta)
                                if delta > self.angle_thresh:
                                    self.grid[gy, gx] = WELL_COVERED

        def reset_callback(self, request, response):
            self.grid.fill(UNKNOWN)
            self.first_yaw.fill(np.nan)
            self.get_logger().info('Coverage grid reset')
            response.success = True
            response.message = 'Coverage grid cleared'
            return response

        def publish_grid(self):
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.info.resolution = self.resolution
            msg.info.width = self.width
            msg.info.height = self.height
            msg.info.origin.position.x = self.x_min
            msg.info.origin.position.y = self.y_min

            # Map cell states to OccupancyGrid values:
            # UNKNOWN=0 → -1 (unknown in OccupancyGrid)
            # SEEN_ONCE=1 → 50
            # WELL_COVERED=2 → 100
            flat = self.grid.flatten()
            data = np.where(flat == UNKNOWN, -1,
                   np.where(flat == SEEN_ONCE, 50, 100)).astype(np.int8)
            msg.data = data.tolist()
            self.grid_pub.publish(msg)

        def publish_status(self):
            total = self.width * self.height
            seen = int(np.sum(self.grid >= SEEN_ONCE))
            well = int(np.sum(self.grid >= WELL_COVERED))

            coverage_pct = (seen / total * 100.0) if total > 0 else 0.0
            well_pct = (well / total * 100.0) if total > 0 else 0.0
            mapped_area = seen * (self.resolution ** 2)
            msg = CoverageStatusStr()
            msg.data = json.dumps({
                "total_cells": total,
                "seen_cells": seen,
                "well_covered_cells": well,
                "coverage_percent": round(coverage_pct, 2),
                "well_covered_percent": round(well_pct, 2),
                "mapped_area_m2": round(mapped_area, 2),
                "grid_resolution": self.resolution,
            })
            self.status_pub.publish(msg)

    def main(args=None):
        rclpy.init(args=args)
        node = CoverageMonitorNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    _entry_main = main

except ImportError:
    _entry_main = None


def main(args=None):
    if _entry_main is not None:
        _entry_main(args)
    else:
        raise RuntimeError('ROS2 (rclpy) is not available')


if __name__ == '__main__':
    main()
