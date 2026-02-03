#!/usr/bin/env python3
# =============================================================================
# Coverage Monitor Node
# =============================================================================
# Subscribes to nvblox mesh and occupancy grid outputs to calculate
# greenhouse coverage metrics. Publishes diagnostics with:
#   - Total mapped volume (m³)
#   - Mapped area (m² from 2D ESDF slice)
#   - Mesh vertex count
#   - Coverage percentage (if target area is configured)
#
# Topic: /coverage/status (diagnostic_msgs/DiagnosticArray)
# =============================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import json
import time


class CoverageMonitorNode(Node):
    def __init__(self):
        super().__init__('coverage_monitor')

        # Parameters
        self.declare_parameter('target_area_m2', 0.0)  # 0 = no target
        self.declare_parameter('publish_rate_hz', 0.5)
        self.declare_parameter('grid_topic', '/nvblox_node/static_occupancy_grid')
        self.declare_parameter('mesh_topic', '/nvblox_node/tsdf_layer_marker')

        self.target_area = self.get_parameter('target_area_m2').value
        rate = self.get_parameter('publish_rate_hz').value
        grid_topic = self.get_parameter('grid_topic').value
        mesh_topic = self.get_parameter('mesh_topic').value

        # State
        self.mapped_cells = 0
        self.total_cells = 0
        self.grid_resolution = 0.0
        self.grid_width = 0
        self.grid_height = 0
        self.mesh_marker_count = 0
        self.last_grid_time = 0.0
        self.last_mesh_time = 0.0
        self.start_time = time.time()

        # QoS: nvblox publishes with RELIABLE, so we must match
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscribers
        self.grid_sub = self.create_subscription(
            OccupancyGrid, grid_topic,
            self.grid_callback, reliable_qos)

        self.mesh_sub = self.create_subscription(
            Marker, mesh_topic,
            self.mesh_callback, reliable_qos)

        # Publishers
        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/coverage/status', 10)
        self.json_pub = self.create_publisher(
            String, '/coverage/json', 10)

        # Timer
        self.timer = self.create_timer(1.0 / rate, self.publish_status)

        self.get_logger().info(
            f'Coverage monitor initialized. Grid: {grid_topic}, Mesh: {mesh_topic}')

    def grid_callback(self, msg):
        self.grid_resolution = msg.info.resolution
        self.grid_width = msg.info.width
        self.grid_height = msg.info.height
        self.total_cells = self.grid_width * self.grid_height

        # Count occupied + free cells (non-unknown)
        # In nvblox ESDF slice: -1 = unknown, 0-100 = known
        self.mapped_cells = sum(1 for c in msg.data if c >= 0)
        self.last_grid_time = time.time()

    def mesh_callback(self, msg):
        # Single Marker: count points in the mesh
        self.mesh_marker_count = len(msg.points) if msg.points else 0
        self.last_mesh_time = time.time()

    def publish_status(self):
        now = time.time()
        elapsed = now - self.start_time

        # Calculate metrics
        mapped_area = self.mapped_cells * (self.grid_resolution ** 2) if self.grid_resolution > 0 else 0.0
        total_area = self.total_cells * (self.grid_resolution ** 2) if self.grid_resolution > 0 else 0.0

        coverage_pct = 0.0
        if self.target_area > 0:
            coverage_pct = min(100.0, (mapped_area / self.target_area) * 100.0)
        elif total_area > 0:
            coverage_pct = (self.mapped_cells / self.total_cells) * 100.0

        grid_age = now - self.last_grid_time if self.last_grid_time > 0 else -1.0
        mesh_age = now - self.last_mesh_time if self.last_mesh_time > 0 else -1.0

        # Determine health
        level = DiagnosticStatus.OK
        message = 'Mapping active'
        if grid_age < 0 and mesh_age < 0:
            level = DiagnosticStatus.WARN
            message = 'No nvblox data received yet'
        elif grid_age > 10.0 or mesh_age > 10.0:
            level = DiagnosticStatus.WARN
            message = 'nvblox data stale'

        # Diagnostic message
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'Coverage'
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key='Mapped Area (m²)', value=f'{mapped_area:.2f}'),
            KeyValue(key='Grid Coverage (%)', value=f'{coverage_pct:.1f}'),
            KeyValue(key='Grid Resolution (m)', value=f'{self.grid_resolution:.3f}'),
            KeyValue(key='Grid Cells (mapped/total)',
                     value=f'{self.mapped_cells}/{self.total_cells}'),
            KeyValue(key='Mesh Markers', value=str(self.mesh_marker_count)),
            KeyValue(key='Elapsed (s)', value=f'{elapsed:.0f}'),
        ]
        if self.target_area > 0:
            status.values.append(
                KeyValue(key='Target Area (m²)', value=f'{self.target_area:.1f}'))
            status.values.append(
                KeyValue(key='Target Coverage (%)', value=f'{coverage_pct:.1f}'))

        diag.status = [status]
        self.diag_pub.publish(diag)

        # JSON for easy parsing
        json_msg = String()
        json_msg.data = json.dumps({
            'mapped_area_m2': round(mapped_area, 2),
            'coverage_pct': round(coverage_pct, 1),
            'mesh_markers': self.mesh_marker_count,
            'grid_resolution': self.grid_resolution,
            'elapsed_s': round(elapsed, 0),
            'healthy': level == DiagnosticStatus.OK,
        })
        self.json_pub.publish(json_msg)

        if int(elapsed) % 30 == 0:
            self.get_logger().info(
                f'Coverage: {mapped_area:.1f}m² mapped, '
                f'{coverage_pct:.1f}% coverage, '
                f'{self.mesh_marker_count} mesh markers')


def main(args=None):
    rclpy.init(args=args)
    node = CoverageMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
