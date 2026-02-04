"""Integration tests using launch_testing.

Verifies that depth_filter, slam_monitor, and pipeline_watchdog nodes
start up, advertise expected topics, publish static TF, and shut down cleanly.
"""

import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the three custom nodes plus the static TF publisher."""

    pkg_dir = os.path.join(os.path.dirname(__file__), '..')

    depth_filter_node = launch_ros.actions.Node(
        package='agv_slam',
        executable='depth_filter_node',
        name='depth_filter',
        output='screen',
    )

    slam_monitor_node = launch_ros.actions.Node(
        package='agv_slam',
        executable='slam_monitor_node',
        name='slam_monitor',
        output='screen',
    )

    watchdog_node = launch_ros.actions.Node(
        package='agv_slam',
        executable='pipeline_watchdog_node',
        name='pipeline_watchdog',
        output='screen',
        parameters=[{
            'log_directory': '/tmp/agv_slam_test_logs/',
            'heartbeat_rate': 2.0,
            'node_timeout_sec': 10.0,
        }],
    )

    static_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x', '0.1', '--y', '0.0', '--z', '0.3',
            '--roll', '0.0', '--pitch', '0.087', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'zed_camera_link',
        ],
    )

    return (
        launch.LaunchDescription([
            depth_filter_node,
            slam_monitor_node,
            watchdog_node,
            static_tf,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'depth_filter': depth_filter_node,
            'slam_monitor': slam_monitor_node,
            'watchdog': watchdog_node,
            'static_tf': static_tf,
        },
    )


class TestNodeStartup(unittest.TestCase):
    """Verify nodes start and are discoverable."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_observer')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_nodes_running(self):
        """All three custom nodes should appear in the node list."""
        expected_nodes = {'/depth_filter', '/slam_monitor', '/pipeline_watchdog'}
        found = set()

        # Poll for up to 10 seconds
        end_time = time.time() + 10.0
        while time.time() < end_time and not expected_nodes.issubset(found):
            node_names = self.node.get_node_names_and_namespaces()
            found = {'/' + name if ns == '/' else ns + '/' + name
                     for name, ns in node_names}
            # Also try without leading slash
            found |= {name for name, ns in node_names}
            time.sleep(0.5)

        for expected in expected_nodes:
            node_name = expected.lstrip('/')
            self.assertTrue(
                any(node_name in f for f in found),
                f"Node {expected} not found. Discovered: {found}"
            )


class TestTopicAdvertisement(unittest.TestCase):
    """Verify expected topics are advertised."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('topic_observer')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for_topic(self, topic_name, timeout=10.0):
        end_time = time.time() + timeout
        while time.time() < end_time:
            topics = self.node.get_topic_names_and_types()
            for name, _ in topics:
                if name == topic_name:
                    return True
            time.sleep(0.5)
        return False

    def test_filtered_depth_topic(self):
        """Depth filter should advertise /filtered/depth."""
        self.assertTrue(
            self._wait_for_topic('/filtered/depth'),
            "/filtered/depth not advertised"
        )

    def test_diagnostics_topic(self):
        """SLAM monitor should advertise /slam/diagnostics."""
        self.assertTrue(
            self._wait_for_topic('/slam/diagnostics'),
            "/slam/diagnostics not advertised"
        )

    def test_heartbeat_topic(self):
        """Watchdog should advertise /watchdog/heartbeat."""
        self.assertTrue(
            self._wait_for_topic('/watchdog/heartbeat'),
            "/watchdog/heartbeat not advertised"
        )


class TestStaticTF(unittest.TestCase):
    """Verify static TF is published."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('tf_observer')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_base_to_camera_tf(self):
        """/tf_static should contain base_link → zed_camera_link."""
        from tf2_ros import Buffer, TransformListener

        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, self.node)

        # Wait for TF to be available
        end_time = time.time() + 10.0
        transform = None
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            try:
                transform = tf_buffer.lookup_transform(
                    'base_link', 'zed_camera_link', rclpy.time.Time())
                break
            except Exception:
                time.sleep(0.5)

        self.assertIsNotNone(transform, "base_link → zed_camera_link TF not found")
        self.assertAlmostEqual(transform.transform.translation.x, 0.1, places=2)
        self.assertAlmostEqual(transform.transform.translation.z, 0.3, places=2)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Verify all nodes shut down cleanly."""

    def test_exit_codes(self, proc_info):
        """All processes should exit with code 0 or SIGINT (negative)."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -15],  # 0, SIGINT, SIGTERM
        )
