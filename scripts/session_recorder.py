#!/usr/bin/env python3
# =============================================================================
# Session Recorder Node
# =============================================================================
# Records session data for offline 3DGS reconstruction:
#   1. ROS2 bag recording (depth, color, camera_info, odometry, TF)
#   2. TUM trajectory format (timestamp tx ty tz qx qy qz qw)
#   3. Session manifest JSON (metadata, paths, timestamps)
#
# SVO2 recording is handled by the ZED node's built-in service.
#
# Services:
#   /session/start  (std_srvs/Trigger) — Start recording
#   /session/stop   (std_srvs/Trigger) — Stop recording
#
# Topics published:
#   /session/status (std_msgs/String) — JSON status
# =============================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import subprocess
import json
import os
import time
from datetime import datetime


class SessionRecorderNode(Node):
    def __init__(self):
        super().__init__('session_recorder')

        # Parameters
        self.declare_parameter('output_dir', '/mnt/ssd/sessions')
        self.declare_parameter('bag_topics', [
            '/zed/zed_node/rgb/color/rect/image',
            '/zed/zed_node/depth/depth_registered',
            '/zed/zed_node/depth/camera_info',
            '/zed/zed_node/left/gray/rect/image',
            '/zed/zed_node/right/gray/rect/image',
            '/zed/zed_node/left/gray/rect/camera_info',
            '/zed/zed_node/imu/data',
            '/visual_slam/tracking/odometry',
            '/tf',
            '/tf_static',
        ])

        self.output_dir = self.get_parameter('output_dir').value
        self.bag_topics = self.get_parameter('bag_topics').value

        # State
        self.recording = False
        self.session_name = ''
        self.session_dir = ''
        self.bag_process = None
        self.tum_file = None
        self.start_time = 0.0
        self.frame_count = 0

        # Odom subscriber for TUM trajectory
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry',
            self.odom_callback, sensor_qos)

        # Services
        self.start_srv = self.create_service(
            Trigger, '/session/start', self.start_recording)
        self.stop_srv = self.create_service(
            Trigger, '/session/stop', self.stop_recording)

        # Status publisher
        self.status_pub = self.create_publisher(String, '/session/status', 10)
        self.status_timer = self.create_timer(2.0, self.publish_status)

        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(
            f'Session recorder ready. Output: {self.output_dir}')

    def odom_callback(self, msg):
        if not self.recording or self.tum_file is None:
            return

        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # TUM format: timestamp tx ty tz qx qy qz qw
        self.tum_file.write(
            f'{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} '
            f'{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n')
        self.frame_count += 1

    def start_recording(self, request, response):
        if self.recording:
            response.success = False
            response.message = 'Already recording'
            return response

        # Create session directory
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_name = f'session_{ts}'
        self.session_dir = os.path.join(self.output_dir, self.session_name)
        os.makedirs(self.session_dir, exist_ok=True)

        bag_dir = os.path.join(self.session_dir, 'bag')
        tum_path = os.path.join(self.session_dir, 'trajectory.tum')

        # Start ros2 bag record
        topic_args = []
        for t in self.bag_topics:
            topic_args.extend([t])

        cmd = ['ros2', 'bag', 'record', '-o', bag_dir] + topic_args
        self.bag_process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env={**os.environ, 'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'}
        )

        # Open TUM trajectory file
        self.tum_file = open(tum_path, 'w')
        self.tum_file.write('# TUM trajectory format: timestamp tx ty tz qx qy qz qw\n')

        self.recording = True
        self.start_time = time.time()
        self.frame_count = 0

        # Write initial manifest
        self._write_manifest()

        self.get_logger().info(f'Recording started: {self.session_dir}')
        response.success = True
        response.message = f'Recording to {self.session_dir}'
        return response

    def stop_recording(self, request, response):
        if not self.recording:
            response.success = False
            response.message = 'Not recording'
            return response

        # Stop bag recording
        if self.bag_process:
            self.bag_process.terminate()
            self.bag_process.wait(timeout=10)
            self.bag_process = None

        # Close TUM file
        if self.tum_file:
            self.tum_file.close()
            self.tum_file = None

        duration = time.time() - self.start_time
        self.recording = False

        # Write final manifest
        self._write_manifest(duration=duration)

        self.get_logger().info(
            f'Recording stopped: {self.frame_count} frames, '
            f'{duration:.1f}s duration')
        response.success = True
        response.message = (
            f'Saved to {self.session_dir}: '
            f'{self.frame_count} frames, {duration:.1f}s')
        return response

    def _write_manifest(self, duration=None):
        manifest = {
            'session_name': self.session_name,
            'created': datetime.now().isoformat(),
            'output_dir': self.session_dir,
            'bag_dir': os.path.join(self.session_dir, 'bag'),
            'trajectory_file': os.path.join(self.session_dir, 'trajectory.tum'),
            'bag_topics': self.bag_topics,
            'recording': self.recording,
            'frame_count': self.frame_count,
        }
        if duration is not None:
            manifest['duration_s'] = round(duration, 1)

        manifest_path = os.path.join(self.session_dir, 'manifest.json')
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)

    def publish_status(self):
        status = {
            'recording': self.recording,
            'session_name': self.session_name,
            'frames': self.frame_count,
            'elapsed_s': round(time.time() - self.start_time, 0) if self.recording else 0,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SessionRecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
