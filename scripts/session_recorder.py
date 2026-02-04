#!/usr/bin/env python3
# =============================================================================
# Session Recorder Node — Recording Orchestration
# =============================================================================
# Records session data for offline 3DGS reconstruction:
#   1. ZED SVO2 recording (via /zed/zed_node/start_svo_rec service)
#   2. ROS2 bag recording (depth, color, camera_info, odometry, TF)
#   3. TUM trajectory format (timestamp tx ty tz qx qy qz qw)
#   4. nvblox PLY save on stop (via /nvblox_node/save_ply service)
#   5. Session manifest with checksums
#
# Services:
#   /session/start_recording  (agv_greenhouse_msgs/srv/StartRecording)
#   /session/stop_recording   (agv_greenhouse_msgs/srv/StopRecording)
# Published:
#   /session/info             (agv_greenhouse_msgs/msg/SessionInfo)
# =============================================================================

import hashlib
import json
import os
import shutil
import time


# ── Standalone pure functions (testable without ROS) ──

def file_checksum(path):
    """Compute SHA256 checksum of a file."""
    if not os.path.exists(path):
        return None
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(8192), b''):
            h.update(chunk)
    return h.hexdigest()


def dir_size_gb(path):
    """Calculate total directory size in gigabytes."""
    if not os.path.exists(path):
        return 0.0
    total = 0
    for dirpath, _, filenames in os.walk(path):
        for f in filenames:
            fp = os.path.join(dirpath, f)
            if os.path.isfile(fp):
                total += os.path.getsize(fp)
    return total / (1024 ** 3)


def build_manifest(session_dir, session_name=None):
    """Build a JSON-serializable manifest dict for a recording session."""
    if session_name is None:
        session_name = os.path.basename(session_dir)

    files = []
    for dirpath, _, filenames in os.walk(session_dir):
        for f in filenames:
            fp = os.path.join(dirpath, f)
            rel = os.path.relpath(fp, session_dir)
            files.append({
                'path': rel,
                'size_bytes': os.path.getsize(fp),
                'checksum': file_checksum(fp),
            })

    return {
        'session_name': session_name,
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'total_files': len(files),
        'total_size_gb': dir_size_gb(session_dir),
        'files': files,
    }


# ── ROS2 Node (requires rclpy + agv_greenhouse_msgs) ──

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from agv_greenhouse_msgs.srv import StartRecording, StopRecording
    from agv_greenhouse_msgs.msg import SessionInfo
    from nav_msgs.msg import Odometry
    import subprocess
    import signal
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
                '/zed/zed_node/left/image_rect_gray',
                '/zed/zed_node/right/image_rect_gray',
                '/zed/zed_node/left/camera_info',
                '/zed/zed_node/right/camera_info',
                '/zed/zed_node/imu/data',
                '/filtered/depth',
                '/filtered/rgb',
                '/filtered/camera_info',
                '/visual_slam/tracking/odometry',
                '/visual_slam/tracking/slam_path',
                '/tf',
                '/tf_static',
            ])
            self.declare_parameter('enable_svo', True)
            self.declare_parameter('enable_ply_save', True)
            self.declare_parameter('disk_warning_gb', 5.0)
            self.declare_parameter('disk_check_interval_sec', 10.0)

            self.output_dir = self.get_parameter('output_dir').value
            self.bag_topics = self.get_parameter('bag_topics').value
            self.enable_svo = self.get_parameter('enable_svo').value
            self.enable_ply_save = self.get_parameter('enable_ply_save').value
            self.disk_warning_gb = self.get_parameter('disk_warning_gb').value

            # State
            self.recording = False
            self.session_id = ''
            self.session_dir = ''
            self.bag_process = None
            self.tum_file = None
            self.start_time = 0.0
            self.frame_count = 0
            self._shutdown = False

            # Odom subscriber for TUM trajectory
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE, depth=1)
            self.odom_sub = self.create_subscription(
                Odometry, '/visual_slam/tracking/odometry',
                self.odom_callback, sensor_qos)

            # Services
            self.start_srv = self.create_service(
                StartRecording, '/session/start_recording', self.start_recording)
            self.stop_srv = self.create_service(
                StopRecording, '/session/stop_recording', self.stop_recording)

            # SessionInfo publisher
            self.info_pub = self.create_publisher(SessionInfo, '/session/info', 10)
            self.info_timer = self.create_timer(1.0, self.publish_info)

            # Disk space check timer
            interval = self.get_parameter('disk_check_interval_sec').value
            self.disk_timer = self.create_timer(interval, self.check_disk_space)

            # Graceful shutdown handler
            signal.signal(signal.SIGINT, self._signal_handler)
            signal.signal(signal.SIGTERM, self._signal_handler)

            os.makedirs(self.output_dir, exist_ok=True)
            self.get_logger().info(
                f'Session recorder ready. Output: {self.output_dir}')

        def _signal_handler(self, signum, frame):
            if self.recording and not self._shutdown:
                self._shutdown = True
                self.get_logger().warn('Signal received — stopping recording gracefully')
                self._do_stop()
            raise SystemExit(0)

        def odom_callback(self, msg):
            if not self.recording or self.tum_file is None:
                return
            stamp = msg.header.stamp
            t = stamp.sec + stamp.nanosec * 1e-9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.tum_file.write(
                f'{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} '
                f'{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n')
            self.tum_file.flush()
            self.frame_count += 1

        def start_recording(self, request, response):
            if self.recording:
                response.success = False
                response.message = 'Already recording'
                return response

            # Create session
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.session_id = request.session_name if request.session_name else f'session_{ts}'
            self.session_dir = os.path.join(self.output_dir, self.session_id)
            os.makedirs(self.session_dir, exist_ok=True)

            bag_dir = os.path.join(self.session_dir, 'bag')
            tum_path = os.path.join(self.session_dir, 'trajectory.tum')
            svo_path = os.path.join(self.session_dir, f'{self.session_id}.svo2')

            # 1. Start SVO2 recording via ZED service
            if self.enable_svo:
                try:
                    from zed_interfaces.srv import StartSvoRec
                    svo_cli = self.create_client(StartSvoRec, '/zed/zed_node/start_svo_rec')
                    if svo_cli.wait_for_service(timeout_sec=3.0):
                        svo_req = StartSvoRec.Request()
                        svo_req.svo_filename = svo_path
                        svo_req.compression_mode = 2  # H265
                        svo_req.target_framerate = 0  # Full rate
                        svo_cli.call_async(svo_req)
                        self.get_logger().info(f'SVO2 recording started: {svo_path}')
                    else:
                        self.get_logger().warn('ZED SVO service not available')
                except Exception as e:
                    self.get_logger().warn(f'SVO2 start failed: {e}')

            # 2. Start ros2 bag record
            cmd = ['ros2', 'bag', 'record', '-o', bag_dir] + self.bag_topics
            self.bag_process = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                env={**os.environ, 'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'})

            # 3. Open TUM trajectory file
            self.tum_file = open(tum_path, 'w')
            self.tum_file.write('# TUM trajectory: timestamp tx ty tz qx qy qz qw\n')

            self.recording = True
            self.start_time = time.time()
            self.frame_count = 0
            self._write_manifest()

            self.get_logger().info(f'Recording started: {self.session_dir}')
            response.success = True
            response.session_id = self.session_id
            response.session_path = self.session_dir
            response.message = f'Recording to {self.session_dir}'
            return response

        def stop_recording(self, request, response):
            if not self.recording:
                response.success = False
                response.message = 'Not recording'
                return response

            duration = self._do_stop()

            response.success = True
            response.session_path = self.session_dir
            response.duration_seconds = duration
            response.total_frames = self.frame_count
            response.message = (
                f'Session saved: {self.frame_count} frames, {duration:.1f}s')
            return response

        def _do_stop(self):
            duration = time.time() - self.start_time

            # 1. Stop SVO2
            if self.enable_svo:
                try:
                    from std_srvs.srv import Trigger
                    svo_cli = self.create_client(Trigger, '/zed/zed_node/stop_svo_rec')
                    if svo_cli.wait_for_service(timeout_sec=2.0):
                        svo_cli.call_async(Trigger.Request())
                        self.get_logger().info('SVO2 recording stopped')
                except Exception as e:
                    self.get_logger().warn(f'SVO2 stop failed: {e}')

            # 2. Stop bag
            if self.bag_process:
                self.bag_process.terminate()
                try:
                    self.bag_process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self.bag_process.kill()
                self.bag_process = None

            # 3. Close TUM file
            if self.tum_file:
                self.tum_file.close()
                self.tum_file = None

            # 4. Save nvblox PLY
            if self.enable_ply_save:
                try:
                    from nvblox_msgs.srv import FilePath
                    ply_cli = self.create_client(FilePath, '/nvblox_node/save_ply')
                    if ply_cli.wait_for_service(timeout_sec=2.0):
                        ply_req = FilePath.Request()
                        ply_req.file_path = os.path.join(self.session_dir, 'coverage.ply')
                        ply_cli.call_async(ply_req)
                        self.get_logger().info('nvblox PLY save requested')
                    else:
                        self.get_logger().warn('nvblox save_ply service not available')
                except Exception as e:
                    self.get_logger().warn(f'PLY save failed: {e}')

            self.recording = False
            self._write_manifest(duration=duration, finalize=True)
            self.get_logger().info(
                f'Recording stopped: {self.frame_count} frames, {duration:.1f}s')
            return duration

        def _file_checksum(self, path):
            return file_checksum(path)

        def _dir_size_gb(self, path):
            return dir_size_gb(path)

        def _write_manifest(self, duration=None, finalize=False):
            tum_path = os.path.join(self.session_dir, 'trajectory.tum')
            svo_path = os.path.join(self.session_dir, f'{self.session_id}.svo2')
            ply_path = os.path.join(self.session_dir, 'coverage.ply')

            manifest = {
                'session_id': self.session_id,
                'created': datetime.now().isoformat(),
                'output_dir': self.session_dir,
                'bag_dir': os.path.join(self.session_dir, 'bag'),
                'trajectory_file': tum_path,
                'svo_file': svo_path if os.path.exists(svo_path) else None,
                'ply_file': ply_path if os.path.exists(ply_path) else None,
                'bag_topics': self.bag_topics,
                'recording': self.recording,
                'frame_count': self.frame_count,
            }
            if duration is not None:
                manifest['duration_s'] = round(duration, 1)

            if finalize:
                manifest['finalized'] = True
                manifest['checksums'] = {}
                if os.path.exists(tum_path):
                    manifest['checksums']['trajectory.tum'] = self._file_checksum(tum_path)
                if os.path.exists(svo_path):
                    manifest['checksums'][f'{self.session_id}.svo2'] = self._file_checksum(svo_path)
                if os.path.exists(ply_path):
                    manifest['checksums']['coverage.ply'] = self._file_checksum(ply_path)
                manifest['total_size_gb'] = round(self._dir_size_gb(self.session_dir), 3)

            with open(os.path.join(self.session_dir, 'manifest.json'), 'w') as f:
                json.dump(manifest, f, indent=2)

        def publish_info(self):
            msg = SessionInfo()
            msg.stamp = self.get_clock().now().to_msg()
            msg.session_id = self.session_id
            msg.session_path = self.session_dir
            msg.recording_active = self.recording
            msg.frame_count = self.frame_count
            if self.recording:
                msg.duration_seconds = time.time() - self.start_time
                msg.disk_usage_gb = self._dir_size_gb(self.session_dir) if self.session_dir else 0.0
            try:
                stat = shutil.disk_usage(self.output_dir)
                msg.disk_free_gb = stat.free / (1024 ** 3)
            except Exception:
                msg.disk_free_gb = -1.0
            msg.status_message = 'Recording' if self.recording else 'Idle'
            self.info_pub.publish(msg)

        def check_disk_space(self):
            if not self.recording:
                return
            try:
                stat = shutil.disk_usage(self.output_dir)
                free_gb = stat.free / (1024 ** 3)
                if free_gb < self.disk_warning_gb:
                    self.get_logger().warn(
                        f'LOW DISK SPACE: {free_gb:.1f} GB free '
                        f'(threshold: {self.disk_warning_gb} GB)')
            except Exception:
                pass

    def main(args=None):
        rclpy.init(args=args)
        node = SessionRecorderNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, SystemExit):
            pass
        finally:
            if node.recording:
                node._do_stop()
            node.destroy_node()
            rclpy.shutdown()

except ImportError:
    # ROS2 not available — standalone utility functions still importable
    pass


if __name__ == '__main__':
    main()
