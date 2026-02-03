# =============================================================================
# Recording-Only Launch — No Nvblox (lighter GPU usage)
# =============================================================================
# Launches: ZED → Depth Filter → cuVSLAM → Session Recorder → Diagnostics
# Skips: nvblox, coverage monitor (saves ~20% GPU for recording quality)
#
# Usage:
#   ros2 launch agv_slam recording_only.launch.py
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction, LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('agv_slam')
    pkg_zed = get_package_share_directory('zed_wrapper')

    declare_foxglove = DeclareLaunchArgument(
        'enable_foxglove', default_value='true')
    declare_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_cam_x = DeclareLaunchArgument('camera_x', default_value='0.1')
    declare_cam_y = DeclareLaunchArgument('camera_y', default_value='0.0')
    declare_cam_z = DeclareLaunchArgument('camera_z', default_value='0.3')
    declare_cam_pitch = DeclareLaunchArgument('camera_pitch', default_value='0.087')

    set_dds = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    set_cyclone = SetEnvironmentVariable(
        'CYCLONEDDS_URI', os.path.join(pkg, 'config', 'cyclonedds.xml'))

    gxf_base = os.path.join(
        get_package_share_directory('isaac_ros_gxf'), 'gxf', 'lib')
    gxf_paths = ':'.join([
        os.path.join(gxf_base, d)
        for d in ['core', 'std', 'cuda', 'serialization', 'multimedia', 'npp', 'network']
        if os.path.isdir(os.path.join(gxf_base, d))])
    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    set_gxf = SetEnvironmentVariable(
        'LD_LIBRARY_PATH', gxf_paths + ':' + existing_ld if existing_ld else gxf_paths)

    set_sim = SetParameter('use_sim_time', LaunchConfiguration('use_sim_time'))

    slam_container = ComposableNodeContainer(
        name='slam_container', namespace='zed',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[], output='screen')

    zed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_zed, 'launch', 'zed_camera.launch.py')),
        launch_arguments={
            'camera_model': 'zed2i', 'camera_name': 'zed',
            'container_name': 'slam_container',
            'ros_params_override_path': os.path.join(pkg, 'config', 'zed2i.yaml'),
            'publish_urdf': 'true', 'publish_tf': 'false',
            'publish_map_tf': 'false', 'publish_imu_tf': 'true',
        }.items())

    depth_filter = TimerAction(period=3.0, actions=[
        Node(package='agv_slam', executable='depth_filter_node',
             name='depth_filter',
             parameters=[os.path.join(pkg, 'config', 'depth_filter.yaml')],
             remappings=[
                 ('/zed/zed_node/depth/depth_registered',
                  '/zed/zed_node/depth/depth_registered'),
                 ('/zed/zed_node/rgb/image_rect_color',
                  '/zed/zed_node/rgb/color/rect/image')],
             output='screen', respawn=True, respawn_delay=5.0)])

    load_cuvslam = TimerAction(period=5.0, actions=[
        LoadComposableNodes(
            target_container='/zed/slam_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='isaac_ros_visual_slam',
                    plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                    name='visual_slam',
                    parameters=[os.path.join(pkg, 'config', 'cuvslam.yaml')],
                    remappings=[
                        ('visual_slam/image_0', '/zed/zed_node/left/gray/rect/image'),
                        ('visual_slam/camera_info_0', '/zed/zed_node/left/gray/rect/camera_info'),
                        ('visual_slam/image_1', '/zed/zed_node/right/gray/rect/image'),
                        ('visual_slam/camera_info_1', '/zed/zed_node/right/gray/rect/camera_info'),
                        ('visual_slam/imu', '/zed/zed_node/imu/data')])])])

    session_recorder = TimerAction(period=8.0, actions=[
        Node(package='agv_slam', executable='session_recorder.py',
             name='session_recorder', output='screen',
             parameters=[{'output_dir': '/mnt/ssd/sessions'}])])

    monitor = TimerAction(period=8.0, actions=[
        Node(package='agv_slam', executable='slam_monitor_node',
             name='slam_monitor', output='screen')])

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--roll', '0.0',
            '--pitch', LaunchConfiguration('camera_pitch'),
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'zed_camera_link'])

    foxglove = Node(
        package='foxglove_bridge', executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{'port': 8765, 'address': '0.0.0.0',
                      'send_buffer_limit': 100000000}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_foxglove')))

    return LaunchDescription([
        declare_foxglove, declare_sim,
        declare_cam_x, declare_cam_y, declare_cam_z, declare_cam_pitch,
        set_dds, set_cyclone, set_gxf, set_sim,
        LogInfo(msg='=== Recording-Only Pipeline (no nvblox) ==='),
        static_tf, slam_container, zed_node,
        depth_filter, load_cuvslam,
        session_recorder, monitor, foxglove,
        LogInfo(msg='=== Ready for recording. Use /session/start_recording ==='),
    ])
