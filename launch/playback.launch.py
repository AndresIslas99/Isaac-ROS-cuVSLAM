# =============================================================================
# Playback Launch — Replay ROS2 Bag Through Nvblox
# =============================================================================
# Replays a recorded bag through nvblox for offline coverage review.
# Uses sim_time from bag playback.
#
# Usage:
#   ros2 launch agv_slam playback.launch.py bag_path:=/mnt/ssd/sessions/session_xxx/bag
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable,
    TimerAction, LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('agv_slam')

    declare_bag = DeclareLaunchArgument(
        'bag_path', description='Path to ROS2 bag directory')
    declare_rate = DeclareLaunchArgument(
        'playback_rate', default_value='1.0',
        description='Playback speed multiplier')

    set_dds = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    set_sim = SetParameter('use_sim_time', True)

    # Bag playback
    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            '--clock',
            '--rate', LaunchConfiguration('playback_rate'),
        ],
        output='screen',
    )

    # nvblox container for offline reconstruction
    nvblox_container = ComposableNodeContainer(
        name='nvblox_container', namespace='',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[], output='screen')

    load_nvblox = TimerAction(period=2.0, actions=[
        LoadComposableNodes(
            target_container='nvblox_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='nvblox_ros', plugin='nvblox::NvbloxNode',
                    name='nvblox_node',
                    parameters=[
                        os.path.join(pkg, 'config', 'nvblox.yaml'),
                        {'use_sim_time': True},
                    ],
                    remappings=[
                        ('camera_0/depth/image', '/zed/zed_node/depth/depth_registered'),
                        ('camera_0/depth/camera_info', '/zed/zed_node/depth/camera_info'),
                        ('camera_0/color/image', '/zed/zed_node/rgb/color/rect/image'),
                        ('camera_0/color/camera_info', '/zed/zed_node/rgb/camera_info'),
                    ])])])

    # Coverage monitor for playback analysis
    coverage = TimerAction(period=3.0, actions=[
        Node(package='agv_slam', executable='coverage_monitor.py',
             name='coverage_monitor', output='screen',
             parameters=[{'use_sim_time': True}])])

    # RViz
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'coverage.rviz')],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        declare_bag, declare_rate,
        set_dds, set_sim,
        LogInfo(msg='=== Playback Mode — Replaying bag through nvblox ==='),
        bag_play, nvblox_container, load_nvblox, coverage, rviz,
    ])
