# =============================================================================
# AGV SLAM Launch File — Complete Industrial Pipeline
# =============================================================================
# Launch order (handled by events):
#   1. ZED 2i Node (sensor driver)
#   2. Depth Filter (C++ preprocessing)
#   3. cuVSLAM (GPU visual-inertial odometry)
#   4. RTAB-Map (mapping + loop closure only)
#   5. SLAM Monitor (diagnostics)
#
# Usage:
#   ros2 launch agv_slam agv_slam.launch.py
#   ros2 launch agv_slam agv_slam.launch.py enable_rviz:=true
#   ros2 launch agv_slam agv_slam.launch.py localization:=true
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    GroupAction,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Package paths ──
    pkg_agv_slam = get_package_share_directory('agv_slam')
    pkg_zed_wrapper = get_package_share_directory('zed_wrapper')

    # ── Launch arguments ──
    declare_localization = DeclareLaunchArgument(
        'localization', default_value='false',
        description='true = localize on existing map, false = build new map')

    declare_rviz = DeclareLaunchArgument(
        'enable_rviz', default_value='false',
        description='Launch RViz for visualization')

    declare_database = DeclareLaunchArgument(
        'database_path',
        default_value='/home/orza/slam_maps/greenhouse.db',
        description='Path to RTAB-Map database file')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time')

    # ── Environment ──
    set_dds = SetEnvironmentVariable(
        'RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    set_cyclone_config = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        os.path.join(pkg_agv_slam, 'config', 'cyclonedds.xml'))

    # ── Global sim time ──
    set_sim_time = SetParameter('use_sim_time', LaunchConfiguration('use_sim_time'))

    # ══════════════════════════════════════════════
    # 1. ZED 2i CAMERA NODE
    # ══════════════════════════════════════════════
    zed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_zed_wrapper, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed',
            'config_path': os.path.join(pkg_agv_slam, 'config', 'zed2i.yaml'),
            'publish_urdf': 'true',
        }.items()
    )

    # ══════════════════════════════════════════════
    # 2. DEPTH FILTER NODE (C++)
    # ══════════════════════════════════════════════
    # Waits 3s for ZED to initialize
    depth_filter_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='agv_slam',
                executable='depth_filter_node',
                name='depth_filter',
                parameters=[
                    os.path.join(pkg_agv_slam, 'config', 'depth_filter.yaml'),
                ],
                remappings=[
                    # Input from ZED
                    ('/zed/zed_node/depth/depth_registered',
                     '/zed/zed_node/depth/depth_registered'),
                    ('/zed/zed_node/rgb/image_rect_color',
                     '/zed/zed_node/rgb/image_rect_color'),
                ],
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
            ),
        ]
    )

    # ══════════════════════════════════════════════
    # 3. ISAAC ROS cuVSLAM (GPU Visual-Inertial Odometry)
    # ══════════════════════════════════════════════
    # Waits 5s for ZED to be fully publishing
    cuvslam_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='isaac_ros_visual_slam',
                executable='visual_slam_node',
                name='visual_slam',
                parameters=[
                    os.path.join(pkg_agv_slam, 'config', 'cuvslam.yaml'),
                ],
                remappings=[
                    # Stereo input from ZED (rectified grayscale)
                    ('visual_slam/image_0', '/zed/zed_node/left/image_rect_gray'),
                    ('visual_slam/camera_info_0', '/zed/zed_node/left/camera_info'),
                    ('visual_slam/image_1', '/zed/zed_node/right/image_rect_gray'),
                    ('visual_slam/camera_info_1', '/zed/zed_node/right/camera_info'),
                    # IMU input
                    ('visual_slam/imu', '/zed/zed_node/imu/data'),
                ],
                output='screen',
            ),
        ]
    )

    # ══════════════════════════════════════════════
    # 4. RTAB-MAP (Mapping + Loop Closure Only)
    # ══════════════════════════════════════════════
    # Waits 8s for cuVSLAM to provide odometry
    #
    # Two modes:
    #   mapping:       Builds new map from scratch
    #   localization:  Localizes on existing map (database_path must exist)

    rtabmap_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                parameters=[
                    os.path.join(pkg_agv_slam, 'config', 'rtabmap.yaml'),
                    {
                        'database_path': LaunchConfiguration('database_path'),
                        # Localization mode: load existing map, don't add new nodes
                        'Mem/IncrementalMemory':
                            PythonExpression([
                                "'false' if '",
                                LaunchConfiguration('localization'),
                                "' == 'true' else 'true'"
                            ]),
                    },
                ],
                remappings=[
                    # RGB-D input (from depth filter)
                    ('rgb/image', '/filtered/rgb'),
                    ('depth/image', '/filtered/depth'),
                    ('rgb/camera_info', '/filtered/camera_info'),
                    # Odometry from cuVSLAM
                    ('odom', '/visual_slam/tracking/odometry'),
                    # IMU for gravity
                    ('imu', '/zed/zed_node/imu/data'),
                ],
                output='screen',
                arguments=[
                    '--delete_db_on_start',  # Remove for persistent mapping
                    '--ros-args', '--log-level', 'info',
                ],
            ),
        ]
    )

    # For localization mode, don't delete the database
    rtabmap_localization = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                parameters=[
                    os.path.join(pkg_agv_slam, 'config', 'rtabmap.yaml'),
                    {
                        'database_path': LaunchConfiguration('database_path'),
                        'Mem/IncrementalMemory': 'false',
                        'Mem/InitWMWithAllNodes': 'true',
                    },
                ],
                remappings=[
                    ('rgb/image', '/filtered/rgb'),
                    ('depth/image', '/filtered/depth'),
                    ('rgb/camera_info', '/filtered/camera_info'),
                    ('odom', '/visual_slam/tracking/odometry'),
                    ('imu', '/zed/zed_node/imu/data'),
                ],
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
            ),
        ]
    )

    # ══════════════════════════════════════════════
    # 5. SLAM MONITOR (Diagnostics)
    # ══════════════════════════════════════════════
    monitor_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='agv_slam',
                executable='slam_monitor_node',
                name='slam_monitor',
                output='screen',
            ),
        ]
    )

    # ══════════════════════════════════════════════
    # 6. RVIZ (Optional)
    # ══════════════════════════════════════════════
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_agv_slam, 'rviz', 'slam.rviz')],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    # ══════════════════════════════════════════════
    # STATIC TF: base_link → camera
    # ══════════════════════════════════════════════
    # Adjust these values to match your physical camera mount
    # x=forward, y=left, z=up from base_link center
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x', '0.1',        # Camera 10cm forward of base center
            '--y', '0.0',
            '--z', '0.3',        # Camera 30cm above base (adjust for your mount)
            '--roll', '0.0',
            '--pitch', '0.087',  # ~5° downward tilt (0.087 rad)
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'zed2i_base_link',
        ],
    )

    # ── Assembly ──
    return LaunchDescription([
        # Arguments
        declare_localization,
        declare_rviz,
        declare_database,
        declare_sim_time,

        # Environment
        set_dds,
        set_cyclone_config,
        set_sim_time,

        # Pipeline
        LogInfo(msg='=== AGV SLAM Pipeline Starting ==='),
        static_tf_base_to_camera,
        zed_node,
        depth_filter_node,
        cuvslam_node,

        # Choose mapping or localization mode
        GroupAction(
            actions=[rtabmap_node],
            condition=UnlessCondition(LaunchConfiguration('localization')),
        ),
        GroupAction(
            actions=[rtabmap_localization],
            condition=IfCondition(LaunchConfiguration('localization')),
        ),

        monitor_node,
        rviz_node,

        LogInfo(msg='=== All nodes launched. Monitor: /slam/diagnostics ==='),
    ])
