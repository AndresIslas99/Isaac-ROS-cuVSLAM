# =============================================================================
# AGV SLAM Launch File — Industrial-Grade Pipeline with Crash Recovery
# =============================================================================
# Launch order (handled by events):
#   1. ZED 2i Node (sensor driver)
#   2. Depth Filter (C++ preprocessing)
#   3. cuVSLAM (GPU visual-inertial odometry)
#   4. RTAB-Map (mapping + loop closure only)
#   5. Pipeline Watchdog (crash recovery)
#   6. SLAM Monitor (diagnostics)
#   7. Foxglove Bridge (optional web visualization)
#
# Usage:
#   ros2 launch agv_slam agv_slam.launch.py
#   ros2 launch agv_slam agv_slam.launch.py enable_rviz:=true
#   ros2 launch agv_slam agv_slam.launch.py localization:=true
#   ros2 launch agv_slam agv_slam.launch.py enable_foxglove:=true
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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

    declare_foxglove = DeclareLaunchArgument(
        'enable_foxglove', default_value='false',
        description='Launch Foxglove Bridge for web visualization')

    declare_database = DeclareLaunchArgument(
        'database_path',
        default_value='/mnt/ssd/slam_maps/greenhouse.db',
        description='Path to RTAB-Map database file (NVMe SSD for performance)')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time')

    # ── Environment ──
    set_dds = SetEnvironmentVariable(
        'RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    set_cyclone_config = SetEnvironmentVariable(
        'CYCLONEDDS_URI',
        os.path.join(pkg_agv_slam, 'config', 'cyclonedds.xml'))

    # Isaac ROS GXF libraries needed by cuVSLAM
    gxf_lib_base = os.path.join(
        get_package_share_directory('isaac_ros_gxf'), 'gxf', 'lib')
    gxf_lib_paths = ':'.join([
        os.path.join(gxf_lib_base, d)
        for d in ['core', 'std', 'cuda', 'serialization', 'multimedia',
                  'npp', 'network', 'behavior_tree']
        if os.path.isdir(os.path.join(gxf_lib_base, d))
    ])
    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    set_gxf_libs = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        gxf_lib_paths + ':' + existing_ld if existing_ld else gxf_lib_paths)

    # ── Global sim time ──
    set_sim_time = SetParameter('use_sim_time', LaunchConfiguration('use_sim_time'))

    # ══════════════════════════════════════════════
    # 1. SLAM CONTAINER (shared by ZED + cuVSLAM)
    # ══════════════════════════════════════════════
    # Both ZED and cuVSLAM run in the same container for intra-process
    # zero-copy image transport (NITROS-compatible).
    slam_container = ComposableNodeContainer(
        name='slam_container',
        namespace='zed',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[],
        output='screen',
    )

    # ══════════════════════════════════════════════
    # 2. ZED 2i CAMERA NODE (loaded into slam_container)
    # ══════════════════════════════════════════════
    zed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_zed_wrapper, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed',
            'container_name': 'slam_container',
            'ros_params_override_path': os.path.join(pkg_agv_slam, 'config', 'zed2i.yaml'),
            'publish_urdf': 'true',
            'publish_tf': 'false',
            'publish_map_tf': 'false',
        }.items()
    )

    # ══════════════════════════════════════════════
    # 3. DEPTH FILTER NODE (C++) — with respawn
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
                    # Input from ZED — topic names match ZED SDK v5.1+
                    ('/zed/zed_node/depth/depth_registered',
                     '/zed/zed_node/depth/depth_registered'),
                    ('/zed/zed_node/rgb/image_rect_color',
                     '/zed/zed_node/rgb/color/rect/image'),
                ],
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                respawn=True,
                respawn_delay=5.0,
            ),
        ]
    )

    # ══════════════════════════════════════════════
    # 4. ISAAC ROS cuVSLAM (GPU Visual-Inertial Odometry)
    # ══════════════════════════════════════════════
    # Loaded into slam_container for intra-process zero-copy transport.
    # Waits 5s for ZED to be fully publishing.
    load_cuvslam = TimerAction(
        period=5.0,
        actions=[
            LoadComposableNodes(
                target_container='/zed/slam_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='isaac_ros_visual_slam',
                        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                        name='visual_slam',
                        parameters=[
                            os.path.join(pkg_agv_slam, 'config', 'cuvslam.yaml'),
                        ],
                        remappings=[
                            # Stereo input from ZED (rectified grayscale) — ZED SDK v5.1+
                            ('visual_slam/image_0', '/zed/zed_node/left/gray/rect/image'),
                            ('visual_slam/camera_info_0', '/zed/zed_node/left/gray/rect/camera_info'),
                            ('visual_slam/image_1', '/zed/zed_node/right/gray/rect/image'),
                            ('visual_slam/camera_info_1', '/zed/zed_node/right/gray/rect/camera_info'),
                            # IMU input
                            ('visual_slam/imu', '/zed/zed_node/imu/data'),
                        ],
                    ),
                ],
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
                        'Mem/IncrementalMemory': 'true',
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
    # 5. PIPELINE WATCHDOG (Crash Recovery)
    # ══════════════════════════════════════════════
    watchdog_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='agv_slam',
                executable='pipeline_watchdog_node',
                name='pipeline_watchdog',
                output='screen',
                parameters=[{
                    'log_directory': '/mnt/ssd/slam_logs/',
                    'heartbeat_rate': 1.0,
                    'node_timeout_sec': 10.0,
                }],
                respawn=True,
                respawn_delay=5.0,
            ),
        ]
    )

    # ══════════════════════════════════════════════
    # 6. SLAM MONITOR (Diagnostics)
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
    # 7. FOXGLOVE BRIDGE (Optional Web Visualization)
    # ══════════════════════════════════════════════
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 100000000,
            'topic_whitelist': [
                '/filtered/rgb',
                '/filtered/depth',
                '/filtered/camera_info',
                '/visual_slam/tracking/odometry',
                '/visual_slam/tracking/slam_path',
                '/rtabmap/grid_map',
                '/rtabmap/cloud_map',
                '/rtabmap/mapData',
                '/slam/diagnostics',
                '/slam/quality',
                '/watchdog/heartbeat',
                '/tf',
                '/tf_static',
            ],
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_foxglove')),
    )

    # ══════════════════════════════════════════════
    # 8. RVIZ (Optional)
    # ══════════════════════════════════════════════
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_agv_slam, 'rviz', 'slam.rviz')],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    # ══════════════════════════════════════════════
    # STATIC TF: base_link -> camera
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
            '--pitch', '0.087',  # ~5 deg downward tilt (0.087 rad)
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
        declare_foxglove,
        declare_database,
        declare_sim_time,

        # Environment
        set_dds,
        set_cyclone_config,
        set_gxf_libs,
        set_sim_time,

        # Pipeline
        LogInfo(msg='=== AGV SLAM Industrial Pipeline Starting ==='),
        static_tf_base_to_camera,
        slam_container,
        zed_node,
        depth_filter_node,
        load_cuvslam,

        # Choose mapping or localization mode
        GroupAction(
            actions=[rtabmap_node],
            condition=UnlessCondition(LaunchConfiguration('localization')),
        ),
        GroupAction(
            actions=[rtabmap_localization],
            condition=IfCondition(LaunchConfiguration('localization')),
        ),

        watchdog_node,
        monitor_node,
        foxglove_bridge,
        rviz_node,

        LogInfo(msg='=== All nodes launched. Monitor: /slam/diagnostics ==='),
        LogInfo(msg='=== Foxglove: ws://192.168.50.100:8765 (if enabled) ==='),
    ])
