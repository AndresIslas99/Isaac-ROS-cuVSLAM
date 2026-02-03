# =============================================================================
# AGV SLAM Launch File — cuVSLAM + nvblox Pipeline
# =============================================================================
# Launch order (handled by events):
#   1. ZED 2i Node (sensor driver)
#   2. Depth Filter (C++ preprocessing)
#   3. cuVSLAM (GPU visual-inertial odometry)
#   4. nvblox (GPU TSDF mesh reconstruction)
#   5. Pipeline Watchdog (crash recovery)
#   6. SLAM Monitor (diagnostics)
#   7. Foxglove Bridge (optional web visualization)
#
# Usage:
#   ros2 launch agv_slam agv_slam.launch.py
#   ros2 launch agv_slam agv_slam.launch.py enable_rviz:=true
#   ros2 launch agv_slam agv_slam.launch.py enable_foxglove:=true
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
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
    declare_rviz = DeclareLaunchArgument(
        'enable_rviz', default_value='false',
        description='Launch RViz for visualization')

    declare_foxglove = DeclareLaunchArgument(
        'enable_foxglove', default_value='true',
        description='Launch Foxglove Bridge for web visualization')

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
    # 1. SLAM CONTAINER (shared by ZED + cuVSLAM + nvblox)
    # ══════════════════════════════════════════════
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
            'publish_imu_tf': 'true',
        }.items()
    )

    # ══════════════════════════════════════════════
    # 3. DEPTH FILTER NODE (C++) — with respawn
    # ══════════════════════════════════════════════
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
                            ('visual_slam/image_0', '/zed/zed_node/left/gray/rect/image'),
                            ('visual_slam/camera_info_0', '/zed/zed_node/left/gray/rect/camera_info'),
                            ('visual_slam/image_1', '/zed/zed_node/right/gray/rect/image'),
                            ('visual_slam/camera_info_1', '/zed/zed_node/right/gray/rect/camera_info'),
                            ('visual_slam/imu', '/zed/zed_node/imu/data'),
                        ],
                    ),
                ],
            ),
        ]
    )

    # ══════════════════════════════════════════════
    # 5. NVBLOX (GPU TSDF Mesh Reconstruction)
    # ══════════════════════════════════════════════
    # Loaded into slam_container for intra-process transport.
    # Uses TF from cuVSLAM (odom -> base_link -> zed_camera_link).
    # Subscribes to ZED depth + color directly (GPU TSDF doesn't
    # need our CPU depth filter preprocessing).
    load_nvblox = TimerAction(
        period=8.0,
        actions=[
            LoadComposableNodes(
                target_container='/zed/slam_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nvblox_ros',
                        plugin='nvblox::NvbloxNode',
                        name='nvblox_node',
                        parameters=[
                            os.path.join(pkg_agv_slam, 'config', 'nvblox.yaml'),
                        ],
                        remappings=[
                            # Depth from ZED
                            ('camera_0/depth/image', '/zed/zed_node/depth/depth_registered'),
                            ('camera_0/depth/camera_info', '/zed/zed_node/depth/camera_info'),
                            # Color from ZED (SDK v5.1+ topic naming)
                            ('camera_0/color/image', '/zed/zed_node/rgb/color/rect/image'),
                            ('camera_0/color/camera_info', '/zed/zed_node/rgb/camera_info'),
                        ],
                    ),
                ],
            ),
        ]
    )

    # ══════════════════════════════════════════════
    # 6. PIPELINE WATCHDOG (Crash Recovery)
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
    # 7. SLAM MONITOR (Diagnostics)
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
    # 8. COVERAGE MONITOR (Python)
    # ══════════════════════════════════════════════
    coverage_monitor = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='agv_slam',
                executable='coverage_monitor.py',
                name='coverage_monitor',
                output='screen',
                parameters=[{
                    'target_area_m2': 0.0,  # Set to greenhouse area for % tracking
                    'publish_rate_hz': 0.5,
                }],
            ),
        ]
    )

    # ══════════════════════════════════════════════
    # 9. SESSION RECORDER (Python)
    # ══════════════════════════════════════════════
    session_recorder = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='agv_slam',
                executable='session_recorder.py',
                name='session_recorder',
                output='screen',
                parameters=[{
                    'output_dir': '/mnt/ssd/sessions',
                }],
            ),
        ]
    )

    # ══════════════════════════════════════════════
    # 10. FOXGLOVE BRIDGE (Optional Web Visualization)
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
                # ZED raw
                '/zed/zed_node/imu/data',
                '/zed/zed_node/left/gray/rect/image',
                '/zed/zed_node/right/gray/rect/image',
                '/zed/zed_node/rgb/color/rect/image',
                '/zed/zed_node/depth/depth_registered',
                '/zed/zed_node/point_cloud/cloud_registered',
                # Depth filter
                '/filtered/rgb',
                '/filtered/depth',
                '/filtered/camera_info',
                # cuVSLAM
                '/visual_slam/tracking/odometry',
                '/visual_slam/tracking/slam_path',
                '/visual_slam/tracking/vo_path',
                '/visual_slam/tracking/vo_pose',
                '/visual_slam/status',
                '/visual_slam/vis/landmarks_cloud',
                '/visual_slam/vis/observations_cloud',
                # nvblox outputs
                '/nvblox_node/mesh',
                '/nvblox_node/static_map_slice',
                '/nvblox_node/mesh_marker',
                '/nvblox_node/static_esdf_pointcloud',
                '/nvblox_node/static_tsdf_pointcloud',
                '/nvblox_node/combined_esdf_pointcloud',
                '/nvblox_node/back_projected_depth',
                '/nvblox_node/map_slice_bounds',
                # Coverage + Session
                '/coverage/status',
                '/coverage/json',
                '/session/status',
                # Diagnostics
                '/slam/diagnostics',
                '/slam/quality',
                '/watchdog/heartbeat',
                # TF
                '/tf',
                '/tf_static',
            ],
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_foxglove')),
    )

    # ══════════════════════════════════════════════
    # 9. RVIZ (Optional)
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
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x', '0.1',
            '--y', '0.0',
            '--z', '0.3',
            '--roll', '0.0',
            '--pitch', '0.087',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'zed_camera_link',
        ],
    )

    # ── Assembly ──
    return LaunchDescription([
        # Arguments
        declare_rviz,
        declare_foxglove,
        declare_sim_time,

        # Environment
        set_dds,
        set_cyclone_config,
        set_gxf_libs,
        set_sim_time,

        # Pipeline
        LogInfo(msg='=== AGV SLAM + nvblox Pipeline Starting ==='),
        static_tf_base_to_camera,
        slam_container,
        zed_node,
        depth_filter_node,
        load_cuvslam,
        load_nvblox,

        watchdog_node,
        monitor_node,
        coverage_monitor,
        session_recorder,
        foxglove_bridge,
        rviz_node,

        LogInfo(msg='=== All nodes launched. Monitor: /slam/diagnostics ==='),
        LogInfo(msg='=== nvblox mesh: /nvblox_node/mesh ==='),
        LogInfo(msg='=== Foxglove: ws://192.168.55.1:8765 (if enabled) ==='),
    ])
