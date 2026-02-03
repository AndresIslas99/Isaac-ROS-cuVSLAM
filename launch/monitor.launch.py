# Monitor launch — live topic rates + TF inspection
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('agv_slam')

    return LaunchDescription([
        # RViz with pre-configured layout
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'slam.rviz')],
        ),

        # rqt for real-time monitoring
        Node(
            package='rqt_topic',
            executable='rqt_topic',
            name='rqt_topic',
        ),
    ])
