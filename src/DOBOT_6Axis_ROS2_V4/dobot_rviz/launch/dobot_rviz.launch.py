"""Optional display only; canonical bringup owns robot TF and joint monitoring."""

import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('[DOBOT RVIZ] ROS_LOCALHOST_ONLY must be exactly 1')
    rviz_config_path = get_package_share_path('dobot_rviz') / 'rviz' / 'urdf.rviz'
    if not rviz_config_path.is_file():
        raise RuntimeError(f'[DOBOT RVIZ] RViz configuration is missing: {rviz_config_path}')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='dobot_rviz',
            output='screen',
            arguments=['-d', str(rviz_config_path)],
            on_exit=[EmitEvent(event=Shutdown(reason='RViz exited'))],
        ),
    ])
