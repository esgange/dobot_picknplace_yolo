"""HMI-owned production processes; excludes the HMI and station dependencies."""

import os

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Source scripts/source_ros_workspace.bash first')
    return LaunchDescription([
        Node(package='robot_controller', executable='robot_controller', name='robot_controller',
             parameters=[{'headless': False}], output='screen', sigterm_timeout='30',
             sigkill_timeout='5', on_exit=Shutdown(reason='Production controller stopped')),
        Node(package='robot_controller', executable='robot_controller_preview',
             name='robot_controller_preview', output='screen',
             on_exit=Shutdown(reason='Controller preview stopped')),
    ])
