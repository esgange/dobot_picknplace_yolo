import os

from launch import LaunchDescription
from launch_ros.actions import Node

from orbbec_camera_launcher.project_config import load_project_config


def generate_launch_description():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError(
            '[ORBBEC CAMERA] Source scripts/source_ros_workspace.bash first; '
            'ROS_LOCALHOST_ONLY must be exactly 1'
        )
    load_project_config(require_camera_ready=False)
    return LaunchDescription([
        Node(
            package='orbbec_camera_launcher',
            executable='camera_launcher_gui',
            name='orbbec_camera_launcher',
            output='screen',
        ),
    ])
