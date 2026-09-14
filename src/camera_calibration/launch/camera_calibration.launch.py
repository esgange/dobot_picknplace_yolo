import os

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError(
            '[CAMERA CALIBRATION] Source scripts/source_ros_workspace.bash first; '
            'ROS_LOCALHOST_ONLY must be exactly 1'
        )
    return LaunchDescription([
        Node(
            package='camera_calibration',
            executable='camera_calibration_gui',
            name='camera_calibration',
            output='screen',
            on_exit=Shutdown(reason='Camera calibration GUI stopped'),
        ),
    ])
