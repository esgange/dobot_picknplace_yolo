"""Dependencies started only by the HMI's explicit Initialize Station action."""

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_controller', executable='robot_controller_station',
             output='screen', sigterm_timeout='100', sigkill_timeout='5',
             on_exit=Shutdown(reason='Station dependencies stopped')),
    ])
