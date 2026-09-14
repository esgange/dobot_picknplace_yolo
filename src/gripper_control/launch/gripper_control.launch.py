from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gripper_control',
            executable='gripper_control_gui',
            name='gripper_control_gui',
            output='screen',
        ),
    ])
