import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context):
    mode = LaunchConfiguration('headless').perform(context)
    if mode not in ('true', 'false'):
        raise RuntimeError('headless must be true or false')
    return [Node(package='remote_setup', executable='remote_setup', name='remote_setup',
                 arguments=['--headless'] if mode == 'true' else [], output='screen',
                 on_exit=Shutdown(reason='Remote Setup exited'))]


def generate_launch_description():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Source scripts/source_ros_workspace.bash first')
    return LaunchDescription([DeclareLaunchArgument('headless', default_value='false'),
                              OpaqueFunction(function=_node)])
