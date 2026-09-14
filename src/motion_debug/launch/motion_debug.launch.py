from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start the motion-debug GUI only.

    The Dobot bringup is intentionally managed by the operator as a separate
    launch. The GUI reports when bringup services are unavailable; it never
    starts a robot driver implicitly.
    """
    return LaunchDescription([
        Node(
            package='motion_debug',
            executable='motion_debug_gui',
            name='motion_debug_gui',
            output='screen',
        ),
    ])
