from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # trajectory_execution_timeout 参数（接收父 launch 传递的参数）
    timeout_arg = DeclareLaunchArgument(
        name='trajectory_execution_timeout',
        default_value='120.0',
        description='Trajectory execution timeout in seconds'
    )

    return LaunchDescription([
        timeout_arg,
        Node(
            package='dobot_moveit',
            executable='action_move_server',
            parameters=[
                {'trajectory_execution_timeout': LaunchConfiguration('trajectory_execution_timeout')}
            ],
            output='screen',
        ),
    ])
