import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch(context):
    headless = LaunchConfiguration("headless")
    dependencies = LaunchConfiguration("start_dependencies")
    cameras = ParameterValue(LaunchConfiguration('require_camera_watchdog'), value_type=bool)
    if ParameterValue(headless, value_type=bool).evaluate(context):
        return [Node(
            package="robot_controller", executable="robot_controller", name="robot_controller",
            parameters=[{"headless": True, 'require_camera_watchdog': cameras}], output="screen",
            sigterm_timeout="30", sigkill_timeout="5",
            on_exit=Shutdown(reason="Robot controller stopped"))]
    owned = ParameterValue(dependencies, value_type=bool).evaluate(context)
    return [
        Node(
            package="robot_controller", executable="robot_controller_gui",
            name="robot_controller_gui", parameters=[{"manage_controller": True,
                                                     "manage_dependencies": owned,
                                                     'require_camera_watchdog': True if owned else cameras}],
            output="screen", sigterm_timeout="200", sigkill_timeout="5",
            on_exit=Shutdown(reason="Controller GUI stopped")),
    ]


def generate_launch_description():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required")
    return LaunchDescription([
        DeclareLaunchArgument('require_camera_watchdog', default_value='false',
                              description='Require fresh owned camera supervisor health'),
        DeclareLaunchArgument(
            "headless", default_value="false",
            description=(
                "true starts only the hardware controller with runtime_teach/ loaded; "
                "neither mode runs Startup automatically")),
        DeclareLaunchArgument(
            "start_dependencies", default_value="true",
            description=(
                "Initialize Station starts owned bringup/TF and cameras after the HMI opens. "
                "Set false when those services are managed separately.")),
        OpaqueFunction(function=_launch),
    ])
