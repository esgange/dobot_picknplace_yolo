import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch(context):
    headless = LaunchConfiguration("headless")
    dependencies = LaunchConfiguration("start_dependencies")
    if (not ParameterValue(headless, value_type=bool).evaluate(context)
            and ParameterValue(dependencies, value_type=bool).evaluate(context)):
        return [Node(
            package="robot_controller", executable="robot_controller_station",
            output="screen", sigterm_timeout="145", sigkill_timeout="5",
            on_exit=Shutdown(reason="Offline station stopped"))]
    return [
        Node(
            package="robot_controller", executable="robot_controller",
            name="robot_controller",
            parameters=[{"headless": ParameterValue(headless, value_type=bool)}],
            output="screen", sigterm_timeout="30", sigkill_timeout="5",
            on_exit=Shutdown(reason="Robot controller stopped")),
        Node(
            package="robot_controller", executable="robot_controller_preview",
            name="robot_controller_preview", condition=UnlessCondition(headless),
            output="screen", on_exit=Shutdown(reason="Controller preview stopped")),
        Node(
            package="robot_controller", executable="robot_controller_gui",
            name="robot_controller_gui", condition=UnlessCondition(headless),
            output="screen", sigterm_timeout="30", sigkill_timeout="5",
            on_exit=Shutdown(reason="Controller GUI stopped")),
    ]


def generate_launch_description():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required")
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless", default_value="false",
            description=(
                "true starts only the hardware controller with runtime_teach/ loaded; "
                "neither mode runs Startup automatically")),
        DeclareLaunchArgument(
            "start_dependencies", default_value="true",
            description=(
                "GUI mode starts the offline station with bringup/robot TF and supervised "
                "cameras. Set false only when those services are managed separately.")),
        OpaqueFunction(function=_launch),
    ])
