import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required")
    headless = LaunchConfiguration("headless")
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless", default_value="false",
            description=(
                "true loads runtime_teach/ into the hardware controller; neither mode "
                "runs Startup automatically")),
        Node(
            package="robot_controller", executable="robot_controller",
            name="robot_controller",
            parameters=[{"headless": ParameterValue(headless, value_type=bool)}],
            output="screen", on_exit=Shutdown(reason="Robot controller stopped")),
        Node(
            package="robot_controller", executable="robot_controller_preview",
            name="robot_controller_preview", condition=UnlessCondition(headless),
            output="screen", on_exit=Shutdown(reason="Controller preview stopped")),
        Node(
            package="robot_controller", executable="robot_controller_gui",
            name="robot_controller_gui", condition=UnlessCondition(headless),
            output="screen", on_exit=Shutdown(reason="Controller GUI stopped")),
    ])
