import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required")
    return LaunchDescription([
        DeclareLaunchArgument("teach_file", default_value="",
                              description="Explicit Tray Teach selection; empty restores its session"),
        Node(package="tray_perception", executable="tray_teach", name="tray_teach",
             parameters=[{"teach_file": ParameterValue(
                 LaunchConfiguration("teach_file"), value_type=str)}],
             output="screen", on_exit=Shutdown(reason="Tray Teach stopped")),
    ])
