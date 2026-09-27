import os

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required")
    return LaunchDescription([
        Node(package="tray_perception", executable="tray_teach", name="tray_teach",
             output="screen", on_exit=Shutdown(reason="Tray Teach stopped")),
    ])
