import os

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "[PLATFORM TEACH] Source scripts/source_ros_workspace.bash first; "
            "ROS_LOCALHOST_ONLY must be exactly 1"
        )
    return LaunchDescription(
        [
            Node(
                package="item_perception_yolo",
                executable="platform_teach",
                name="platform_teach",
                output="screen",
                on_exit=Shutdown(reason="Platform teach GUI stopped"),
            )
        ]
    )
