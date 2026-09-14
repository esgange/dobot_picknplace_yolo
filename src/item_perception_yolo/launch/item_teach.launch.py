import os

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required"
        )
    return LaunchDescription([
        Node(package="item_perception_yolo", executable="item_teach", name="item_teach",
             output="screen", on_exit=Shutdown(reason="Item teach stopped")),
    ])
