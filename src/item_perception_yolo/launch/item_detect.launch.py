import os

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("ROS_LOCALHOST_ONLY=1 is required")
    return LaunchDescription([Node(
        package="item_perception_yolo", executable="item_detect", name="item_detect",
        output="screen",
        on_exit=Shutdown(reason="Required item detector stopped"))])
