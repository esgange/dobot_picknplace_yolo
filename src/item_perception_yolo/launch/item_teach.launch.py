import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required"
        )
    return LaunchDescription([
        DeclareLaunchArgument("teach_file", default_value="",
                              description="Explicit Item Teach selection; empty restores its session"),
        DeclareLaunchArgument("bin_teach_file", default_value="",
                              description="Explicit Bin Teach selection for Item Teach"),
        Node(package="item_perception_yolo", executable="item_teach", name="item_teach",
             parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=str)
                          for name in ("teach_file", "bin_teach_file")}],
             output="screen", on_exit=Shutdown(reason="Item teach stopped")),
        Node(package="item_perception_yolo", executable="robot_camera_box",
             name="robot_camera_box", output="screen"),
    ])
