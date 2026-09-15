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
        DeclareLaunchArgument(
            "item_teach_file", default_value="", description="Explicit saved item YAML",
        ),
        DeclareLaunchArgument("bin_teach_file", default_value="", description="Portable bin YAML"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="Load runtime_teach/ and initialize permanently Live"),
        Node(package="robot_controller", executable="robot_controller", name="robot_controller",
             parameters=[{"item_teach_file": ParameterValue(
                 LaunchConfiguration("item_teach_file"), value_type=str,
             ), "bin_teach_file": ParameterValue(LaunchConfiguration("bin_teach_file"),
                                                 value_type=str),
                 "headless": ParameterValue(LaunchConfiguration("headless"), value_type=bool)}],
             output="screen", on_exit=Shutdown(reason="Robot controller stopped")),
    ])
