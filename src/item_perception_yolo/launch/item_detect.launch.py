import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("ROS_LOCALHOST_ONLY=1 is required")
    arguments = [DeclareLaunchArgument(name) for name in
                 ("item_teach_file", "platform_teach_file", "bin_teach_file")]
    arguments += [DeclareLaunchArgument("armed", default_value="false"),
                  DeclareLaunchArgument("trusted_model", default_value="false")]
    values = {name: ParameterValue(LaunchConfiguration(name), value_type=str) for name in
              ("item_teach_file", "platform_teach_file", "bin_teach_file")}
    values.update({name: ParameterValue(LaunchConfiguration(name), value_type=bool)
                   for name in ("armed", "trusted_model")})
    return LaunchDescription(arguments + [Node(package="item_perception_yolo",
                                               executable="item_detect", name="item_detect", output="screen", parameters=[values],
                                               on_exit=Shutdown(reason="Required item detector stopped"))])
