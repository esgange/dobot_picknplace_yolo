# ROS 2 source packages

Every ROS 2 package in this workspace lives below `src/` and has a package-local `README.md` beside its `package.xml`. The vendor boundaries are intentional: they keep the official Dobot and Orbbec snapshots identifiable and make offline refreshes reviewable.

## Dobot CR10 packages

All packages in `DOBOT_6Axis_ROS2_V4/` come from the vendored Dobot ROS 2 V4 snapshot. The project profile retains only the standard CR10 robot assets.

| Package | Role |
| --- | --- |
| [`cr10_moveit`](DOBOT_6Axis_ROS2_V4/cr10_moveit/) | CR10 MoveIt planning and controller configuration |
| [`cra_description`](DOBOT_6Axis_ROS2_V4/cra_description/) | CR10 URDF/Xacro description and meshes |
| [`dobot_bringup_v4`](DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/) | Dobot TCP/IP ROS 2 driver and services |
| [`dobot_demo`](DOBOT_6Axis_ROS2_V4/dobot_demo/) | Python demonstration nodes |
| [`dobot_gazebo`](DOBOT_6Axis_ROS2_V4/dobot_gazebo/) | Gazebo simulation launchers and world |
| [`dobot_kinematics_plugin`](DOBOT_6Axis_ROS2_V4/dobot_kinematics_plugin/) | MoveIt kinematics plugin |
| [`dobot_moveit`](DOBOT_6Axis_ROS2_V4/dobot_moveit/) | MoveIt launchers and motion action helpers |
| [`dobot_msgs_v4`](DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/) | Dobot messages and service definitions |
| [`dobot_rviz`](DOBOT_6Axis_ROS2_V4/dobot_rviz/) | CR10 RViz visualization |
| [`servo_action`](DOBOT_6Axis_ROS2_V4/servo_action/) | Joint trajectory action client |

## Orbbec packages

All packages in `OrbbecSDK_ROS2/` come from the vendored Orbbec ROS 2 wrapper snapshot.

| Package | Role |
| --- | --- |
| [`orbbec_camera`](OrbbecSDK_ROS2/orbbec_camera/) | Orbbec camera driver and Gemini 330-series launchers |
| [`orbbec_camera_msgs`](OrbbecSDK_ROS2/orbbec_camera_msgs/) | Camera messages and service definitions |
| [`orbbec_description`](OrbbecSDK_ROS2/orbbec_description/) | Camera URDF/Xacro descriptions and meshes |

Build from the workspace root so `colcon` discovers all package manifests:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Runtime event files are isolated under `logs/<package-name>/events.jsonl`; see [`logs/README.md`](../logs/README.md). Cross-package compilation is provided by `scripts/compile_logs.py`. Do not flatten or move a package out of its vendor group without recording the architecture change in [`docs/WORKFLOW_RULES_BLUEPRINT_DIARY.md`](../docs/WORKFLOW_RULES_BLUEPRINT_DIARY.md).
