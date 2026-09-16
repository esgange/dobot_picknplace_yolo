# ROS 2 source packages

Every ROS 2 package in this workspace lives below `src/` and has a package-local `README.md` beside its `package.xml`. The vendor boundaries are intentional: they keep the official Dobot and Orbbec snapshots identifiable and make offline refreshes reviewable.

## Project packages

| Package | Role |
| --- | --- |
| [`motion_debug`](motion_debug/) | CR10 motion/status GUI; uses Dobot bringup services but never starts bringup |
| [`gripper_control`](gripper_control/) | Dobot DO1, DO2, DO13, and DO14 gripper/suction IO diagnostic GUI; requires live bringup |
| [`orbbec_camera_launcher`](orbbec_camera_launcher/) | Gemini 335 configuration GUI and strict three-attempt complete-set supervisor |
| [`camera_calibration`](camera_calibration/) | Manual-prefix ChArUco camera-to-hand (`base_link`) and camera-on-hand (`Link6`) calibration |
| [`item_perception_yolo`](item_perception_yolo/) | Fixed/on-hand ChArUco `platform_teach`, mode-matched four-marker `bin_teach`, and staged perception integration |
| [`robot_controller`](robot_controller/) | Deterministic headless Home/Pick hardware authority plus separate API-only GUI and TF-only preview |
| [`robot_controller_interfaces`](robot_controller_interfaces/) | Typed controller actions, lifecycle/configuration services, preview service, and transient-local status |
| [`item_perception_interfaces`](item_perception_interfaces/) | Ranked candidate message and read-only GetItemPoses service |

`item_perception_yolo` also installs the `item_teach` GUI and shared headless
`item_detect` runtime, with class checkboxes, RGB/depth preview and explicitly
armed calibrated pose requests. The controller can request/log a candidate
batch without robot execution. `robot_controller` requires explicit Startup in
both GUI and headless deployments; launch itself never enables or moves.
Imported `item_pick` is retained as reference source with `COLCON_IGNORE`;
its unaligned execution path is not part of the root build or installed runtime.

## Dobot CR10 packages

All packages in `DOBOT_6Axis_ROS2_V4/` come from the vendored Dobot ROS 2 V4 snapshot. The project profile retains only the standard CR10 robot assets.

| Package | Role |
| --- | --- |
| [`cra_description`](DOBOT_6Axis_ROS2_V4/cra_description/) | CR10 URDF/Xacro description and meshes |
| [`dobot_bringup_v4`](DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/) | Dobot TCP/IP ROS 2 driver and services |
| [`dobot_msgs_v4`](DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/) | Dobot messages and service definitions |
| [`dobot_rviz`](DOBOT_6Axis_ROS2_V4/dobot_rviz/) | CR10 RViz visualization |

Gazebo/robot simulation, MoveIt, vendor demonstration nodes, `servo_action`,
and the Dobot `ServoJ`/`ServoP` interfaces are intentionally absent from the
hardware-only CR10 profile.

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
colcon build
source scripts/source_ros_workspace.bash
```

Runtime event files are isolated under `logs/<package-name>/events.jsonl`; see [`logs/README.md`](../logs/README.md). Cross-package compilation is provided by `scripts/compile_logs.py`. Do not flatten or move a package out of its vendor group without recording the architecture change in [`docs/WORKFLOW_RULES_BLUEPRINT_DIARY.md`](../docs/WORKFLOW_RULES_BLUEPRINT_DIARY.md).
