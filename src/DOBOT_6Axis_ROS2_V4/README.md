# DOBOT 6Axis ROS 2 V4 — project profile

Vendored Dobot ROS 2 V4 source for this project's standard CR10 robot. It is
derived from the official Dobot source snapshot recorded in the repository
blueprint diary and remains subject to the upstream [MIT License](LICENSE).

This hardware-only profile is deliberately limited to the CR10 description,
robot bringup, retained protocol interfaces, and actual-feedback visualization.
Gazebo/robot simulation, MoveIt, vendor demonstration nodes, `servo_action`,
and the streaming `ServoJ`/`ServoP` interfaces are excluded.

## Fixed platform and robot profile

| Requirement | Value |
| --- | --- |
| Operating system | Ubuntu 22.04 LTS |
| ROS distribution | ROS 2 Humble |
| Robot | Dobot CR10 |
| LAN1 controller IP | `192.168.20.204` |
| LAN2 diagnostic IP | `192.168.200.1` |
| Dashboard port | `29999` |
| Feedback port | `30004` |

## Build

Build this package group only as part of the repository workspace:

```bash
cd ~/PicknPlace
source /opt/ros/humble/setup.bash
colcon build
source scripts/source_ros_workspace.bash
```

The canonical root `.env` is required at runtime. It contains exactly these
project settings:

- `ROS_LOCALHOST_ONLY`
- `DOBOT_ROBOT_LAN1_IP`
- `DOBOT_ROBOT_LAN2_IP`
- `DOBOT_CONNECTION_TIMEOUT_MS`
- `DOBOT_ROBOT_TYPE`
- `DOBOT_ROBOT_NUMBER`
- `DOBOT_TRAJECTORY_DURATION`
- `DOBOT_ROBOT_NODE_NAME`

There is no package-local configuration or compatibility path.

## Retained packages

| Package | Role |
| --- | --- |
| `cra_description` | CR10 URDF/Xacro and visual/collision meshes |
| `dobot_bringup_v4` | Controller TCP/IP driver, feedback, and ROS services |
| `dobot_msgs_v4` | Dobot ROS messages and services |
| `dobot_rviz` | Read-only actual-feedback CR10 and ROS TF viewer |

## Safe entry points

Start controller bringup only after completing the real-robot safety and network
checks documented in the repository root README:

```bash
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

After bringup is connected and publishing actual joint feedback, visualize the
robot without commanding it:

```bash
ros2 launch dobot_rviz dobot_rviz.launch.py
```

The current feedback path is:

```text
Dobot controller TCP feedback
  → dobot_bringup_v4
  → /joint_states
  → robot_state_publisher and RViz
```

## Safety

The retained bringup services are capable of robot motion. Do not run a motion
executable or call a motion service unless the user explicitly requests it and
the robot/network/safety preconditions are checked.

## Upstream attribution

Original project: [Dobot-Arm/DOBOT_6Axis_ROS2_V4](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4)

The exact upstream commit, local pruning decisions, and verification record are
maintained in `docs/WORKFLOW_RULES_BLUEPRINT_DIARY.md` at the repository root.
