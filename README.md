# Dobot Pick-and-Place YOLO

ROS 2 workspace for a Dobot 6-axis robot and an Orbbec Gemini 335 depth camera. This initial bootstrap keeps the manufacturers' ROS 2 repositories as Git submodules so their history, licenses, and exact revisions remain available without copying or modifying upstream source.

## Workspace contents

```text
src/
├── DOBOT_6Axis_ROS2_V4/  # Dobot Robotics official ROS 2 SDK (main)
└── OrbbecSDK_ROS2/       # Orbbec official ROS 2 wrapper (v2-main)
```

| Component | Official repository | Tracked branch |
| --- | --- | --- |
| Dobot 6Axis ROS 2 V4 | [Dobot-Arm/DOBOT_6Axis_ROS2_V4](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4) | `main` |
| Orbbec ROS 2 wrapper | [orbbec/OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) | `v2-main` |

Orbbec's support matrix lists Gemini 335 under the Gemini 330 series. The `v2-main` branch is the recommended branch for new designs and provides the `gemini_330_series.launch.py` launch file.

## Clone this workspace

```bash
git clone --recurse-submodules https://github.com/esgange/dobot_picknplace_yolo.git
cd dobot_picknplace_yolo
```

If the repository was cloned without submodules, initialize them with:

```bash
git submodule update --init --recursive
```

The superproject records the exact upstream commit for each submodule. To deliberately move to newer upstream revisions, fetch and review the change before committing the updated submodule pointers:

```bash
git submodule update --remote --merge
git diff --submodule
```

## Build (Ubuntu 22.04 / ROS 2 Humble)

The Dobot SDK documents Ubuntu 22.04 with ROS 2 Humble. After sourcing ROS 2, install dependencies and build from the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Initial hardware checks

Set the Dobot variables for the robot model and network being used (the values below are the upstream examples):

```bash
export DOBOT_TYPE=cr5
export IP_address=192.168.5.1
```

For a Gemini 335 connected over USB, the official Orbbec wrapper provides:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

For first-time USB setup, install the udev rules supplied by Orbbec:

```bash
sudo bash src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh
```

Refer to the upstream READMEs for complete robot safety, networking, camera, and launch instructions. Pick-and-place and YOLO application packages will be added in subsequent steps.
