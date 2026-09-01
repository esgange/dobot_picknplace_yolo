# Dobot Pick-and-Place YOLO

ROS 2 workspace for a Dobot CR10 robot and an Orbbec Gemini 335 depth camera. The Dobot source is an intentionally pruned CR10-only vendor profile; the Orbbec source is vendored in full. A transferred copy therefore contains the project source without requiring network access.

## Workspace contents

```text
src/
├── DOBOT_6Axis_ROS2_V4/  # Dobot official SDK, pruned to CR10
└── OrbbecSDK_ROS2/       # Orbbec official ROS 2 wrapper snapshot
```

| Component | Official repository | Snapshot |
| --- | --- | --- |
| Dobot 6Axis ROS 2 V4 (CR10 profile) | [Dobot-Arm/DOBOT_6Axis_ROS2_V4](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4) | `main` at `def21d05`, locally pruned |
| Orbbec ROS 2 wrapper | [orbbec/OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) | `v2-main` at `8e7cad2b` |

Orbbec's support matrix lists Gemini 335 under the Gemini 330 series. The `v2-main` branch is the recommended branch for new designs and provides the `gemini_330_series.launch.py` launch file.

## Package organization

All 13 ROS packages are below `src/`, grouped under the official vendor snapshot they came from. Each package has a package-local README describing its role and safe entry points. See [`src/README.md`](src/README.md) for the complete package index. The vendor grouping is intentional and must remain intact for offline provenance and refreshes.

## Clone this workspace

```bash
git clone https://github.com/esgange/dobot_picknplace_yolo.git
cd dobot_picknplace_yolo
```

No submodule initialization or network access is required after cloning this repository. The vendored sources are ordinary tracked files. Preserve the upstream `LICENSE`, `NOTICE`, and README files when updating them. Do not reintroduce non-CR10 Dobot model configurations unless the project scope is explicitly changed and recorded in the blueprint diary.

## Updating vendored sources (online maintenance only)

Vendor updates must be deliberate and reviewable. Use a separate temporary clone of the official repository, compare it with the current snapshot, then commit the resulting source changes together with an entry in [`docs/WORKFLOW_RULES_BLUEPRINT_DIARY.md`](docs/WORKFLOW_RULES_BLUEPRINT_DIARY.md). Do not add the vendor repositories back as submodules.

```bash
tmp_dir="$(mktemp -d)"
git clone --branch main https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4.git "$tmp_dir/dobot"
git clone --branch v2-main https://github.com/orbbec/OrbbecSDK_ROS2.git "$tmp_dir/orbbec"
# Review differences before copying; keep any project-specific changes out of vendor directories.
git diff --no-index src/DOBOT_6Axis_ROS2_V4 "$tmp_dir/dobot" || true
git diff --no-index src/OrbbecSDK_ROS2 "$tmp_dir/orbbec" || true
```

Record the new upstream commit IDs in the diary whenever a snapshot is intentionally refreshed.

## Offline transfer

For a full Git repository transfer, create a bundle while online and copy that single file to the offline machine:

```bash
git bundle create ../dobot_picknplace_yolo.bundle --all
git clone ../dobot_picknplace_yolo.bundle dobot_picknplace_yolo
```

The bundle includes the vendored source and project history; it does not need GitHub or submodule URLs. A source-only archive can also be made with `git archive --format=tar.gz --output=../dobot_picknplace_yolo.tar.gz HEAD`. The offline PC still needs a compatible Ubuntu/ROS 2 installation and any system dependencies (for example MoveIt and Gazebo) already available locally; `rosdep` cannot download missing packages without an offline package mirror or cache.

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

Create the project-wide runtime configuration once per checkout. The file is ignored by Git, so it can contain the address of the robot connected to that machine:

```bash
cp .env.example .env
# Edit .env and set DOBOT_ROBOT_IP for the robot/network being used.
```

The Dobot bringup launch requires the repository `.env` automatically. It hard-fails when the file is missing, malformed, or missing a valid `DOBOT_ROBOT_IP`; there is no IP fallback:

```bash
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

The file uses strict `KEY=value` lines and does not require a Python dotenv package. Do not use shell exports, alternate key names, or alternate configuration paths.

For a Gemini 335 connected over USB, the official Orbbec wrapper provides:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

For first-time USB setup, install the udev rules supplied by Orbbec:

```bash
sudo bash src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh
```

Refer to the upstream READMEs for complete robot safety, networking, camera, and launch instructions. Pick-and-place and YOLO application packages will be added in subsequent steps.
