# robot_controller

## Read-only candidate requests

The controller now has one perception client, not a Dobot command client.
After explicitly selecting a strict schema-3 item profile, request a fresh
batch from the independently armed GUI or headless detector:

```bash
ros2 service call /robot_controller/request_item_poses std_srvs/srv/Trigger '{}'
```

It sends the profile SHA-256 and `retry_limit` as the maximum pose count, checks
the returned frame, identity, timestamps, uniqueness and ordering, and returns
the batch as JSON while recording bounded package events. Missing service,
deadline, invalid data and profile changes fail without retries. No returned
pose is stored as an executable robot target. The initial validation-only
robot-motion contract below remains: no motion, enable/disable, I/O or home
execution, and no claim that sole-command ownership migration is complete.

Initial, non-actuating controller stage for strict item-profile validation.
It does not launch bringup, create Dobot command clients, load model weights,
move home, pick, or actuate I/O. Detection requests are read-only. This is not yet the complete
controller or enforcement of sole robot-command ownership. Existing motion and
gripper clients still need migration before that claim can be made.

## Build and launch

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_controller
source scripts/source_ros_workspace.bash
ros2 launch robot_controller robot_controller.launch.py
```

In another sourced terminal, open the editor:

```bash
ros2 launch item_perception_yolo item_teach.launch.py
```

Both processes require `ROS_LOCALHOST_ONLY=1` and the canonical root `.env`.
Start unconfigured, then explicitly Save/Load a profile in Item Teach and press
**Validate Saved Profile in Controller**. Alternatively, explicitly supply
`item_teach_file:=<absolute path to the saved YAML>` when launching the controller.
There is no implicit last-profile selection or controller-unavailable bypass.

## Public interface

- Parameter `item_teach_file`: only a strict schema-3 YAML directly under
  `offline_teach/item_teach/`, with its same-stem, SHA-256-verified `.pt` copy.
  The GUI uses `/robot_controller/set_parameters_atomically`; malformed,
  missing or tampered profiles are rejected without defaults or retries.
- `/robot_controller/validate_profile` (`std_srvs/srv/Trigger`): explicitly
  recheck the selected YAML/model pair. Failure reports `PROFILE_INVALID`.
- `/robot_controller/status` (`std_msgs/msg/String`): JSON, reliable and
  transient-local, published every second. States are `UNCONFIGURED`,
  `PROFILE_VALIDATED_NOT_ARMED`, and `PROFILE_INVALID`. Execution and inference
  are always false. A published validation summary describes the last check;
  its periodic publication does not continuously rehash model files.

The home record is six canonical `joint1` through `joint6` positions in radians.
The source robot IP and feedback publisher are recording provenance, not a
station restriction: the user explicitly permits the same home joints on their
identical robots. Loading never commands them. Any future home move requires
explicit controller execution and normal safety preconditions; a recorded home
does not establish collision-free travel on a different station.

The summary records `requested_pose_count = retry.retry_limit` and the separate
`yolo.max_detections` cap. Candidate requests occur only on the explicit Trigger action.
Events are UTC JSONL in `logs/robot_controller/events.jsonl`, overwriting before
record 1,001. There is no additional configuration file or GUI state store.

## Remaining execution decisions

Inference and pose generation now belong to the shared item detector. Still
finalize controller-side vertical attitude/height equations, home/travel/place
safety, and the complete I/O confirmation/timeout sequence before any execution.
The corrected map is DO1 exhaust, DO2 finger close, DO13 suction, DO14 finger
open, DI1 suction detection and DI12 full-open confirmation. No prototype
Grip/Release pattern or `item_pick` execution path is approved by this scaffold.

Hardware-free tests cover profile integrity, portable homes, parameter/service
rejections, the 1,000-event limit, and absence of hardware/model-loading clients.
