# gripper_control

`gripper_control` is a small operator diagnostic GUI for the DOBOT gripper and
suction tooling. It sends the retained Dobot V4 `dobot_msgs_v4/srv/DO` service
and reads the retained bringup `FeedInfo` JSON stream for output and input
feedback. Bringup must already be alive and connected before this package is
started.

## Executable

| Executable | Purpose |
| --- | --- |
| `gripper_control_gui` | Tkinter GUI for toggling `DO1`, `DO2`, `DO13`, and `DO14` and viewing generic `DI1`/`DI2` checks. |

## IO Map

| Channel | Purpose |
| --- | --- |
| `DO1` | Suction exhaust |
| `DO2` | Gripper close |
| `DO13` | Finger close |
| `DO14` | Gripper open |
| `DI1` | Digital input check (name not assigned) |
| `DI2` | Digital input check (name not assigned) |

The `Digital Inputs` panel reads `digital_input_bits` from the canonical
`/dobot_bringup_ros2/msg/FeedInfo` topic. DO state is read from that same
payload's `digital_outputs` field. DI1 is bit `0` and DI2 is bit `1`, using the
usual `N` to bit `N-1` mapping. Semantic names for these inputs are deferred.

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select gripper_control
source scripts/source_ros_workspace.bash
```

## Run

```bash
ros2 launch gripper_control gripper_control.launch.py
```

The launch file has no service/topic/configuration overrides. It starts only
the GUI. The process exits after a five-second bounded startup check if the
canonical DO service, connected RobotStatus, or FeedInfo feedback is missing.

Direct run:

```bash
ros2 run gripper_control gripper_control_gui
```

## Interface

| Interface | Type | Default |
| --- | --- | --- |
| DO service | `dobot_msgs_v4/srv/DO` | `/dobot_bringup_ros2/srv/DO` |
| Robot status | `dobot_msgs_v4/msg/RobotStatus` | `/dobot_msgs_v4/msg/RobotStatus` |
| FeedInfo | `std_msgs/msg/String` JSON payload | `/dobot_bringup_ros2/msg/FeedInfo` |

Expected DI status payload:

```json
{"digital_input_bits":1,"digital_outputs":4098,"robot_mode":5}
```

In this example DI1 is HIGH, DI2 is LOW, and DO2 plus DO13 are ON
(`2 + 4096 = 4098`).

## GUI Behavior

- Provides independent controls for `DO1`, `DO2`, `DO13`, and `DO14`.
- Shows read-only HIGH/LOW indicators for generic `DI1` and `DI2` checks.
- Sends canonical `DO(index,status,time=0)` requests and confirms each result
  from the FeedInfo `digital_outputs` bitmask.
- Uses explicit GUI timers followed by a second OFF service call for pulses;
  it does not rely on undocumented controller timing semantics.
- Supports per-channel auto-off timing in milliseconds.
- Always requests `DO1`, `DO2`, `DO13`, and `DO14` OFF during shutdown,
  regardless of cached UI state.

The window contains only Outputs, Digital Inputs, status and the auto-off hint.
The Quick Actions panel and its automatic Grip/Release sequences are removed.

## Diagnostics

Check the canonical FeedInfo source topic:

```bash
ros2 topic echo --field data /dobot_bringup_ros2/msg/FeedInfo
```

DI1 is reported from bit `0` and DI2 from bit `1` of `digital_input_bits`.
Their semantic names are intentionally not assigned yet.

## Notes

- Start the canonical `dobot_bringup_v4` launch before opening the GUI. The
  gripper process does not start bringup, prompt for it, or wait indefinitely.
- Confirm output channel wiring before enabling suction or gripper hardware.
- Startup requires a connected `RobotStatus`, a valid FeedInfo record, and the
  DO service within five seconds. Failure is logged to the package datalog and
  exits without creating the GUI.
- Each action is serialized and stops at its first failed service response or
  missing FeedInfo output confirmation. Shutdown attempts OFF calls for
  `DO1`, `DO2`, `DO13`, and `DO14` and records any failures.

## Datalog

Runtime events are written by this package to:

```text
WORKSPACE_ROOT/logs/gripper_control/events.jsonl
```

Records use UTC timestamps. The file is overwritten before event 1,001.
