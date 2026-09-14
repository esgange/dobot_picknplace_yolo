# motion_debug

`motion_debug` is the primary operator GUI for live DOBOT state, manual motion
commands, IO/motion service calls, and motion script editing/playback. It is a
diagnostic and commissioning tool, not an autonomous pick workflow.

## Executable

| Executable | Purpose |
| --- | --- |
| `motion_debug_gui` | Tkinter GUI for status, manual control, and script playback. |

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select motion_debug
source scripts/source_ros_workspace.bash
```

## Run

```bash
ros2 launch motion_debug motion_debug.launch.py
```

This launch starts the GUI only. It never starts the Dobot driver or opens a
robot TCP connection. Start the bringup separately when hardware access is
intended:

```bash
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

If the bringup service is not available, the GUI shows a prompt with this
command and keeps its control actions disabled until the service appears.

## Startup Initialization

After bringup reports controller mode `4` (Disabled), `5` (Enabled), or `11`
(Jogging), the GUI locks its controls and runs this one-time ordered sequence:

1. `StopMoveJog` (best effort).
2. `DisableRobot` (best effort); a successful response is followed by a
   five-second wait for mode `4` confirmation.
3. `EnableRobot`; mode `5` must be confirmed within five seconds.
4. Set SpeedFactor to `50%`.
5. Activate Tool `0`.
6. Set Tool `1` TCP to `{0,0,0,0,0,0}`.
7. Set CP to `100%`.

`StopMoveJog` and `DisableRobot` are the only explicit non-fatal exceptions.
If either service is unavailable, fails, throws, times out, or (for disable)
does not produce mode `4`, the package writes a `WARNING` event and continues
to `EnableRobot`. This means only that the startup precondition is treated as
non-blocking; it is not proof that the robot or connection is healthy.

`EnableRobot`, its mode `5` confirmation, and every setting step are strict.
Any failure or five-second timeout stops all later startup steps and writes an
`ERROR` event. There is no automatic retry; restart `motion_debug` explicitly
after resolving the cause. The GUI does not send motion commands as part of
this initialization.

Direct run:

```bash
ros2 run motion_debug motion_debug_gui
```

## Inputs

| Topic | Type |
| --- | --- |
| `/joint_states` | `sensor_msgs/msg/JointState` |
| `dobot_msgs_v4/msg/ToolVectorActual` | `dobot_msgs_v4/msg/ToolVectorActual` |
| `dobot_msgs_v4/msg/RobotStatus` | `dobot_msgs_v4/msg/RobotStatus`; source of connection and enable state |
| `dobot_bringup_ros2/msg/FeedInfo` | `std_msgs/msg/String`; JSON feedback containing the controller `robot_mode` |

## Robot Services

The GUI calls services under:

```text
/dobot_bringup_ros2/srv
```

Services used:

- `EnableRobot`
- `DisableRobot`
- `ClearError`
- `Stop`
- `StopMoveJog`
- `StartDrag`
- `StopDrag`
- `Tool`
- `SetTool`
- `SetPayload`
- `CP`
- `SpeedFactor`
- `VelJ`
- `VelL`
- `AccJ`
- `AccL`
- `MovJ`
- `MovL`

Current controller mode comes from the bringup `FeedInfo` JSON feedback. The
GUI calls `GetErrorID` only when the reported mode is `9`.

## Motion Scripts

The main window keeps script controls hidden by default. Select the `Scripts`
button flush right in the full-width `Robot Status` header to expand the same
window to the right and reveal the Scripts panel. Select `Hide Scripts` to
retract the panel without unloading the current script.

Scripts are stored only in:

```text
WORKSPACE_ROOT/config/debug_script
```

This is the only script save/load directory. The former
`config/motion_calibrate` name is not read as a fallback.

Script files are JSON and can include a top-level speed profile:

```json
{
  "speed_profile": {
    "cp": 80,
    "speed_factor": 40
  }
}
```

Script behavior:

- loading a script updates the GUI CP and SpeedFactor controls;
- running a script applies CP and SpeedFactor first;
- script points execute in order;
- the run button becomes a stop control while playback is active.

## Relationship to Other Packages

- `tray_intercept` handles tray tracking and intercept motion.
- `item_pick` handles item-pick execution.
- `movement_calibration` consumes motion scripts created here.

## Notes

- Start `dobot_bringup_v4` separately before using live robot commands.
- Use conservative speed settings during commissioning.
- Motion scripts should be reviewed before playback because they send real robot
  motion commands.
