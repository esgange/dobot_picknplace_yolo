# orbbec_camera_launcher

`orbbec_camera_launcher` is the project-level operator GUI and strict runtime
supervisor for exactly two USB-connected Orbbec Gemini 335 cameras. It uses the
vendored `orbbec_camera` package and its exact
`gemini_330_series.launch.py` entry point.

## Canonical workflow

From the repository root:

```bash
source scripts/source_ros_workspace.bash
ros2 launch orbbec_camera_launcher camera_launcher.launch.py
```

The GUI is the only camera-configuration editor. It scans USB devices, displays
the detected serials, validates every Orbbec setting, and atomically updates
only the canonical `ORBBEC_*` lines in the ignored root `.env`. The detected
serial selector is informational: it does not assign a slot or change the
clipboard. Enter the exact serial in the intended camera row and save it.

Device-scan results, raw scan output, configuration actions, and supervisor
activity share one chronological read-only GUI log. Select text and press
`Ctrl+C` to copy only that selection, or use **Copy Log** to copy the complete
displayed log.

Opening the GUI never starts camera nodes automatically. It provides two
explicit launch modes:

- **Launch Camera 1** or **Launch Camera 2** saves the complete configuration,
  then launches only that configured camera in a separate GNOME terminal. This is a
  direct `orbbec_camera gemini_330_series.launch.py` process with `device_num=1`
  and no watchdog. Stop it with Ctrl-C in that terminal. Closing the GUI does
  not terminate this operator-owned terminal.
- The lower **Launch Both (Watchdog)** action saves the configuration and starts
  the complete configured set headlessly through the package's internal
  `camera_headless.launch.py` supervisor.

Only one launch mode may run at a time. The internal supervisor takes no launch
arguments and reads `.env` without modifying it. Do not use the headless launch
file as a separate configuration or override workflow.

## Strict startup and supervision

One attempt covers the complete configured camera set:

1. Scan with `ros2 run orbbec_camera list_devices_node`.
2. Require both configured slots' exact serial numbers.
3. Start Camera 1 only, following canonical `.env` slot order.
4. Require both `/<camera_1_name>/color/image_raw` and
   `/<camera_1_name>/depth/image_raw` within Camera 1's exact five-second
   startup deadline.
5. Only after Camera 1 is ready, start Camera 2 and give it its own independent
   exact five-second deadline. Camera 1's process and stream freshness remain
   supervised during this step.
6. Once the complete set is ready, continuously monitor every owned process and
   every required color/depth stream.

The entire launcher run gets exactly three total attempts: the initial attempt
and two retries. A failed attempt stops the complete owned camera set and rests
exactly three seconds after shutdown completes, rescans both serials, and starts
again from Camera 1. The third failure records a terminal error and exits.
There is no supervised partial-camera startup, individual-camera retry,
connection-check bypass, exponential backoff, manual budget reset, or unlimited
restart. The explicit single-camera terminal mode is diagnostic and
intentionally has no watchdog; its driver startup and output remain visible to
the operator in that terminal.

## Local-only and USB-only behavior

The GUI launch, direct single-camera terminal, internal headless launch, and
supervisor all use `ROS_LOCALHOST_ONLY=1`. Every child process inherits that
environment. The vendor camera launch always receives
`enumerate_net_device:=false`, so this package does not search for Orbbec
network cameras. A missing `/usr/bin/gnome-terminal` hard-fails a single-camera
launch; there is no alternate-terminal fallback.

`ROS_LOCALHOST_ONLY` restricts ROS 2 DDS discovery and communication; it does
not replace operating-system firewall policy for unrelated applications.

## Configuration

All project runtime settings live in root `.env`, created from `.env.example`.
The camera configuration includes:

- exactly two canonical ROS camera names and exact serial numbers;
- the absolute Gemini 335 color/depth stream profile;
- registration, alignment, frame-sync, temporal-filter, and point-cloud flags;
- scan, startup, health, polling, and shutdown timeouts;
- fixed `ORBBEC_MAX_ATTEMPTS=3`, `ORBBEC_RETRY_DELAY_SEC=3`, and
  `ORBBEC_ENUMERATE_NET_DEVICE=false` policy values.

Missing, duplicate, malformed, unsupported, or inconsistent configuration
hard-fails. Both camera slots always exist; there is no camera-count setting or
GUI selector. An empty serial is allowed only so the GUI can open for first-time
configuration; camera launch remains disabled until both slots are complete and
unique.

The project launch files expose no public override arguments. The GUI and
headless supervisor read only root `.env`, then pass the following values to
the official Gemini 330-series vendor launch:

| Root setting | Vendor launch argument |
| --- | --- |
| Active camera name and serial | `camera_name`, `serial_number` |
| Preset | `device_preset` |
| Color enable/width/height/FPS | `enable_color`, `color_width`, `color_height`, `color_fps` |
| Depth enable/width/height/FPS | `enable_depth`, `depth_width`, `depth_height`, `depth_fps` |
| Registration and alignment | `depth_registration`, `align_target_stream`, `align_mode` |
| Frame sync and temporal filter | `enable_frame_sync`, `enable_temporal_filter` |
| Point cloud and USB-only enumeration | `enable_point_cloud`, `enumerate_net_device` |

The complete supervised set always uses internal `device_num=2`; this is not an
`.env` setting or launch override. A direct single-camera diagnostic terminal
always uses `device_num=1`.
Scan, startup, health, check-period, retry-delay, attempt-count, and shutdown
values belong to the GUI/supervisor and are not vendor camera arguments.

Orbbec advertises Gemini 335 maximum stream modes of 1920x1080 at 30 FPS for
RGB and 1280x800 at 30 FPS for depth. The configured pair still must be a mode
supported simultaneously by the attached device and firmware; this project
passes the explicit requested values and does not silently clamp them.

## Process ownership and logging

The GUI stops only the headless supervisor process group it created. The
supervisor tracks and stops only the vendor camera launch process groups it
created. It never searches the global process table or kills a process by a
matching command string. A direct single-camera terminal is stopped by the
operator in that terminal and is not terminated when the GUI closes.

During one supervised run, the watchdog records the exact DDS publisher GIDs
that appeared after each owned process started. A retry may ignore only those
exact retired-owned GIDs while discovery expires; any other publisher on a
required topic remains a strict collision. The GUI reads the supervisor's
terminal event, so exhaustion of all three attempts is shown as failure even
when the enclosing `ros2 launch` wrapper itself returns zero.

GUI configuration, scans, ordered camera launches, per-camera stream readiness,
startup timeouts, runtime failures, retired-endpoint filtering, numbered
complete-set recovery, process shutdown, and terminal failures are written to:

```text
logs/orbbec_camera_launcher/events.jsonl
```

Records use UTC timestamps. The file is overwritten before record 1,001, in
accordance with the repository package-log contract.

The supervisor publishes:

| Interface | Type | Purpose |
| --- | --- | --- |
| `/camera_watchdog/status` | `diagnostic_msgs/msg/DiagnosticArray` | Per-camera phase, stream age, PID, attempt budget, and last error |
| `/camera_watchdog/healthy` | `std_msgs/msg/Bool` | `true` only when every configured camera has fresh color and depth streams |

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select orbbec_camera_launcher
source scripts/source_ros_workspace.bash
colcon test --packages-select orbbec_camera_launcher
colcon test-result --verbose
```

These tests use simulated device scans and processes; they do not start camera
or robot hardware.
