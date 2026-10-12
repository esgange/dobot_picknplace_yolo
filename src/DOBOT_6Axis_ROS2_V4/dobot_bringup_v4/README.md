# dobot_bringup_v4

ROS 2 TCP/IP driver for the physical Dobot CR10. The node publishes joint state, robot status, and tool-vector feedback and exposes the retained Dobot command services. Streaming `ServoJ` and `ServoP` services are intentionally not built or exposed.

The canonical launch starts the driver and an owned headless session with the
actual-joint monitor and CR10 `robot_state_publisher`. Their existing executable
and model assets remain in `dobot_rviz`; `robot_state.launch.py` contains only
these two nodes. Bringup never starts RViz and needs no graphical desktop.
Headless perception can obtain timestamped `base_link -> Link6` TF without a viewer.

Ctrl-C in the bringup terminal or driver exit stops the owned TF session.
Monitor or publisher failure stops that session, reports unavailable robot TF,
and leaves the driver and its command/feedback services running. No automatic
restart is added. The monitor retains its five-second startup, one-second
receipt freshness, exact joint-message and sole-publisher checks. A TF failure
must be resolved before an explicit operator restart.

The separate `ros2 launch dobot_rviz dobot_rviz.launch.py` command starts only
the optional viewer. Opening, closing, crashing or reopening it has no effect on
the driver, monitor or TF publisher and cannot create duplicate robot TF.
The launch adds no Enable, Home, motion, Stop or gripper commands. Connection
retries, service responses, feedback publication and controller containment
remain unchanged.

The bringup launch is strict and requires the repository root `.env`:

```bash
cp .env.example .env
# Set every required project value from .env.example in the root .env.
source scripts/source_ros_workspace.bash
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

`ROS_LOCALHOST_ONLY=1` is required so ROS discovery and DDS traffic remain on
this computer. The robot driver still uses its explicit TCP connection to the
configured controller addresses.

The parser validates the complete project `.env` schema, including the
canonical Orbbec keys and the three `ITEM_TEACH_*_CALIBRATION` filename keys,
so every package sees one strict configuration file. Calibration values must be
all empty or all local YAML basenames; bringup does not load those artifacts.
Bringup itself uses only the Dobot values.

LAN1 is attempted first, followed by LAN2 when LAN1 cannot establish both Dobot TCP channels. Each channel is bounded by the required `DOBOT_CONNECTION_TIMEOUT_MS`; the canonical value is `3000` ms. The node prints and logs an attempt before entering the bounded TCP call, so an unreachable LAN1 is visible and cannot wait for the operating system's default timeout. Each successful connection writes a `robot_connection_result` record to this package's `logs/dobot_bringup_v4/events.jsonl` containing `status=connected`, the selected `interface`, and its `ip`. If both addresses fail, one result for that continuous outage contains `status=failed` and both attempted addresses, then the driver retries the same explicit sequence. Missing, malformed, unsupported, duplicate, or invalid configuration fails before the node starts. Confirm the robot, network, remote-control mode, workspace clearance, and emergency-stop readiness before launching against hardware.

Inspect the most recent connection results with:

```bash
grep '"event":"robot_connection_result"' logs/dobot_bringup_v4/events.jsonl | tail
```

For live attempt visibility:

```bash
tail -f logs/dobot_bringup_v4/events.jsonl
```

Stopping bringup while a TCP connection attempt is still pending cannot produce a connection result; the attempt remains recorded as `robot_connection_attempt`.

The logger is package-local; use the repository-level `scripts/compile_logs.py` only when a timestamp-ordered cross-package file is needed.
