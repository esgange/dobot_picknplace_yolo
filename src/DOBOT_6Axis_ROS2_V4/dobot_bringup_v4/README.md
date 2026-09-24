# dobot_bringup_v4

ROS 2 TCP/IP driver for the physical Dobot CR10. The node publishes joint state, robot status, and tool-vector feedback and exposes the retained Dobot command services. Streaming `ServoJ` and `ServoP` services are intentionally not built or exposed.

The canonical launch also starts the existing `dobot_rviz` viewer, including
its actual-joint monitor and CR10 `robot_state_publisher`. A graphical desktop
is required to display RViz. Do not start a second viewer while this one is
running: it would duplicate the robot TF publisher.

The viewer runs in an owned child launch session. Closing RViz, a viewer crash,
or a failed viewer feedback check stops that viewer session and its robot TF
publisher while the Dobot driver continues. The terminal reports viewer exit;
there is no automatic restart. The standalone
`ros2 launch dobot_rviz dobot_rviz.launch.py` command can reopen it after it has
exited. Ctrl-C in the bringup terminal stops both sessions, and driver exit also
shuts down the owned viewer. Closing RViz is not a robot Stop command.

RViz remains read-only. The launch adds no Enable, Home, motion, or gripper
commands. Connection retries, service responses and feedback publication retain
their existing behavior; the viewer retains its five-second startup and
one-second stream-freshness checks. Display/GPU failures remain local to the
viewer, but applications requiring its robot TF lose that TF when it exits.

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
canonical Orbbec keys, so every package sees one strict configuration file.
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
