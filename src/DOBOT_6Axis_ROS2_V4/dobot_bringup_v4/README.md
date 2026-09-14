# dobot_bringup_v4

ROS 2 TCP/IP driver for the physical Dobot CR10. The node publishes joint state, robot status, and tool-vector feedback and exposes the retained Dobot command services. Streaming `ServoJ` and `ServoP` services are intentionally not built or exposed.

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
