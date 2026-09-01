# dobot_bringup_v4

ROS 2 TCP/IP driver for the Dobot CR10. The node publishes joint state, robot status, and tool-vector feedback and exposes the Dobot command services.

The bringup launch is strict and requires the repository root `.env`:

```bash
cp .env.example .env
# Set all required Dobot values in the root .env.
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

LAN1 is attempted first, followed by LAN2 when LAN1 cannot establish both Dobot TCP channels. If both fail, the outage is recorded in this package's `logs/dobot_bringup_v4/events.jsonl` and the driver retries the explicit two-address sequence. Missing, malformed, unsupported, duplicate, or invalid configuration fails before the node starts. Confirm the robot, network, remote-control mode, workspace clearance, and emergency-stop readiness before launching against hardware.

The logger is package-local; use the repository-level `scripts/compile_logs.py` only when a timestamp-ordered cross-package file is needed.
