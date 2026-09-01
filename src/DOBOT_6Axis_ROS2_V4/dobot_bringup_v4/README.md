# dobot_bringup_v4

ROS 2 TCP/IP driver for the Dobot CR10. The node publishes joint state, robot status, and tool-vector feedback and exposes the Dobot command services.

The bringup launch is strict and requires the repository root `.env`:

```bash
cp .env.example .env
# Set DOBOT_ROBOT_IP=... in .env
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

Missing, malformed, unsupported, duplicate, or invalid configuration fails before the node starts. Confirm the robot, network, remote-control mode, workspace clearance, and emergency-stop readiness before launching against hardware.
