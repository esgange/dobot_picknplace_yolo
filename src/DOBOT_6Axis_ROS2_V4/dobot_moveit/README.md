# dobot_moveit

MoveIt integration helpers for the CR10, including launch files, the motion action server, pose-target utilities, and DH verification tooling.

For a planning-only check:

```bash
ros2 launch dobot_moveit moveit_demo.launch.py
```

Real execution requires a separately started, safety-checked `dobot_bringup_v4` driver and the robot/network configuration in the root `.env`.
