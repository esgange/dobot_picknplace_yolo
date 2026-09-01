# dobot_gazebo

Gazebo simulation support for the CR10 profile. It provides the simulation world, robot-state launchers, and MoveIt/Gazebo integration without requiring a physical Dobot connection.

Start the simulation with:

```bash
ros2 launch dobot_gazebo dobot_gazebo.launch.py
```

Use simulation first for launch and planning checks; this package does not replace the hardware bringup safety procedure.
