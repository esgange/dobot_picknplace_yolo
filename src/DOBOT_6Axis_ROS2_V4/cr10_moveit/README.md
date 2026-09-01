# cr10_moveit

MoveIt configuration for the Dobot CR10. This package contains the CR10 robot model, SRDF, joint limits, kinematics, planning, controller, and RViz configuration used by the Dobot planning stack.

Useful non-hardware launches:

```bash
ros2 launch cr10_moveit demo.launch.py
ros2 launch cr10_moveit moveit_rviz.launch.py
```

Use the package-local launch files to inspect planning behavior before enabling any real-robot execution. Robot-specific assets are intentionally limited to CR10.
