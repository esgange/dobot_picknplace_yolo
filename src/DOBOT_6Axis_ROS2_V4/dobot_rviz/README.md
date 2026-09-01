# dobot_rviz

RViz visualization for the Dobot CR10. It includes the CR10 URDF, meshes, and visualization launch configuration.

Visualize the model without hardware:

```bash
ros2 launch dobot_rviz dobot_rviz.launch.py
```

Live joint-state display requires a running and safety-checked Dobot bringup node.
