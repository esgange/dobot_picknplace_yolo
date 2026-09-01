# orbbec_camera

Official Orbbec ROS 2 camera driver snapshot. It provides the SDK-backed camera node, parameters, filters, tools, and launch files for Orbbec devices including the Gemini 330 series.

For a Gemini 335 camera:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

USB permissions, camera firmware, and host SDK prerequisites must be provisioned on the target machine. This package has no Dobot hardware dependency.
