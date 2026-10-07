# orbbec_camera

Official Orbbec ROS 2 camera driver snapshot. It provides the SDK-backed camera node, parameters, filters, tools, and launch files for Orbbec devices including the Gemini 330 series.

For a Gemini 335 camera:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

USB permissions, camera firmware, and host SDK prerequisites must be provisioned on the target machine. This package has no Dobot hardware dependency.

## Local registered-depth publication guard

PicknPlace patch, 2026-10-07, against OrbbecSDK_ROS2 `v2-main` snapshot
`8e7cad2b` (<https://github.com/orbbec/OrbbecSDK_ROS2>). Under `ANY` frame
aggregation the upstream callback can receive depth without RGB, skip software
alignment, then publish raw depth/CameraInfo under the aligned RGB frame name.

`src/ob_camera_node.cpp` now requires completed software D2C and validates the
current output's dimensions and intrinsics against its RGB target through
`include/orbbec_camera/aligned_depth_guard.hpp`, before any aligned output or
point-cloud queue. A failed bundle is discarded with a throttled warning. RGB and
registered-depth distortion may differ; zero-distortion depth remains supported.
The separate `depth/image_unaligned` topic remains explicitly raw. Hardware
alignment, C2D and registration-disabled paths are unchanged.

The project launcher selects `frame_aggregate_mode=full_frame` for registered
depth. The guard also protects direct vendor launches using `ANY`; aggregation
alone cannot prove that alignment succeeded. No startup intrinsics are cached or
substituted. `test/aligned_depth_guard_test.cpp` uses synthetic SDK frames and an
actual SDK Align operation without opening hardware. Run its CTest target
`aligned_depth_guard` after building with `BUILD_TESTING=ON`. Upstream SDK binaries,
licenses and attribution remain unchanged. See the project diary for validation.
