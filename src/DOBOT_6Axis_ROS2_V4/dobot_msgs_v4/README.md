# dobot_msgs_v4

ROS 2 interface definitions for the hardware-only Dobot V4 driver. The package contains the robot status and tool-vector messages plus retained command, point-to-point/jog motion, I/O, and configuration service definitions consumed by the other Dobot packages. Streaming `ServoJ` and `ServoP` definitions are intentionally excluded.

This package has no hardware side effects; it can be built and inspected independently.
