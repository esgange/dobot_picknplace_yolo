# robot_controller_interfaces

Typed ROS 2 interfaces for `robot_controller` v2. Long-running Home and Pick
operations are actions; bounded lifecycle, configuration, Stop, preview, and
speed operations are services. `ControllerStatus` replaces the former JSON
status payload.

This package contains definitions only. It never connects to or commands the
robot.
