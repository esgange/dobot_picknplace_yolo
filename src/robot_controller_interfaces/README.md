# robot_controller_interfaces

Typed ROS 2 interfaces for `robot_controller` v2. Long-running Home and Pick
operations are actions; bounded lifecycle, configuration, Stop, preview, and
speed operations are services. The shared `Command` service type is also used
for the controller-owned `/robot_controller/pause` and
`/robot_controller/continue` endpoints; `/robot_controller/stop` remains a
direct call with no Pause prerequisite. `ControllerStatus` replaces the former
JSON status payload and reports the explicit `PAUSED` lifecycle state.

This package contains definitions only. It never connects to or commands the
robot.
