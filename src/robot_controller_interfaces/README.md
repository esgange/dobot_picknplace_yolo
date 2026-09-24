# robot_controller_interfaces

Typed ROS 2 interfaces for `robot_controller` v2. Long-running Home and Pick
operations are actions; bounded lifecycle, configuration, Stop, preview, and
speed operations are services. The shared `Command` service type is also used
for the controller-owned `/robot_controller/pause` and
`/robot_controller/continue` and `/robot_controller/return_item` endpoints;
`/robot_controller/stop` remains a direct call with no Pause prerequisite. `ControllerStatus` replaces the former
JSON status payload and reports `PAUSING`, `PAUSED` and `RETURNING_ITEM` alongside
the other lifecycle states. Pause/return service success acknowledges the request;
status reports parking/return completion. Continue rebuilds commands after a
confirmed Stop; no vendor paused queue is resumed. `candidate_ids` and
`candidate_states` are parallel arrays for the retained candidate ledger;
`can_return_item` reports availability of trusted held source context. Rebuild
this interface package and restart all clients after updating these fields.

`ControllerStatus` also carries observed enable/running/queue/error/collision
flags and raw `digital_input_bits` / `digital_outputs` from one validated
canonical feedback snapshot. They are valid only when `feedback_fresh` is true;
otherwise their default zeros mean unavailable. Output bits are observed I/O,
not commanded values, and DI1 is raw rather than the debounced holding decision.
The GUI uses this topic for its robot-state label and two LEDs for DI1 suction
detection and DI12 finger fully open. The additional robot flags and output bits
remain available to API consumers. The stream updates periodically at 5 Hz and
cannot guarantee display of every short pulse.

This package contains definitions only. It never connects to or commands the
robot.

`ReplayCalibration` adds the attended calibration action: a fresh run ID plus
ordered `CalibrationPosition` messages (six canonical joints in radians), with
position/phase feedback and captured-count/final-state results. It uses the same
success/cancel/fault outcome values as GoHome. `CaptureCalibration` is the
controller-to-calibrator PREPARE/CAPTURE handshake, scoped by run ID and one-based
position index; CAPTURE carries the earliest allowed observation timestamp.
`ControllerStatus.state` additionally reports CALIBRATING. Rebuild interfaces
and restart controller/calibration clients before using the new workflow.
