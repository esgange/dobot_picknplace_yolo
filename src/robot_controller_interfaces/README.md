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

This package contains definitions only. It never connects to or commands the
robot.
