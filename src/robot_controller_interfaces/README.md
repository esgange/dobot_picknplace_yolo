# robot_controller_interfaces

Typed ROS 2 interfaces for `robot_controller` v2. Long-running Home, Pick, Tray Detect Position and Place
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
explicit Recover marks unfinished candidates `CANCELED` and returns Home with
gripper outputs preserved. It never resumes the old batch. A trusted held item
remains `HELD`; an unconfirmed release is never relabeled `PLACED` or `RETURNED`.
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

`Preview` is served only by the read-only preview process at
`/robot_controller/preview_v2`: HOME=1, PICK=2, CLEAR=3 and PLACE=4. Requests carry
selected Item/Bin/Tray paths plus placement X/Y millimetres and rotation degrees.
Responses list planned TF frames; no robot command or gripper output is issued.
CLEAR can cancel an in-flight observation. The GUI's single Preview toggle routes
Home/Pick/Place to this service while ON; hardware actions remain separate.
Rebuild this package and `robot_controller`, then restart controller/preview/GUI
together. The old preview endpoint cannot satisfy the new GUI.

`GoTrayDetectPosition` remains available to external clients, carrying the
configuration ID and saved Tray Teach observation joints. The GUI has no separate
Tray Detect Position button; Place includes that travel. `PlaceItem` carries
the configuration ID plus positive `x_mm`, `y_mm` and
`rotation_deg` in [−180, +180]. Rotation zero is the saved observation tool attitude,
with a local tool-Z offset. It is independent of detected item/tray axes.
`Configure.tray_teach_file` is optional for Home/Pick and required for tray actions.
Status includes `tray_configured`, `tray_position_recorded`, `TRAY_POSITIONING` and
`PLACING`. Headless Place requires HOLDING with a trusted HELD candidate.
`manual_placement_enabled` is true only for a non-headless controller: attended
Place may start from READY or HOLDING without a picked item, and does not require
suction before release. It still executes real hardware commands and validates
tray/depth, release I/O and final Home. Confirmed release records PLACED only when
a held candidate exists. Placement Pause stops in place; Continue and Recover preserve
release progress. Neither interface nor launch automatically starts motion.

`ControllerStatus.item_detector_ready` and `tray_detector_ready` report availability
of the corresponding pose service from exactly one canonical root provider. They
are refreshed at 5 Hz; unavailable, ambiguous or foreign providers report false.
Item Teach/Tray Teach advertise while Armed; Item Detect/Tray Detect are the
headless equivalents. These flags do not mean a pose has already been captured,
that sources match, or that an item is held. The GUI combines them with lifecycle,
held-item and tray-position guards; action admission independently rechecks current
availability. Rebuild this package and Robot Controller and restart all status
clients together; do not mix old/new ControllerStatus definitions.
