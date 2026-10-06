# robot_controller_interfaces

Typed ROS 2 interfaces for `robot_controller`. Long-running Home, Pick, Tray Detect Position and Place
operations are actions; bounded lifecycle, configuration, Stop, preview, and
speed/CP operations are services. The shared `Command` service type is also used
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

`AutoRun` is the native `/robot_controller/auto_run` action. Its goal carries the
exact configuration ID, a quantity of 1–10000, placement X/Y/rotation and debug-image
choice for both item and tray requests. It requires unheld READY, a complete
Item/Bin/Tray configuration and both canonical pose providers. One operation
owner combines Pick/Place, prefetches one
next bin batch during placement, and appends Home/next Pick as soon as it is ready,
without waiting for intermediate placement/Home arrival. Final Home is confirmed.
Feedback and result include requested/completed quantities; SUCCESS means all
requested placements and final Home completed. NO_PICK ends after three exhausted
Pick batches; other failures/cancellation retain a partial count and use normal
Stop containment. The manual Place action's acceptance-only result is unchanged.

`PickItem`, `PlaceItem` and `AutoRun` each accept `bool save_debug_images` (default
false). `PlaceItem` forwards it to every tray pose/depth request; `AutoRun` applies
it to all item/tray requests, including prefetch and acquisition retries. Saved
images go to `debug/pick_img/` and `debug/tray_img/` respectively. The flag is
retained through Pause/Continue; Preview never requests captures. Rebuild this
package and Robot Controller, then restart controller and GUI together: the
`PlaceItem` goal definition has changed. Perception service definitions are unchanged.

`ControllerStatus.auto_run_active`, `auto_run_requested`, `auto_run_completed` and
operation `auto_run` identify the owner while lifecycle states describe its current
phase. Manual Pause/Continue/Return reject during Auto Run; direct Stop and action
cancellation always pre-empt. This is a wire-interface change: rebuild interfaces
and controller, and manually restart every status/action client together.

`ControllerStatus` also carries observed enable/running/queue/error/collision
flags and raw `digital_input_bits` / `digital_outputs` from one validated
canonical feedback snapshot. They are valid only when `feedback_fresh` is true;
otherwise their default zeros mean unavailable. Output bits are observed I/O,
not commanded values, and DI1 is raw rather than the debounced holding decision.
The GUI uses this topic for its robot-state label and two LEDs for DI1 suction
detection and DI12 finger fully open. The additional robot flags and output bits
remain available to API consumers. The stream updates periodically at 5 Hz and
cannot guarantee display of every short pulse.

Operator eligibility fields supplement the unchanged internal state names:
`configuration_editable` identifies non-headless configuration, `pick_configured`
requires Item/Bin selection, `motion_ready` and `motion_block_reason` describe
fresh enabled/idle/empty-queue readiness, and `preview_ready` permits stationary
disabled feedback. `at_tray_detect` compares every fresh canonical joint with the
saved target at ±1°; it is false while moving or feedback is unavailable.
`can_continue` / `continue_block_reason` describe retained Pause eligibility,
including uncertain-release refusal. They are read-only hints; requests revalidate
sources, ownership, outputs and feedback. The GUI adds local pending requests,
input validity and service reachability to its button gates. Rebuild interfaces
and controller, then manually restart controller/preview/GUI together.

`/robot_controller/configure` now validates and loads teach files **and prepares
real hardware** through the existing Startup sequence, under one operation owner.
Its unchanged response reports success only after confirmed READY. Stop cancels
preparation with no chained Startup request. Invalid files preserve the previous
configuration; initialization failure keeps the new files but reports failure.
The GUI blocks Configure in Preview. Headless construction remains read-only and
INACTIVE; `/robot_controller/startup` remains for external supervisors. Rebuild
and manually restart the controller and GUI together for the changed semantics.

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
Tray Detect Position button. Place checks fresh idle status and all six saved tray
joints within ±1°, proceeding immediately or waiting up to three seconds for
arrival. It never commands observation travel; timeout reports "Not at Tray Detect
position" without a detection request. `PlaceItem` carries
the configuration ID plus positive `x_mm`, `y_mm` and
`rotation_deg` in [−180, +180]. Rotation zero is the saved observation tool attitude,
with a local tool-Z offset. It is independent of detected item/tray axes.
`Configure.tray_teach_file` is optional for Home, and a recorded Tray Detect Pose
is required for Pick and tray actions. Pick skips its initial Home queue when
fresh idle status and all six joints already match Home. Successful Pick lifts
through pre-pick/clearance, then moves directly to saved Tray Detect and finishes
HOLDING there. Pick requests no tray observation; only Place requires tray arming.
PICK preview requires the same Tray Teach path and includes the success target.
Status includes `tray_configured`, `tray_position_recorded`, `TRAY_POSITIONING` and
`PLACING`. Headless Place requires HOLDING with a trusted HELD candidate.
`manual_placement_enabled` is true only for a non-headless controller: attended
Place may start from READY or HOLDING without a picked item, and does not require
suction before release. It still executes real hardware commands and validates
tray/depth and release I/O. It queues approach/drop/retract without final Home.
`PlaceItem.SUCCESS` acknowledges all three ordered command acceptances; its
`final_state` is normally PLACING. `ControllerStatus.operation_active` remains true
until the completion worker confirms idle final retract, neutral outputs and DI1
LOW, then enters READY and records PLACED only when a held candidate exists.
Later failures report through status/events and Stop containment; the returned
action result cannot be changed. Direct Stop and Pause remain available while
the queue executes. Placement Pause stops in place; Continue retains release
progress and only retreats after confirmed release. Explicit Recover cancels,
returns Home and resets the gripper. Neither interface nor launch automatically starts motion.

Three unavailable tray observations before placement motion cause a confirmed
in-place Pause at Tray Detect. The original Place action remains active and status
reports `PAUSED`, operation `place`, phase `TRAY_ACQUISITION_PAUSED`, plus the last
failure reason. Existing `/continue` explicitly grants another three-request
acquisition batch; ordinary Pause does not reset a partly used budget. The GUI's
**Place Item (Retry)** button uses that continuation; Pick Item stays disabled.
The paused **RETURN ITEM** control uses the existing return service and disables
itself while pending/executing. A separate permanent red **STOP** remains available.
`can_return_item` additionally permits
`/return_item` in this specific pause with a trusted HELD source and no issued
placement release. It returns to the saved bin pose and Home, ending Place with
CANCELED/final READY. No new interface fields or endpoints are added. Unknown held
items, invalid sources/pose evidence, robot faults and direct Stop keep their guards.

`ControllerStatus.item_detector_ready` and `tray_detector_ready` report availability
of the corresponding pose service from exactly one canonical root provider. They
are refreshed at 5 Hz; unavailable, ambiguous or foreign providers report false.
Item Teach/Tray Teach advertise while Armed; Item Detect/Tray Detect are the
headless equivalents. These flags do not mean a pose has already been captured,
that sources match, or that an item is held. The GUI combines them with lifecycle,
held-item and tray-position guards; action admission independently rechecks current
availability. Rebuild this package and Robot Controller and restart all status
clients together; do not mix old/new ControllerStatus definitions.

`SetGlobalCP` at `/robot_controller/set_global_cp` takes `uint8 percent` constrained
to 0–100. It requires idle READY/HOLDING with exclusive ownership, like global
speed. The response carries `success`, `message` and signed `confirmed_percent`;
`ControllerStatus.global_cp_percent` reports the same last accepted setting.
Both use -1 for unknown, since 0 is valid. Startup sets 100; Recover retains the
last confirmed value (100 if unknown). All motion inherits this setting without
per-command overrides. Preview and Auto Run disable the manual control.
Rebuild interfaces/controller and manually restart controller, preview and GUI
together; clients must use the same new ControllerStatus definition.
