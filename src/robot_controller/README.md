# Robot Controller v2

Visual guide: [Controller finite state machine](../../docs/ROBOT_CONTROLLER_FSM.md).
It covers the current lifecycle, candidate ledger and operation/recovery routes;
update it in the same change whenever those behaviors change.
Ready-to-view versions: [visual HTML](../../docs/ROBOT_CONTROLLER_FSM.html) and
[visual PDF](../../docs/ROBOT_CONTROLLER_FSM.pdf), generated from that document.

`robot_controller` is the sole production application-level authority for the
physical CR10. It provides Home, Pick Item, Tray Detect Position and Place Item.
It does not launch Dobot bringup, cameras, Item Detect, or RViz.

Launching the package never enables, recovers, homes, or moves the robot. An
operator or supervisor must load configuration and make an explicit Startup
service call before a hardware action can be accepted.

## Emergency-stop feedback

The controller translates a Dobot command response `res=-3` or confirmed robot
alarm **1537** into **“Emergency stop pressed — cannot start or recover. Release
the emergency-stop button, then click Recover / Clear Error.”** The failed service
response, operator log and FAULT status carry this explanation; the GUI shows
EMERGENCY STOP / PRESSED / Cannot start / recover in its status panel.
Generic mode 9, ErrorStatus, collision, disabled state and unrelated alarms are
not enough to label an emergency stop.

Startup first makes a read-only canonical `GetErrorID` query before any hardware
initialization commands. Its V4.6.5 payload must be `{[integer alarm IDs]}` (`{[]}`
when clear); missing/malformed/failed responses block startup. An emergency stop
reported by StopMoveJog is fatal even though ordinary StopMoveJog rejections are
best effort. Stop rejection still means Stop is unconfirmed.

Recover retains its explicit Stop → conditional ClearError → verified alarm
clearance → Enable sequence. An acknowledged ClearError alone is not success.
If the alarm persists, GetErrorID distinguishes an emergency stop from the old
generic clearance timeout. Recover remains clickable in FAULT so it can clear a
latched alarm after physical release; no stale UI flag blocks that attempt. It
cannot reach Enable, settings, output reset or motion unless clearance passes.
All queries use canonical ownership, feedback/cancellation and five-second
response checks, with no retries. Held-item and independent Stop guards remain.
Restart the controller and GUI after rebuilding `robot_controller` to load this
change; detection/teach files and ROS interfaces are unchanged.


## Tray placement

Load **Item Teach**, **Bin Teach** and a complete **Tray Teach** in the controller,
then Startup. In GUI mode, Pick Item is optional before placement; headless Place
requires a successful Pick. **Tray Detect Position** moves to the joints recorded
in Tray Teach using one direct queued joint-target MovL. There is no preliminary
Z rise or elevated transit; the bin routes retain their existing clearance logic.
**Place Item** also reaches that observation pose when needed and
requests one fresh tray/depth observation from Armed Tray Teach or headless Tray
Detect. Run exactly one provider. All hardware commands remain controller-owned.

Tray configuration uses the active eye-on-hand calibration saved in the shared
`ITEM_TEACH_ROBOT_CAMERA_CALIBRATION` selection, independently of the camera
recorded when the tray plane was taught. Use the same active camera in Tray Teach
or headless Tray Detect; returned camera hashes must still match. Recalibration
preserves the saved plane and detect joints. Reload controller configuration and
restart headless detection after changing calibration; GUI Tray Teach can load
the replacement explicitly and must be armed again. Loading never moves the robot.

Pick Item is enabled only in idle READY with its Item Teach/Detect service
available. Place Item needs a recorded Tray Detect Pose and its Tray Teach/Detect
service. The normal GUI launch (`headless=false`) is attended debug mode: Place
is available from idle READY or HOLDING, with or without an item or picked-item
record. No suction-presence prerequisite applies during observation or approach.
The sequence still commands real hardware; final neutral I/O and DI1 are checked
at Home. Headless mode retains HOLDING, trusted picked-item source and held-suction
guards before the placement queue begins, including travel to Tray Detect Pose.
The controller owns this policy; merely attaching a GUI to a headless controller
does not relax it. Status reports `manual_placement_enabled`; tooltips explain it.

The controller reports `item_detector_ready` and `tray_detector_ready` in typed
status at 5 Hz. It checks service availability and exactly one allowed provider:
root Item Teach or Item Detect for Pick; root Tray Teach or Tray Detect for Place.
Teaching disarm removes the service. Missing, foreign, namespaced or duplicate
providers disable the corresponding button. The controller checks again at action
admission, so stale GUI readiness cannot bypass it. These are read-only checks;
no detector trigger, model inference or automatic arming occurs. Fresh request-time
source/pose/depth validation still applies. Home and Tray Detect Position remain
available under their usual guards even when detection is disarmed.

Rebuild `robot_controller_interfaces` and `robot_controller`, then restart all
controller/GUI clients together for these status fields.

Tray requests use `/tray_detect/get_tray_pose_v2`, the depth-capable contract.
The earlier endpoint is never used as a fallback. Restart Tray Teach/Detect and
Robot Controller together after rebuilding; an old provider leaves Place disabled
instead of accepting an incompatible request. No robot motion is sent by detection.

Enter positive **X (mm)** and **Y (mm)** from the detected tray origin along its
inward short-X and long-Y axes, and **Rotation (−180° to +180°)**. At 0° the tool uses
the recorded Tray Detect Pose attitude; the offset rotates about that tool's Z.
Item axes and pick rotation have no effect on placement orientation.

Placement keeps requested X/Y and measures surface Z using the Item Teach depth
sampling diameter and quality settings. Release Z is surface + standoff + prepick
height: the equivalent of item pre-pick above the detected tray surface (rule 169).
Pre-place/retract uses the placement X/Y and orientation at taught Home Z,
matching the first Item Pick approach (`pN_transit`) before pre-pick (rule 171).
That initial Pick route skips its lower clearance/initial point. For surface Z
250 mm, standoff 10 mm, prepick 50 mm and Home Z 800 mm, release is Z 310 mm and
approach/retract Z 800 mm. This is a Home-Z target over the tray, not a preliminary
vertical rise before traveling to Tray Detect Pose. The tray attitude remains
detect-relative until the final Home command restores Home's full pose.
Home Z must be above drop Z for percentage I/O on both legs; invalid geometry
blocks the placement queue. No additional height field or teach-file changes.
Tray arrival uses fresh idle/empty-queue feedback and saved joint
angles within ±1°, with execution evidence and no added settling interval. Then
request fresh tray/depth and queue exactly four Cartesian commands through Home:

| Command | Target | Timed outputs |
| --- | --- | --- |
| MovL | Pre-place | Preserve existing outputs |
| MovLIO | Release height | At 80%: DO2 OFF, DO14 ON, DO13 OFF, DO1 ON |
| MovLIO | Back to pre-place | At 20%: DO2 OFF, DO14 OFF, DO1 OFF, DO13 OFF |
| MovL | Taught Home XYZ/orientation | Neutral |

Tray Detect Position and all four placement commands use **speed 100%**,
independent of Item Teach speed settings. Global SpeedFactor still scales them;
placement never changes that slider. Acceleration remains Item Teach travel for
Tray Detect Position, then travel / approach / retract / travel for the four-command
queue. Item Pick retains its taught speeds. No teach-file edit is required.
There is no placement settling, separate release call, fixed-duration exhaust
pulse, extra retract-height waypoint or separate Home action. The 80% trigger
starts release before the nominal lower point; exhaust duration follows the
motion until the 20% upward trigger. All commands inherit CP(100); control points
can blend. Admit each service in order, without waiting for intermediate arrival.
Physically confirm only final Cartesian Home before reporting READY/SUCCESS.

Rule 170 removes intermediate release-confirmation gates in both modes. Send the
entire approach → pre-pick-equivalent drop → approach → Home queue without waiting
for OPEN/exhaust, DI12 or DI1 transitions. Missing/late release evidence, suction
changes and gaps in output history do not interrupt this queue. Any observed
coherent release feedback is retained as diagnostic/recovery evidence only.
Keep command acceptance/order, live enabled/error/collision/freshness checks,
opposing-output protection, motion watchdogs and direct Stop/Pause. These are
hardware/transport checks, not intermediate arrival or release confirmations.

Only at physically confirmed idle Home, require neutral DO1/DO2/DO13/DO14 and
DI1 LOW before READY/SUCCESS; DI12 need not be HIGH. A bad final grip reports a
specific Home-reached fault without trying to release again. Successful Home
completion records PLACED for an existing held candidate and clears holding;
the placement_home_completed event separately records release_feedback_observed.
PLACED now describes the completed queue and clear final grip, not proof that
the item was physically deposited on the tray. An empty test creates no candidate.

Placement Pause stops in place and preserves outputs. Continue reobserves when
the release command has not been issued. Once issued, never repeat descent or
release, even when its feedback was missed.
After confirmed release, Continue neutralizes outputs, retracts upward
from actual position to at least pre-place height if needed, and returns Home.
An interrupted partial release with insufficient confirmation remains blocked;
no automatic release or bin put-back is inferred. Direct Stop requires explicit
Recover, which cancels the placement and preserves current outputs while lifting
to Home height and returning Home, then relaxes all four gripper outputs. It never
replays the expired placement history
or waits for continuous exhaust to switch itself OFF. A failed
tray/depth observation performs no placement or release. Existing Pick settling,
retries, camera/bin avoidance, rotation and Home paths remain unchanged.

The controller UI's strict schema-3 last-session store preserves Item/Bin/Tray
filenames and the last validated X/Y/Rotation as unapplied prefill. Explicit
in-memory import of schema 1/2 preserves their selections; disk changes only on
configuration save or a valid Place request. First-use X/Y remain empty.

New APIs are `GoTrayDetectPosition` at `/robot_controller/go_tray_detect_position`
and `PlaceItem` at `/robot_controller/place_item`, both with the active configuration
ID. Configure accepts `tray_teach_file`; status includes `tray_configured`,
`tray_position_recorded`, `manual_placement_enabled`, `TRAY_POSITIONING` and `PLACING`.
Only headless placement requires a trusted HELD candidate.

Rebuild `tray_perception_interfaces`, `robot_controller_interfaces`,
`tray_perception` and `robot_controller`, then restart the tray provider and
controller/GUI together. Launch alone still sends no robot commands. Software
validation uses synthetic feedback and isolated ROS; physical placement requires
separate commissioning.

## Processes

- `robot_controller` is the background hardware authority. It alone creates Dobot
  motion, Pause/Continue/Stop, robot-setting, and gripper-output clients.
- `robot_controller_preview` calculates and broadcasts TF-only Home/Pick plans.
  It has no Dobot command client and cannot actuate the robot.
- `robot_controller_gui` is a client of the controller and preview APIs. It has
  no Dobot command client.

The normal launch starts all three:

```bash
ros2 launch robot_controller robot_controller.launch.py
```

Headless mode starts only the hardware controller and loads exactly one Item
Teach YAML/paired `.pt` plus one Bin Teach YAML from the flat root
`runtime_teach/` directory. The shared detector/controller catalog selects them
by `item_teach_` and `bin_teach_` filename prefix; Item YAML/PT must have the
same stem. It still remains `INACTIVE` until Startup:

A complete optional `tray_teach_*.yaml`/same-stem `.pt` pair may coexist in the
catalog for Tray Detect and controller placement. Its profile/camera hashes and
recorded observation joints become part of the controller configuration.

Missing Item YAML, Item model and Bin YAML inputs are reported separately.
Duplicate errors list the conflicting filenames. Manual or external deployment
must finish one complete visible set before launch; replacement requires stopping
the process, replacing the catalog, and restarting it. Hidden dot-prefixed staging
files are ignored. There is no directory watcher, retry, fallback or automatic
copy.

```bash
ros2 launch robot_controller robot_controller.launch.py headless:=true
```

Canonical Dobot bringup and Item Detect are separate processes. Do not run the
maintenance `motion_debug` or `gripper_control` application alongside
production Startup; their presence is rejected as competing command ownership.
Launch headless Item Detect separately with no arguments; it reads the same
immutable `runtime_teach/` selection and advertises only the read-only pose
service.

## Typed API

Actions:

- `/robot_controller/go_home` — goal contains `configuration_id`.
- `/robot_controller/pick_item` — goal contains `configuration_id` and the
  one-shot `save_debug_images` flag. Candidate count cannot be supplied by the
  caller; it comes from Item Teach `retry.pose_candidates`.

Services:

- `/robot_controller/configure` loads or reloads explicit Item/Bin Teach in GUI
  mode. Reload is accepted only from idle, unheld `READY` or from `INACTIVE`;
  success performs no robot command, returns to `INACTIVE`, and requires Startup
  again. A rejected replacement preserves the current configuration and state.
- `/robot_controller/startup` performs the deterministic cold Startup sequence.
- `/robot_controller/recover` cancels the interrupted action and remaining batch,
  preserves gripper outputs, restores readiness, lifts vertically to Home height
  and moves to taught Home, then turns DO1/DO2/DO13/DO14 OFF. Confirmed neutral
  outputs and DI1 LOW finish READY. It accepts `FAULT`, `RECOVERY_REQUIRED` and
  `HELD_UNKNOWN`; unknown DI1 HIGH still blocks enabling/motion.
- `/robot_controller/pause` accepts a managed stop-and-park request. The service
  reports acceptance; status reaches `PAUSED` only after parking completes.
- `/robot_controller/continue` accepts replanning from confirmed `PAUSED`.
- `/robot_controller/return_item` accepts controlled return of a trusted held
  candidate to its source. Observe `RETURNING_ITEM` and then `READY` in status.
- `/robot_controller/stop` pre-empts and confirms Stop while preserving outputs.
  It is always direct and never requires a preceding Pause.
- `/robot_controller/set_global_speed` accepts an integer 1–100 only while
  stationary in `READY` or `HOLDING`.
- `/robot_controller/preview` belongs to the TF-only preview node.

`/robot_controller/status` uses
`robot_controller_interfaces/msg/ControllerStatus`, reliable/transient-local
QoS. It reports state, phase, waypoint, candidate index, configuration ID,
holding context, Startup completion, global factor, and feedback freshness.
The same snapshot includes observed `robot_enabled` (FeedInfo EnableStatus),
`robot_running`, `robot_queue_active`, `robot_error`, `robot_collision`, and raw
64-bit `digital_input_bits` / `digital_outputs`. These fields are valid only
when `feedback_fresh` is true; zeroed fields in an unavailable snapshot mean
unknown, not confirmed disabled/OFF/LOW. These are feedback values, independent
of expected/commanded outputs and the debounced held-item state. The periodic
status rate remains 5 Hz, with additional state/progress updates.
`candidate_ids` and `candidate_states` are parallel ordered arrays for the retained
batch; `can_return_item` identifies trusted held source context.
The removed Trigger/JSON/Live/Enable/validation/pose-proxy/debug-image endpoints
have no compatibility wrappers.

Example CLI flow for a non-headless controller:

```bash
ros2 service call /robot_controller/configure \
  robot_controller_interfaces/srv/Configure \
  "{item_teach_file: '/absolute/item.yaml', bin_teach_file: '/absolute/bin.yaml'}"
ros2 service call /robot_controller/startup \
  robot_controller_interfaces/srv/Command '{}'
ros2 topic echo --once /robot_controller/status \
  robot_controller_interfaces/msg/ControllerStatus
ros2 action send_goal /robot_controller/go_home \
  robot_controller_interfaces/action/GoHome \
  "{configuration_id: '<exact status configuration_id>'}" --feedback
ros2 action send_goal /robot_controller/pick_item \
  robot_controller_interfaces/action/PickItem \
  "{configuration_id: '<exact status configuration_id>', save_debug_images: false}" \
  --feedback
ros2 service call /robot_controller/pause \
  robot_controller_interfaces/srv/Command '{}'
ros2 service call /robot_controller/continue \
  robot_controller_interfaces/srv/Command '{}'
ros2 service call /robot_controller/stop \
  robot_controller_interfaces/srv/Command '{}'
ros2 service call /robot_controller/recover \
  robot_controller_interfaces/srv/Command '{}'
ros2 service call /robot_controller/set_global_speed \
  robot_controller_interfaces/srv/SetGlobalSpeed '{percent: 50}'
```

## State and safety contract

The states are `UNCONFIGURED`, `INACTIVE`, `STARTING`, `READY`, `HOMING`,
`PICKING`, `HOLDING`, `PAUSING`, `PAUSED`, `RETURNING_ITEM`, `STOPPING`,
`RECOVERY_REQUIRED`, `RECOVERING`,
`HELD_UNKNOWN`, and `FAULT`. One immutable configuration snapshot and one
operation generation exist at a time. Action configuration IDs prevent a stale
GUI or supervisor from executing a replaced profile.

The non-headless GUI labels the configuration control **Reload Teach
Configuration** after a profile is installed. It remains enabled in idle,
unheld `READY`, so the operator can revalidate the selected files or install a
different pair without restarting the controller process. A successful reload
clears TF preview output and invalidates prior Startup, global-speed, and output
assumptions; it does not call Dobot and the controller must be explicitly
started again. Reload is blocked during Home/Pick, Pause, holding, Stop,
recovery, unknown-held, and fault states. Headless configuration remains fixed
to its startup `runtime_teach/` snapshot.

Startup validates sole canonical services and publishers, then performs:

1. best-effort `StopMoveJog`;
2. strict `Stop` with stationary, empty queue confirmation (a latched pause flag
   does not invalidate a confirmed Stop);
3. cold DI1 check (active DI1 preserves I/O and enters `HELD_UNKNOWN`);
4. Disable and conditional ClearError;
5. Enable and confirm fresh enabled mode. A latched `isPauseCmdFlag` does not
   block READY when mode 5 is enabled and the queue is empty/not running;
6. SpeedFactor 100, User 0, Tool 0, Tool 1 TCP zero, and CP 100;
7. DO1/DO2/DO13/DO14 reset only when no item is held;
8. 200 ms of coherent `READY` feedback.

Recover cancels the interrupted pick/place/put-back and marks unfinished
candidates CANCELED. It confirms Stop with two distinct stationary, queue-empty
samples with unchanged gripper outputs and raw DI1, then adopts that fresh I/O
state. It does not replay old placement history. Opposing output pairs, unknown
DI1 HIGH, DI1 HIGH after confirmed release, or a prior dropped source that merely
regains suction block motion. Sustained fresh LOW permits empty recovery while
preserving the outputs: LOW is not proof that an object physically left the fingers.
A trusted HELD source with live suction and vacuum stays held throughout recovery travel.

After conditional ClearError, verified alarm clearance, Enable and readiness,
Recover issues an upward-only RelMovLUser at unchanged XY/attitude if below Home Z.
Confirm that lift before a separate queued joint-target MovL to taught Home.
Use taught travel speed/acceleration and the last confirmed global factor (100%
if unset). Already at/above Home height skips the lift; already at taught Home
skips its move. Current outputs and suction policy are monitored throughout travel.
After stationary Home is confirmed, reset DO1, DO2, DO13 and DO14 to OFF, in that
order, confirming each response/output. Both finger outputs OFF means relaxed;
there is no OPEN command or exhaust pulse. The intentional reset permits suction
to decay. Require all four outputs OFF, DI1 LOW and continued Home arrival before
reporting READY. Cancel the former held ledger entry without claiming PLACED or
RETURNED. A still-HIGH DI1 reports HELD_UNKNOWN; clear the item/obstruction and retry.
Stop/cancellation prevents remaining commands during travel or output reset; failures
never report success. Another Recover replans from fresh stopped feedback. There is
no automatic next pick or replay of an interrupted placement/put-back release.

Recover remains available in HELD_UNKNOWN for a fresh check after the operator
secures/clears the item or obstruction. Stale feedback, active E-stop/other alarms,
changed teach sources, unexpected I/O or failed commands prevent movement.
The completion prompt reports Home reached, fingers relaxed, suction/exhaust OFF
and DI1 LOW. Recover itself does not put an item back; use the separate Return Item
operation for a deliberate return to the source.

Native action cancellation and Stop invalidate the active command generation,
use the independent Stop client, wait for acknowledgement and two distinct
feedback samples showing an unchanged tool pose and stationary empty queue, and
preserve every gripper output. They never Home, release, disable, or resume a
discarded queue. The result is
`RECOVERY_REQUIRED`. A late motion acknowledgement causes another Stop;
unconfirmed stopping is `FAULT`. Held-item DI1 and expected-output integrity are
checked throughout Stop and Recover. Unexpected running/nonempty queue feedback
while otherwise idle is immediately routed through the same Stop confirmation
path rather than merely changing the state label.

Managed Pause uses the current operation executor. It immediately requests
independent Stop, blocks later dispatch, resolves any already-admitted command
reply, then confirms a final stationary empty-queue Stop before repositioning.
The old queue and its unexecuted output events are discarded. Unanswered/rejected
admission prevents all parking/return commands; late responses retain Stop
containment. Vendor Pause and Continue clients are removed.

Each new batch initializes `PENDING` candidates. The first accepted approach
command marks `ACTIVE`; final-pick settling without suction marks `FAILED`;
Pause marks the active candidate `INTERRUPTED` and keeps it eligible for retry.
Continue retries that same candidate; its next accepted approach marks it
`ACTIVE` again. Repeated Pause does not consume it or increase the distinct
attempted-candidate count. Confirmed suction marks `HELD`; a paused loss marks
`DROPPED`; a completed intentional held return marks `RETURNED`. FAILED, DROPPED
and RETURNED candidates remain ineligible.
Parking never marks a candidate attempted or changes an interrupted state.
The ledger and source plans are in memory, never written into teach artifacts.

Without an item, active Pick Pause neutralizes all four canonical outputs,
rises vertically at measured X/Y/attitude to at least taught Home Z, then crosses
at that height to `park_transit` above the interrupted candidate, or the next
pending candidate if no interrupted approach remains. This pause endpoint has
that candidate's X/Y and planned pick orientation, with Z equal to
the higher of stopped Z and Home Z. Both the safety rise and final transit are
confirmed; no descent or fixed settling dwell is added while parking. With no
remaining candidate it parks at safety height. Continue commands finger CLOSE
OFF then OPEN ON, confirms the outputs, then queues pre-pick followed by final
pick with the existing timed SUCK and taught settling. A retained candidate
batch is never replaced during Continue. Standalone Home replans its Home goal;
idle READY Pause remains stationary.

Held Pause preserves the outputs and rises vertically at current X/Y/attitude
to Home Z, without descending if already above it. Paused feedback, actual pose,
DI1 and outputs remain supervised. DI1 LOW lasting 50 ms on advancing feedback
during the rise requests Stop; confirmed loss while parked also starts the
same put-back routine described below. A shorter LOW followed by HIGH cancels
the pending loss. Confirmed loss is latched even if DI1 rises again.
Stale feedback or output faults are not
converted into ordinary drops. An eligible acquisition during the initial Stop
can establish trusted holding; late DI1 from a latched miss cannot.

Put-back retains the original held candidate pose even after successful Pick has
already completed at Home. From confirmed Stop it rises to Home Z if necessary,
queues the entry transit, then releases at the exact saved pre-pick pose:
nominal final-pick Z plus taught `motion.prepick_height`, at that candidate's
X/Y/attitude. Rule 113 removes the fixed +50 mm release offset and its minimum
pre-pick check. Preview and hardware use the same release geometry. No extra
descent below pre-pick or duplicate pre-pick target is queued. Holding outputs
are preserved until the release pose is physically confirmed. For an already
latched drop, DI1 LOW is expected but output/fault/feedback checks continue.

Release commands CLOSE OFF, SUCK OFF, OPEN ON, then `DO(1,1,50)`: the robot
controller ends EXHAUST after 50 ms. OPEN is established before EXHAUST, not at
an exactly simultaneous electrical edge. The pulse is independent of
`timing.pick_settling`. A bounded feedback history records the pulse ON/OFF even
when it finishes before its ROS response arrives. Missing pulse evidence, exhaust
remaining ON or DI1 remaining HIGH blocks return motion. Fingers stay OPEN during
release; the first real upward `MovLIO` return segment commands all four outputs
NEUTRAL at its start. The clearance/exit-transit/exact joint-Home
route is admitted in order and physically confirms only terminal Home.
Put-back queues entry `park_transit` before pre-pick/release and exit
`return_park_transit` after clearance, including at/near Home Z. If taught
retract height is zero, omit the coincident clearance and neutralize on the
upward exit-transit segment instead. If neither clearance nor safety Z provides
an upward retreat, reject the geometry before picking; never attach neutral
I/O to a zero-distance move. Otherwise the exit has no timed I/O, and it remains
queued even when coincident with clearance. All segments retain global CP(100).
Under rule 112, every put-back target uses
`v=100` and taught travel acceleration: initial safety rise, entry transit,
taught pre-pick release, neutral clearance retreat, exit transit and
exact joint Home. Put-back never inherits the slow approach/retract rates.
This applies to explicit return, paused drop and automatic loss return.
Explicit Recover instead preserves outputs and uses taught travel rates to Home,
then neutralizes the gripper at Home.
If another candidate follows, its normal travel and final-pick approach rates
resume after the old item's exit transit. Global SpeedFactor still applies
and is never automatically raised by a put-back.

Explicit `/return_item` cancels the interrupted action with a CANCELED result and
finishes READY at Home. The GUI uses **RETURN ITEM & STOP** while paused with
trusted holding. A paused drop runs that same release/return route automatically
and stays PAUSED at Home. Continue then attempts remaining candidates (including
from the retained batch of a completed Pick), or finishes without a pick if none
remain. A dropped candidate stays `DROPPED`; completing the motions is not proof
that the physical item was placed back. A new Pick replaces the old unheld batch.
The source context is not restored after a process restart.

Put-back also retains its destination and `APPROACH`, `RELEASING` or `RELEASED`
progress until Home completes or the next-candidate route takes ownership.
Stop before/during release preserves the source even when suction is already
intentionally OFF. A normal candidate becomes RETURNED only when release is
confirmed; a candidate with latched loss stays DROPPED. Explicit Recover abandons
that return progress and cancels remaining candidates. It uses fresh stopped I/O,
never repeats the release/exhaust, and lifts to Home height before moving Home.
Then it resets the gripper to neutral. New DI1 HIGH after confirmed release still
blocks recovery travel.

Direct `/stop`, action cancellation, shutdown, and another Stop during parking
or return cancel all further host-side commands and require recovery. The
already-requested controller pulse can still switch EXHAUST OFF on its timer.
Other faults never automatically invoke put-back or release. Confirmed held DI1
loss during an active Pick uses the automatic routine below. The idle vendor
`isPauseCmdFlag` remains contextual telemetry and does not enable Continue.

Concurrent Stop callers may share the same ongoing attempt. Every later
explicit Stop/action-cancel gets a new dispatch and physical confirmation,
including after an earlier failure or unanswered Stop. Old results cannot
complete a newer attempt or change a newly started operation's state.

Confirmed held-item suction loss marks the owning candidate
`DROPPED` while preserving its source and outputs. This records loss of vacuum
confirmation, not proof that the physical item left the gripper. A subsequent
HIGH does not clear the latch. Physical Stop confirmation still requires its
accepted response and stationary empty queue; a DI1 loss is logged separately
and does not mislabel a successful Stop or poison later Stop confirmation.
Freshness and output-integrity failures retain their strict handling.

During active Pick, rule 111 keeps that loss inside the original action. Request
Stop immediately, resolve any in-flight admission, confirm a stationary empty
queue and preserved outputs, and require fresh enabled feedback and unchanged
sources before returning to the saved item. Use the normal safety rise, entry
transit, original taught pre-pick release, OPEN and confirmed 50 ms exhaust.
Queue neutral retreat and the old exit transit before the next eligible item's
entry/clearance/pre-pick/pick, or before exact Home if none remain. Both transits
retain CP(100). The lost candidate stays DROPPED even if DI1 returns HIGH.
Repeated losses advance through the finite retained batch without new detection.
The action completes normally with SUCCESS or NO_PICK, so this loss alone does
not open the GUI's Action ended dialog and needs no Recovery click. No automatic
disable, enable, ClearError or settings sequence is issued. This also applies
to active picking resumed through Continue. Explicit Recover cancels the batch.

The dedicated held-suction-loss condition cannot turn output/readiness faults,
invalid feedback, service failures or missing source context into automatic
motion. During a pending motion reply, request Stop immediately but still require
that response to be accepted within its original five-second deadline; keep the
loss latched, send no later old-group command, then confirm Stop before put-back.
A rejected/unanswered response, unconfirmed Stop, failed release or other fault
ends the action through normal containment and explicit Recovery. Direct Stop,
action cancellation and shutdown always pre-empt; pending Pause retains its
existing put-back-and-remain-paused behavior. Idle HOLDING and standalone Home
losses continue to require explicit Recovery.

Explicit Recover is a separate cancel-and-Home-then-relax operation under rule 176. It
never invokes the automatic put-back or next-candidate routine. A repeated
Recover after interruption preserves the cancelled state and uses a new Stop,
fresh gripper feedback and a newly measured motion origin. Output changes,
unknown suction, stale feedback or failed commands stop further motion.

The GUI SpeedFactor slider tracks the handle position and sends it once on
release. Keyboard and groove changes are debounced for 350 ms. Controller status
cannot overwrite an active or pending edit, and unchanged selections do not send
another SpeedFactor request.

The GUI presents START/CONTINUE and PAUSE/STOP dynamically. Continue is enabled
only once parking completes. Immediately after requesting Pause/return and
during `PAUSING` or `RETURNING_ITEM`, **STOP NOW**
pre-empts without waiting for the managed request's response. While paused and
holding, **RETURN ITEM & STOP** requests put-back without first canceling its
owning Pick action. A pending Pause response takes priority over a newly arrived
`PAUSED` status, so the label always matches the direct-Stop click handler.
External clients can always call direct `/stop`.

The top row places Robot status and Gripper status side by side. Robot status
shows only the controller state (READY, PICKING, HOLDING, PAUSED, etc.). Hover
for its message, feedback availability, configuration ID, action detail and
ordered candidate ledger. Gripper status contains exactly two read-only LEDs:
DI1 Suction and DI12 Finger open. Green/HIGH and gray/LOW reflect raw inputs;
amber/UNKNOWN means feedback is unavailable. Text accompanies every color.
DO commands and the logical held-item state never drive these LEDs. LOW DI12
does not establish that fingers are closed, and DI1 retains its raw display
while held-item decisions retain their 50 ms loss debounce. Short input pulses
may occur between the periodic 5 Hz status updates. Full robot flags and DI/DO
bits remain available in the existing typed message for other consumers.

Item/Bin Teach fields, Browse buttons and Load/Reload occupy the smaller
top-right panel. Full paths remain editable and available in field tooltips;
prefill, explicit load, reload gates and validated persistence are unchanged.
Lifecycle/action controls, global speed and the command-log toggle remain below.
The GUI consumes only controller APIs. It adds no Dobot/camera subscription or
I/O command. Missing/stale canonical feedback turns both LEDs UNKNOWN; a
controller status older than one second by source timestamp or local receipt
also shows robot UNAVAILABLE and disables controls that depend on status. Direct Stop remains
available whenever its service is reachable. No automatic Stop or command is
sent by this display logic. Rebuild interfaces and restart controller, GUI and
preview together for the extended message definition.

Feedback is condition-driven from the approximately 100 Hz FeedInfo stream.
Policies are: five seconds for service discovery and each Dobot
service response (including Stop), five seconds for output
feedback, one-second feedback age, two-second expected mode changes, three
consistent error/collision samples, three-second no-progress watchdog, and a
300-second physical-motion cap. Home
arrival is within one degree on every taught joint; Cartesian arrival is within
5 mm and one degree, plus enabled, queue-idle and stationary confirmation.

The bridge publishes `RobotStatus.is_enable` as `robot_mode == 5`; it is an
idle-mode alias on a separate, slower publisher rather than an independent
enable latch. Fresh RobotStatus remains mandatory for connection/liveness, and
idle READY/final-arrival checks require its Boolean to converge. Active motion
uses authoritative FeedInfo `EnableStatus`, mode, error/collision, user/tool and
queue fields so an asynchronous status sample cannot cancel an accepted move.

Every actual canonical Dobot call has paired audit output in the ROS console and
`logs/robot_controller/events.jsonl`. Each `SEND` and terminal accepted,
rejected, timed-out, canceled, errored or late-response record contains a
process-local request ID, exact endpoint/request fields, ROS response `res`,
available `robot_return`, and elapsed milliseconds. This covers Startup/Recover
settings, DO, all three motion services, and the independent Stop channel. A
successful response is still only command acceptance; fresh robot feedback
remains required for completion.

The authority publishes the same timestamped human-readable state, phase and
Dobot audit lines on reliable transient-local
`/robot_controller/operator_log` (`std_msgs/msg/String`) with a retained depth of
1,000. The GUI lower panel is a read-only, no-wrap 1,000-line view of that topic,
collapsed by default to keep service request/reply traffic out of the main view.
**Show command log** expands it and exposes Copy Log; **Hide command log**
collapses it again. Messages continue accumulating while hidden, and expanding
shows the full retained buffer. The toggle is a transient diagnostic view;
no operator setup file or runtime logging policy changes.
Text is selectable with Ctrl+C, **Copy Log** copies the entire displayed buffer,
and incoming messages auto-scroll only while the operator is already following
the bottom. The topic is observability-only and does not replace typed status,
actions/services, or `events.jsonl`.

## Home and Pick

The explicit Home action is permitted from `READY` and trusted
`HOLDING`. It obtains current Link6 XYZ/RPY from fresh, stationary FeedInfo.
Unless that Cartesian pose is already within 5 mm/1° of the FK-derived taught
Home pose, it sends exactly two Cartesian-mode `MovL` targets in one named
motion group: `(current X, current Y, Home Z, Home Rx, Home Ry, Home Rz)`, then
`(Home X, Home Y, Home Z, Home Rx, Home Ry, Home Rz)`. Each service must return
`res=0` before the next is sent, without an added delay, so both enter the Dobot
queue in order and inherit global `CP(100)`. Only the final Home target receives
the 5 mm/1° stationary, empty-queue physical confirmation. Both use the taught
travel `v`/`a`, no timed I/O, and preserve/monitor suction when holding.
The same fresh stationary pose used to plan `home_align` is passed into the
motion group as its confirmed origin; the action does not acquire a duplicate
origin sample before dispatch.
This action does not send `RelMovLUser` or joint-mode Home. Cartesian arrival
does not prove the joints match the recorded Home tuple. The first segment may
rotate the tool or descend at current XY; the controller has no collision model
for an arbitrary starting pose, and CP may round that alignment control point,
so the operator must verify that the complete blended path is clear.

Pick's initial Home and standalone shared Home calls use the joint-Home rule. The
initial step skips if all six fresh actual joints are within ±1° of the taught
tuple in one feedback sample with idle mode 5, RobotStatus enabled,
`EnableStatus=1`, fault/collision clear, user/tool zero, queue empty/not
running, and held-item I/O intact where applicable. Otherwise, more than 5 mm
below taught Home Z, `RelMovLUser` first rises at current XY/attitude and confirms
5 mm/1° Cartesian arrival plus stationary/empty-queue feedback. Within 5 mm
below Home Z or anywhere above it, skip that preliminary rise and send exact
taught Home joints directly using joint-mode `MovL` and ±1° joint confirmation.
The planner and first dispatch share one fresh confirmed FeedInfo pose, so a
second origin reading cannot turn a planned upward correction into a rejected
downward move. When a rise is needed, final joint Home acquires its origin after
that rise finishes. Successful/final-miss/put-back returns always queue an
explicit Cartesian exit transit, even within 5 mm of Home Z or at an identical
clearance position. Successful and exhausted Pick returns queue pre-pick,
clearance, exit transit and joint Home together, using the confirmed stopped
pick pose as the group origin. Only final Home is physically confirmed.

Pick is permitted only from `READY` with DI1 clear:

1. run the same Home function;
2. request one fresh profile/model/camera/platform/bin-hash-matched batch from
   `/item_detect/get_item_poses`, advertised by exactly one root node: headless
   `/item_detect` or explicitly Armed `/item_teach`;
3. transform platform-relative targets into base coordinates;
4. offset Link6 green/Y by the taught `pick_rotation` from each item's short-axis
   line while preserving taught tool Z;
5. attempt up to Item Teach `pose_candidates` in detector rank order;
6. after an intermediate miss, retract to that candidate's final clearance and
   proceed through the next candidate's safety-Z transit, clearance, pre-pick
   and final pick without returning Home;
7. after success, retract and return Home holding with suction on.

The detector pose uses local X for the measured short axis and local Y for the
long axis. It is an in-plane heading, not a TCP attitude. The controller
composes that heading through the destination platform transform, projects the
short X axis perpendicular to the exact taught Home tool Z, then considers the
taught unsigned 0–90° `pick_rotation` on either side of that line. Since a
rectangle has no directed end, both modulo-180° directions are equivalent.
Every candidate independently chooses the legal attitude with the least CW/CCW
travel from taught Home; travel ties within 1e-12 radians prefer the CCW offset
so frame relabeling cannot magnify floating-point noise into a different turn.
All candidate poses are therefore ready before execution
and their rotations never accumulate across retries. Every candidate's
transit/descent/retract targets share its selected attitude; exact
joint Home restores the taught orientation. Platform tilt is not copied into
TCP roll/pitch, and all waypoint heights remain referenced to base Z.

Rule 157 changes item coordinates from long-X/short-Y to short-X/long-Y and
updates this planner together, so the physical gripper orientation remains the
same. Teach-file dimensions, pick points, Home and motion rates are unchanged.
Rebuild/restart perception and all controller processes together. Pose requests
send `pose_convention=item_short_x_long_y_v1`, and matching response evidence is
mandatory before candidate planning; older or unspecified conventions fail
visibly instead of being interpreted as a new item frame.

The current station also requires the latest strict schema-7 robot-camera
transform `Link6 <- robot_camera_link` (transform only). For every candidate,
preview and hardware independently compose its calibrated camera-link origin
at Link6's pick height (`item Z + standoff_height`) into `platform_reference`.
The normal Home-relative pick attitude is used if that origin is inside/on the
green Bin Teach ROI; otherwise the exact 180° tool-Z mirror is used if safe.
The mirror preserves the undirected short-axis line and unchanged tool Z but
may exceed the normal 90° Home-relative travel limit. If both origins are
outside, planning fails before hardware candidate motion; the detector must
have excluded that pose before ranking, allowing the next safe candidate to
take its place. Robot-camera calibration SHA-256 is part of configuration and
detector evidence. This origin-only constraint does not model the housing;
maintain actual physical safety margin inside green. Blue inset checks still
apply solely to the item pick point.

The schema-9 geometry uses pick Z equal to item Z plus `standoff_height`,
pre-pick adds `prepick_height`, and clearance adds `retract_height`. Home/travel
uses taught travel rates and final descent uses approach rates. A successful
pick's first rise to pre-pick uses taught retract speed/acceleration. A missed
pick's same rise uses `v=100` and taught travel acceleration. The next clearance
rise uses `v=100` with taught travel acceleration for both outcomes. All rates
remain subject to global SpeedFactor. Put-back uses `v=100` and travel
acceleration throughout, including release descent and empty retreat.

The outputs are explicit mutually exclusive states. Finger OPEN is DO2 OFF then
DO14 ON, CLOSE is DO14 OFF then DO2 ON, and NEUTRAL is both OFF. Vacuum SUCK is
DO1 exhaust OFF then DO13 ON, EXHAUST is DO13 OFF then DO1 ON, and NEUTRAL is
both OFF. Timed state changes preserve that order, and feedback showing either
opposing pair ON together faults the motion. `use_grip` controls CLOSE only;
every candidate is presented with OPEN and no DI12 wait.

The first `candidate_1_home_to_pick` group contains three control points: item
X/Y at Home Z with OPEN at 50%, pre-pick with no I/O, then final pick with SUCK
at 20%. It intentionally skips the item-clearance point on this initial descent.
DI1 is eligible only after the candidate's SUCK transition. A DI1 acquisition
during descent invokes Stop-and-confirm. Otherwise the terminal pose must remain
within tolerance with advancing queue-idle feedback and the commanded final
outputs for the profile's `pick_settling` interval while DI1 is monitored. This
is the complete final-pick confirmation interval; there is no fixed 300 ms pick
gate before it or separate sensor wait after it. If DI1 is still low when the
interval ends, that attempt is irrevocably missed.

DI1 HIGH-to-LOW uses one fixed `SUCTION_LOSS_DEBOUNCE_SEC = 0.050` filter owned
by the canonical feedback monitor. After HIGH has been seen, the first advancing
LOW sample starts a monotonic timer. Advancing LOW feedback at least 50 ms later
confirms loss; any HIGH resets the pending interval immediately. Re-reading a
snapshot or publishing the same controller timer cannot complete the debounce.
A feedback gap beyond the existing freshness limit cannot count toward it, and
stale/invalid feedback keeps its existing failure handling.

The internal snapshot's `suction_present` value is used after acquisition Stop
and throughout held motion, motion-origin/Home checks, Stop/recovery, idle
holding and managed Pause/Continue/put-back. Raw `digital_input_bits` and output
history remain unchanged: first HIGH pickup detection, cold/untrusted DI1,
missed-pick suction reset, release/exhaust confirmation and unheld checks still
use raw DI1. DO13 loss, opposing outputs and other faults receive no new delay.
Direct Stop is sent immediately and its stationary confirmation never waits for
this timer. The debounce is independent of taught `pick_settling` and the 50 ms
exhaust pulse; it introduces no teach setting, launch argument or schema change.

On success, `use_grip=true, grip_onpick=true` enters CLOSE immediately after
confirmed containment. With `grip_onpick=false`, CLOSE instead occurs at 100%
of the clearance rise (DO14 OFF before DO2 ON). `use_grip=false` never enters
CLOSE. The held return uses the exhausted-miss route: actual stopped
pose to pre-pick, clearance, exit transit and exact joint Home, in one
`candidate_N_pick_to_home` group. Both initial rises preserve stopped X/Y and
attitude and never descend. The first held lift uses taught retract rates;
the clearance rise uses `v=100` with taught travel acceleration.
Exit `pN_transit_exit` keeps that X/Y/attitude and
uses `max(stopped Z, taught Home Z)` with taught travel rates and no I/O.
It is always queued, including when no further rise is needed. No intermediate
arrival wait is added. SUCK stays ON without reissuing it; no EXHAUST or NEUTRAL release
events are sent. Continuous holding checks, including the 50 ms DI1 loss
debounce, remain active through dispatch and final Home confirmation. Held
Continue queues an exit transit at the parked X/Y/attitude before joint Home.

A missed non-final candidate starts one
`candidate_N_pick_to_retry_M_pick` CP-blended group: old final to old pre-pick,
old pre-pick to old clearance, N's exit `park_transit`, M's entry `park_transit`,
then descend through M's clearance and pre-pick to M's final pick. The exit
preserves actual stopped X/Y/attitude; the entry uses M's X/Y/attitude. Both use
`max(stopped Z, taught Home Z)` and taught travel rates. The exit is named
`pN_transit_exit` and has no I/O. The entry's command name remains `pM_transit`
so admission marks the correct candidate
ACTIVE; a Pause there retains M for Continue. The same geometry helper supplies
Pause's `park_transit`, which neutralizes its I/O. Unheld Continue reuses that
already-confirmed entry transit. Every pick and put-back has both entry and exit
transits queued. Both are blended under CP(100) and can be rounded without an
intermediate arrival wait or dwell; coincident coordinates still get separate
requests. Direct Stop can always prevent later requests from being sent.
At 80% of the first rise the group enters EXHAUST. At 0% of the second rise it
enters both finger and vacuum NEUTRAL. At 50% of the transfer to M's transit it
enters OPEN; M's clearance has no I/O. At 20% of M's final descent
it enters SUCK. Each service must return `res=0` before the next is sent, while
only M's final target is physically checked. A late DI1 from candidate N is
ignored throughout its latched-miss recovery; candidate M is armed only after
DO13 OFF and a clear DI1 have been observed before its new SUCK. A final
candidate miss queues its old pre-pick EXHAUST rise, old-clearance NEUTRAL rise,
explicit exit transit and exact joint Home as one
`candidate_N_pick_to_home` group. Each request still requires ordered `res=0`
acceptance, but only exact joint Home is physically confirmed; clearance and
transits are blended control points. Later DI1 cannot reclassify the latched miss
as success. Successful returns use the same geometry, ordered group and
final-Home-only confirmation, with taught retract rates on the first lift and
held-item outputs and monitoring throughout.

All `MovL`, `MovLIO`, and `RelMovLUser` requests in one named batch are admitted
in target order. Each must return `res=0` before the next is sent, with no
additional inter-command delay. This is an admission barrier, not an
intermediate physical-arrival wait: it prevents separate ROS services from
reversing their dashboard TCP queue order, as observed in a failed Home return.
A response error, rejection, cancellation, or five-second response deadline invokes independent
Stop containment; an outstanding late response remains contained by another
Stop. Batch start, every dispatch/response, complete group admission,
interruption and terminal completion are recorded with the batch name.
An armed new-candidate DI1 or cancellation during admission prevents all later
targets in that group from being sent; late DI1 from a latched miss is ignored.
The independent Stop path bypasses normal admission. During a held-item return, a
timed DO2/DO14 transition requested by MovLIO is accepted only as the exact
old-to-commanded state change after that MovLIO has been sent, and becomes the
new expected state when observed;
uncommanded output changes, lost DI1/DO13, and wrong terminal states still fail.

Motion requests carry only `user=0`, `tool=0`, and their selected `v`/`a` rates;
they never carry a per-command `cp` or `r`. The global `CP(100)` established by
Startup/Recover therefore controls all transitions. As specified by the Dobot
protocol, smoothing can bypass exact intermediate pick coordinates and timed
I/O can occur during a blended transition. Successful and exhausted Pick returns
both queue through Home and physically verify only exact joint Home.
Serialized service responses may let
a short pick segment decelerate despite global CP 100; queue order takes
precedence over uninterrupted blending.

No-I/O targets use `MovL`. `MovLIO` is used only for a real non-empty timed DO
tuple. Initial/shared Home's conditional rise uses `RelMovLUser`; item exit
transits use Cartesian `MovL`. The controller never calls
`InverseKin` or vendor `Continue`; controller Continue rebuilds the remaining route.
Service acknowledgement is acceptance only; actual
feedback confirms every result. A coherent miss advances to the next pending
candidate; Continue retries an approach interrupted by Pause. All command,
feedback, state, cancellation, and result events
are written to ignored `logs/robot_controller/events.jsonl`, capped at 1,000.
Each candidate plan also records its source quaternion, transformed short axis,
commanded green axis, configured offset, selected CW/CCW side, rotation from the
taught Home reference, and target RPY.

Joint Home completion uses ±1° independently on every joint. Cartesian target
completion uses 5 mm Euclidean translation and 1° orientation. Home, clearance
and every other non-pick endpoint complete on the first fresh enabled,
queue-idle feedback sample within the applicable tolerance after the complete
group's acceptance, with a newer sequence and advancing controller timer.
`MovL` exposes its queue ID in the existing reply: require that exact
`FeedInfo.currentCommandId` at completion. The fixed vendor `MovLIO` and
`RelMovLUser` response schemas expose only `res`; those endpoints instead require
live execution evidence latched during dispatch/travel (running/queued status,
changed queue ID or actual pose movement). Short/zero-distance commands can
complete without ever observing a running flag when the stream shows their
execution. No new query service or midpoint wait is added. Optional managed
safety rises within the existing 5 mm tolerance are skipped before dispatch
using the actual pose. Only a final pick
uses a timed settling interval: its taught `pick_settling` duration under the
same feedback gates. Home remains joint-only and does not additionally compare
Cartesian FK with the streamed actual tool pose.

Before every independently acquired motion-batch origin or stopped-pose
measurement, the controller waits up to two seconds for coherent idle feedback
and an advancing FeedInfo `controller_timer` (brief duplicate publishes are
allowed, but no source freeze longer than 150 ms). There is no additional dwell
duration. The final-pick confirmation sample supplies its stopped pose and is
carried directly into the immediate retract/return batch. The controller uses
`tool_vector_actual` from the same validated sample; it never sends a separate
`GetPose` request or subscribes to the slower, unstamped `ToolVectorActual`
topic. A timeout names every current
RobotStatus/FeedInfo blocker, or reports that otherwise-valid fields could not
produce one coherent advancing sample. Fresh connected RobotStatus, joints,
FeedInfo, user/tool zero, held-item integrity, and the normal final-arrival
checks remain mandatory. Startup/Recover retains its separate 200 ms READY
lifecycle coherence. A live comparison of streamed pose with
`GetPose(user=0,tool=0)` is a separate commissioning check, not an automatic
fallback or an extra runtime service dependency.

Software tests use synthetic services/feedback and must never commission
physical motion. Real commissioning requires separate explicit authorization,
clear workspace, functional physical emergency stop, verified wiring, and an
attentive operator.

`robot_controller` is the sole production/runtime Dobot command issuer.
`motion_debug` may issue direct commands only as a mutually exclusive maintenance
application; production Startup rejects competing maintenance command owners.
