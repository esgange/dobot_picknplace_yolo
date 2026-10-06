# Robot Controller

Visual guide: [Controller finite state machine](../../docs/ROBOT_CONTROLLER_FSM.md).
It covers the current lifecycle, candidate ledger and operation/recovery routes;
update it in the same change whenever those behaviors change.
Ready-to-view versions: [visual HTML](../../docs/ROBOT_CONTROLLER_FSM.html) and
[visual PDF](../../docs/ROBOT_CONTROLLER_FSM.pdf), generated from that document.

`robot_controller` is the sole production application-level authority for the
physical CR10. The GUI provides Home, Pick Item and Place Item with one Preview toggle,
plus Auto Run with an adjacent quantity field and completed count.
It does not launch Dobot bringup, cameras, Item Detect, or RViz.

## Auto Run

Start from configured, unheld READY with recorded tray joints and both pose
providers available. Select a whole quantity from 1 to 10000; the run freezes the
current placement X/Y/rotation and debug-image choice. One native `AutoRun` action
owns all hardware commands until completion/failure/cancellation. The GUI disables
manual motion, Preview, teach loading, speed/CP, Pause/Continue/Return and input edits;
permanent STOP remains enabled. External manual actions cannot acquire the operation
slot. After three unavailable tray observations, Auto Run instead enters the
normal acquisition Pause and permits Continue or Return Item, as described below.
Other manual queue-control requests reject while Auto Run is active.
Preview cannot start Auto Run. Launch/prefill never starts it.

Successful tray placement discards unused poses; the next manual or Auto Run
Pick uses a fresh bin batch. Original poses remain eligible for misses,
Pause/Continue and drop/return recovery until a successful placement. Preserve
initial Pick Home skip/arrival, candidate plans, timed I/O, acquisition Stop
acknowledgement, lifts and saved Tray Detect destination.
Tray pose acquisition begins only after Pick confirms saved tray joints and idle
feedback. Auto Run always requires the trusted picked item for placement, including
in GUI mode. Placement retains its existing approach/release/retract queue and rates.

After Pick confirms Tray Detect joints/idle, acquire the tray pose and placement
depth first, including any of the three allowed observation attempts. After the
validated result and observation-position check, start one read-only worker requesting
a fresh next-bin candidate batch whenever another item is needed, even if the
old batch still contains eligible poses (rule 218 supersedes rule 215).
It runs while the owning action plans and sends the placement approach/release/retract
queue, overlapping command admission and execution. The worker never sends hardware
commands. Require ordered acceptance of all three placement commands before consuming
even an already-ready result or appending next-Pick motion. Tray failure or Stop
before the worker starts prevents the request; a subsequent placement admission or
execution failure cancels/discards it. Capture remains a fresh post-request
RGB/depth/TF observation.
Keep the fixed bin camera's view clear during placement; no automatic occlusion
test is added. Manual Pick/Place do not launch this lookahead.

Keep the original batch and held source until placement execution/release is
confirmed, so tray failure or an interrupted drop can still use its saved poses.
At successful placement, mark the old item PLACED and cancel unused candidates.
Never select an old remaining candidate for the next normal placement cycle.
The prefetched result gets its own ledger; its candidates cannot acquire ownership
until the old placement crosses the execution/release boundary.

Consume the result using the existing handoff/error path. When valid poses are ready,
append next entry → pre-pick → final pick directly **without Home or a placement
arrival wait**. The dashboard executes these requests after every accepted
placement command, with the selected global CP. Entry travels from the planned
tray retract to item X/Y at Home Z and retains finger OPEN at 50%; final descent
retains SUCK at 20%. If detection is slower than placement, confirm/count retract
and wait unheld, then use the same direct next Pick without an initial Home.
The fresh batch is retained for retries/recovery until the first successful placement. Manual
Pick → Place cycles discard the old batch at placement completion and acquire
fresh poses on the next explicit Pick.
Source/hash/attitude checks and three nonempty batches per Pick remain in force.
Each Pick has one retry for an empty pose result: confirm Home before requesting
again. An empty prefetch uses the owned placement-to-Home route and counts the
completed placement before this retry. Another empty result ends NO_PICK at Home
with the completed quantity. Empty observations do not consume physical-pick batches.

The handoff retains the previous placement and source until advancing FeedInfo
reports the next pre-pick MovL's returned queue ID (or a later ID), with observed neutral
gripper outputs and DI1 LOW since placement admission. This proves execution passed
the placement queue without inventing a MovLIO queue ID or requiring a midpoint
stop. Entry remains MovLIO and returns no ID; pre-pick is the second appended
command. Apply this handoff before interpreting any new-pick suction as old-item
feedback, including when feedback jumps directly to final descent. Only then count
that placement, mark the old candidate PLACED, activate the
next candidate and permit its suction acquisition. Cancel the old batch's unused
poses and install the fresh ledger. Old held DI1 cannot
trigger the next Pick. Missing boundary/release evidence faults and Stops the run.
An interrupted handoff retains the correct source for explicit Recover. Counted
placement is robot execution/release evidence, not camera proof of item delivery.

Initial Pick and empty-result recovery retain Home. For the final item, append
MovJ Home immediately after placement admission without
requesting another item batch. Confirm final Home, neutral outputs and DI1 LOW
before Auto Run SUCCESS/READY. Status publishes `auto_run_active`, requested and
completed quantities; the action result preserves the final/partial count.
Three exhausted nonempty Pick batches, or an empty result after the one Home
acquisition retry is used, end READY/NO_PICK at Home. Three unavailable tray
requests confirm Stop and enter **PAUSED at Tray Detect**, retaining the item,
source, target, operation owner and completed count. The same paused controls
offer **Continue**, **Return Item** and permanent **STOP**; Place Item (Retry) is
an alias for Continue. Continue grants three fresh tray requests for that same
item and resumes the run after a valid result. Another exhausted batch pauses
again. No new Pick or next-bin request starts while waiting. Return Item uses
the shared return queue, ends Home/READY and cancels Auto Run with its unchanged
partial count. Return requires no tray provider; Continue requires the provider
and unchanged safe parked state. A fault or held loss while awaiting the operator
cannot resume counted production automatically; normal Stop containment applies.
Malformed successful detector evidence and robot/transport faults still require
explicit recovery; no fourth physical-pick batch, extra empty-result retry
or automatic restart is granted. STOP
cancels the run and outstanding prefetch, discarding its result. Tray exhaustion
prevents prefetch; held loss or another fault during placement discards it.
Explicit Recover cancels remaining poses; reloading configuration and process
restart invalidate reuse. Successful Return Item excludes the returned candidate
but retains remaining poses for a later explicit Pick/Auto Run. Manual Place shares this
acquisition-pause/retry workflow. Rebuild and manually restart controller/GUI
after this change; no teach-file or interface changes are required.

Launching the package never enables, recovers, homes, or moves the robot. An
operator explicitly loads teach configuration to prepare the robot to READY.
Headless supervisors retain the explicit Startup service after deployment loading.

## Cycle audit and acquisition performance

The 2026-10-01 audit before rule 202 checked manual Pick/Place, missed candidates, held-loss
put-back, Return Item, Auto Run handoff and final Home against their shared
planners/executors. The latest complete recorded Auto Run finished 3/3 in 53.65 s
(12:52:05.898–12:52:59.552 Dubai time), with first-candidate picks and first-attempt
tray/depth results. The two next-item requests started after placement admission
and overlapped execution; their validated results arrived 1.69 s and 1.08 s after
retract completion. Final Home was appended immediately behind the last placement.
Rule 209 now starts the worker after validated tray/depth acquisition, overlapping
placement admission as well as execution while retaining the complete placement
admission gate before any next-Pick motion. No new physical cycle timing has been measured.

Repeated source validation was a measured CPU cost. Strict catalog scans now
share one implementation and reuse camera-prefix parsing only for identical
file bytes. Every scan still reads files and checks their names, symlinks and
selection; selected calibration parsing, schemas, hashes and validation checkpoints
remain unchanged. No perception result is cached. An alternating seven-pair
offline benchmark on the local configured artifacts reduced median validation
from 425.09 ms to 248.66 ms (41.5%). This measures validation only, not a new robot
cycle time; first-time parsing still runs normally. Rebuild Item Perception and
Robot Controller, then manually restart the controller to use this optimization.

Further work suggested by the audit: measure per-request validation, preview-lock
wait, inference, geometry and reply validation separately; avoid overlay generation
in production requests when no display/debug image consumes it; record failed depth
sample counts/rejection reasons; retain a complete run trace across the current
1,000-event log reset. These are follow-ups, not changes to this cycle's behavior.

## Return Item compared with tray placement

Explicit Return Item requires a trusted held source, including after failed tray
acquisition. It never requests another bin/tray pose. The original saved pre-pick
matrix supplies its exact drop position and attitude; `trayplace_height` is unused.

| Motion | Return Item | Place Item |
| --- | --- | --- |
| MovL approach | Saved item X/Y and pick attitude at Home Z | Tray target X/Y and tray attitude plus rotation at Home Z |
| MovLIO descent | Exact saved pre-pick target at 100% | Detected surface Z + trayplace_height at 100% |
| At 80% descent | DO2 OFF, DO14 ON (open), DO13 OFF, DO1 ON (exhaust) | Same |
| Queue admission | One complete ordered group; no drop-arrival wait | Same |
| MovLIO retract | Same item X/Y/attitude back to Home Z | Same tray X/Y/attitude back to Home Z |
| At 0% ascent (start) | DO2/DO14/DO1/DO13 OFF | Same |
| Final MovJ | Exact taught Home joints | None for manual Place |
| Physical completion | Home joints/idle/execution, neutral outputs and DI1 LOW | Final retract/idle/execution, neutral outputs and DI1 LOW |

Return uses one ordered CP (default 100%) group: approach/drop/retract/Home. When
starting more than 5 mm below Home Z, prepend a vertical MovL at current X/Y/attitude
to the same group. Confirm only final Home from fresh joints, idle and execution
feedback. No intermediate arrival, settling, release-I/O,
DI12 or exhaust-pulse wait is added. All motions use speed 100%, scaled by global
SpeedFactor. Approach/Home
use taught travel acceleration, descent uses approach acceleration, and retract
uses retract acceleration. Finger OPEN, suction OFF and exhaust ON occur at 80%;
exhaust ends as retract begins. No separate 100% event is sent.
The shared planner/observer keeps the placement I/O timing and release supervision
identical. Observed release means finger-open/exhaust outputs ON, finger-close
and suction OFF, and DI1 LOW; DI12 can be HIGH or LOW.
Only final Home completion marks the candidate RETURNED and clears its
source; queue acceptance alone cannot report READY or successful return.

Direct Stop pre-empts admission/execution and retains observed release/source
progress. Explicit Recover cancels the interrupted queue, preserves current grip
through its existing lift/Home, and resets at Home; it never repeats the drop.
Rule 212 makes this paused Return Item route the shared reference for automatic
suction-loss returns too. Paused drop appends Home and remains PAUSED; an active
drop appends the next saved candidate instead. No configuration or interface
changes are required; restart the controller after rebuilding to apply.

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
clearance → Enable sequence. Only Stop acceptance is required before clearing;
stationary joints and an empty queue are confirmed after enabled feedback,
before settings or recovery motion. An acknowledged ClearError alone is not success.
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
which automatically prepares the robot to READY. Explicit Place Item accepts an
empty or held robot in either launch mode. Pick finishes at the recorded Tray Detect joints.
**Place Item commands travel to the observation pose when needed.** It checks fresh
RobotStatus idle and all six `/joint_states` within ±1° of saved Tray Detect,
just like Pick's immediate Home skip. If matched, observe immediately; otherwise
send the existing direct joint-target MovL at 100% with taught travel acceleration
and preserved outputs. Confirm execution/idle/joints before requesting the tray.
Failure or interruption performs existing containment and blocks detection and
placement. There is no separate observation-position button; the typed
Tray Detect Position action remains available to external clients with its direct
joint-target MovL. The bin routes retain their existing clearance logic.
After confirmation, request fresh tray/depth from Armed Tray Teach or headless
Tray Detect, with
**three requests per acquisition batch**. Retry a missing pose/depth result,
provider no-result/error/BUSY reply or response timeout at the same observation
position. One attempt is **fresh tray pose → fresh placement depth**. A depth
failure consumes that attempt even when a tray was found; the next attempt repeats
tray detection and depth sampling from a new observation, with no saved-pose reuse.
The first attempt plus two retries share one budget, not separate pose/depth budgets.
Each request keeps its taught request timeout plus one second for the
reply and requires RGB/depth captured after that request. A timed-out local future
is discarded; a late reply cannot supply a later attempt. The provider serializes
inference. Ordinary Pause/Continue keeps the request count and discards interrupted
results after completion or their original deadline. After three unavailable
observations, confirm Stop and enter **PAUSED at the saved Tray Detect position**,
with the original Place/Auto Run action and command ownership retained. Preserve outputs
and any held source; queue no placement, release or parking rise. Report the final
reason in status, action feedback and operator log. **Continue / Place Item (Retry)** starts
another batch of up to three fresh requests, after the usual position/source checks.
There is no automatic fourth request. Another exhausted batch pauses again.
Local source/ownership/feedback faults and invalid successful pose
evidence remain terminal. Run exactly one provider; commands remain controller-owned.

During this acquisition pause, Pick Item stays disabled. The existing Pause
control shows **RETURN ITEM**, with **CONTINUE** in the same split-button menu
as an ordinary held pause. Place Item is also labelled **Place Item (Retry)**.
Return stays available if tray detection becomes unavailable; retry requires its
provider. The ongoing Place/Auto Run action does not block these choices. Auto Run
shows PAUSED with its retained completed/requested count. Pending return disables
Return and Retry; permanent red STOP always stops/cancels. Without a trusted held
source, the primary action is Continue and Return stays disabled.

A trusted held item may be returned to its saved
bin source through one queued approach at Home Z → saved pre-pick drop →
retract to Home Z → taught joint Home, without intermediate arrival waits or
settling. Use tray placement's 80%-descent finger OPEN/suction OFF/exhaust ON
and 0%-ascent neutral events,
with a preliminary vertical rise in the same queue
when needed. Return finishes READY
and ends the waiting Place/Auto Run action as CANCELED, without claiming placement
success or incrementing Auto Run's completed count.
The gripper is preserved until the motion-timed release begins.
GUI placement's relaxed item-presence policy does not apply to the bin return;
trusted held-item checks apply before dispatch; queued release supervision matches placement.
Return is blocked during ordinary placement Pause and after release admission.
Direct Stop always pre-empts; stopped/faulted operations still require Recover.

Tray configuration uses the active eye-on-hand calibration saved in the shared
`ITEM_TEACH_ROBOT_CAMERA_CALIBRATION` selection, independently of the camera
recorded when the tray plane was taught. Use the same active camera in Tray Teach
or headless Tray Detect; returned camera hashes must still match. Recalibration
preserves the saved plane and detect joints. Reload controller configuration and
restart headless detection after changing calibration; GUI Tray Teach can load
the replacement explicitly and must be armed again. Loading never moves the robot.

Pick Item is enabled only in idle READY with its Item Teach/Detect service
available and a loaded Tray Teach with recorded Tray Detect joints. Pick does
not require tray detection to be armed. Place Item needs a recorded Tray Detect Pose and its Tray Teach/Detect
service. Explicit Place, including through a headless controller,
is available from idle READY or HOLDING, with or without an item or picked-item
record. No suction-presence prerequisite applies during observation or approach.
The sequence still commands real hardware; final neutral I/O and DI1 are checked
at final retract. Auto Run retains its trusted picked-item source and held-suction
guards. Status reports `manual_placement_enabled=true` for explicit Place in both
launch modes; it does not permit placement from a fault or unknown-item state.

The controller reports `item_detector_ready` and `tray_detector_ready` in typed
status at 5 Hz. It checks service availability and exactly one allowed provider:
root Item Teach or Item Detect for Pick; root Tray Teach or Tray Detect for Place.
Teaching disarm removes the service. Missing, foreign, namespaced or duplicate
providers disable the corresponding button. The controller checks again at action
admission, so stale GUI readiness cannot bypass it. These are read-only checks;
no detector trigger, model inference or automatic arming occurs. Fresh request-time
source/pose/depth validation still applies. Home and Tray Detect Position remain
available under their usual guards even when detection is disarmed.
Auto Run also displays its blocking reason beside the controls. It can start away
from Tray Detect, but both detectors must be available. CameraInfo changes disarm
Tray Teach; re-arm its validated setup before using Place or Auto Run.

Rebuild `robot_controller_interfaces` and `robot_controller`, then restart all
controller/GUI clients together for these status fields.

Tray requests use `/tray_detect/get_tray_pose_v3`, the depth-capable contract.
The earlier endpoint is never used as a fallback. Restart Tray Teach/Detect and
Robot Controller together after rebuilding; an old provider leaves Place disabled
instead of accepting an incompatible request. No robot motion is sent by detection.

Enter positive **X (mm)** and **Y (mm)** from the detected tray origin along its
inward short-X and long-Y axes, and **Rotation (−180° to +180°)**. At 0° the tool uses
the recorded Tray Detect Pose attitude; the offset rotates about that tool's Z.
Item axes and pick rotation have no effect on placement orientation.

Placement keeps requested X/Y and measures surface Z using the Item Teach depth
sampling diameter and quality settings. Valid original pixels after range/MAD
filtering must cover the taught percentage of the full sampling circle (default
50%); no fixed sample-count threshold remains. Empty samples fail. The detector,
service and controller apply the same rule. Release Z is surface Z + the explicit
Item Teach `motion.trayplace_height` in millimetres (rule 184). This placement
clearance is independent of standoff, pre-pick and retract heights; Preview uses
the same formula.
Pre-place/retract uses the placement X/Y and orientation at taught Home Z,
matching the first Item Pick approach (`pN_transit`) before pre-pick (rule 171).
That initial Pick route skips its lower clearance/initial point. For surface Z
250 mm, trayplace_height 40 mm and Home Z 800 mm, release is Z 290 mm and
approach/retract Z 800 mm. The final pose stays above the tray at this height
with the detect-relative attitude; there is no final Home command.
Home Z must be above drop Z for percentage I/O on both legs; invalid geometry
blocks the placement queue. Require a complete schema-12 Item Teach profile with
a finite, nonnegative `trayplace_height`. To use older profiles, open them in
Item Teach, explicitly fill this blank recovery field, Save and reload/redeploy
the updated pair. Production readers do not supply a fallback height.
Schema-10 profiles retain that height during GUI recovery; review the retained
**Minimum valid depth (%)** and Save as schema 12 to remove the fixed-count field.
The arrival check adds no fixed settling interval, new FeedInfo tick or pose query.
Recheck idle and saved joints during observation and before using its result.
Then send three Cartesian commands in one queue:

| Command | Target | Timed outputs |
| --- | --- | --- |
| MovL | Pre-place | Preserve existing outputs |
| MovLIO | Release height | At 80%: DO2 OFF, DO14 ON (open), DO13 OFF, DO1 ON |
| MovLIO | Back to pre-place | At 0% (start): DO2 OFF, DO14 OFF, DO1 OFF, DO13 OFF |

All placement segments and Tray Detect Position use **speed 100%**, scaled by
global SpeedFactor; placement never changes that slider. Acceleration remains
Item Teach travel for Tray Detect Position, then travel / approach / retract
for placement. Item Pick retains its taught speeds. No teach-file edit is required.
There is no placement settling, separate release call, fixed-duration exhaust
pulse, extra retract-height waypoint or separate Home action. At 80% of descent,
open fingers, turn suction OFF and exhaust ON. Exhaust lasts
until the upward command starts. Zero uses the existing distance-mode
start trigger (`{1,0,channel,0}`), carried by MovLIO. All commands inherit CP (default 100%); control points
can blend through the complete group. There is no drop-arrival, settling or
release-I/O wait between descent and retract.
Return **PlaceItem SUCCESS after all three commands are accepted**,
normally with `final_state=PLACING`. The action result acknowledges the final queue.
A completion worker retains the operation slot and PLACING status while the robot
moves, so another motion cannot overlap. It uses the existing joint-FK/idle,
execution, freshness, output and watchdog checks. Direct Stop, Pause and shutdown
still pre-empt. This worker adds no ROS executor thread.

Rule 170 removes intermediate release-confirmation gates in both modes. Send the
entire approach → pre-pick-equivalent drop → retract queue without waiting
for finger-open/exhaust, DI12 or DI1 transitions. Missing intermediate release evidence and history gaps alone do not interrupt
the queue. Rule 211 interrupts confirmed held loss before observed commanded
suction OFF; release admission and finger transitions do not disable monitoring. Any observed
coherent release feedback is retained as diagnostic/recovery evidence only.
Keep command acceptance/order, live enabled/error/collision/freshness checks,
opposing-output protection, motion watchdogs and direct Stop/Pause. These are
hardware/transport checks, without intermediate arrival or release-I/O confirmation.

Only at physically confirmed idle retract, require neutral DO1/DO2/DO13/DO14 and
DI1 LOW before READY; DI12 need not be HIGH. A bad final grip reports a
specific retract-reached fault without trying to release again. Successful retract
completion records PLACED for an existing held candidate and clears holding;
the `placement_retract_completed` event records `release_feedback_observed`.
Failures after action acceptance use Stop containment and report through typed
status/events; they cannot change the already returned action result.
PLACED now describes the completed queue and clear final grip, not proof that
the item was physically deposited on the tray. An empty test creates no candidate.

Placement Pause stops in place and preserves outputs. Continue reobserves when
the release command has not been issued, after rechecking saved Tray Detect
position within the same three-second window. It never moves back there. Once
issued, never repeat descent or release, even when its feedback was missed.
After confirmed release, Continue neutralizes outputs, retracts upward
from actual position to at least pre-place height if needed, then ends there.
An interrupted partial release with insufficient confirmation remains blocked;
no automatic release or bin put-back is inferred. Direct Stop requires explicit
Recover, which cancels the placement and preserves current outputs while lifting
to Home height and returning Home, then relaxes all four gripper outputs. It never
replays the expired placement history
or waits for continuous exhaust to switch itself OFF. A failed
tray/depth observation performs no placement or release. Pick settling,
within-batch retries, camera/bin avoidance, rotation and Home paths are unchanged.

The controller UI's strict schema-3 last-session store preserves Item/Bin/Tray
filenames and the last validated X/Y/Rotation as unapplied prefill. Explicit
in-memory import of schema 1/2 preserves their selections; disk changes only on
configuration save or a valid Place request. First-use X/Y remain empty.

New APIs are `GoTrayDetectPosition` at `/robot_controller/go_tray_detect_position`
and `PlaceItem` at `/robot_controller/place_item`, both with the active configuration
ID. Configure accepts `tray_teach_file`; status includes `tray_configured`,
`tray_position_recorded`, `manual_placement_enabled`, `TRAY_POSITIONING` and `PLACING`.
Auto Run placement requires a trusted HELD candidate; explicit Place does not.

Rebuild `tray_perception_interfaces`, `robot_controller_interfaces`,
`tray_perception` and `robot_controller`, then restart the tray provider and
controller/GUI together. Launch alone still sends no robot commands. Software
validation uses synthetic feedback and isolated ROS; physical placement requires
separate commissioning.

## Processes

- `robot_controller` is the background hardware authority. It alone creates Dobot
  motion, Pause/Continue/Stop, robot-setting, and gripper-output clients.
- `robot_controller_preview` calculates and broadcasts TF-only Home/Pick/Place plans.
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

The **Save item/tray debug RGB/depth** checkbox is captured when Pick Item,
Place Item or Auto Run starts. Item requests save under `debug/pick_img/`;
tray requests save under `debug/tray_img/`. Auto Run passes the same flag to
initial/next-item detection and every tray acquisition. Tray observation retries
and Pause/Continue retain the original choice. Unchecked requests and controller
Preview do not save images. Saving uses each detector's existing request snapshot
and diagnostic error reporting, without another inference or background archive.
`PlaceItem` now includes `bool save_debug_images`; rebuild interfaces/controller
and restart controller and GUI together. Detector service definitions are unchanged.

Actions:

- `/robot_controller/go_home` — goal contains `configuration_id`.
- `/robot_controller/pick_item` — goal contains `configuration_id` and the
  one-shot `save_debug_images` flag. Candidate count cannot be supplied by the
  caller; it comes from Item Teach `retry.pose_candidates`. Each full candidate
  nonempty batch is one physical-pick attempt, with **three attempts total**.
  Successful tray placement invalidates unused poses; its next Pick requests
  fresh ones. Poses retained after interruption/return may still be used. Then
  ensure Home before candidate motion. A valid empty
  result grants one Home-and-acquisition retry per Pick, preserved across Pause
  and later physical misses. Another empty result ends READY/NO_PICK at Home.
  Empty results do not consume physical-pick attempts. After exhausting a
  nonempty batch, confirm Home and request a fresh batch.
  Success ends immediately in HOLDING; three exhausted nonempty batches end
  READY/NO_PICK at Home. `attempted_candidates` totals actual candidate attempts
  in that Pick action across all batches, including canceled/faulted results.
  Successful placement cancels unattempted poses; the next manual/Auto Run Pick
  acquires a fresh batch.
  Pause/Continue retains
  the current batch and attempt count; direct Stop/Recover ends the action.
  Item detector errors/timeouts, source/pose validation failures and robot faults
  remain terminal. Reject reused IDs in new detector replies; intentionally
  continuing the retained batch is permitted. No teach schema, retry
  setting or action-interface field is added; progress reports attempt N/3.

- `/robot_controller/place_item` — goal contains `configuration_id`, placement
  `x_mm`, `y_mm`, `rotation_deg` and `save_debug_images`. The debug flag is passed
  to every `/tray_detect/get_tray_pose_v3` observation for the action, including
  explicit acquisition retries. It does not change motion or release timing.

Services:

- `/robot_controller/configure` loads or reloads explicit Item/Bin Teach in GUI
  mode, then runs guarded robot preparation under the same operation ownership.
  Accepted from unheld UNCONFIGURED/INACTIVE/READY; success means confirmed READY.
  Invalid files preserve the old configuration. Preparation failure retains the
  newly loaded files but reports failure, requiring explicit Recover.
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
- `/robot_controller/set_global_cp` (`SetGlobalCP`) accepts integer 0–100 with
  the same idle READY/HOLDING and single-operation ownership gates.
- `/robot_controller/preview_v2` belongs to the TF-only preview node. It accepts
  HOME, PICK, PLACE and CLEAR, selected Item/Bin/Tray files and placement X/Y/rotation.
  The former preview endpoint is not used as a fallback.

## Global CP control

The **Global CP** slider below Global SpeedFactor adjusts continuous-path blending
from 0 through 100%. It sends the live position on release or after 350 ms of
keyboard/groove inactivity, displays the last accepted setting, and preserves
an in-progress edit against status refresh. READY and HOLDING permit changes with
fresh enabled idle feedback and no active operation. Preview, Auto Run, motion,
Pause and faults disable it; STOP remains independent.

The controller sends one `CP(r)` through its existing serialized transport, with
strict ownership, response deadline, cancellation and held-output checks. A failed
response never claims the requested setting. Status `global_cp_percent` and the
service response's signed `confirmed_percent` use -1 for unknown and 0 for valid
zero blending. Accepted settings apply to all subsequent queues; motion requests
still omit per-command `cp`/`r`. Poses, rates, I/O timings and arrival checks stay
unchanged. Smaller CP values can reduce blending and change cycle timing.

Load/Startup starts at 100%. Recover reapplies the last accepted CP, including 0,
or 100 if none is known. No teach schema, `.env` key or disk persistence is added.
Restart the rebuilt controller, preview and GUI together for the changed status
interface. Launch alone never sends CP or another robot command.
The [official Dobot V4 API](https://github.com/Dobot-Arm/TCP-IP-Python-V4/blob/main/dobot_api.py#L584-L594)
defines CP as 0–100; the vendored ROS service's older 1–100 comment is not a
runtime limit (its integer `r` is sent unchanged).

## Operator status and buttons

Load/Reload Teach Configuration is the explicit preparation command. After the
files validate, the controller runs its existing Startup sequence under the same
operation owner: alarm/ownership/feedback checks, Stop, cold DI1 protection,
disable/clear/enable, global settings, guarded neutral outputs and confirmed READY.
It does not queue Home, Pick or Place. The service reports success only at READY.
Invalid files preserve the old configuration; preparation failure retains the
new configuration and reports FAULT or HELD_UNKNOWN with explicit Recover guidance.
Stop during validation, installation or preparation cancels the sequence; no later
Startup request can re-enable the robot. Launch and filename prefill remain inert.
Headless launch still loads into INACTIVE and needs the existing external Startup
service. The GUI has no Startup client or button.

The lifecycle row has exactly **Recover / Clear Error**, **Pause** and permanent
red **STOP**. In an ordinary empty pause, the middle button becomes **CONTINUE**.
With a trusted held item, its main action is **RETURN ITEM**, with **CONTINUE** in
the split-button menu. If Return is unavailable but Continue is valid, Continue
becomes the main action. Each choice has its own eligibility guard. Acquisition
failure uses that same Continue/Return menu, plus the **Place Item (Retry)**
shortcut, in manual Place and Auto Run. Pending requests disable conflicting
actions; STOP remains direct.

Load/Reload is disabled in Preview because loading now prepares real hardware.
Turning Preview OFF never starts preparation; click Load/Reload explicitly.
Preview motion buttons retain their read-only behavior, including while disabled.

The window title is **Robot Controller**. The main status is NOT READY, READY,
BUSY, HOLDING ITEM, PAUSED, ATTENTION REQUIRED or OFFLINE. Starting, homing,
picking, placing, parking, returning and recovering appear in the activity line;
PAUSED appears only after confirmed parking/Stop. The reason is always visible.
Confirmed emergency-stop evidence overrides the main label in red.

The lifecycle row is Recover / Clear Error, a managed Pause/Continue/Return Item
control, and a **permanent red STOP**. STOP always calls the independent Stop service when
reachable, including while another request is pending, with stale status, and in
Preview. It never changes into Pause or Return. A pending Pause/Return disables
the managed button and shows Pausing…/Returning item…; STOP remains available.

| Operator status | Available controls, subject to the prerequisites below |
| --- | --- |
| NOT READY, no configuration | Select/load teach files; STOP |
| NOT READY, configured | Reload to prepare; Preview; STOP |
| READY | Home; Pick Item; Place Item; Auto Run; Preview; reload; speed; CP; STOP |
| AUTO RUN | STOP; quantity/progress visible; manual controls and edits disabled |
| BUSY, Home/Pick/Place/tray travel | Pause; STOP |
| BUSY, startup/recovery/return/parking/stopping | STOP |
| HOLDING ITEM | Home preserving grip; Place Item; Pause; Preview; speed; CP; STOP |
| PAUSED, ordinary | Continue; Return Item with trusted held source; STOP |
| PAUSED, tray acquisition exhausted (Place / Auto Run) | Continue / Place Item (Retry); Return Item with trusted held source; STOP |
| ATTENTION REQUIRED | Recover when configured with fresh feedback and no active operation; STOP |
| OFFLINE | STOP if reachable; loading/preparation waits for fresh robot feedback |
| EMERGENCY STOP PRESSED | STOP; explicit Recover rechecks alarms after physical release |

Home/Pick/Place and speed/CP require started configuration and fresh enabled,
fault-free, idle robot feedback with an empty queue. Pick also requires an unheld
READY state, configured Item/Bin, saved Tray Detect joints and exactly one armed
item provider. Place requires saved tray joints, positive finite X/Y, legal rotation
and one tray provider. It accepts READY/HOLDING with or without an item, regardless
of current Tray Detect position; the action moves there if needed and confirms
arrival before acquisition. Missing conditions, including Auto Run blockers,
appear beside the controls and in tooltips.

Continue requires retained managed Pause context, unchanged parked pose/outputs,
no pending suction loss, and no uncertain placement release. A paused observation
also needs the tray provider. Retry uses the original target and a new three-request
budget; target fields stay locked. Return remains independent of tray perception.
An old E-stop message never permanently disables Recover: the controller rechecks
current alarms on the explicit request. Unknown suction is still checked by
Recover and never authorizes an inferred item source or output reset.

Preview ON explicitly says **No robot motion** and routes Home/Pick/Place only to
TF planning. Enable it after loading configuration. It requires fresh stationary
feedback but permits a disabled robot and no Startup. Selected files, placement inputs and detector availability gate
preview requests; the preview node validates the selected sources and position.
Preview OFF and STOP remain available; Load/Recover/Pause/Continue/Return/speed/CP stay disabled.
A stopped fault can also be inspected with read-only Preview. All buttons check
service availability and recheck their policy on click. Pending requests disable
conflicting controls immediately. These status hints do not replace execution's
source, ownership, I/O and feedback validation or guarantee a detection result.

## Unified motion preview

The operations grid contains Home (top left), Preview toggle (top right), Pick
Item (bottom left) and Place Item (bottom right). Preview defaults OFF on launch;
it is a GUI routing choice, not a hardware-controller lifecycle or persisted teach
setting. Turn it ON only with no active/pending hardware operation. Home/Pick/Place
then call only the preview service, even before Startup. No fallback to hardware
is allowed on unavailable/rejected preview. Load, Continue, Recover, managed
Pause/Return and speed/CP changes cannot dispatch while ON. The Stop control sends
direct Stop and clears preview; it never parks or puts back an item in this mode.
The separate typed hardware API retains its existing guards and behavior.

The preview process has no Dobot command clients. It reads the same three canonical
robot feedback streams, requires fresh stationary/empty-queue inputs, and obtains
the current Link6 pose from joint FK. It can preview while the robot is disabled;
it never enables it. Shared planners supply the conditional vertical clearance
and exact joint Home for both Home and Pick, every candidate's approach/retract,
successful Tray Detect, missed entry/exit transits and Home/put-back targets, and
Place's three approach/drop/retract targets when already at Tray Detect. Away from
the saved joints, Place preview shows only its observation-travel TF and explains
that placement targets need a fresh observation at that pose. It never moves the
robot, samples placement from the wrong pose or fabricates the unknown drop height.
Targets skipped by Home arrival checks are omitted. Each target is broadcast under
`base_link` as a distinct `robot_controller_preview_*` frame.

Pick preview shows one fresh candidate batch and its possible nominal branches,
not simulated suction outcomes or unknown future retry observations. Actual early
contact/Stop positions depend on hardware feedback and cannot be predicted.
Place uses fresh tray/depth from the camera's **current physical position**, with
the same three-request bound and evidence validation. A hidden tray cannot be
observed by previewing a future camera pose; a missing observation fails visibly
without fabricated targets or movement. Preview does not require a held item.

Turning OFF, editing teach/placement inputs or clearing cancels pending preview
work and stops broadcasting. CLEAR pre-empts blocked perception and late results
cannot reinstall canceled targets. Changed sources, invalid/stale feedback or
robot movement clear installed previews; the existing RViz TF timeout retires
old display frames. No output commands, hardware state changes, new teach schema
or automatic application restart are introduced. Rebuild both controller packages
and restart controller/preview/GUI together for the versioned preview contract.

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
`PICKING`, `TRAY_POSITIONING`, `PLACING`, `HOLDING`, `PAUSING`, `PAUSED`, `RETURNING_ITEM`, `STOPPING`,
`RECOVERY_REQUIRED`, `RECOVERING`,
`HELD_UNKNOWN`, and `FAULT`. One immutable configuration snapshot and one
operation generation exist at a time. Action configuration IDs prevent a stale
GUI or supervisor from executing a replaced profile.

The non-headless GUI labels the configuration control **Reload Teach
Configuration** after a profile is installed. It remains enabled in idle,
unheld `READY`, so the operator can revalidate the selected files or install a
different pair without restarting the controller process. A successful reload
clears TF preview output and repeats robot preparation, restoring SpeedFactor
100% and validated neutral outputs before reporting READY. Reload is blocked during Home/Pick, Pause, holding, Stop,
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
candidates CANCELED. It sends Stop and requires an accepted response, then adopts
validated fresh gripper I/O without waiting for stationary joints or an empty
queue. It does not replay old placement history. Opposing output pairs, unknown
DI1 HIGH, DI1 HIGH after confirmed release, or a prior dropped source that merely
regains suction block motion. Sustained fresh LOW permits empty recovery while
preserving the outputs: LOW is not proof that an object physically left the fingers.
A trusted HELD source with live suction and vacuum stays held throughout recovery travel.

Keep gripper and feedback checks active through conditional ClearError, verified
alarm clearance and Enable. After enabled feedback, confirm two distinct fresh
stationary joint samples, an empty queue and unchanged gripper outputs/raw DI1.
This check has the existing bounded wait and Stop cancellation; rejection,
timeout, stale feedback or I/O changes prevent settings and recovery motion.
Use the original accepted Stop; do not send another Stop just for this check.
Restore settings/readiness, then issue an upward-only RelMovLUser at unchanged
XY/attitude if below Home Z. Direct Stop, managed Pause, drop containment and
Startup retain their existing physical Stop confirmation order.
Confirm that lift before a separate queued joint-target MovJ to taught Home.
Use taught travel speed/acceleration and the last confirmed global speed and CP (each 100%
if unset, preserving CP=0). Already at/above Home height skips the lift; already at taught Home
skips its move. Current outputs and suction policy are monitored throughout travel.
After stationary Home is confirmed, reset DO1, DO2, DO13 and DO14 to OFF, in that
order, confirming each response/output. Both finger outputs OFF means relaxed;
there is no OPEN command or exhaust pulse. The intentional reset permits suction
to decay. Require all four outputs OFF, DI1 LOW and continued Home arrival before
reporting READY. This final check waits up to five seconds for newer joint/status
feedback and the output command queue to finish; it does not fail immediately
on temporary I/O queue activity. A failure reports idle/freshness conditions and
the maximum Home joint error. Cancel the former held ledger entry without claiming PLACED or
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
joint samples showing unchanged joints and a stationary empty queue, and
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
command marks `ACTIVE`; settling followed by the 50% pickup lift without suction marks `FAILED`;
Pause marks the active candidate `INTERRUPTED` and keeps it eligible for retry.
Continue retries that same candidate; its next accepted approach marks it
`ACTIVE` again. Repeated Pause does not consume it or increase the distinct
attempted-candidate count. Confirmed suction marks `HELD`; a paused loss marks
`DROPPED`; a completed intentional held return marks `RETURNED`, and confirmed
placement marks `PLACED`. FAILED, DROPPED, RETURNED, PLACED and CANCELED
candidates remain ineligible. Successful placement retains the other poses for
later manual/Auto Run picks under the exact same configuration object; source
validation still runs before use. Reload/recovery/restart invalidates reuse.
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
DI1 and outputs remain supervised. Drop monitoring activates when first-retract
height is reached, including during this parking rise. DI1 LOW lasting 500 ms
after activation requests Stop; confirmed loss while parked also starts the
same put-back routine described below. A shorter LOW followed by HIGH cancels
the pending loss. Confirmed loss is latched even if DI1 rises again.
Stale feedback or output faults are not
converted into ordinary drops. An eligible acquisition during the initial Stop
can establish trusted holding; late DI1 from a latched miss cannot.

All item returns use paused **Return Item** as their reference (rule 212), with
the exact original saved pre-pick X/Y/Z/attitude. The shared queue contains:

1. Optional vertical rise at current X/Y/attitude when more than 5 mm below Home Z.
2. Approach above the saved source at Home Z.
3. Descend to saved pre-pick; at 80%, DO2 OFF, DO14 ON, DO13 OFF, DO1 ON.
4. Retract vertically to Home Z; at 0%, DO2/DO14/DO1/DO13 OFF.

Every return target uses speed 100%. Rise/approach/Home use taught travel
acceleration, descent uses approach acceleration, and retract uses retract
acceleration. Global SpeedFactor and CP apply. Home Z must exceed saved pre-pick Z;
preview and candidate validation use the same geometry. The fixed +50 mm offset,
separate 50 ms exhaust pulse, explicit release DO calls and intermediate arrival/
release-I/O waits are absent. Exhaust ends with the retract's timed neutralization.

Explicit Return and paused drop append exact joint Home in that same group,
confirming Home execution/idle/neutral outputs/DI1 LOW. Explicit Return marks the
held candidate RETURNED and ends READY; a paused drop stays DROPPED and PAUSED.
An active drop instead appends the next retained entry/clearance/pre-pick/pick in
the same ordered group, without Home or a return-arrival wait. The original
source remains DROPPED and owned until advancing feedback reaches the next
clearance's returned MovL queue ID and neutral outputs with raw DI1 LOW have
been observed after the retract command was issued. Only then can the next
candidate own acquisition. Early command acceptance or old suction cannot do so.
If no candidate remains, confirm the shared retract above the bin and neutral/
DI1 LOW. Physical item placement is not measured.

Stop pre-empts either route and retains issued release/source evidence. Explicit
Recover cancels that operation, preserves grip through its existing lift/Home,
then neutralizes at Home; it never replays the interrupted release. Unexpected
outputs, stale feedback or rejected/unanswered commands remain faults.

Explicit `/return_item` cancels the interrupted action with a CANCELED result and
finishes READY at Home. The GUI uses **RETURN ITEM** while paused with
trusted holding. A paused drop runs the separate pulse-based route described here
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

Rule 211's automatic loss interruption uses rule 223's delayed activation and
500 ms debounce: after first-retract height, monitor clearance/travel, idle
holding, tray detection and placement approach/descent. Confirmed loss latches
DROPPED and sends Stop directly in the FeedInfo callback. Command
admission shares the latch lock; no further interrupted command can be submitted.
Resolve outstanding replies within their original deadlines, acknowledge Stop,
then send a final Stop and confirm stationary joints/empty queue before returning.
Feedback/source/output failures or rejected/unanswered commands block recovery.

Monitoring ends only on observed, issued suction OFF, never on release-command
submission or finger motion. Planned release is exempt. Once loss is latched,
DI1 HIGH or a late planned release cannot erase it or mark the candidate PLACED.
Keep its original source plan. Use the shared paused Return Item approach,
80%-descent release and 0%-neutral retract to Home Z, then append the next eligible
entry/clearance/pre-pick/pick in the same group. Preserve
saved order and exclude failed/dropped/returned poses. No Home, new detection or
operator action is inserted while eligible poses remain. Exhausted automatic
return confirms the exit above the bin; active Pick may then use its existing
bounded new-batch policy and ensure Home before new candidate motion.

Auto Run cancels its speculative detector worker and a separate appended next-session
ledger; it preserves the shared saved ledger when continuing that same batch,
retains the old source/batch and leaves its placement count unchanged. It places
the replacement item after a successful retained Pick. Manual Place cancels its
interrupted placement and leaves the replacement at Tray Detect; an already
accepted Place result remains acceptance-only and status reports the recovery.
Idle HOLDING uses the same recovery under exclusive operation ownership. Direct
Stop/cancel always wins. Paused drop and explicit Return retain their existing
Home/paused endpoints. Explicit Recover cancels the batch.

The dedicated held-suction-loss condition cannot turn output/readiness faults,
invalid feedback, service failures or missing source context into automatic
motion. During a pending motion reply, request Stop immediately but still require
that response to be accepted within its original five-second deadline; keep the
loss latched, send no later old-group command, then confirm Stop before put-back.
A rejected/unanswered response, unconfirmed Stop, failed release or other fault
ends the action through normal containment and explicit Recovery. Direct Stop,
action cancellation and shutdown always pre-empt; pending Pause retains its
put-back-and-remain-paused endpoint. Idle HOLDING uses the automatic shared return;
standalone Home losses continue to require explicit Recovery.

Explicit Recover is a separate cancel-and-Home-then-relax operation under rule 176. It
never invokes the automatic put-back or next-candidate routine. A repeated
Recover after interruption preserves the cancelled state and uses a new Stop,
fresh gripper feedback and a newly measured motion origin. Output changes,
unknown suction, stale feedback or failed commands stop further motion.

The GUI SpeedFactor slider tracks the handle position and sends it once on
release. Keyboard and groove changes are debounced for 350 ms. Controller status
cannot overwrite an active or pending edit, and unchanged selections do not send
another SpeedFactor request.

The GUI has separate managed Pause/Return and permanent STOP controls. See the
operator button policy above. The two gripper LEDs show raw DI1 Suction and DI12
Finger open as Detected / Not detected / Unknown. DO commands and logical holding
do not drive these LEDs. DI12 Not detected does not prove that fingers are closed.
The held-item decision uses its 500 ms loss debounce independently of the raw
DI1 display. Full raw robot flags and DI/DO remain in typed status.

Item/Bin/Tray Teach fields, Browse buttons and Load/Reload occupy the smaller
top-right panel. Full paths remain editable and available in field tooltips;
prefill, explicit load, reload gates and validated persistence are unchanged.
Lifecycle/action controls, global speed/CP and the command-log toggle remain below.
The GUI consumes only controller APIs. It adds no Dobot/camera subscription or
I/O command. Missing/stale canonical feedback turns both LEDs Unknown; a
controller status older than one second by source timestamp or local receipt
also shows robot OFFLINE and disables controls that depend on status. Direct Stop remains
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

The explicit Home action is permitted from `READY` and trusted `HOLDING`.
It now uses the same conditional vertical-clearance and exact joint Home plan
as Pick. Preview shows those same endpoints. Every final Home command uses
`/dobot_bringup_ros2/srv/MovJ`, `mode=true`, with the six unwrapped taught angles
in degrees, taught travel `v`/`a`, and no timed I/O. Return routes that already
use speed 100% retain that rate. All final Home destinations, including recovery,
missed-pick exhaustion, item return and Auto Run, use this joint interpolation.
Tray Detect retains its existing joint-target `MovL` service.

The initial step skips if all six fresh actual joints are within ±1° of the taught
tuple in fresh canonical joint feedback with RobotStatus idle,
`EnableStatus=1`, fault/collision clear, user/tool zero and held-item I/O intact
where applicable. Queue/execution confirmation remains required for sent moves.
Otherwise, more than 5 mm below taught Home Z, `RelMovLUser` first rises at current XY/attitude and confirms
5 mm/1° Cartesian arrival plus stationary/empty-queue feedback. Within 5 mm
below Home Z or anywhere above it, skip that preliminary rise and send exact
taught Home joints directly using joint-mode `MovJ` and ±1° joint confirmation.
The planner and first dispatch share one fresh confirmed joint-derived pose, so a
second origin reading cannot turn a planned upward correction into a rejected
downward move. When a rise is needed, final joint Home acquires its origin after
that rise finishes. The initial Home skip reads one fresh idle RobotStatus and
canonical joint sample immediately; it waits for no extra sample, dwell, FK or
service query. FeedInfo remains the fresh fault/frame/I/O guard. Miss returns
retain an explicit Cartesian exit transit, even at Home Z.
Item returns use their shared Home-Z retract instead.
Exhausted Pick queues pre-pick, clearance, exit transit and joint Home together,
using the confirmed stopped pose as origin and confirming only final Home.
Successful Pick uses the same vertical Safety Z exit after its two lifts, then
finishes at Tray Detect instead of Home.

Pick requires `READY`, DI1 clear and a loaded Tray Teach with recorded detect
joints. Its tray detector need not be armed; Pick only travels to the saved pose:

1. after successful tray placement, acquire fresh poses; interruption/return may
   retain eligible original poses under the same configuration. Request a fresh
   profile/model/camera/platform/bin-hash-matched
   batch from `/item_detect/get_item_poses`, advertised by exactly one root node:
   headless `/item_detect` or explicitly Armed `/item_teach`;
2. run the same Home function once poses are available, skipping motion when already
   matched. If the result is empty, confirm Home and retry acquisition once per
   Pick; another empty result ends NO_PICK at Home;
3. for a new batch, transform platform-relative targets into base coordinates;
4. offset Link6 green/Y by the taught `pick_rotation` from each item's short-axis
   line while preserving taught tool Z;
5. attempt eligible saved candidates in detector rank order, excluding terminal
   states; new batch size still comes from Item Teach `pose_candidates`;
6. after an intermediate miss, retract to that candidate's final clearance and
   proceed through the next candidate's safety-Z transit, clearance, pre-pick
   and final pick without returning Home;
7. after success, lift to pre-pick, lift to clearance, then move directly to saved
   Tray Detect joints and finish HOLDING there with suction on;
8. after full exhaustion, confirm Home and repeat from a fresh batch, up to three
   nonempty batches total; three physically exhausted batches finish READY/NO_PICK.

The empty-result retry is separate from physical-pick attempts and cannot reset
after a miss or Pause. This permits at most four pose requests per Pick when one
empty observation occurs among three nonempty batches. If both initial requests
are empty, stop after those two. Detector errors/timeouts and invalid evidence
remain terminal; no hardware command is retried after uncertain acceptance.

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
preview and hardware independently compose its calibrated camera-link body
at Link6's pick height (`item Z + standoff_height`) into `platform_reference`.
Use the shared Gemini 335 housing: RGB optical XYZ size 90/25/30 mm, center
(+11, 0, −12.79) mm relative to `robot_camera_color_optical_frame`. Compose the
documented nominal RGB-to-camera-link transform with the saved mounting transform;
this yields camera-link center (−10.77, −25, 0) mm and XYZ size 30/90/25 mm.
This fixed mechanical model does not replace the factory optical TF used for
RGB/aligned-depth measurement or change any saved calibration. The geometry
sources and derivation are in the [Item Perception README](../item_perception_yolo/README.md).
Transform all eight corners and project their convex outline onto
platform XY, including mount/platform tilt. The normal Home-relative pick
attitude is used if the entire outline is inside/on the
green Bin Teach ROI; otherwise the exact 180° tool-Z mirror is used if safe.
The mirror preserves the undirected short-axis line and unchanged tool Z but
may exceed the normal 90° Home-relative travel limit. If neither body fits,
planning fails before hardware candidate motion; the detector must
have excluded that pose before ranking, allowing the next safe candidate to
take its place. Robot-camera calibration SHA-256 is part of configuration and
detector evidence. Blue inset checks still
apply solely to the item pick point.

The camera-body visualization belongs to Item Teach. Its launch starts the
read-only `item_perception_yolo/robot_camera_box` node, publishing
`/item_teach/robot_camera_body` from Item Teach's exact validated calibration
selection. Controller GUI/headless launches do not start it. The display follows
live Link6 with the shared RGB-offset housing; invalidated/missing teaching
updates hide it. Controller planning and calibration selection remain independent
of Item Teach's display. Canonical RViz includes the new topic; reload its config
if already open. No camera TF, command client or hardware action is added.
This pick-pose body check does not model mounts/cables or the swept travel path.

Item Teach schema 12 also requires the nearby depth radius/height settings
(defaults 150/60 mm). The detector rejects any candidate with a usable depth
point within/on its base-XY radius and at least the configured base-Z height above
the final pick, including standoff, before ranking. Hardware and Preview receive
only the filtered profile-bound batch; the controller has no depth subscription
or separate scene scan. Open older profiles in Item Teach, review the proposed
new defaults, Save and manually redeploy/reload matching profiles. Motion and
tray placement filtering are unchanged.

The schema-12 geometry uses pick Z equal to item Z plus `standoff_height`,
pre-pick adds `prepick_height`, and clearance adds `retract_height`. Home/travel
uses taught travel rates and final descent uses approach rates. A successful
pick's first rise to pre-pick uses taught retract speed/acceleration. A missed
pick's same rise uses `v=100` and taught travel acceleration. The next clearance
rise uses `v=100` with taught travel acceleration for both outcomes. All rates
remain subject to global SpeedFactor. All item returns use `v=100`, with travel
acceleration for approach/rise/Home,
approach acceleration for release descent and retract acceleration for ascent.

The outputs are explicit mutually exclusive states. Finger OPEN is DO2 OFF then
DO14 ON, CLOSE is DO14 OFF then DO2 ON, and NEUTRAL is both OFF. Vacuum SUCK is
DO1 exhaust OFF then DO13 ON, EXHAUST is DO13 OFF then DO1 ON, and NEUTRAL is
both OFF. Timed state changes preserve that order, and feedback showing either
opposing pair ON together faults the motion. `grip_onpick` controls immediate
pickup closing independently of `use_grip`, which selects finger holding during
transport. Every candidate is presented with OPEN and no DI12 wait.

The first `candidate_1_home_to_pick` group contains three control points: item
X/Y at Home Z with OPEN at 50%, pre-pick with no I/O, then final pick with SUCK
at 20%. It intentionally skips the item-clearance point on this initial descent.
DI1 is eligible only after the candidate's SUCK transition. The first eligible
HIGH during descent or settling immediately sends Stop to discard the old queue.
After its successful command acknowledgement, mark the candidate HELD and start
the upward return from the latest fresh joint-derived pose. Do not wait for
stationary joints, idle status, an empty-queue sample or the remaining settling
time before this return. Command acceptance is not a physical-stop confirmation.
If acquisition occurs while a motion reply is outstanding, resolve that reply
and the initial Stop, then acknowledge one further Stop to discard any late
admission before returning. Unanswered/rejected commands or cancellation prevent
the return; delayed normal motion callbacks cannot Stop its new queue.
Otherwise the terminal pose must remain
within tolerance with advancing queue-idle feedback and the commanded final
outputs for the profile's `pick_settling` interval while DI1 is monitored. This
is the complete final-pick confirmation interval; there is no fixed 300 ms pick
gate before it or separate sensor wait after it. If DI1 is still low when the
interval ends, keep the attempt ACTIVE and suction ON. At taught final-approach
speed and acceleration, command one upward MovL through **50% of the remaining
distance from actual settled Z to saved pre-pick Z**, preserving measured XY and
attitude. A 40 mm remaining retract gives a 20 mm lift. Keep all finger/vacuum
outputs unchanged and eligible DI1 monitored throughout the transition, admission
and lift. DI1 HIGH takes the same acquisition Stop path and starts the normal held
lift/Tray Detect queue from the actual stopped pose; gripper behavior begins only
after acquisition. Stop and Pause still pre-empt, retaining the original source.

Only fresh feedback confirming the probe queue ID, endpoint and idle state without
DI1 latches FAILED; no second settling interval is added. With no upward distance
available, skip the probe instead of inventing a height or descending. Events
record actual lift millimetres and rates, including a zero-distance check. Existing
failed retract/retry/Home runs from the probe endpoint, with its existing release
events and late-DI1 isolation after failure. This applies to first candidates,
retries, resumed unheld picks, return continuations and queued Auto Run picks.
No new setting, schema, detection request or attempt count is added.

The operator log and structured candidate event report `FAILED — no DI1 pickup
detected after 300 ms settling and the 50% upward-lift check.` for a 0.3-second
setting; milliseconds always use the loaded `pick_settling`. The event's
`candidate_state` field and typed status remain `FAILED`.

DI1 HIGH-to-LOW uses one fixed `SUCTION_LOSS_DEBOUNCE_SEC = 0.500` filter owned
by the canonical feedback monitor, not a teach-file setting. On pickup, retain
HELD/source context but defer loss detection until a fresh joint-FK sample reaches
the first retract/pre-pick height, or the higher actual pickup origin. Observe
the upward crossing even during CP blending; do not split the lift/clearance/Tray
Detect queue or wait at a midpoint. LOW time before that crossing does not count.
At activation, start a new timer if DI1 is already LOW; otherwise the next LOW
starts it. Advancing LOW feedback at least 500 ms later
confirms loss; any HIGH resets the pending interval immediately. Re-reading a
snapshot or publishing the same controller timer cannot complete the debounce.
A feedback gap beyond the existing freshness limit cannot count toward it, and
stale/invalid feedback keeps its existing failure handling.

Zero remaining first lift arms on new position feedback at that height. Pause
acquisition uses the same gate, and a parking rise may complete it. Direct Stop
retains the pending height. Suction OFF clears pending deferral so release/reset
cannot carry it into another item. Raw acquisition, release and output guards
remain unchanged. Events `drop_detection_deferred` and `drop_detection_armed`
record the height and 500 ms threshold for run auditing.

The internal snapshot's `suction_present` value is used after acquisition Stop
and throughout held motion, motion-origin/Home checks, Stop/recovery, idle
holding and managed Pause/Continue/put-back. Raw `digital_input_bits` and output
history remain unchanged: first HIGH pickup detection, cold/untrusted DI1,
missed-pick suction reset, release/exhaust confirmation and unheld checks still
use raw DI1. DO13 loss, opposing outputs and other faults receive no new delay.
Direct Stop is sent immediately and its stationary confirmation never waits for
this timer. The debounce is independent of taught `pick_settling` and motion-timed
release; it introduces no teach setting, launch argument or schema change.

On success, `grip_onpick=true` enters CLOSE immediately after pickup Stop
acknowledgement and confirmed suction, regardless of `use_grip`. Both output
calls require acceptance and output feedback before the lift queue.

| `use_grip` | `grip_onpick` | After pickup | At 50% of first lift to pre-pick |
| --- | --- | --- | --- |
| true | true | CLOSE | Keep closed |
| true | false | Keep open | CLOSE |
| false | true | CLOSE | RELAX (DO2 OFF, DO14 OFF) |
| false | false | Keep open | RELAX (DO2 OFF, DO14 OFF) |

The halfway event uses MovLIO: CLOSE is `{0,50,14,0}` then `{0,50,2,1}`;
RELAX is `{0,50,2,0}` then `{0,50,14,0}`. Clearance has no finger event.
Held Pause preserves outputs; Continue restores CLOSE for `use_grip=true` or
RELAX for false before direct Tray Detect travel, including when Pause canceled
the first lift's event. Suction remains on throughout.

After valid tray pose/depth and placement validation, `use_grip=false` sends
DO2 OFF then DO14 ON and confirms both before placement motion. Auto Run starts
fresh next-bin inference before these calls; all placement motions must still
be accepted before appending the next Pick. `use_grip=true` stays closed until
the existing 80% descent release. The reopen never switches vacuum or waits for
DI12; Stop/drop interruption prevents later commands. Release recovery never
reopens or repeats release. Place and Return retain 80% release/0% retract reset.

Successful Pick queues latest measured pose → pre-pick → clearance →
Safety Z exit → saved Tray Detect joints as one `candidate_N_pick_to_tray` group.
The vertical targets preserve measured X/Y and attitude and never descend.
Safety Z is max(taught Home Z, current height); retain the explicit exit even if
coincident with clearance. The first held lift uses taught retract rates;
clearance uses `v=100` with travel acceleration. Exit and Tray Detect use taught
travel speed/acceleration, scaled by global SpeedFactor. There is no final Home
in this success route. The Safety Z exit is a queued control point and may be
rounded by the selected global CP; it has no separate physical arrival gate.
The final command is joint-target MovL, restoring the saved Tray Detect attitude.
Confirm only its saved joints (±1°), fresh idle RobotStatus and executed/empty
queue after admission; no midpoint wait or fixed arrival dwell is added.
SUCK stays ON without reissuing it; no EXHAUST/NEUTRAL release events are sent.
Holding/output checks remain active throughout. DI1-loss detection begins at
measured first-retract height, then uses the shared 500 ms debounce.
Held Continue moves directly from the confirmed safety-height parked pose to
Tray Detect, without replaying the pick, lifts or a Home detour. Successful Pick
requests no tray observation and does not place; the next Place checks saved
Tray Detect joints/idle without queuing observation travel. Preview requires
the same saved tray destination and includes its TF for every candidate success.

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
already-confirmed entry transit. Missed retries retain both entry and exit
transits queued. Shared item return uses its Home-Z approach and retract. Both
are blended under CP (default 100%) and can be rounded without an
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
as success. Successful Pick instead uses the two lifts, Safety Z exit and Tray
Detect route above; its held-item outputs and monitoring remain active.

All `MovJ`, `MovL`, `MovLIO`, and `RelMovLUser` requests in one named batch are admitted
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
they never carry a per-command `cp` or `r`. The global CP setting (default 100%)
controls all transitions; the idle slider changes it and Recover retains it.
As specified by the Dobot protocol, smoothing can bypass exact intermediate pick coordinates and timed
I/O can occur during a blended transition. Successful Pick physically confirms
only Tray Detect after its lift group; exhausted Pick confirms only exact joint Home.
Serialized service responses may let
a short pick segment decelerate even at global CP 100%; queue order takes
precedence over uninterrupted blending.

Final Home targets use `MovJ(mode=true)`; other no-I/O targets use `MovL`.
`MovLIO` is used only for a real non-empty timed DO tuple. Initial/shared Home's conditional rise uses `RelMovLUser`; item exit
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

Joint Home and Tray Detect completion use ±1° independently on every joint.
Home never wraps angles modulo 360°: a matching tool pose with J6 one turn away
must still execute the exact saved joint target. MovJ uses joint interpolation;
the final Home path is not constrained to a straight Cartesian line.
Cartesian targets use 5 mm Euclidean translation and 1° orientation, calculated
from the same canonical `/joint_states` sample using the existing CR10 model.
RobotStatus `is_enable` is the vendor's mode-5 idle indication. Home, clearance
and every other non-pick endpoint complete on the first qualifying joint/status
update after the complete group's acceptance. Require a newer joint source stamp
and a newer RobotStatus receipt; duplicate/backward joint stamps cannot refresh
position evidence. Both topic callbacks wake the wait immediately, independently
of FeedInfo updates. There is no added stability interval.
`MovJ` and `MovL` expose their queue ID in the existing reply: require that exact
`FeedInfo.currentCommandId` at completion. The fixed vendor `MovLIO` and
`RelMovLUser` response schemas expose only `res`; those endpoints instead require
live execution evidence latched during dispatch/travel (running/queued status,
changed queue ID or joint movement). Queue-empty confirmation remains a separate
execution guard. Short/zero-distance commands can
complete without ever observing a running flag when the stream shows their
execution. No new query service or midpoint wait is added. Optional managed
safety rises within the existing 5 mm tolerance are skipped before dispatch
using the actual pose. Only a final pick
uses a timed settling interval: its taught `pick_settling` duration under the
same feedback gates. Joint-mode Home remains joint-only; no position check
compares against FeedInfo `tool_vector_actual`.

Before every independently acquired motion-batch origin or stopped-pose
measurement, the controller waits up to two seconds for newer joint/status
feedback, idle RobotStatus and an empty command queue. There is no additional dwell
duration. A missed final pick supplies its settled pose directly to the return.
Successful acquisition instead uses fresh joint feedback after the Stop command
acknowledgement, without waiting for idle or stationary feedback, and carries
that pose into the immediate lift/return batch. The controller uses
FK from that validated joint sample; it never sends a separate
`GetPose` request or subscribes to the slower, unstamped `ToolVectorActual`
topic. A timeout names every current
idle, queue and joint/status advancement blockers. Fresh connected RobotStatus, joints,
FeedInfo, user/tool zero, held-item integrity, and the normal final-arrival
checks remain mandatory. Startup/Recover retains its separate 200 ms READY
lifecycle coherence. Outside successful pickup's acknowledgement-only handoff,
Stop confirms two distinct joint source samples unchanged
within 0.05° and the stopped/empty-queue state; fault/disabled stopping remains
confirmable. Paused/placement hold checks also use joint-derived pose. The FK
model is nominal geometry; this change is software-validated, not a new physical
calibration or an automatic runtime service dependency.

Software tests use synthetic services/feedback and must never commission
physical motion. Real commissioning requires separate explicit authorization,
clear workspace, functional physical emergency stop, verified wiring, and an
attentive operator.

`robot_controller` is the sole production/runtime Dobot command issuer.
`motion_debug` may issue direct commands only as a mutually exclusive maintenance
application; production Startup rejects competing maintenance command owners.
