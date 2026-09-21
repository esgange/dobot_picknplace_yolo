# Robot Controller v2

`robot_controller` is the sole production application-level authority for the
physical CR10. It provides two deterministic operations: Home and Pick Item.
It does not launch Dobot bringup, cameras, Item Detect, or RViz.

Launching the package never enables, recovers, homes, or moves the robot. An
operator or supervisor must load configuration and make an explicit Startup
service call before a hardware action can be accepted.

## Processes

- `robot_controller` is headless hardware authority. It alone creates Dobot
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
- `/robot_controller/recover` restores readiness. After a confirmed held-item
  suction loss with retained source, it also puts back the uncertain item and
  continues eligible saved candidates, or Homes when none remain.
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

Recover repeats Stop/error-clear/enable/settings/readiness, retaining the last
confirmed global factor. Without confirmed suction loss, trusted holding
recovery preserves suction/finger outputs and verifies DI1. With confirmed loss
and a retained source, the explicit click also owns the put-back/continuation
described below. Cold-start DI1 remains `HELD_UNKNOWN`;
after the operator physically resolves it, another explicit Stop observes DI1
clear and changes the state to `RECOVERY_REQUIRED`, where Recover is allowed.

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
This applies to explicit return, paused drop, Recovery and automatic loss return.
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

Direct `/stop`, action cancellation, shutdown, and another Stop during parking
or return cancel all further host-side commands and require recovery. The
already-requested controller pulse can still switch EXHAUST OFF on its timer.
Other faults never automatically invoke put-back or release. Confirmed held DI1
loss during an active Pick uses the automatic routine below. The idle vendor
`isPauseCmdFlag` remains contextual telemetry and does not enable Continue.

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
to active picking resumed through Continue or explicit Recovery.

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

An explicit Recovery click with that retained source confirms Stop, conditional
ClearError, Enable, settings and READY feedback while preserving all outputs.
Only this scoped readiness step permits uncertain DI1; every output check stays
strict and normal suction validation is restored on success or exception. Then
it uses the existing put-back route with outputs preserved until the taught
pre-pick release pose, followed by finger OPEN and the confirmed native 50 ms EXHAUST
pulse. Do not proceed until pulse OFF and raw DI1 LOW are confirmed.

If the same retained batch has an eligible PENDING or INTERRUPTED candidate,
queue neutral retreat through old clearance, the old item's exit transit,
then the next candidate's entry transit, clearance, pre-pick and final pick in
one ordered group. Both transits share safety Z. No intervening
Home or new detector request occurs; only final pick uses taught settling and
normal acquisition monitoring. FAILED, DROPPED and RETURNED stay excluded.
The subsequent successful or exhausted return uses rules 108 and 110. With no eligible
candidate after release, queue the retreat through Home and finish READY.
The recovery service completes when this operation completes; status reports
RECOVERING, RETURNING_ITEM, then PICKING when applicable, and HOLDING/READY.
Pause/Continue remains available during the resumed Pick. Direct Stop pre-empts
recovery, release and subsequent picking. An interrupted pre-release put-back
retains its source for another explicit Recovery, including after DI1 bounces
HIGH. Failed/ambiguous services, changed sources, invalid feedback or changed
outputs prevent later commands and invoke Stop containment. No automatic retry,
source reconstruction or process-restart restoration is permitted.

The GUI SpeedFactor slider tracks the handle position and sends it once on
release. Keyboard and groove changes are debounced for 350 ms. Controller status
cannot overwrite an active or pending edit, and unchanged selections do not send
another SpeedFactor request.

The GUI presents START/CONTINUE and PAUSE/STOP dynamically. Continue is enabled
only once parking completes. During `PAUSING` or `RETURNING_ITEM`, **STOP NOW**
pre-empts without waiting for the managed request's response. While paused and
holding, **RETURN ITEM & STOP** requests put-back without first canceling its
owning Pick action. External clients can always call direct `/stop`.

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
1,000. The GUI lower panel is a read-only, no-wrap 1,000-line view of that topic.
Text is selectable with Ctrl+C, **Copy Log** copies the entire displayed buffer,
and incoming messages auto-scroll only while the operator is already following
the bottom. The topic is observability-only and does not replace typed status,
actions/services, or `events.jsonl`.

## Home and Pick

The explicit Hardware Home action is permitted from `READY` and trusted
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

The detector pose uses local X for the measured long axis and local Y for the
short axis. It is an in-plane heading, not a TCP attitude. The controller
composes that heading through the destination platform transform, projects the
short axis perpendicular to the exact taught Home tool Z, then considers the
taught unsigned 0–90° `pick_rotation` on either side of that line. Since a
rectangle has no directed end, both modulo-180° directions are equivalent.
Every candidate independently chooses the legal attitude with the least CW/CCW
travel from taught Home. All candidate poses are therefore ready before execution
and their rotations never accumulate across retries. Every candidate's
transit/descent/retract targets share its selected attitude; exact
joint Home restores the taught orientation. Platform tilt is not copied into
TCP roll/pitch, and all waypoint heights remain referenced to base Z.

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
queue-idle feedback sample within the applicable tolerance. Only a final pick
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
