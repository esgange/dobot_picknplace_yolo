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
`runtime_teach/` directory. It still remains `INACTIVE` until Startup:

```bash
ros2 launch robot_controller robot_controller.launch.py headless:=true
```

Canonical Dobot bringup and Item Detect are separate processes. Do not run the
maintenance `motion_debug` or `gripper_control` application alongside
production Startup; their presence is rejected as competing command ownership.

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
- `/robot_controller/recover` stops/clears/enables/restores settings without
  moving Home.
- `/robot_controller/pause` pauses and confirms the current Dobot queue without
  canceling the active Home/Pick action.
- `/robot_controller/continue` resumes only a confirmed `PAUSED` queue.
- `/robot_controller/stop` pre-empts and confirms Stop while preserving outputs.
  It is always direct and never requires a preceding Pause.
- `/robot_controller/set_global_speed` accepts an integer 1–100 only while
  stationary in `READY` or `HOLDING`.
- `/robot_controller/preview` belongs to the TF-only preview node.

`/robot_controller/status` uses
`robot_controller_interfaces/msg/ControllerStatus`, reliable/transient-local
QoS. It reports state, phase, waypoint, candidate index, configuration ID,
holding context, Startup completion, global factor, and feedback freshness.
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
`PICKING`, `HOLDING`, `PAUSED`, `STOPPING`, `RECOVERY_REQUIRED`, `RECOVERING`,
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

Recover repeats Stop/error-clear/enable/settings/readiness but never moves Home.
It retains the last confirmed global factor. Trusted holding recovery preserves
suction/finger outputs and verifies DI1. Cold-start DI1 remains `HELD_UNKNOWN`;
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

Pause is different from Stop. From `READY`, `HOLDING`, `HOMING`, or `PICKING`,
it waits for the canonical Pause response plus pause-flagged, stationary
feedback, enters `PAUSED`, and retains the queued path and active action context.
No later host-side waypoint or I/O command is dispatched while paused. Continue
is accepted only from that confirmed state; it waits for the canonical response
and three fresh cleared-pause samples before restoring the suspended state.
Feedback, held suction, and expected outputs remain supervised. An ambiguous
Pause/Continue is contained by direct Stop. Intentional pause duration is not
charged to sensor, no-progress, arrival, or hard-motion deadlines.

`isPauseCmdFlag` is contextual telemetry rather than a global command gate.
Live evidence showed `EnableRobot()` latching it to one with an idle empty queue,
and a raw `Continue()` returned `-1`. Only a controller-issued, acknowledged
Pause with retained operation context enters `PAUSED`; an idle latch never does.

The GUI SpeedFactor slider tracks the handle position and sends it once on
release. Keyboard and groove changes are debounced for 350 ms. Controller status
cannot overwrite an active or pending edit, and unchanged selections do not send
another SpeedFactor request.

The GUI presents these services as two dynamic controls. `START` calls Startup
from `INACTIVE` and becomes `CONTINUE` in `PAUSED`. The amber `PAUSE` control
immediately becomes red `STOP` on its first click. A rapid second click records
a Stop request locally, waits for the Pause response, and then calls direct
Stop—never overlapping the two vendor requests. External nodes do not use this
two-click policy and may call `/robot_controller/stop` immediately in any state.

Feedback is condition-driven from the approximately 100 Hz FeedInfo stream.
Policies are: five seconds for service discovery and each Dobot
service response (including Stop and Pause/Continue), five seconds for output
feedback, one-second feedback age, two-second expected mode changes, three
consistent pause/error samples, three-second no-progress watchdog, and a
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
settings, DO, all three motion services, Pause, Continue, and the
independent Stop channel. A successful response is still only command acceptance;
fresh robot feedback remains required for completion.

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

Pick's initial and return Home paths retain the shared joint-Home rule. The
initial step skips if all six fresh actual joints are within ±1° of the taught
tuple in one feedback sample with idle mode 5, RobotStatus enabled,
`EnableStatus=1`, fault/collision clear, user/tool zero, queue empty/not
running, and held-item I/O intact where applicable. Otherwise, below taught
Home Z, `RelMovLUser` first rises at current XY/attitude and confirms 5 mm/1°
Cartesian arrival plus stationary/empty-queue feedback; at/above it, that rise
is skipped. Exact taught Home joints then use joint-mode `MovL` and ±1° joint
confirmation.
Successful held-item Pick returns first confirm the above-item
approach/clearance before this rule.

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
   proceed directly to the next candidate without returning Home;
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
uses taught travel rates, final descent uses approach rates, and successful
pick-to-prepick uses retract rates; missed-pick rises use `v=100` with taught
travel acceleration.

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

On success, `use_grip=true, grip_onpick=true` enters CLOSE immediately after
confirmed containment. With `grip_onpick=false`, CLOSE instead occurs at 0% of
the later clearance-to-item-X/Y-at-Home-Z move. `use_grip=false` never enters
CLOSE. The held return runs actual stopped pose to pre-pick, clearance, item X/Y
at Home Z, then the shared Home barriers without reissuing SUCK.

A missed non-final candidate starts one
`candidate_N_pick_to_retry_M_pick` CP-blended group: old final to old pre-pick,
old pre-pick to old clearance, lateral transfer to M's clearance, descend to
M's pre-pick, then descend to M's final pick. At 80% of the first rise it enters
EXHAUST. At 0% of the second rise it enters both finger and vacuum NEUTRAL. At
50% of the next-clearance transfer it enters OPEN. At 20% of M's final descent
it enters SUCK. Each service must return `res=0` before the next is sent, while
only M's final target is physically checked. A late DI1 from candidate N is
ignored throughout its latched-miss recovery; candidate M is armed only after
DO13 OFF and a clear DI1 have been observed before its new SUCK. A final
candidate miss queues its old pre-pick EXHAUST rise, old-clearance NEUTRAL rise,
conditional relative Home-Z segment and exact joint Home as one
`candidate_N_pick_to_home` group. The conditional segment is derived from the
planned clearance endpoint. Each request still requires ordered `res=0`
acceptance, but only exact joint Home is physically confirmed; clearance and
Home Z are blended control points. Later DI1 cannot reclassify the latched miss
as success. Successful held-item returns retain their separate clearance and
Home-Z confirmation barriers.

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

Motion requests carry only `user=0`, `tool=0`, and their taught `v`/`a` rates;
they never carry a per-command `cp` or `r`. The global `CP(100)` established by
Startup/Recover therefore controls all transitions. As specified by the Dobot
protocol, smoothing can bypass exact intermediate pick coordinates and timed
I/O can occur during a blended transition. Successful held-item returns keep
their physically verified above-item clearance and Home-Z barriers. An
exhausted final miss uses the documented queued-through-Home exception and
physically verifies only exact joint Home. Serialized service responses may let
a short pick segment decelerate despite global CP 100; queue order takes
precedence over uninterrupted blending.

No-I/O targets use `MovL`. `MovLIO` is used only for a real non-empty timed DO
tuple. Pick's conditional Home rise uses `RelMovLUser`. The controller never calls
`InverseKin`; `Continue` is reserved solely for explicit resume from `PAUSED`.
Service acknowledgement is acceptance only; actual
feedback confirms every result. Only coherent missed suction advances to the
next candidate. All command, feedback, state, cancellation, and result events
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
