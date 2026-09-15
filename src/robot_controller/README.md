# robot_controller

Service-driven Home, vertical pick and Stop controller. GUI and headless expose
the same action interfaces. The GUI starts with **Live OFF**: TF previews only,
with no robot-command clients or startup initialization. Headless starts
permanently **Live ON** and performs robot initialization before READY. No mode launches bringup,
cameras, detection or RViz, loads model weights, automatically picks, or performs
placement.

## Launch

Build from root and source the canonical workspace loader:

```bash
colcon build --packages-up-to robot_controller
source scripts/source_ros_workspace.bash
ros2 launch robot_controller robot_controller.launch.py
```

The GUI explicitly loads Item Teach and optionally Bin Teach. Home needs only
Item Teach; Pick also validates Bin Teach and the latest bound station calibration.
Files come from `offline_teach/item_teach/` and `offline_teach/bin_teach/`.
Restored filenames in `logs/robot_controller/last_session.json` (strict schema 1)
are unapplied prefill; malformed GUI state fails before real initialization.
Loading never moves the robot. Launch paths can be supplied
explicitly using `item_teach_file:=... bin_teach_file:=...` in GUI mode.

Headless loads one complete deployment set automatically from flat root
`runtime_teach/`: exactly one strict schema-6 item YAML, its same-stem hash-bound
`.pt`, and one strict schema-3 bin YAML. Extra/ambiguous artifacts, symlinks and
partitions are rejected; no file overrides or implicit profile choice. Station
camera/platform calibration comes from the shared latest selector in
`calibration/`, not the portable bin's source station. Models are hashed only.
The GUI's Home, Pick, Stop/Clear, red-on-active Live and Debug Images controls
call the same ROS services listed below; there is no private GUI execution path.
Live ON automatically executes the ordered startup EnableRobot. A persistent
post-startup readiness blocker triggers one guarded Stop, conditional ClearError
and EnableRobot attempt. Stop/Clear confirms the Stop reply, fresh stationary/
empty queue feedback, conditionally clears a remaining alarm, re-enables, then
uses the shared Home function from fresh actual GetPose/FK when DI1 is OFF and
an unchanged Home profile is loaded. With no profile it re-enables without motion
and asks the operator to load a Home; DI1 ON preserves the last-prepick return/
release policy instead. If global SpeedFactor is unknown after a failed/ambiguous
setting response, explicit Stop/Clear re-runs the complete ordered initialization
to restore a confirmed 100% factor before Home; the enable-only service cannot
bypass that unknown state. No discarded waypoint queue is resumed with Continue.
The separate GUI Enable Robot button is removed. The optional headless
`/robot_controller/enable_robot` service remains enable-only after completed
settings; Stop/Clear is the shared recovery route. Blocked Home/Pick GUI buttons
remain clickable only to report the exact safety refusal and prompt; no motion
is sent until READY. A failed recovery also prompts once in the GUI with an
emergency-stop check, without assuming the stop is actually pressed.

```bash
ros2 launch robot_controller robot_controller.launch.py headless:=true
ros2 service call /robot_controller/go_home std_srvs/srv/Trigger '{}'
ros2 service call /robot_controller/pick_item std_srvs/srv/Trigger '{}'
ros2 service call /robot_controller/stop std_srvs/srv/Trigger '{}'
ros2 service call /robot_controller/set_debug_images std_srvs/srv/SetBool '{data: true}'
ros2 service call /robot_controller/set_global_speed dobot_msgs_v4/srv/SpeedFactor '{ratio: 50}'
```

In the GUI, Home/Pick while Live is OFF preview TFs. Setting Live ON performs the one-time
disable/re-enable initialization and sets SpeedFactor 100%; it does not itself
Home or pick. Check the physical safety area and independently start canonical
bringup first. Live OFF is accepted only while idle and not carrying an item;
it removes the controller's command clients but does not send DisableRobot.
Every later GUI Live ON runs initialization again. Headless automatically starts
Live, rejects Live OFF and never automatically homes or picks. Do not run Motion
Debug or Gripper Control while Live is ON.

The **Global speed** slider sets SpeedFactor from 1–100%, independently of the
item profile's per-command `v=` and `a=` values. Mouse changes apply after release;
keyboard changes apply after a 300 ms pause. Each request waits for the actual
robot response before another setting/action can proceed. No motion is issued
by the slider, and taught speeds/acceleration are not rewritten.
It is available only with Live ON, completed startup and an idle READY/HOLDING/
NO_PICK controller. Startup, motion, Stop recovery, pending responses, stale/
faulted/paused feedback or nonzero user/tool reject changes. A held item must
retain DI1 while the setting response is awaited. Normal command serialization
and independent safety Stop remain intact.
The shared headless service above returns `res: 0` only after success; `res: -1`
means rejection/failure, with the precise reason on status/events. A failed or
ambiguous accepted setting blocks further actions without an automatic retry.
Status reports `global_speed_percent` and `global_speed_message`; the factor is
null while unknown or Live OFF, not a guessed readback. It is the acknowledged
command factor, not measured motion speed. The slider is a transient robot
command, not reusable setup: it is not saved to Item Teach, `.env` or UI prefill,
and every Live initialization explicitly resets SpeedFactor to 100%.
Reduced global speed does not extend motion deadlines or candidate freshness.

Schema 6 retains its historical non-executing controller_contract as validation
metadata, not movement permission. Only explicit Live ON plus an action service
call authorize execution; loading a teach file never authorizes it.

An independently armed GUI/headless item detector must use the same item/model,
bin and station hashes. The controller does not auto-start or arm it.
Detect/Teach service availability and provider are checked before Pick's
preliminary Home travel; fresh acquisition happens after Home.
Item Detect supplies the requested annotated image pair when Debug Images is ON;
it remains independently launched and armed.

Debug Images is separate from the motion gate and defaults OFF in both modes.
The setting is sampled once for each candidate request. When ON, the detector
saves the exact rendered RGB and registered-depth result panes for that request
as a same-batch PNG pair under ignored `debug/pick_img/`. It does not subscribe
to another camera stream, continuously archive frames, change filtering/ranking,
or change the motion sequence. Absolute saved paths, or a warning if persistence
fails, appear in `/robot_controller/status`; persistence failure does not invalidate
an otherwise valid candidate batch.

## Startup and motion

Live initialization order: StopMoveJog, DisableRobot, EnableRobot/enabled confirmation,
SpeedFactor 100%, Tool 0, Tool 1 TCP zero, CP 100%. Only StopMoveJog and
DisableRobot are best effort; strict startup failures leave a FAILED controller
without pretending it is READY. Live performs at most one post-settings Stop/
ClearError/EnableRobot recovery for a persistent readiness blocker; explicit
Stop/Clear may re-run incomplete initialization as a new operator action.
The same applies to an unknown global SpeedFactor after a failed setting; an
unanswered earlier normal response still blocks every subsequent command.
Motion Debug's independent 50% startup rule is unchanged.
Wait up to five seconds for all strict startup services before preconditioning.
Every normal call is response-serialized. Missing/rejected best-effort services
or missing Disabled confirmation warn and continue, but an unanswered response
timeout stops startup without dispatching later calls—even for StopMoveJog or
DisableRobot. A late response never automatically advances the sequence. Safety
Stop remains independent so it can interrupt a pending motion acknowledgement.
The status names each active startup call and failed calls are logged with their
service name. After all settings respond, validate enabled/fault/pause/user/tool
feedback before claiming READY. Await final coherent idle/enabled feedback for
at most five seconds without reissuing commands, so a transient asynchronous
RobotStatus update does not immediately fail startup. Persistent failure then
attempts one Stop/conditional ClearError/EnableRobot recovery. If that attempt
fails, status says `Startup calls completed; recovery failed:` followed by the
exact blocker, not a guessed failed startup service. For example,
`isPauseCmdFlag=1` means queue-paused feedback; the controller never silently
ignores it or sends Continue to resume an unknown paused queue.

Canonical `/joint_states`, RobotStatus and FeedInfo must be uniquely provided by
the configured bringup node, fresh within one second. FeedInfo controller_timer
must advance: republishing an old packet does not make the robot live. Real
actions require enabled, fault-free feedback and user/tool 0. FeedInfo must
include EnableStatus=1. Vendor RobotStatus.is_enable means idle mode 5, so its
False value during moving modes 7/8 is not itself disabled feedback; the explicit
EnableStatus remains mandatory. Idle readiness still requires RobotStatus enabled.
Static canonical
CR10 FK derives Home from the recorded joints; GetPose(user=0,tool=0) and IK must
agree with that model within 2 mm/0.5 degrees or movement is blocked. No guessed
Home pose, live RViz dependence, alternate model or IK fallback.
In the vendored bringup bridge, the raw TCP error ID is the ROS service `res`;
GetPose and InverseKin `robot_return` contain only `{six,finite,values}`, without
the TCP prefix or command echo. A nonzero `res` or malformed result blocks motion.

Every Home first determines current Link6 Z. If current Z is below taught Home Z,
RelMovLUser changes only base Z to Home Z while preserving actual XY/attitude;
MovLIO joint mode then reaches the exact six taught Home joints. If current Z is
equal to or above Home Z, the controller skips RelMovLUser and issues only the
direct joint-mode Home target, per the user's confirmed safe-above-Home rule.
The six taught Home joints are stored in radians and sent as degrees through
MovLIO joint mode. Pick/transit/retract targets use mm/degrees in MovLIO
Cartesian pose mode; neither mode guesses a TCP or alternate kinematic target.
The conditional relative-Z segment is the user-approved exception to MovLIO-only
picking. All pick/transit/retract segments use MovLIO. Before queueing, validate
all endpoints with nearest-previous-solution IK and canonical FK while idle.
Wait for each service response before submitting the next waypoint, but do not
wait for arrival between waypoints. Completion requires fresh whole-queue idle,
stationary tail pose and exact Home joints, not service acceptance. Per-command
cp=0 preserves vertical corners/speed boundaries; startup CP remains 100%.
The vendor MovLIO interface returns only res, not queue IDs: completion uses
the sole owned queue's idle feedback plus its terminal pose/joints, never a
guessed queue ID or a motion-acknowledgement shortcut.

Item Teach saves separate `speed` and `acceleration` groups, each with explicit
integer `travel_percent`, `approach_percent`, `retract_percent` in 1–100.
Initial speed values are 100/6/6; initial acceleration is 100/100/100. Travel
includes Home, XY transit, initial positioning and descent to pre-pick. Approach
means only pre-pick to pick; retract applies only to pick-to-prepick upward
movement, including early-contact adjustment and missed-pick recovery. The
remaining clearance and Home movements use travel rates.
Each command carries `v=<speed>` and `a=<acceleration>` in param_value, including
Home-height RelMovLUser; no per-phase global setting changes. These are vendor
percentages, not mm/s or mm/s². Missing values never fall back: production requires
schema 6 and rejects schemas 1–5. Rates do not relax target-age/motion deadlines
or collision checks; slow motion can exhaust a batch's configured freshness.

Pick holds Home orientation and uses robot base Z, never the item's long-axis
orientation as tool attitude. Convert the complete platform XYZ into base before
adding millimetre offsets:

- Link6 pick Z = item Z + standoff_height.
- Pre-pick Z = pick Z + prepick_height.
- Above-item clearance Z = pre-pick Z + retract_height.

zheight_offset is removed. Require Home Z at or above every candidate clearance.
Complete the shared Home function before acquiring one fresh pose batch.
The forward queue transits item XY at Home Z, then descends vertically through
clearance/pre-pick at travel rates and final approach at approach rates. The
return queue rises to pre-pick at retract rates, then clearance at travel rates,
and appends the shared Home function (conditional relative rise plus exact joint
Home). Invalid settings block
execution without editing the teach file. These checks are not collision
planning; a taught Home cannot guarantee safe travel on another station.

## Suction, fingers, retries and Stop recovery

Keep exhaust DO1 off. MovLIO carries a start-distance event `{1,0,13,1}` to turn
suction DO13 on at the start of final approach, monitoring
active-high DI1 while descending (including delayed command acknowledgement).
On DI1, interrupt with Stop and confirm fresh stationary feedback before
retracting from the actual stopped Z. If nominal pick completes without DI1,
wait the saved pick_settling interval (e.g. 0.2 seconds) before declaring a miss.
Unexpected DI1 before suction, stale feedback, failed Stop or lost suction are
faults, not missed picks. Intermediate/final retract never moves downward from
an early contact. Every Pick first completes Home, then requests one fresh ranked
batch. A success completes retract and returns Home with suction on. A missed
candidate completes retract and returns Home before vacuum OFF and the next
candidate. DI1 arriving during an already-classified missed return is a fault:
Stop without releasing or retrying a possibly held item. Exhausting
the batch returns Home and reports failure. No new batch is acquired automatically.
Initial and missed-pick vacuum OFF also require DI1 clear before dispatch,
during the response wait and output confirmation; a late DI1 blocks retries.

use_grip=false leaves DO2/DO14 untouched and grip_onpick has no effect. With
use_grip=true, MovLIO opens DO2 OFF/DO14 ON at 50% of the clearance move.
grip_onpick=true turns DO14 OFF/DO2 ON after confirmed DI1 acquisition/Stop and
before retract. grip_onpick=false uses 100% motion events on retract-to-prepick
to close, ONLY if DI1 confirmed pickup; a missed suction pick never closes.
Motion I/O must be confirmed by fresh output feedback before batch success.
Early suction clears the remaining forward queue; no further descent is submitted.
A motion acknowledgement arriving after acquisition Stop receives a new safety
Stop, whose acknowledgement/stationary feedback is confirmed before retract.
Return targets preserve actual stopped XY/attitude and clamp Z upward; a contact
above nominal pre-pick also updates the remembered Stop-recovery height.
User deferred DI12 full-open checks
and damage diagnosis in this stage; no old Grip/Release or purge pattern.

The controller requests up to retry.pose_candidates distinct ranked poses.
Only missed suction advances after confirmed final retract and Home. Every
candidate must remain within the profile's result age
for both RGB/depth; expired batches require another explicit request, never
cached/reacquired fallback. Hardware/I/O/stop/retract faults cancel later commands
and fail closed. On cancellation, attempt canonical Stop and preserve vacuum,
never disable/release a possibly held item. Late accepted motion gets another
safety Stop, not another movement. Stop request is not a confirmed emergency
stop; physical emergency-stop functions remain independent.
Ctrl-C/SIGTERM notify the controller before ROS context teardown, allowing a
Stop request while DDS is still alive; this still does not guarantee stopping.

The Stop service immediately cancels the active routine, clears previews and,
while Live is ON, sends canonical Stop. Recovery starts only after Stop acceptance,
fresh stationary/empty queue feedback and termination of the interrupted action.
DI1 OFF conditionally clears remaining alarm feedback, re-enables and, when a
validated loaded Home exists, plans the shared conditional-vertical Home from
fresh actual GetPose/FK. Unexpected DI1 blocks that transit. Without a Home
profile, Stop/Clear re-enables only and reports the missing Home. DI1 ON requires
the remembered last pre-pick target: keep suction active, return there, then set DO13 OFF and
DO1 exhaust ON; with use_grip=true also set DO2 OFF and DO14 ON. A missing target,
lost suction, motion/I/O/freshness/Stop failure never releases the item. Stop is
not an emergency stop, and this explicitly requested return path is not collision
planning.
Another Stop during confirmation/return cancels recovery and sends Stop again;
shutdown also forbids resuming recovery or releasing. No second return routine is
started. Debug-image persistence remains subject to the original source-generation,
result-age and request-deadline checks, so saving cannot make expired targets valid.

## Interfaces and verification

- `/robot_controller/go_home`, `/pick_item`, `/stop` (Trigger): shared GUI/headless
  action acceptance or cancellation; follow status for asynchronous completion.
- `/robot_controller/enable_robot` (Trigger): optional headless explicit enable-only
  request after settings; Stop/Clear is the shared recovery path.
- `/robot_controller/set_global_speed` (dobot_msgs_v4/SpeedFactor): Live/idle-only
  integer ratio 1–100; res=0 after successful robot response, otherwise -1.
- `/robot_controller/set_live` (SetBool): shared GUI/headless actuation gate. True
  begins GUI initialization; false returns GUI to TF-only mode only while
  idle/not holding. Headless starts true and rejects false.
- `/robot_controller/set_debug_images` (SetBool): enable/disable one annotated
  RGB/depth pair per requested candidate batch, independently of Live.
- `/robot_controller/status` (String JSON): transient-local state, holding status,
  explicit Live/headless mode, `startup_settings_applied`, `debug_images`,
  `global_speed_percent`, `global_speed_message`,
  `debug_capture_status`, validation
  summary and debug TF frame names.
- `/robot_controller/request_item_poses`: retained explicit read-only batch request.
- `/robot_controller/validate_profile`: retained explicit idle integrity recheck.
- `item_teach_file`: GUI-mode explicit profile parameter; launch modes/runtime
  catalog are immutable. Bin selection belongs to GUI/load-time configuration.

Debug always publishes `base_link -> robot_controller_debug_home`; only while
below Home Z does it also publish `robot_controller_debug_home_height`. Pick
previews publish
`robot_controller_debug_pN_transit/initial/prepick/pick/retract/final` at 10 Hz.
Per-candidate `robot_controller_debug_pN_home_height` (only below Home Z) and
`robot_controller_debug_pN_home` show the shared return tail too.
These are teaching targets, not actual robot/platform TF or collision validation.
Debug Home requires fresh actual joints but issues no GetPose/motion/I/O command.
Cancel, changed artifacts, selection changes and exit stop TF publication.
Full hashes are verified at load/actions; the TF timer checks file signatures
instead of repeatedly hashing large model weights.

Source/model integrity, portable Home provenance, runtime catalogs, initialization,
geometry, early Stop, finger rules, confirmed completion, expiry and fault/retry
behavior are tested with synthetic artifacts, fake services/feedback and offscreen
GUI only. No hardware commissioning is claimed. Sole application command
ownership remains an architecture decision: duplicate providers and known legacy
apps are rejected, but existing direct clients still require migration. Events
are bounded UTC JSONL under ignored `logs/robot_controller/events.jsonl`.
