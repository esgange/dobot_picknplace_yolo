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
`runtime_teach/`: exactly one strict schema-5 item YAML, its same-stem hash-bound
`.pt`, and one strict schema-3 bin YAML. Extra/ambiguous artifacts, symlinks and
partitions are rejected; no file overrides or implicit profile choice. Station
camera/platform calibration comes from the shared latest selector in
`calibration/`, not the portable bin's source station. Models are hashed only.
The GUI's Home, Pick, Stop, red-on-active Live and Debug Images controls call the
same ROS services listed below; there is no private GUI execution path.

```bash
ros2 launch robot_controller robot_controller.launch.py headless:=true
ros2 service call /robot_controller/go_home std_srvs/srv/Trigger '{}'
ros2 service call /robot_controller/pick_item std_srvs/srv/Trigger '{}'
ros2 service call /robot_controller/stop std_srvs/srv/Trigger '{}'
ros2 service call /robot_controller/set_debug_images std_srvs/srv/SetBool '{data: true}'
```

In the GUI, Home/Pick while Live is OFF preview TFs. Setting Live ON performs the one-time
disable/re-enable initialization and sets SpeedFactor 100%; it does not itself
Home or pick. Check the physical safety area and independently start canonical
bringup first. Live OFF is accepted only while idle and not carrying an item;
it removes the controller's command clients but does not send DisableRobot.
Every later GUI Live ON runs initialization again. Headless automatically starts
Live, rejects Live OFF and never automatically homes or picks. Do not run Motion
Debug or Gripper Control while Live is ON.

Schema 5 retains its historical non-executing controller_contract as validation
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
DisableRobot are best effort; subsequent failures terminate startup, no retries.
Motion Debug's independent 50% startup rule is unchanged.
Wait up to five seconds for all strict startup services before preconditioning.
Every normal call is response-serialized. Missing/rejected best-effort services
or missing Disabled confirmation warn and continue, but an unanswered response
timeout stops startup without dispatching later calls—even for StopMoveJog or
DisableRobot. A late response never automatically advances the sequence. Safety
Stop remains independent so it can interrupt a pending motion acknowledgement.
The status names each active startup call and failed calls are logged with their
service name. After all settings respond, validate enabled/fault/pause/user/tool
feedback before claiming READY. Failure then says `Startup calls completed;`
followed by exact blockers, not a guessed failed service. For example,
`isPauseCmdFlag=1` means queue-paused feedback; the controller never silently
ignores it or sends Continue to resume an unknown paused queue.

Canonical `/joint_states`, RobotStatus and FeedInfo must be uniquely provided by
the configured bringup node, fresh within one second. FeedInfo controller_timer
must advance: republishing an old packet does not make the robot live. Real
actions require enabled, fault-free feedback and user/tool 0. Static canonical
CR10 FK derives Home from the recorded joints; GetPose(user=0,tool=0) and IK must
agree with that model within 2 mm/0.5 degrees or movement is blocked. No guessed
Home pose, live RViz dependence, alternate model or IK fallback.

Every Home first determines current Link6 Z. If current Z is below taught Home Z,
RelMovLUser changes only base Z to Home Z while preserving actual XY/attitude;
MovLIO joint mode then reaches the exact six taught Home joints. If current Z is
equal to or above Home Z, the controller skips RelMovLUser and issues only the
direct joint-mode Home target, per the user's confirmed safe-above-Home rule.
The conditional relative-Z segment is the user-approved exception to MovLIO-only
picking. All pick/transit/retract segments use MovLIO, with
confirmed queue-idle, fresh stationary/target feedback, not service acceptance.

Item Teach saves separate `speed` and `acceleration` groups, each with explicit
integer `travel_percent`, `approach_percent`, `retract_percent` in 1–100.
Initial speed values are 100/6/6; initial acceleration is 100/100/100. Travel
includes Home, XY transit, initial positioning and descent to pre-pick. Approach
means only pre-pick to pick; retract applies to both intermediate and final
retract, including early-contact adjusted targets and missed-pick recovery.
Each command carries `v=<speed>` and `a=<acceleration>` in param_value, including
Home-height RelMovLUser; no per-phase global setting changes. These are vendor
percentages, not mm/s or mm/s². Missing values never fall back: production requires
schema 5 and rejects schemas 1–4. Rates do not relax target-age/motion deadlines
or collision checks; slow motion can exhaust a batch's configured freshness.

Pick holds Home orientation and uses robot base Z, never the item's long-axis
orientation as tool attitude. Convert the complete platform XYZ into base before
adding millimetre offsets:

- Link6 pick Z = item Z + standoff_height.
- Initial/final Z = pick Z + zheight_offset.
- Pre-pick Z = pick Z + prepick_height.
- Intermediate retract Z = pick Z + retract_height.

Require zheight_offset >= prepick_height and retract_height, and Home Z at or
above all candidate clearance heights. Transit XY at Home Z, then descend
vertically through initial/pre-pick/final approach. Invalid settings block
execution without editing the teach file. These checks are not collision
planning; a taught Home cannot guarantee safe travel on another station.

## Suction, fingers, retries and Stop recovery

Keep exhaust DO1 off. Turn suction DO13 on at final approach, monitoring
active-high DI1 while descending (including delayed command acknowledgement).
On DI1, interrupt with Stop and confirm fresh stationary feedback before
retracting from the actual stopped Z. If nominal pick completes without DI1,
wait the saved pick_settling interval (e.g. 0.2 seconds) before declaring a miss.
Unexpected DI1 before suction, stale feedback, failed Stop or lost suction are
faults, not missed picks. Intermediate/final retract never moves downward from
an early contact. Every Pick first completes Home, then requests one fresh ranked
batch. A success completes retract and returns Home with suction on. A missed
candidate completes retract and returns Home before the next candidate. Exhausting
the batch returns Home and reports failure. No new batch is acquired automatically.

use_grip=false leaves DO2/DO14 untouched and grip_onpick has no effect. With
use_grip=true, start with DO2 off/DO14 on (open); grip_onpick=false stays open,
true turns DO14 off/DO2 on only after DI1. User deferred DI12 full-open checks
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
fresh stationary feedback and termination of the interrupted action. DI1 OFF
leaves the robot stopped with no return motion. DI1 ON requires the remembered
last pre-pick target: keep suction active, return there, then set DO13 OFF and
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
- `/robot_controller/set_live` (SetBool): shared GUI/headless actuation gate. True
  begins GUI initialization; false returns GUI to TF-only mode only while
  idle/not holding. Headless starts true and rejects false.
- `/robot_controller/set_debug_images` (SetBool): enable/disable one annotated
  RGB/depth pair per requested candidate batch, independently of Live.
- `/robot_controller/status` (String JSON): transient-local state, holding status,
  explicit Live/headless mode, `debug_images`, `debug_capture_status`, validation
  summary and debug TF frame names.
- `/robot_controller/request_item_poses`: retained explicit read-only batch request.
- `/robot_controller/validate_profile`: retained explicit idle integrity recheck.
- `item_teach_file`: GUI-mode explicit profile parameter; launch modes/runtime
  catalog are immutable. Bin selection belongs to GUI/load-time configuration.

Debug always publishes `base_link -> robot_controller_debug_home`; only while
below Home Z does it also publish `robot_controller_debug_home_height`. Pick
previews publish
`robot_controller_debug_pN_transit/initial/prepick/pick/retract/final` at 10 Hz.
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
