# Dobot Pick-and-Place YOLO

ROS 2 workspace for a physical Dobot CR10 robot and an Orbbec Gemini 335 depth camera. The Dobot source is an intentionally pruned hardware-only CR10 vendor profile; the Orbbec source is vendored in full. A transferred copy therefore contains the project source without requiring network access.

## Workspace contents

```text
src/
├── motion_debug/          # project GUI; does not start Dobot bringup
├── gripper_control/       # gripper/suction IO GUI; requires live bringup
├── orbbec_camera_launcher/ # Gemini 335 configuration and bounded supervisor GUI
├── camera_calibration/    # manual-prefix two-mode ChArUco calibration GUI
├── item_perception_yolo/  # platform teaching and perception integration
├── tray_perception/       # tray teaching and headless, profile-bound tray pose requests
├── robot_controller/      # deterministic Home/Pick hardware authority + GUI/preview clients
├── robot_controller_interfaces/ # typed controller actions, services, and status
├── item_pick/             # imported reference only; excluded by COLCON_IGNORE
├── DOBOT_6Axis_ROS2_V4/  # Dobot official SDK, pruned to CR10
└── OrbbecSDK_ROS2/       # Orbbec official ROS 2 wrapper snapshot
```

| Component | Official repository | Snapshot |
| --- | --- | --- |
| Dobot 6Axis ROS 2 V4 (CR10 profile) | [Dobot-Arm/DOBOT_6Axis_ROS2_V4](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4) | `main` at `def21d05`, locally pruned |
| Orbbec ROS 2 wrapper | [orbbec/OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) | `v2-main` at `8e7cad2b` |

Orbbec's support matrix lists Gemini 335 under the Gemini 330 series. The `v2-main` branch is the recommended branch for new designs and provides the `gemini_330_series.launch.py` launch file.

## Package organization

The root build contains 17 ROS packages below `src`: seven packages grouped under the official vendor snapshots and the project-level `motion_debug`, `gripper_control`, `orbbec_camera_launcher`, `camera_calibration`, `item_perception_yolo`, `tray_perception`, `robot_controller`, `robot_controller_interfaces`, `item_perception_interfaces`, and `tray_perception_interfaces` packages. Imported `item_pick` is reference-only and excluded by `COLCON_IGNORE`. Gazebo/robot simulation, MoveIt, vendor demonstration nodes, `servo_action`, and the Dobot `ServoJ`/`ServoP` streaming interfaces are deliberately excluded. Each retained package has a package-local README describing its role and safe entry points. See [`src/README.md`](src/README.md) for the complete package index. The vendor grouping is intentional and must remain intact for offline provenance and refreshes.

## Robot Controller

The motion controls form one four-button grid:

| Left | Right |
| --- | --- |
| Home | Preview: OFF / ON |
| Pick Item | Place Item |

**Auto Run**, below this grid, takes a quantity of 1–10000 and uses the displayed
placement X/Y/rotation for every cycle. Start from unheld READY with both detectors
available. Each cycle confirms Tray Detect and acquires fresh tray pose/depth.
If another item is needed, immediately start a fresh bin pose request while
planning and sending the placement approach/release/retract queue. Once all
placement commands are accepted and the fresh poses are ready, append the next
entry → pre-pick → pick directly behind placement, without Home or an arrival wait. Successful placement
cancels any unused poses from its old batch. After the last placement, append
Home immediately and finish only at confirmed Home; no extra bin request is made.
The displayed count requires placement execution and neutral/released feedback,
not acceptance alone. Auto Run disables manual controls during motion; permanent
STOP remains available. Three unavailable tray observations pause at Tray Detect
with the item held and the count retained. **Continue** retries tray detection
for that item; **Return Item** puts it back and ends the run at Home/READY with
its partial count. Three exhausted Pick batches, robot faults and STOP end the
run; no automatic restart.

Auto Run displays elapsed seconds beside its completed count, then retains the
total when it ends. Time includes detection, retries, pauses and final Home (or
failure/Stop handling); it resets for the next run. The controller owns this
monotonic timer, so headless clients and a reopened GUI receive the same duration.
The final/partial count and time remain available until the next run or controller
restart. Rebuild `robot_controller_interfaces` and `robot_controller`, then restart
controller, GUI and other status/action clients together for the new timing fields.

Keep the fixed bin camera clear during placement-time detection. Missed picks,
Pause/Continue and automatic drop return keep eligible original poses until
successful placement. Separate manual Pick → Place cycles also discard unused
poses on placement; the next explicit Pick requests a fresh batch.
Reload/recovery or process restart invalidates the retained batch; no poses are
persisted to disk.
Rebuild interfaces/controller and manually restart their clients after upgrading.

**Save item/tray debug RGB/depth** applies to Pick Item, Place Item and Auto Run.
Checked requests save item images under `debug/pick_img/` and tray images under
`debug/tray_img/`, using the detectors' existing per-request capture. The choice
is retained through acquisition retries and Pause/Continue; unchecked requests
and controller Preview do not save images. Rebuild `robot_controller_interfaces`
and `robot_controller`, then restart controller and GUI together after this update
because `PlaceItem` now carries `save_debug_images`.

**Preview ON** makes all three motion buttons publish planned TF targets without
moving the robot or changing gripper outputs. Home shows any required vertical
clearance and its final taught joint target; Pick shows a fresh batch's candidate,
entry/exit and return targets; Place shows its approach, drop and retract targets
from the saved Tray Detect position.
Preview uses fresh canonical joint feedback for the current pose and the same
geometry as hardware.
Pick/Place still need their armed read-only detector and fresh visible targets;
preview does not move the camera to obtain them. Load/Continue, Recover and speed/CP
changes are blocked while previewing; direct Stop remains available. Switch modes
only while no hardware operation is active. Preview starts OFF and turning it OFF
or editing inputs clears its TFs. Hardware Place moves to Tray Detect if needed;
away from that pose, Place preview shows only the observation-travel TF because
fresh placement depth is not yet available. There is no separate Tray Detect Position button. Lifecycle
controls remain separate.
Rebuild `robot_controller_interfaces` and `robot_controller`, then restart the
controller/preview/GUI together for the `/robot_controller/preview_v2` contract.

See the [controller FSM and workflow diagrams](docs/ROBOT_CONTROLLER_FSM.md)
for the current lifecycle, Pick, Pause/Continue, Stop/Recovery and item-return paths.
Open the [visual HTML](docs/ROBOT_CONTROLLER_FSM.html) in a browser or the
[visual PDF](docs/ROBOT_CONTROLLER_FSM.pdf) directly; both work offline.

Every final Home destination uses `MovJ(mode=true)` with the exact six taught
joint angles, including the Home button, recovery, returns and Auto Run. Necessary
vertical clearance remains linear. Home confirmation compares each joint within
±1° without wrapping full turns; matching the tool pose alone is insufficient.
Auto Run skips Home between successful placement and the next pick, even when
perception finishes after retract. Initial Pick, final Auto Run and empty-result
retry Home remain. Next-pick ownership requires the pre-pick command ID and
observed placement release; acceptance alone cannot count a placement.

Motion groups retain CP (default 100%) blending and confirm their final endpoint using
fresh RobotStatus idle and joint feedback after acceptance. Saved joint targets
are checked directly; Cartesian targets and current poses use the existing CR10
forward kinematics from `/joint_states`. FeedInfo still supervises command
execution, queue completion, faults and I/O. Joint/status callbacks wake checks
immediately; no fixed arrival dwell is added outside taught final-pick settling.
Eligible DI1 HIGH during Pick sends Stop immediately and starts the lift/Tray
Detect return after command acknowledgement, without stationary confirmation.
If final-pick settling ends without DI1, keep suction on and try a last-chance
lift through 50% of the remaining distance to pre-pick at final-approach rates.
DI1 interrupts settling or this lift immediately; only completing the lift without
DI1 fails the candidate. Pending motion admission is resolved and discarded before return.
Explicit Recover cancels the interrupted action, preserves current gripper
outputs during a confirmed vertical lift to Home height and return to taught Home.
Recover first waits only for Stop acceptance, validates fresh gripper feedback,
clears alarms if needed and enables the robot. After enabled feedback, it confirms
stationary joints, an empty queue and unchanged gripper I/O before settings or
Home motion. Collision mode cannot block alarm clearing at the old pre-enable
standstill check. Direct Stop and Startup retain their physical Stop checks.
At confirmed Home, Recover / Clear Error turns exhaust OFF and suction ON,
preserving the fingers, and checks DI1 for up to one second. A clear test relaxes
all four outputs and finishes READY after neutral-output/DI1 LOW confirmation.
Detected suction means an item or obstruction, not a definitive clog diagnosis.
The controller stays PAUSED at Home with suction/fingers preserved. **RETURN ITEM**
uses the saved unreleased pickup location when available; otherwise clear the
item/obstruction manually. Once DI1 is LOW, **RETEST SUCTION** repeats the check.
Neither choice resumes the cancelled job. STOP remains available throughout.
The final reset check waits up to five seconds for fresh Home joints, idle status
and the output queue to finish. Temporary I/O activity no longer produces an
immediate “Home position lost” failure; a timeout identifies the failed conditions.
Later Stop clicks send a new Stop with a fresh physical confirmation.

Confirmed emergency stops report **“Emergency stop pressed — cannot start or
recover.”** Release the physical button, then explicitly use Recover / Clear
Error; the controller verifies alarm clearance before enabling. See the
[emergency-stop feedback details](src/robot_controller/README.md#emergency-stop-feedback).

`robot_controller` is the production hardware authority for Home, Pick, Tray Detect
Position and Place Item operations. Placement supports positive tray-local X/Y
and a −180° to +180° tool rotation referenced to the saved Tray Detect Pose, with
live depth and Item Teach `motion.trayplace_height`. The drop target is the tray
surface Z + `trayplace_height` (mm), independent of pick heights. Pre-place/retract
uses the placement X/Y at taught Home Z, matching the first item-pick approach
before pre-pick.
Place checks fresh RobotStatus idle and all six joints within ±1° of saved Tray
Detect. Skip travel when matched; otherwise send absolute-joint
MovJ at 100%, preserving outputs, and confirm execution/idle/joints before detection.
Failed or interrupted arrival prevents tray acquisition and placement. The external
Tray Detect Position action uses this same direct route. Successful Pick also uses
MovJ to the exact saved Tray Detect joints after its linear lifts and Safety Z exit,
retaining taught travel rates. Placement descent and upward retract stay linear.
After fresh detection, placement queues pre-place → release (open fingers,
suction OFF and exhaust ON at 80% of descent) → retract at Home Z
(neutral at its 0% start) in one ordered group, without a drop-arrival wait or settling.
There is no Home move. All placement segments and Tray Detect Position use speed
100%, scaled by the operator's global SpeedFactor. Item Teach acceleration
settings and Item Pick speeds remain unchanged. No separate release-I/O wait or
extra retract-height waypoint is added. **Place returns SUCCESS when the retract
command is accepted.**
Status remains PLACING with exclusive command ownership while a completion worker
verifies the final retract, neutral outputs and DI1 LOW, then enters READY above
the tray. Missing intermediate DI12/DI1 release evidence cannot stop this queue.
Stop, robot faults, command responses and feedback freshness remain supervised;
failures after acceptance appear in status and logs.
Pick Item requires an available armed Item Teach or
headless Item Detect provider. Pick first reuses an eligible saved pose; only an
absent/exhausted batch requests fresh bin poses before Home. Then ensure Home
before approaching the candidate. The first valid empty result grants one
Home-and-acquisition retry per Pick; another empty result ends READY/NO_PICK at
Home. An empty result does not consume a physical-pick batch. Pick uses at most
**three nonempty candidate batches**:
exhausting all poses counts as one attempt, then return Home and request a fresh
batch. Stop on the first held success; three exhausted batches finish READY/NO_PICK.
Place retries a missing tray/depth result or response timeout with at most **three
requests total**, while staying at Tray Detect Pose. Stop/Pause, source validation
and robot feedback checks remain active; malformed pose evidence remains fatal.
After three unavailable observations, confirm Stop and enter **PAUSED at Tray
Detect**, preserving outputs and any trusted held source, in both manual Place
and Auto Run. **Continue**, also available as **Place Item (Retry)**,
starts another three-request acquisition batch for the same target. The paused
control shows **RETURN ITEM** for a known held item, with **Continue** in its menu.
Return uses the existing saved-bin put-back routine, ending Home/READY and
canceling the placement/run without counting it. Pick Item stays disabled. No placement or
release is queued while waiting for the operator; robot/source faults remain terminal.
Place Item requires an available armed Tray Teach
or headless Tray Detect provider. **Explicit Place permits placement with or without an
item** in either launch mode, from idle READY or HOLDING; no successful Pick is required. This is real
hardware debug operation with unchanged tray/depth, release I/O and motion checks.
Auto Run still requires a successful Pick before each placement and both armed
detectors before starting. The GUI displays Auto Run's blocking reason beside
the controls, including a missing/disarmed tray provider.
See the
[placement workflow](src/robot_controller/README.md#tray-placement). Normal launch
separates it into a background controller,
a TF-only preview process with no Dobot clients, and an API-only GUI. Headless
launch starts only the controller and strictly loads the flat `runtime_teach/`
catalog. Crucially, neither launch mode enables or moves the robot: both require
an explicit typed `/robot_controller/startup` call for Home/Pick. Attended
calibration replay has its separate already-enabled admission described below.

Headless `item_detect` uses the same prefix-classified deployment catalog:
exactly one `item_teach_*.yaml` with its same-stem `item_teach_*.pt`, and one
`bin_teach_*.yaml`. Its launch has no file, trust or arming arguments. Starting
that dedicated read-only process loads the deployed model once, validates fresh
camera/TF inputs and advertises the pose service; inference remains request-driven.
A complete optional `tray_teach_` YAML/model pair may coexist for the independent
headless Tray Detect consumer. Robot Controller also binds that optional pair
for Tray Detect Position and Place Item; Item Detect still selects only Item/Bin.

Tray reference planes and Tray Detect joints can also survive camera-only
recalibration. Tray Teach, headless Tray Detect and controller tray configuration
use the active robot-camera selection independently of the teach-time camera.
Saved capture history stays unchanged; see the [tray calibration workflow](src/tray_perception/README.md).

Home and Pick are native ROS actions, and each goal carries the exact active
configuration SHA-256 so stale clients cannot execute replaced teach files.
When no eligible saved pose remains, Pick requests a fresh hash-matched batch from the sole provider:
headless `item_detect`, or explicitly Armed `item_teach` during attended use.
They share `/item_detect/get_item_poses`; running both providers is rejected.
Candidate count always comes from Item Teach `pose_candidates`. Typed Startup,
Recover, Pause, Continue, direct Stop, Configure and global speed/CP services support
those actions, while reliable transient-local typed status reports the state and
operation phase. The old Trigger/JSON/Live/Enable/validation/pose-proxy/debug-image
endpoints are removed.

In the non-headless GUI, **Load Teach Configuration** becomes **Reload Teach
Configuration** after the first successful load. Reload is available only while
the controller is idle and unheld in `READY` (or already `INACTIVE`). Load/Reload
now validates files and runs the existing guarded robot initialization automatically,
reporting READY only after it succeeds. It may enable the robot and reset unheld
gripper outputs, but never queues Home/Pick/Place. There is no Start button.
Invalid files preserve the current configuration; initialization failures show
the cause and require explicit Recover. Stop cancels preparation without a delayed
restart. Load/Reload is disabled while Preview is ON. Headless `runtime_teach/` configuration is
immutable until the process is restarted.

The GUI top row shows **Robot status** and **Gripper status** side by side,
with compact Item/Bin/Tray Teach loading at the right. Robot status uses NOT READY,
READY, BUSY, HOLDING ITEM, PAUSED, ATTENTION REQUIRED and OFFLINE, with activity
and failure reasons always visible. Emergency stop has a prominent red override.
The lifecycle row has Recover, a Pause/Continue/Return Item control and a **permanent
red STOP**. Empty pauses show Continue; held pauses show Return Item with Continue
in its drop-down. Failed tray acquisition uses the same controls, plus Place Item (Retry).
STOP always stops directly, including while Pause/Return is pending.
Motion buttons are enabled only when their current prerequisites are met: Place
requires confirmed Tray Detect position, valid inputs and an armed tray provider;
Pick requires unheld READY, Item/Bin/Tray configuration and its provider. Disabled
actions show reasons. See the [full button policy](src/robot_controller/README.md#operator-status-and-buttons).
Gripper LEDs **DI1 Suction** and **DI12 Finger open** say Detected, Not detected or
Unknown; DI12 Not detected does not establish that the fingers are closed.
Values arrive through `/robot_controller/status` at a periodic 5 Hz, so brief
input pulses may fall between updates. The command log is collapsed by default;
**Show command log** opens the complete retained diagnostic stream and Copy Log.
Controller logging and feedback checks continue while the log is hidden.

```bash
ros2 launch robot_controller robot_controller.launch.py
ros2 launch robot_controller robot_controller.launch.py headless:=true
```

Startup performs strict Stop/queue confirmation, DI1 protection,
disable/conditional-clear/enable, SpeedFactor 100/User 0/Tool 0/Tool-1-zero/CP
100, unheld output reset, and coherent READY confirmation. Recover cancels the
interrupted action and remaining candidates, confirms Stop with fresh gripper
feedback, clears alarms/enables, then lifts vertically to Home height and returns
Home with every gripper output preserved. It never repeats release or resumes
the old batch. Active Pick's automatic loss-return behavior remains unchanged.
Pause discards the current queue and
parks through the operation executor. Unheld Pick parks at `park_transit`, above
the interrupted candidate at safety Z (the higher of stopped Z and Home Z), or
the next unattempted candidate when no interrupted approach remains.
Held Pause rises at current X/Y to Home Z. Continue opens fingers and descends
through the parked candidate's pre-pick to final pick. Candidate states are
visible in the GUI and typed status. An `INTERRUPTED` candidate remains eligible:
Continue restarts its approach using the same saved pose and marks it `ACTIVE`
on acceptance. Confirmed failed candidates stay skipped.
Direct Stop and native cancellation preserve all gripper outputs, discard queued
motion, and finish in `RECOVERY_REQUIRED`; Stop never requires Pause first and
never automatically Homes, releases, or resumes. Trusted held-item DI/output
feedback is checked throughout recovery and Stop confirmation. If idle
supervision sees an unexpected running/nonempty queue, it pre-empts that motion
with the independent Stop path before requiring recovery.

After Stop with a trusted held item and no confirmed suction loss, Recover
preserves the grip during travel, then tests suction at confirmed Home. A positive
test pauses before releasing the gripper and offers **RETURN ITEM** for the saved
source. A clear test finishes the gripper reset and reaches READY. The permanent **STOP**
remains available while Pause/return is pending and always pre-empts without
put-back. Unknown suction instead produces an instruction to keep the robot
stopped, safely secure/clear the item or check the suction sensor for obstruction.
Recover remains available in `HELD_UNKNOWN`; after fresh DI1 LOW, click it again
without an extra Stop click. HIGH or unavailable feedback never permits an
unheld output reset. Close competing maintenance applications such as Gripper
Diagnostics before controller recovery.

Held-item DI1 loss has a fixed 500 ms falling-edge debounce, stored in the
controller rather than a teach file. Advancing FeedInfo
must continue reporting LOW for that interval; HIGH cancels the pending loss
immediately. One shared filter covers held motion, Home preflight, Stop/recovery,
idle holding and Pause. Confirmed held-item loss stays latched even if DI1 rises
again. Pickup HIGH detection, release/reset LOW checks, DO/output integrity and
feedback-freshness checks retain their existing immediate behavior. The debounce
adds no sleep and never delays an explicit Stop command.

The Dobot `isPauseCmdFlag` bit is not a general readiness gate. Hardware evidence
shows that `EnableRobot()` may latch it to one while mode 5 is enabled and the
queue is empty/not running; `Continue()` is rejected in that condition. The
controller enters PAUSED only after its own managed parking completes. Neither
Pause nor Continue calls the vendor queue Pause/Continue services.

A trusted held item's source pose survives completion of its Pick action.
`/robot_controller/return_item` requests a controlled put-back and ends READY at
Home. The GUI exposes it as **RETURN ITEM** while paused with an item.
Explicit Return Item shares tray placement's timed approach/release/retract:
above the original item at Home Z → exact saved pre-pick pose → same X/Y at
Home Z → taught joint Home. At 80% of descent, open fingers, turn suction OFF
and exhaust ON. At the 0% start of ascent, neutralize all four outputs.
Queue optional vertical rise, approach, drop, retract and taught Home together,
with no intermediate arrival wait or settling. All motions use speed
100% and inherit CP (default 100%); final Home requires neutral outputs and DI1 LOW.
Accelerations are travel, approach, retract and travel respectively. There is
no separate 50 ms pulse or release-I/O/clearance wait. Return after failed
tray acquisition uses the same sequence, without another perception request.

Paused **Return Item is the reference for every item return**. Automatic drop
recovery uses the same optional rise, Home-Z approach, exact saved pre-pick
release and Home-Z retract, with identical 80%/0% timed outputs and segment
accelerations. There is no separate exhaust pulse or release-arrival wait.
Home Z must be above saved pre-pick so the retract is upward; no fixed 50 mm
release offset or minimum pre-pick height is introduced. All return speeds are
100%, scaled by global SpeedFactor.

A drop during Pause appends taught Home and remains PAUSED. Active automatic
recovery instead joins the next saved candidate directly after the shared
retract, in the same ordered queue. It retains the dropped source until execution
crosses into the next clearance and neutral outputs/DI1 LOW have been observed;
queue acceptance alone cannot transfer ownership or arm the next pickup.

After pickup, defer drop detection until fresh joint feedback reaches the first
retract/pre-pick height. LOW during that lift does not count toward the 500 ms
debounce. Keep the complete lift/clearance/Tray Detect queue; no midpoint wait is
added. Then monitor continuously through travel, idle holding, tray acquisition
and placement approach/descent. A confirmed 500 ms DI1 loss immediately
sends Stop from the feedback callback and blocks further interrupted commands.
Submission of a timed release does not end monitoring: observed commanded suction
OFF does. Planned release is never classified as a drop.

Resolve all issued command replies, then send a final Stop and confirm stationary
joints and an empty queue before automatic put-back. Preserve the original source
and mark its candidate DROPPED even if DI1 returns HIGH. Execute the shared paused
Return Item approach, 80%-descent release and 0%-neutral retract to Home Z.
**No Home is included in this active automatic return route.** Join the retract
directly to the next eligible saved candidate in original order, without a
detection request or operator action.
A fully exhausted return ends above the bin; active Pick's existing bounded
new-batch policy can then acquire more poses and ensure Home before approaching.

Auto Run discards speculative next-bin results and any separate appended next-pick
session when the old item drops. A shared saved ledger is preserved with its
remaining candidates. It retains the old batch, does not count that placement,
and resumes placement after a successful replacement pick. Manual Place ends its
interrupted placement and leaves a successful replacement pick at Tray Detect.
Direct Stop and cancellation still pre-empt every stage; Pause retains its existing
paused-drop policy. No enable, disable or settings commands are added.

Only confirmed held DI1 loss uses this path. A service rejection/timeout,
unconfirmed Stop, changed source, invalid/stale feedback or output fault still
requires recovery. If loss occurs while a motion reply is pending, Stop is sent
immediately, that reply must still return `res=0` within its existing five-second
deadline, and later commands in that old group are withheld. Put-back begins
only after a final confirmed Stop. DI1 confirms vacuum, so LOW does not prove
that an item has physically left the fingers. Outside active Pick, or after an
interrupted put-back, explicit Recover cancels the previous operation and goes
Home while preserving the current outputs. Fresh unknown DI1 HIGH, opposing
outputs, stale feedback and active alarms block it. A confirmed prior loss is
not erased by later DI1 HIGH; no unknown item gains a fabricated source pose.
During parking/return, **STOP**, direct Stop, cancellation and shutdown
pre-empt immediately. Software verification does not validate physical clearance,
actual pulse width, or successful physical placement.

The vendored `RobotStatus.is_enable` field is also not a separate enable latch:
the bridge calculates it as `robot_mode == 5` on its slower status publisher.
Active-motion supervision therefore uses fresh FeedInfo `EnableStatus`, mode and
fault fields, while idle READY and final arrival still wait for the status field
to converge. Every controller-issued Dobot service request is printed as a
correlated `SEND` plus terminal result in the ROS console and in the bounded
controller event log, including exact request fields, response code/payload and
duration. `motion_debug` remains a direct-command maintenance tool only and must
not run concurrently with the production controller.

The GUI shows those timestamped state, phase and service-audit lines in a
read-only controller command log below status. The view keeps up to 1,000 lines,
supports normal selection and Ctrl+C, and **Copy Log** copies the complete
displayed history. Headless deployments expose the same human-readable stream on
`/robot_controller/operator_log`; the bounded JSONL file remains authoritative.

The global SpeedFactor slider sends its live 1–100 position on mouse release;
keyboard and groove edits use a 350 ms debounce. Status updates do not snap the
control back while an edit or service confirmation is in progress.

The **Global CP** slider below SpeedFactor adjusts path blending from **0–100%**
with the same release/debounce behavior. Change it only while idle in READY or
HOLDING; Preview and Auto Run disable it. All subsequent motion queues inherit
the accepted value. Load/Startup sets 100%; Recover restores the last confirmed
value, including 0. The setting is not saved across process restarts or reloads.
Rebuild interfaces/controller and restart the controller, preview and GUI together
for the added CP status field.

The explicit Home action uses the shared Home route. With fresh idle feedback,
skip only when all six actual joints match their taught values within ±1°.
Otherwise obtain the current Link6 pose from canonical joint FK; if more than
5 mm below Home Z, confirm an upward-only linear rise at unchanged X/Y/attitude.
Then send `MovJ(mode=true)` to the six exact saved Home angles, preserving grip
and taught travel rates. Confirm its returned command ID, exact joint arrival
and an empty queue. Final Home uses joint interpolation. A
successful Pick queues actual stopped pose → pre-pick lift → clearance lift →
vertical Safety Z exit → saved Tray Detect joints in one CP (default 100%) group.
Safety Z is taught Home Z or the higher actual height; the exit preserves current
X/Y and attitude. It omits final Home, confirms only Tray Detect and finishes
HOLDING there. The queued exit remains subject to the selected CP blending.
Pick requires a loaded Tray Teach with recorded detect joints, but does not
request tray detection or placement. Its initial Home check uses one fresh idle
RobotStatus and all six `/joint_states` within ±1°: skip the entire Home queue
immediately when matched, without an extra feedback tick, service query or dwell.
Otherwise use the existing conditional rise and joint Home after acquiring poses,
before candidate motion. If no poses are returned, confirm Home first and retry
acquisition once. Missed-candidate routes and the three-nonempty-batch limit remain.
Finger states are OPEN (DO2 OFF then DO14 ON), CLOSE (DO14 OFF then DO2 ON),
or NEUTRAL (both OFF); vacuum states are SUCK (DO1 OFF then DO13 ON), EXHAUST
(DO13 OFF then DO1 ON), or NEUTRAL (both OFF). Opposing outputs are never
intentionally active together and such feedback aborts motion. The first pick
group goes Home-Z item X/Y with OPEN at 50%, directly to pre-pick, then to final
pick with SUCK at 20%. A miss latches only after final-pose `pick_settling`
and the 50% last-chance upward lift both finish without DI1.
Retry rises to the old pre-pick with EXHAUST at 80%, enters both NEUTRAL states
at the start of the old-clearance rise, enters OPEN at 50% of travel to the
next candidate's safety-Z transit, and enters SUCK at 20% of the next final
descent. A late DI1 from an already missed attempt cannot become success or
block that retry.
Each candidate rotates only around the unchanged taught tool Z. Its green/Y axis
uses the detected item short-axis line plus the taught unsigned `pick_rotation`;
the planner evaluates both clockwise and counter-clockwise offsets and both
modulo-180° line directions. Every candidate independently minimizes rotation
from Home, so retry rotations never accumulate. Platform tilt never
becomes TCP tilt and waypoint heights remain in base Z. The latest strict
`Link6 <- robot_camera_link` calibration is transform-only: no robot-camera
stream is subscribed. The Gemini 335 housing uses documented 90 × 25 × 30 mm
RGB optical XYZ dimensions and center offset (+11, 0, −12.79) mm from the RGB
optical origin. The nominal mechanical RGB-to-link transform places its center
at (−10.77, −25, 0) mm in the saved camera-link frame. At the planned pick height, project
all eight rotated corners onto platform XY and require the entire outline
inside/on the green Bin ROI; otherwise test the equivalent 180° tool-Z mirror.
If neither body fits, the detector removes that pose before
ranking so the next safe item is eligible, and the controller independently
rejects a disagreement before motion. A magenta `CAM`/`CAM 180` footprint is
shown on bin-camera RGB/depth; light blue remains pick-point-only. A confirmed
miss advances to another candidate; Pause/Continue retries an interrupted approach.
Item Teach launch starts the separate read-only `robot_camera_box` node in
`item_perception_yolo`. It publishes a magenta CUBE marker on
`/item_teach/robot_camera_body`, attached to live Link6 using Item Teach's selected
calibration, RGB-referenced dimensions and center offset. RGB/aligned-depth
measurements retain their actual factory optical TF; the nominal mechanical
transform is used only for the housing. Item Teach sends its validated mounting
pose at 1 Hz; edits, failed validation and shutdown clear it. The display expires
after 2.5 seconds without fresh mounting evidence and stops with Item Teach's
launch. It never chooses a calibration independently. Controller and headless
launches create no camera-body display; their clearance checks remain independent.
Canonical RViz includes the display. Reload its configuration if already open.
This is a pick-pose footprint check, not a swept-path or full robot collision planner.
Final Home and Tray Detect moves use MovJ in absolute joint mode; other no-I/O
moves use MovL. Timed-output
moves use non-empty MovLIO. The conditional rise for initial/shared Home uses
RelMovLUser; item exit transits use Cartesian MovL.
Continue replans the remaining operation from its confirmed parked pose;
the controller never uses vendor Continue or InverseKin. See the
[controller README](src/robot_controller/README.md) for its typed APIs, state
machine, raw CLI examples, timing policy and commissioning requirements.

The first Home-to-pick group contains item-X/Y transit at Home Z, pre-pick and
final pick; only the terminal pick/stopped pose is checked. OPEN occurs at 50%
of transit and SUCK at 20% of final descent. DI1 is eligible only after SUCK and
is evaluated while the final pose, queue-idle state and commanded outputs remain
coherent for the taught `pick_settling` time. This one profile-driven interval
replaces the fixed 300 ms final-pick gate and has no later suction wait.
Missed picks queue entry and exit `park_transit` targets. Item returns use the
shared Home-Z approach/retract described above. On an
intermediate miss, one group rises through the old item's pre-pick and clearance
at `v=100`, then queues the old item's exit transit followed by the next item's
entry transit before descending through its clearance and pre-pick to final pick.
Both transits use the higher of taught Home Z and stopped Z. The exit preserves
actual stopped X/Y/attitude; the entry uses the next candidate's X/Y/attitude.
Both use taught travel rates and CP (default 100%) blending, with no intermediate arrival
wait or dwell. Even coincident transit coordinates are sent as separate commands.
The group enters EXHAUST at 80% of the first rise,
finger/vacuum NEUTRAL at the start of the second rise, OPEN at 50% of transfer,
and SUCK at 20% of the new final descent. A final miss queues the same
EXHAUST/NEUTRAL rise, explicit exit transit and exact joint Home in one
ordered group. Clearance and transits remain blended control points; only exact
joint Home is physically confirmed. A confirmed pickup lifts through
pre-pick, clearance and the Safety Z exit, then moves to saved Tray Detect in one group
while holding SUCK. Held Continue moves directly from its parked pose to Tray
Detect. Unheld Continue reuses the entry transit already confirmed by Pause.
Motion services are admitted in order: each response must be `res=0`
before the next request is sent, with no extra inter-command delay. This
prevents independent ROS services
from reversing the dashboard queue, as observed in a failed Home return.
Responses confirm queue acceptance, not physical arrival. A rejection,
response error, or five-second group timeout invokes independent Stop containment.
The independent Stop path is never delayed by motion admission. Planned timed DO
changes are tracked as commanded transitions, so finger movement requested by
MovLIO is not mistaken for an external output change while held-item integrity
monitoring remains active. A missed attempt is latched after settling: its later
DI1 is ignored, and the next item is armed only after DO13 OFF and DI1 clear are
observed before the new SUCK. A Stop or cancellation during group admission
prevents any later group command from being sent.

Pick's initial Home skip uses fresh idle RobotStatus and every actual joint within
±1° of its taught value, with no added wait. Queued joint Home and Tray Detect
arrivals additionally require post-acceptance advancing position feedback and
empty-queue/execution evidence. Explicit Home uses the same exact joint gate;
Cartesian tool-pose agreement cannot hide a full wrist-turn difference.
Queued return-to-Home paths after a pick attempt
are not skipped. Each independently acquired motion-origin pose waits up to two
seconds for stationary idle feedback with advancing joint/status streams, with
no added dwell; stale or frozen feedback cannot supply a motion origin. The
final-pick confirmation sample is reused for its immediate retract/return. The
controller no longer calls the Dobot `GetPose` service or subscribes to the
slower, unstamped `ToolVectorActual` topic.

## Item Teach and controller

In separate terminals after building and sourcing the workspace:

```bash
ros2 launch item_perception_yolo item_teach.launch.py
ros2 launch robot_controller robot_controller.launch.py
```

Item Teach selects `.pt` from any directory, edits grouped item/YOLO settings,
and records all six actual home joints from fresh canonical bringup feedback.
Save creates a strict schema-13 YAML and SHA-256-bound `.pt` copy under
`offline_teach/item_teach/`, with matching timestamped names and a confirmation
dialog. Transfer both files together; the original model path is not needed.
Home joints are portable between the user's identical robots: source IP/node
are provenance, not a station restriction. Loading never replays joint positions.

**Load Item Teach** also loads its paired `.pt` and reads the model classes after
one combined replacement/trust confirmation. Saved class selection, geometry and
settings are preserved; no separate Load Model click is needed. Missing, changed
or incompatible pairs are rejected. The 1 Hz teaching preview starts when ready;
Armed remains OFF, and startup prefill still does not execute model weights.
Armed ON is highlighted red so
the advertised production pose-service state is conspicuous; validation and
fresh-input rules remain unchanged.

A complete validated loaded item teach, including startup restoration of the
named profile, already counts as saved: no redundant Save is needed before
Simulate Trigger or manual Armed. Model verification, YOLO ON and valid fresh
station inputs are still required. Edits disarm and require Save again.
**Save Item Teach** updates the loaded YAML/.pt pair when its item name is
unchanged; renaming the item (or starting a new document) creates a new pair.
Updates keep one hidden previous-version ZIP beside the pair and reject files
changed externally since loading. Unchanged paired weights are not rewritten.

Old or partially invalid item files can open in the GUI as **recovery drafts**.
Independently valid fields are kept; missing/ambiguous fields are blank (unknown
checkboxes show a partial state). The old `retry_limit` count is recovered as
`pose_candidates` only when unambiguous. Missing/bad model pairing clears the
model field; it is never silently trusted. Review the recovery warning/log,
complete the form, and Save a valid schema-13 YAML/.pt pair before simulating,
arming or sending it to the controller. The same known item name updates the
loaded file with a previous-version backup; an unknown original name creates a
new pair. Loading alone never rewrites files. Detector/controller loaders
accept only complete schema-13 profiles; they never recover old files.
The removed zheight_offset is not recovered. Old retract_height is blank in GUI
drafts because it now means extra clearance above pre-pick, not above pick.
Schema-7 and older drafts also leave `pick_rotation` blank; explicitly enter
0–90° before saving schema 13. Schema-8 and older drafts leave all four optional
bin-wall clearance fields blank for explicit review. Schema-9 and older drafts
leave `trayplace_height` blank; enter the intended tray clearance explicitly and
Save before loading the new profile into the controller or deploying it for
headless use. No automatic sum of pick heights or default is substituted.

Item Teach section **4  Item size / pick depth** adds **Nearby depth radius filter
(mm)**, default **150**, and **Maximum nearby floor-height difference (mm)**, default
**60**. Reject a candidate if any usable original depth pixel lies within/on that
camera-XY radius and at least that floor-relative height difference above the item.
Evaluate the platform Z=0 floor at each point’s physical camera XY and measure
height parallel to camera Z. Standoff/tool-length compensation is excluded from this check.
Include the outer bin border and inset margin, including outside masks;
exclude depth outside the outer bin. The check is independent of RViz
voxels. First rank candidates that pass the ordinary geometry checks, then measure
nearby height in that order. Skip blocked candidates and stop immediately once
the requested number pass (for example, three), or the ranked list is exhausted.
Later candidates stay unchecked; the controller receives only checked poses.
Teaching preview, Simulate Trigger and headless detection share this sequence.
Schema 13 saves these as
`geometry.nearby_depth_radius_mm` and `geometry.nearby_depth_height_mm`.
Older profiles open as recovery drafts proposing the new defaults for review.
Save and manually redeploy the pair, then reload all consumers; runtime readers
require complete schema 13. Existing operator artifacts are not rewritten.
Schema 13 also requires `geometry.depth_frame_count`: 1, 3 (default) or 5.
Production/simulation use distinct post-request frames and a strict-majority
float32 median for all pose, clearance and image paths. Item-only depth limits
start at max(500 mm, configured minimum); tray limits stay unchanged.
Review these changed measurements and frame settings explicitly before Save.
Debug-disabled headless production skips rendering; fresh active controller status suspends
new Item Teach background jobs. Source validation still reads/verifies contents.
See the [capture and timing contract](src/item_perception_yolo/README.md#schema-13-capture-and-request-scheduling)
for deployment/restart and measurement details.

To inspect this check, Item Teach overlays both RGB and registered depth:
solid yellow = radius at the item surface, dashed orange = the height limit
above it, red depth points/X = blocking obstacles. Small labels show only the
detection ID; full measurements and rejection reasons remain in diagnostics.
Capture a request to inspect its checks; click RGB to resume passive video.
Simulate Trigger and saved `debug/pick_img/` pairs include these overlays,
including blocked items when no valid poses remain. Enable
**Save item/tray debug RGB/depth** on the controller to save request images.
The request supplies complete overlays without another YOLO prediction.
Between captures, both teaching windows show clean live RGB/depth
without masks, borders, axes, circles or labels. Simulate Trigger and actual
controller requests served by the Armed teaching window display their exact
completed annotated pair for five seconds, then return to passive video. New
results replace older captures and restart the hold; click RGB to resume sooner.
Both windows share compact source/result/count, age/timing/countdown and rejection
feedback, with full details in tooltips and diagnostics. Debug saving controls
writing files only; the same request image buffers supply the captured view.
Restart both teaching windows and their workers to activate these changes.
Item capture uses `pose_candidates` as its acquisition limit and reports how many
remaining candidates were left unchecked. `valid_count` reports validated returned poses, not an
exhaustive count of every potentially usable item in the image.

Item Teach schema 13 provides optional inward clearances for Bin Teach edges
P1→P2, P2→P3, P3→P4 and P4→P1. Blank means no inset on that edge. A configured
valid inset is projected in light blue on both RGB and registered depth. The
green ROI ignores a detection only when its platform-plane footprint is fully
disjoint; overlap or edge contact is accepted. The final depth-derived pick
point must remain inside/on green, and the light-blue region additionally rejects
that exact point near a wall. To keep frozen feedback conservative under
parallax, the unchanged RGB pick pixel must also appear inside/on the projected
green/light-blue allowed-pick polygon. Simulate Trigger, Armed Item Teach and
headless Item Detect share this filter before ranking poses, so Robot Controller
receives only accepted candidates and does not reinterpret the border.

Item Teach also edits per-motion speed and acceleration percentages (integers
1–100). New profiles explicitly start with travel/Home speed 100%, final-approach
speed 6% and retract speed 6%. Final pick uses taught approach rates. After a
successful pickup, the first lift from final pick to pre-pick uses taught retract
speed and acceleration. Without a picked item, that retract uses speed 100% and
taught travel acceleration. The following clearance rise also uses speed 100%
and travel acceleration in both cases. Successful Pick travel to Tray Detect and
missed-pick exit-transit/Home moves use taught travel rates. Every item return uses
speed 100%: approach/rise/Home use travel acceleration, release descent uses
approach acceleration, and upward retract uses retract acceleration.
Acceleration starts at 100%
for all three phases. Save records separate `speed` and `acceleration` groups.
The controller passes each target's `v=`/`a=` to MovJ, MovL, MovLIO or the Home-height
RelMovLUser exception, independently of the controller's global SpeedFactor
(100% at initialization, adjustable explicitly while idle). Loaded rates are
preserved; missing/invalid rates in old GUI recovery drafts remain blank,
never silently defaulted. Production rejects schemas 1–10.
Motion saves `standoff_height`, `prepick_height`, `retract_height` and
`trayplace_height`. The new required, finite, nonnegative millimetre field is
below `pick_rotation` at the bottom of **Vertical motion — mm**.
Pick Z = item Z + standoff; pre-pick Z = pick Z + prepick;
clearance Z = pre-pick Z + retract. Placement drop Z = tray surface Z +
trayplace_height, shared by hardware and Preview. Offsets are in robot base Z.
Queued motion commands omit per-command `cp`/`r`, so the selected global CP
(default 100%) governs every transition. Intermediate waypoints are
therefore blended planning control points rather than guaranteed exact stops;
the terminal pick/stopped pose and final saved-joint destination are physically
confirmed. Successful Pick ends at Tray Detect after its two lifts and Safety Z exit;
exhausted Pick uses the same exit transit and exact Home. Successful travel preserves
SUCK and grip behavior; exhausted returns use EXHAUST then
NEUTRAL. `grip_onpick=true` closes immediately after confirmed pickup, independently
of `use_grip`. At 50% of the first successful lift to pre-pick, `use_grip=false`
relaxes both finger outputs; `use_grip=true, grip_onpick=false` closes them.
With both flags true they stay closed. Clearance has no timed finger event.
After valid tray pose/depth, `use_grip=false` reopens before placement motion,
while next-bin inference can run. Suction stays on until the normal 80% release.
Motion requests wait for
queue-admission responses in order but not intermediate physical arrival;
short segments may still decelerate despite CP 100. See the
controller README for feedback/Stop confirmation and
deployment safety requirements.

Browse a standalone `.pt` to load it and read its classes automatically. Available
model paths restored from the saved Item Teach also load automatically, with the
paired YAML/model hashes checked. The load takes the next worker slot while the
automatic bin border is updating, with queued/loading progress and no repeated retry.
Connect RGB before or after loading; the 1 Hz YOLO preview starts automatically
once its inputs/settings are ready. **YOLO Detect** can stop or resume it.
Armed remains OFF until explicitly enabled.
There is one detection view: no Detect All/Filtered controls or Resume Live button.
The visual-first window keeps setup in one scrollable left column and gives
most space to resizable, side-by-side RGB/depth panes. Result/frozen status,
ages, inference settings and selected pose feedback occupy the top black band
inside each pane, not the camera pixels. Text wraps at the pane width; masks,
axes, bin borders and sampling circles stay on the images. Load/Save remain
visible at the top; the bounded activity log can be expanded at the bottom.
Pick fields can remain blank for initial detection. The visible YOLO settings apply;
initial confidence/IoU/cap are 0.25/0.70/100 and new-profile inference size is 640.
Edits to confidence, IoU, cap, class selection, dimensions, quality or geometry refresh an
enabled preview after a 300 ms typing pause, without toggling YOLO. Invalid
values pause inference with a reason; correcting them resumes it. Old-setting
results are discarded, and edits disarm without automatically saving or re-arming.
Background diagnostics retain all model classes and size failures. Item borders are green
within the taught size tolerance, red outside it, and gray if dimensions or
plane measurement are unavailable. Green means size-valid, not a validated pose.
Select the station platform/bin and mask/OBB to measure short-X/width and
long-Y/height on a plane parallel to the floor at the measured item-center height.
Items are assumed flat and parallel to the taught floor. The same filtered depth
determines this plane and the pick point; size tolerance is checked after depth.
Registered depth is displayed alongside RGB; missing/mismatched or insufficient
depth leaves RGB detections visible with size unavailable and blocks the pose.
There is no floor-size fallback. Taught dimensions are physical millimetres;
review existing values/tolerances after changing from floor-projected sizing.
Registered depth and RGB must share their optical frame, dimensions and K.
Their lens-distortion coefficients may differ: the worker uses both CameraInfo
models to map the physical sampling circle and RGB mask onto native depth pixels.
Depth is not resized or interpolated; the original RGB pick center is unchanged.
Use Simulate Trigger or a controller request served by Armed Item Teach to
capture the exact RGB/depth result. Class, size, ROI, freshness and MAD depth
checks must pass before a pose is accepted; no newer image/depth or TF is
substituted. The returned pair displays for five seconds, then clean passive
video resumes. Click RGB to resume sooner. Captured geometry includes short-X/
long-Y axes, pick dots, bin/inset borders and the metric cyan depth-sampling
circle. Accepted depth points are black and rejected points red. Full pose,
size and rejection evidence remain in tooltips and diagnostics. Simulated batches
publish their teaching-only TFs during the same five-second hold; real request
images add no simulated TFs. No extra platform-reference authority is created.
Platform/Bin Teach use the same compact
setup/large-video presentation, with Save/capture visible and extra calibration
details expandable; their explicit Apply and capture/save workflow is unchanged.

Item Teach also publishes a default **1 Hz RViz preview**: a colored **10 mm voxel
cloud** of the valid depth view, including surroundings outside the bin,
plus TF/axis markers for
**all valid item poses**, up to the YOLO detection cap. The canonical Dobot RViz
configuration includes both displays. Topics are `/item_teach/voxel_cloud`
(`PointCloud2`), `/item_teach/valid_items` (`MarkerArray`) and
`/item_teach/rviz_diagnostics` (JSON in `String`, including all candidates and
rejection reasons). Frames are `base_link -> item_teach_live_candidate_N`.
Cloud XYZ uses the complete calibrated transforms; colors come from the original
RGB through the separate RGB/depth distortion models. Only the display cloud is
voxelized; pose calculation uses the full-resolution registered depth and the
existing class, size, bin, quality and robot-camera-clearance checks. Poses need
selected classes, dimensions, Home and planning settings; the cloud can run with
YOLO OFF. The view explains missing prerequisites. No `pose_candidates` truncation
applies to this teaching preview.

One existing worker job processes the latest pair at most once per second, with
no extra YOLO inference, new executor, backlog or automatic image saving. Slow
processing lowers the update rate. Each published result remains a snapshot;
the teaching view and diagnostics report its age. RViz has no item text or
number overlays; confidence and other candidate details remain in diagnostics.
The latest cloud stays visible indefinitely and turns **grey after 5 seconds**
without new validated voxel data. A fresh snapshot replaces it and restores its
colors; repeated/frozen frames cannot reset that timer. The reliable depth-one
cache also supplies the cloud to RViz viewers opened while Item Teach runs.
Diagnostics retain original source timestamps and report source/refresh ages;
candidate markers and TF pause during gaps or while grey. Source/settings/
CameraInfo changes, terminal failure or orderly exit grey the cloud immediately.
RViz uses zero cloud decay and expires stopped TF frames after 2.5 seconds.
These frame-local IDs are not tracked identities or production service results. Headless Item
Detect stays request-driven, publishes none of these visualization topics and
saves RGB/depth images only when `GetItemPoses.save_debug_images=true`.
RGB keeps mask shading and one mask-derived rectangle (or the native oriented
rectangle for OBB), with centered short-X/long-Y lines and a pick-point dot.
No extra axis-aligned YOLO box is drawn. The green loaded bin ROI appears on the
same image, including with YOLO OFF. Item Teach provides **Browse…** selectors
for platform, bin-camera and robot-camera calibration files inside root
`calibration/`. Platform geometry and the active bin-camera file are independent.
The complete selection automatically validates and remembers its filenames in root
`.env`; descriptive mode-prefixed camera filenames are supported. Selection
changes stop YOLO, disarm and clear old previews. Invalid selections leave the
saved filenames unchanged. After moving/recalibrating only the camera, select
its new calibration while retaining the platform and bin geometry, provided the
robot base, platform and bin stayed fixed. Historical teaching camera filenames
and hashes are provenance, not runtime dependencies. There is no catalog scan
or newer-file substitution in Item Teach.
Select the portable bin file to connect its calibrated camera preview.
The bin file records its teaching platform's filename, SHA-256 and transform.
If the selected platform's SHA-256 differs, Item Teach shows an amber warning
under the file selectors with both filenames (full hashes in its tooltip and
Activity event). Intentional cross-station reuse stays allowed: verify the
same physical origin, X/Y directions, bin size and placement. This checks file
identity, not physical alignment; it never substitutes the original transform.
The saved bin selection reconnects this preview using the `.env` calibration
choices at startup. The border appears when fresh RGB,
CameraInfo and required TF arrive. Model loading runs independently from its selected
path; neither restoration nor browsing launches cameras or arms the pose service. The
bin-border geometry uses the same shared platform-Z=0 construction as Bin Teach:
saved metric XY is placed using only this station's full platform/camera chain,
preserving tilt and height. No source-station transform, marker detection, depth
or resizing is used for the loaded edge. Completed
inference images remain visible as age-labelled result snapshots until replaced;
slow CPU inference does not discard their annotations. Pose-service freshness
checks are unchanged. The editable `image_size` control is removed:
new profiles use 640 internally; loaded profiles preserve their recorded value.
Explicit **Armed ON** always validates the production profile and advertises the item-pose service
only for a saved/loaded matching profile with valid fresh inputs. Length/width
are projected onto platform Z=0; pick XYZ uses the exact rectangle center and
MAD-filtered registered depth. Accepted samples are black, rejected samples red.
Candidates are ranked nearest the bin center first. OFF removes the service.
Teaching previews/clicked poses never become service responses.

The main controls are **YOLO Detect | Simulate Trigger | Armed**. With a complete
saved profile, matching loaded model, YOLO ON and a valid station, **Simulate
Trigger** runs the same fresh RGB/depth/TF candidate pipeline as a real pose request.
It works with Armed OFF and never advertises a service or commands the robot.
Both views freeze for **5 seconds after the result is displayed**, then return
to live automatically. A countdown is shown; click RGB to resume sooner. This
includes empty results and clears the simulated pose guides on expiry.
The frozen views show only the returned ranked P1…Pn overlays, capped by
`pose_candidates`, and the green bin border. Every returned pose also publishes
a teaching-only TF at 10 Hz: `base_link -> item_teach_candidate_1` through
`item_teach_candidate_N`, matching the image priorities. The blue-UP guide display
visualizes these poses; enable **TF → Show Axes** to inspect their actual frames.
Item Teach does not launch RViz. These frozen poses preserve the platform's full
tilt/height, replace any clicked-item/batch preview, and stop publishing on resume,
edits/source changes, arming changes, YOLO OFF or failure/exit. Empty batches
publish no candidate frames. TF/RViz may briefly retain old frames after stopping.
Compact result/count/age/timing feedback is in the top bands; full pose and
rejection details remain in tooltips and Activity. A shortage or zero
valid items is explicit. Click RGB again to resume; edits/source changes or YOLO
OFF cancel old results. The armed service continues acquiring independent fresh
observations—it never returns this frozen teaching batch.
Unsaved text-box edits are not autosaved; restart prefills the selected saved
teach YAML and camera prefix. Available selected calibrations and models load
automatically, with preview starting when ready; arming remains manual.

`item_detect.launch.py` loads the same three calibration filenames saved by Item
Teach in root `.env`. An empty, missing, invalid or mismatched selection fails
startup; no latest-file scan or replacement occurs. Its Item YAML, matching
model and Bin YAML still come from the shared flat `runtime_teach/` catalog.
Launch it with:

```bash
ros2 launch item_perception_yolo item_detect.launch.py
```

It accepts no artifact, trust, arming or platform arguments. Platform/Bin Teach
retain their explicit calibration selection. Headless Item Detect loads the
model and enables YOLO/arming after validating artifacts and fresh camera/TF
inputs; inference runs only on requests. Restart to load changed `.env` choices.
Robot Controller retains its independent latest-calibration selection, and its
hash checks require the same artifacts as the detector. The controller requests
one fresh batch directly
from `/item_detect/get_item_poses`, always using the taught `pose_candidates`
limit and exact profile/station hashes. Detector/teach nodes remain read-only;
explicit controller Load/Reload (or headless Startup) prepares hardware before
a typed motion action. No training is included. Private inference
wheels are pinned/verified/extracted offline; exact Torch/system dependencies
still require separate provisioning. The extracted native runtime is always
materialized as ordinary files in the package install prefix, including when
the surrounding workspace uses `colcon build --symlink-install`; this prevents
OpenCV's loader from recursively importing a symlinked build-tree `cv2` package.
The retired training-oriented teacher
and separate `item_detect_yolo_debug` sources/launchers have been removed.
See [Item Teach](src/item_perception_yolo/README.md) and
[robot_controller](src/robot_controller/README.md) for details.

Populate `runtime_teach/` manually, or with an external deployment program,
before starting headless Item Detect or Robot Controller. A missing Item YAML,
Item model or Bin YAML produces its own fatal startup error; a duplicate error
lists every conflicting filename. There is no watcher or startup retry. Stop the
consumer before replacing the catalog, stage incomplete transfers under hidden
dot-prefixed names, expose exactly one complete set, and then restart it.

Teaching GUIs and headless detectors share their pose-generation code. To keep
their results aligned, deploy the exact saved Item/Bin files and Tray YAML/model
pair, and use matching active calibration selections. Saving teaching changes
does not synchronize `runtime_teach/`; compare file hashes before switching modes.
Headless requires production profiles, whereas GUIs can retain incomplete drafts
and unsaved edits.

Depth coverage uses **Minimum valid depth (%)** in Item Teach (default 50%).
Item picks and tray placement depth apply this percentage after range/outlier
filtering over the full physical sampling circle, with no fixed pixel-count
minimum. Schema 11 removes the count field; open older Item profiles, review and
Save before controller reload or manual headless deployment. Restart Tray
Teach/Detect and Robot Controller together for `/tray_detect/get_tray_pose_v3`.

## Tray Teach

The separate `tray_perception` package currently provides the read-only
`tray_teach` GUI:

```bash
ros2 launch tray_perception tray_teach.launch.py
```

Enter the setup's camera prefix and **Connect RGB**, then browse a YOLO model
to load it automatically. RGB detection works independently of calibration and a
completed profile; live settings update after a 300 ms typing pause. Side-by-side
RGB/depth views support click inspection with manually entered dimensions/tolerance.
Drag the center divider to resize the two camera panes, as in Item Teach.
On relaunch, Tray Teach automatically reopens the last loaded or saved tray file,
including its reference plane, recorded joint pose, settings, calibration and
verified paired model, just like Item Teach. Save continues updating that file.
With no remembered tray file, form entries and file choices restore instead,
including incomplete text; available calibration/model files load automatically.
Session edits are remembered after a 300 ms pause and on orderly close; a remembered
tray file takes precedence over unsaved form edits. YOLO preview starts when ready.
Armed stays OFF. The camera/model selectors use
**Browse…** with no extra Load button or model confirmation.
Browse matching camera calibration. Leave size filters blank to measure first.
The **Tray Detect Pose** panel records all six current robot joint angles, just
like Item Teach Home: displayed in degrees and saved in radians. It reads fresh
canonical `/joint_states` only; recording never moves the robot. Tray Detect Pose
is recorded independently of Item Teach Home. Save it for future controller
travel to the tray observation position. This pose is optional for detection and
arming; controller motion integration remains separate. The sidebar separates
camera/model settings from this pose and reference-plane data.
Use **Capture 4-corner snapshot…** in Reference Plane, select four surface corners
in the existing RGB pane, then Create. The RGB/depth panes hold one captured
observation during selection; Create or Cancel restores live display. Each
corner uses its remaining valid depth samples, even one. The sidebar identifies
whether the plane has been saved. The saved plane's green outline and P1–P4
labels are hidden from RGB/depth previews and request images; corner selection
and depth-sample evidence remain visible during explicit plane capture.
Enter physical width/X and length/Y, save the desired size filters, then
use Simulate Trigger to inspect the returned tray and its rejection diagnostics. Plane capture and
measurement need no recorded robot pose. Streams and background preview keep running.
The saved plane and
corner points are expressed in `base_link`. Subsequent tray measurements use
that plane, without live surface depth, and reject detections outside the taught
length/width tolerance. Select one valid tray nearest the image center.
**Detection Mask Clean** runs before tray measurement in preview, Simulate
Trigger and both pose-service providers. It uses each raw binary segmentation
mask, avoiding polygon conversion that joins disconnected blobs. A 3×3 opening
at native mask resolution breaks thin bridges; only the largest region remains,
and it must retain at least 80% of the original foreground pixels. Empty or
ambiguous masks produce an explicit rejection. The cleaned outline supplies RGB
and depth annotations, dimensions and poses. Original connected-region image-edge
clipping still rejects the tray. Cleanup status is shown in inspection/simulation
and request diagnostics. OBB models and Item Teach are unchanged; existing teach
files need no migration.
Calibrated synchronized RGB/depth also supplies 1 Hz colored 10 mm RViz voxels on
`/tray_teach/voxel_cloud`, including with YOLO off. Retain the latest cloud until
replaced; grey it after five seconds without fresh data or immediately on invalidation.

The detected origin is the rectangle corner nearest the robot base by 3D
distance. Both positive axes run inward along adjacent tray edges, independent
of image left/right: red X follows the short edge and green Y the long edge.
Z follows their right-handed cross product. After plane teaching, all measurable
trays show these axes and mm dimensions, including before expected sizes are
entered; clicks report that tray's measurements without changing your settings.
Before metric geometry is ready, mask/OBB detections show grey rectangles and
red/green axes labelled **2D**, without mm dimensions or a base-frame pose.
Only the eligible center-prioritized tray is returned by the pose service.
RViz's separate **XY and blue UP guides** show the same X/Y and a blue normal
pointing toward positive `base_link` Z. Item Teach uses the same convention for
live, clicked and simulated poses. All guides are 200 mm long with 20 mm shafts,
matching the canonical robot-joint TF axis length and width at Marker Scale 1.
These are display arrows only: actual TF,
controller-facing poses, calibration and pick/place coordinates stay unchanged.
Canonical RViz sets **TF → Show Axes = false** to hide the duplicate downward axes;
this also hides robot-frame axes while keeping the RobotModel and TF data intact.
For an already-open viewer, uncheck **TF → Show Axes** or reload the installed
canonical configuration. Enable Show Axes explicitly to inspect the real frames.
If an older RViz window has no Tray Teach display, reload the installed canonical
`dobot_rviz/rviz/urdf.rviz` configuration to subscribe to `/tray_teach/voxel_cloud`.
**Save Tray Teach** needs only a valid tray name. It writes a named YAML under
`offline_teach/tray_teach/` and copies a loaded model unchanged to a same-stem `.pt`.
Incomplete fields are saved as a draft, including any created plane and recorded
joint pose. Further saves update that loaded/saved file and retain one hidden
`.previous.zip` backup; changing the tray name creates a new file/pair. Once all
required data validates, Save makes the same file a complete detection profile.
Load Tray Teach restores either form; incomplete detection data cannot arm.
Tray Detect Pose is optional. Loading complete detection data can
immediately Arm or Simulate without another Save, including an older GUI draft
whose only missing data was that position. The original YAML/hash stays unchanged;
missing detection settings, model, calibration or plane still block pose requests.
Save produces a production profile for headless deployment even without a position.
Reopening needs no source Item Teach file. Controller Home remains in the
controller's Item Teach file. Teaching has no motion commands or placement
variables. **Simulate Trigger** runs the same fresh observation pipeline as the
controller-facing service and holds its exact returned RGB/depth result for
**5 seconds after display**, including empty results. Then live preview resumes
and the simulated pose clears. Real controller requests served by Armed Tray
Teach use the same five-second captured display and replace older captures.
Click RGB to resume sooner; detection and controller execution never wait for
the display timer. Passive RGB/depth has no overlays. **Armed ON**
advertises `/tray_detect/get_tray_pose_v3`; requests supply the saved YAML SHA-256
and receive one tray or an explicit no-tray result. Settings changes disarm.
The versioned endpoint carries the placement-depth contract; there is no fallback
to the old endpoint. Restart Tray Teach/Detect and Robot Controller together after
updating. Tray executor failures disarm and report a traceback in the package log
instead of silently leaving an open preview without ROS reception.

For request-driven headless use, deploy one tray YAML/model pair to `runtime_teach/`
and run `ros2 launch tray_perception tray_detect.launch.py`. It loads the bound
calibration and arms after fresh-input validation, without an Item Teach dependency.
It also publishes calibrated scene voxels at up to 1 Hz on
`/tray_detect/voxel_cloud`, with status on `/tray_detect/rviz_diagnostics`.
Cloud refresh shares the native worker and yields to pose requests; YOLO still
runs only on requests. The canonical RViz **Tray Detect - 10 mm colored voxels**
display uses the same retention/greying behavior as Tray Teach. Reload the
installed RViz configuration in an existing viewer to add this display.
Run only one armed tray provider. Debug images are saved only when a request asks;
Robot Controller now consumes fresh tray/depth observations for placement. See [Tray Perception](src/tray_perception/README.md)
for the complete
workflow, source checks, plane sampling limits and frame convention.

## Clone this workspace

```bash
git clone https://github.com/esgange/dobot_picknplace_yolo.git
cd dobot_picknplace_yolo
```

No submodule initialization or network access is required after cloning this repository. The vendored sources are ordinary tracked files. Preserve the upstream `LICENSE`, `NOTICE`, and English README files when updating them. Do not reintroduce non-CR10 Dobot model configurations unless the project scope is explicitly changed and recorded in the blueprint diary.

## Development checkpoints

After each completed, tested change, commit and push the scoped source, tests
and documentation to the current branch's configured remote. This is the
user-authorized standing workflow; another confirmation is not required.
Review status and the staged diff, run `git diff --check`, and record workflow
or architecture changes in the blueprint diary. Keep unrelated user edits,
local configuration, generated output, station teaching/calibration artifacts
and operator model weights out of these source commits. Report the commit and
verified push, or explain any validation/push blocker. Never force-push or
rewrite shared history.

## Updating vendored sources (online maintenance only)

Vendor updates must be deliberate and reviewable. Use a separate temporary clone of the official repository, compare it with the current snapshot, then commit the resulting source changes together with an entry in [`docs/WORKFLOW_RULES_BLUEPRINT_DIARY.md`](docs/WORKFLOW_RULES_BLUEPRINT_DIARY.md). Do not add the vendor repositories back as submodules.

```bash
tmp_dir="$(mktemp -d)"
git clone --branch main https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4.git "$tmp_dir/dobot"
git clone --branch v2-main https://github.com/orbbec/OrbbecSDK_ROS2.git "$tmp_dir/orbbec"
# Review differences before copying; keep any project-specific changes out of vendor directories.
git diff --no-index src/DOBOT_6Axis_ROS2_V4 "$tmp_dir/dobot" || true
git diff --no-index src/OrbbecSDK_ROS2 "$tmp_dir/orbbec" || true
```

Record the new upstream commit IDs in the diary whenever a snapshot is intentionally refreshed.

## Offline transfer

For a full Git repository transfer, create a bundle while online and copy that single file to the offline machine:

```bash
git bundle create ../dobot_picknplace_yolo.bundle --all
git clone ../dobot_picknplace_yolo.bundle dobot_picknplace_yolo
```

The bundle includes the vendored source and project history; it does not need GitHub or submodule URLs. A source-only archive can also be made with `git archive --format=tar.gz --output=../dobot_picknplace_yolo.tar.gz HEAD`. The offline PC still needs a compatible Ubuntu/ROS 2 installation and retained system dependencies already available locally; `rosdep` cannot download missing packages without an offline package mirror or cache. Gazebo and MoveIt are not dependencies of this project profile.

## Build (Ubuntu 22.04 / ROS 2 Humble)

The Dobot SDK documents Ubuntu 22.04 with ROS 2 Humble. After sourcing ROS 2, install dependencies and build from the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source scripts/source_ros_workspace.bash
```

This hardware-only profile builds without Gazebo, MoveIt, or servo control. The
removed simulation packages/assets, `servo_action`, `ServoJ`/`ServoP`
interfaces, and MoveIt motion stack must not be restored unless the project
scope and blueprint diary are explicitly changed together.

`scripts/source_ros_workspace.bash` is the canonical post-build environment
loader. It requires the root `.env`, clears inherited ROS workspace paths,
sources exactly ROS 2 Humble and this workspace, and exports the required
`ROS_LOCALHOST_ONLY` value. Source it in every terminal used for ROS commands.

## Initial hardware checks

Create the project-wide runtime configuration once per checkout. The file is ignored by Git, so it can contain the address of the robot connected to that machine:

```bash
cp .env.example .env
# Edit .env and set the explicit robot/network and single-CR10 bringup values.
# LAN1 is tried first; LAN2 is the diagnostic failover.
```

`ROS_LOCALHOST_ONLY=1` is mandatory. It restricts ROS 2 discovery and DDS
communication to the local computer. It does not block the Dobot driver's
explicit TCP connection to the configured robot LAN addresses.

The Dobot bringup launch requires the repository `.env` automatically. It reads all bringup settings from that file and tries `DOBOT_ROBOT_LAN1_IP` first, then `DOBOT_ROBOT_LAN2_IP`. Each TCP channel uses the required `DOBOT_CONNECTION_TIMEOUT_MS`; the project value is `3000`, so an unreachable LAN1 cannot block LAN2 behind the operating system's long default TCP timeout. Every attempt is printed immediately and logged. Each successful connection records a `robot_connection_result` containing the selected interface/IP. If both interfaces fail, one result is recorded for the continuous outage with both attempted addresses and the driver continues its explicit retry loop; missing, malformed, duplicate, unsupported, or invalid configuration hard-fails before the node starts:

```bash
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

Read the bounded package log after bringup has completed an attempt:

```bash
grep '"event":"robot_connection_result"' logs/dobot_bringup_v4/events.jsonl | tail
```

Follow connection attempts and results live with:

```bash
tail -f logs/dobot_bringup_v4/events.jsonl
```

The bringup command also opens the read-only actual-robot and TF viewer on the
graphical desktop. It owns a separate viewer launch session: closing RViz or a
viewer failure stops the viewer and its robot TF publisher while the driver
continues. Ctrl-C in the bringup terminal stops both, and driver exit also stops
the owned viewer. No robot enable or motion command is added, and closing RViz
is not a robot Stop command. Viewer exit is reported without automatic restart.

To reopen a viewer that has exited, use a second sourced terminal while bringup
is still publishing. Run only one viewer at a time to avoid duplicate robot TF
publishers:

```bash
ros2 launch dobot_rviz dobot_rviz.launch.py
```

The fixed CR10 `robot_state_publisher` consumes canonical `/joint_states`
directly and RViz displays the resulting robot model plus every TF available on
the local ROS graph. The viewer has no launch arguments, manual joint sliders,
relay topic, zero-position generator, or non-hardware mode. It requires the
configured Dobot bringup node to be the only `/joint_states` publisher; missing,
malformed, ambiguous, or stale feedback terminates the viewer and is recorded
under `logs/dobot_rviz/events.jsonl`.

`motion_debug` is launched separately and does not start bringup. If bringup is
not running, the GUI prompts the operator to start the Dobot bringup command and
waits for its services. When a safe startup controller mode is available, it performs
one exact initialization cycle before enabling its controls:
`StopMoveJog` (best effort), `DisableRobot` (best effort), `EnableRobot`, mode
confirmation, SpeedFactor `50%`, Tool `0`, Tool `1` TCP zero, and CP `100%`.
Only the first two preconditioning calls may continue after a logged warning;
Enable and all setting steps stop on failure. See
[`src/motion_debug/README.md`](src/motion_debug/README.md) for the full timeout
and logging contract.

The file uses strict `KEY=value` lines and does not require a Python dotenv package. Do not use shell exports, alternate key names, or alternate configuration paths. Every key shown in `.env.example` is required. Active Orbbec serial numbers may remain empty only while opening the camera GUI for first-time configuration; camera startup remains disabled until the configured set is complete.

Item Teach writes `ITEM_TEACH_PLATFORM_CALIBRATION`,
`ITEM_TEACH_BIN_CAMERA_CALIBRATION` and `ITEM_TEACH_ROBOT_CAMERA_CALIBRATION` after
automatic selection validation succeeds. Values are filenames directly
inside `calibration/`, never machine-specific paths. All three empty means no
selection yet; otherwise all three must be populated. Existing workspaces must
add these keys from `.env.example`, initially empty. The shell loader, bringup,
Motion Debug and camera launcher accept and validate them. Item Teach and
headless Item Detect use them to choose calibrations; Item Detect requires a
complete selection and never writes `.env`.

Runtime datalogs are isolated per package under `logs/<package-name>/events.jsonl`; the bringup launch creates the package files before starting the node. Compile the timestamped package records into a separate universal file only when needed:

```bash
python3 scripts/compile_logs.py --workspace-root . --output /tmp/picknplace-events.jsonl
```

Each package log overwrites itself before record 1,001. The compiler is a standalone script, not a ROS package; its output location is explicit.

The `gripper_control` GUI requires the separately launched bringup to already be
connected. It uses only the default Dobot V4 DO service and feedback topics; if
those interfaces are not live within five seconds, it writes a failure to its
package log and exits. It exposes raw DO1, DO2, DO13 and DO14 controls and
monitors DI1 and DI2 as generic digital inputs. Each output action requires
service acceptance (`res=0`) and observed FeedInfo output confirmation.

**Wiring correction / migration pending:** the user-confirmed physical map is
DO1 exhaust, DO2 finger close, DO13 suction, DO14 finger open, DI1 suction
detection, and DI12 finger fully open. The diagnostic GUI's input display and
maintenance-client migration remain pending. Its Quick Actions panel and
automatic Grip/Release sequences have been removed; individual output controls
and auto-off timing remain.

Configure and launch the Gemini 335 camera set through the project GUI:

```bash
ros2 launch orbbec_camera_launcher camera_launcher.launch.py
```

The GUI is the only camera configuration editor and saves canonical `ORBBEC_*`
values to root `.env`. The camera set is fixed at exactly two slots; camera
count is not an `.env` value or GUI control. Each camera-row button launches
only that saved camera
through the vendor driver in a separate visible terminal for diagnostics; it
has no watchdog and is stopped with Ctrl-C in that terminal. The lower action
launches the complete configured set headlessly under the mandatory watchdog,
starting Camera 1 and requiring both streams before Camera 2 is started with its
own five-second deadline. Camera 1 remains supervised throughout Camera 2
startup. Any camera failure stops both, then the full ordered sequence is
retried; there are three total full-set attempts separated by fixed
three-second rests after shutdown completes.
Launch modes are mutually exclusive. Every supervised serial and both
color/depth streams are required; supervised partial startup and unlimited
restart are forbidden. The package forces USB-only Orbbec enumeration while
all ROS 2 communication remains local.
With depth registration enabled, both launch paths request complete RGB/depth
bundles (`frame_aggregate_mode=full_frame`). The driver discards unsuccessful
software-alignment output before publishing aligned depth or CameraInfo. This
prevents occasional raw-depth intrinsics from disarming perception. Detector
calibration checks remain active; `.env` and teaching files need no new settings.
Rebuild `orbbec_camera` and `orbbec_camera_launcher` with camera processes stopped,
then relaunch the cameras and re-arm perception to activate this change.
See [`src/orbbec_camera_launcher/README.md`](src/orbbec_camera_launcher/README.md).

Calibrate one camera at a time with the local-only ChArUco GUI:

```bash
ros2 launch camera_calibration camera_calibration.launch.py
```

The standalone GUI follows the pinned
[MoveIt ROS 2 calibration pipeline](https://github.com/moveit/moveit_calibration/tree/3f9d48ebe843caf1de060bfafe78160585c7c26f)
without adding MoveIt dependencies. Manual calibration remains read-only;
explicit automatic capture sends guarded Dobot maintenance commands directly.
Calibration uses only RGB ChArUco and color CameraInfo: there is no depth subscription, synchronization,
plane fitting, fusion, or depth panel. Camera-launcher configuration is unchanged.

Enter the camera prefix, mode, dictionary and measured board geometry manually;
names are never inferred from root `.env`. Camera-to-hand references
`base_link`; camera-on-hand references `Link6`. Apply Settings saves validated
form fields as strict schema-4 `logs/camera_calibration/last_session.json`.
Startup restores unapplied prefill only, not samples or a solution.

The modern detector uses grayscale, color intrinsics/distortion, legacy board
layout, no marker-corner refinement, two adjacent markers, no marker recovery,
and iterative board PnP with at least four non-collinear ChArUco corners.
There is no editable corner minimum, target stability history or pose averaging.
Capture requires a board pose at most 0.5 seconds old plus robot TF and exact
six-joint feedback at most 1 second old. Both robot and camera-relative board
orientations must differ by at least 5 degrees from every earlier sample;
translation alone never qualifies. Hold stationary for capture and collect
rotations around multiple axes.

The fifth accepted sample and every later capture/removal automatically
recompute using fixed Tsai hand-eye solving. Robot TF reception remains on an
independent executor thread. The single RGB view overlays the calculated
camera XYZ/RPY and **FIT RMS**, and broadcasts only
`base_link -> <prefix>_link` or `Link6 -> <prefix>_link`, never a board TF.
Diagnostics include fit maxima/IDs, previous-solution changes, pose coverage,
leave-one-out from six samples, and separately labelled adjacent-pair
**AX=XB RMS** in mm/degrees. These are consistency metrics, not accuracy grades.
Below five samples the solution is cleared and TF broadcasting stops.
A failed leave-one-out omission preserves the full preview but disables saving.

Save YAML confirms the exact new timestamped path under `calibration/`.
Strict schema 7 stores pinned pipeline provenance, settings, diagnostics,
stable C# IDs, one RGB board pose, robot transform and six joint positions per
sample. Load Calibration explicitly replaces confirmed samples, validates the
five-sample/angular rules, and recomputes using live internal camera TF.
Existing schema 1–6 files are preserved but rejected without conversion.
After loading, **Start Automatic Capture** reuses the ordered joint positions
directly through canonical Dobot bringup, collects fresh samples after each confirmed
joint position and a one-second stationary/idle hold before each capture attempt.
Reported TCP is checked for live movement only, never against nominal CR10 forward
kinematics or the previous calibration's TCP. A position
gets three automatic attempts; only after all three fail does Continue/Stop appear.
Continue tries another three at the same position and keeps earlier samples.
Temporary RGB/CameraInfo unavailability uses the same retry workflow, with
confirmed Stop before retrying a move interrupted by lost camera readiness.
Loss of stationary hold also confirms Stop and retries the same saved joints,
with a new one-second hold and fresh capture. Only the interrupted attempt's
unconfirmed data is discarded; earlier accepted samples remain.
An isolated backward robot timestamp logs a warning and discards only that
feedback message. The next valid feedback continues the same route without
Stop, a repeated move or a consumed capture attempt. Discarded messages cannot
refresh the one-second feedback timeout or conceal a robot fault/I/O change.
The robot stays at the final position on completion. The separate
**Save as New Calibration** button opens an editable filename dialog with the
existing timestamped naming rule as its default. It saves inside `calibration/`
and never overwrites the source or another file. Replay is an attended maintenance
exception: close Robot Controller and other command tools first. Confirmed Start
requires fresh canonical joint/robot feedback, then runs Motion Debug's startup:
StopMoveJog, DisableRobot/disabled confirmation, EnableRobot/enabled confirmation,
SpeedFactor 50%, Tool 0, Tool 1 TCP zero and CP 100%. Only StopMoveJog and
DisableRobot are best effort; failures warn and continue. Enable and every
setting are strict, with five-second response/confirmation bounds. Before motion, confirm
stationary enabled/idle feedback, an empty queue, no faults, user/tool zero and
DI1 LOW. A failed five-second readiness check names the remaining blockers and
requests direct Stop. Replay uses 20% joint speed and acceleration with global
SpeedFactor 50%, preserves gripper outputs, and offers direct Stop throughout.
Loading and launch never move or enable the robot. See the
[calibration package workflow](src/camera_calibration/README.md#automatic-recalibration-from-a-loaded-file)
for capture gates and prerequisites.
RGB frames and overlays are transient memory only, never an accumulating archive.

The build verifies and extracts the existing exact vendored
`opencv-python 4.10.0.84` wheel offline into the package-private prefix.
The extracted runtime is copied into the install prefix as ordinary files even
for `colcon build --symlink-install`; its `cv2` loader and native binary must not
resolve back into `build/`, which would recursively import the Python package.
One lifetime spawned worker owns all OpenCV 4.10.0 detection/drawing/pose/solve
operations with one thread and OpenCL disabled; the ROS/Qt parent never imports
`cv2`. Runtime/protocol drift, native exit or timeout remains terminal with no
worker restart, fallback runtime, detector or solver.
See [package documentation](src/camera_calibration/README.md) and
[upstream attribution](src/camera_calibration/NOTICE.md).

Teach the platform origin after producing a valid camera calibration:

```bash
ros2 launch item_perception_yolo platform_teach.launch.py
```

At startup, Platform Teach reminds the operator to freshly calibrate the bin
camera in its current position before teaching or re-teaching the platform.
`platform_teach` explicitly loads one schema-7 camera-to-hand or camera-on-hand
YAML directly from root `calibration/`, inherits its camera prefix and ChArUco
geometry, and shows the same RGB marker/corner/board-axis feedback. A fixed
camera uses its calibrated `base_link <- camera_link` directly. An on-hand
camera additionally requires a live `base_link <- Link6` TF no older than one
second and composes it with calibrated `Link6 <- camera_link`. Place the board
origin at the bin-mount corner chosen as the pick-area origin, keep the robot
and board stationary, capture once, inspect the calculated
`base_link <- platform_reference` XYZ/RPY and RViz TF preview, then confirm
Save YAML. It performs no robot motion and does not subscribe to joint states.

The strict schema-4 result is saved beside the camera artifacts as
`calibration/platform_calibration_<UTC_TIMESTAMP>_<DOBOT_ROBOT_LAN1_IP>.yaml`.
It stores the fixed base/platform transform separately from `teaching_provenance`:
the source camera calibration/hash, ChArUco settings and original capture chain.
Loading the platform does not require that source camera file. It stores no RGB
frame or overlay. Existing schema-3 platforms remain readable without a source-file
dependency; explicit migration only reorganizes metadata, preserving geometry. See
[`src/item_perception_yolo/README.md`](src/item_perception_yolo/README.md).

After saving a platform calibration, teach a bin ROI with four 5x5 ArUco
markers (IDs 0–3):

```bash
ros2 launch item_perception_yolo bin_teach.launch.py
```

Select the platform calibration and an independent **Active camera calibration**,
choose the exact 5x5 dictionary, enter the measured common marker size, and apply.
The camera selector prefills the active Item Teach `.env` selection. The node uses
that camera's mode: fixed-camera mode uses its static calibrated
mount, while on-hand mode requires fresh `base_link <- Link6` TF. All four IDs
must be visible; their ID order does not define bin-corner order. The private
worker undistorts all detected image corners using color CameraInfo. Bin Teach
intersects their rays with the existing platform Z=0 plane, without flattening
the platform relative to the robot base or using marker PnP depths for ROI XY.
The GUI selects each plane corner farthest from the four-marker centroid, draws the
resulting polygon, and saves four clockwise metric XY points in
`platform_reference` as strict schema 3:

```text
offline_teach/bin_teach/bin_teach_<UTC_TIMESTAMP>_<DOBOT_ROBOT_LAN1_IP>.yaml
```

The node is RGB-only and launches no camera, robot, RViz, or motion process.
After a successful capture, it dynamically publishes the complete RViz preview
`base_link -> platform_reference -> bin_corner_1..4` at 10 Hz. The corner frames
use the exact saved clockwise P1–P4 XY coordinates, Z=0, and platform-aligned
orientation. Retake, re-Apply, terminal failure, or node exit stops publication.
Run `dobot_rviz` separately to inspect the frames.
Platform outputs use schema 4; portable Bin Teach remains schema 3. The platform records the
shared ChArUco-origin/axis convention; the bin file separates portable XY
geometry from its original station's teaching evidence. Each station must
reproduce the same origin, axes, bin size and bin offset. Platform and corner
markers are approximately coplanar within a station; absolute station height
may differ.

Teaching's yellow border and loading's green border project the same platform
XY/Z=0 geometry, with 32 lens-distorted samples per edge. A tilted platform is
preserved. Parallel, behind-camera or invalid ray intersections block capture.
The planar projection assumes the physical corners lie on the taught plane;
an incorrect platform/camera calibration can still distort metric dimensions.
Existing files are unchanged: capture and save a new bin file to replace geometry
previously taught by dropping marker-PnP Z. There is no automatic repair/migration.

Bin ROI files are teaching data and are saved/loaded only in
`offline_teach/bin_teach/`. Camera and platform calibrations stay in
`calibration/`. To inspect a copied bin template at another station, put it in
that station's `offline_teach/bin_teach/`, Apply that station's own
platform calibration in Bin Teach, then select **Load Bin ROI** and confirm the
physical reference arrangement. RViz previews the unchanged XY points using the
destination platform transform. Live RGB shows a green border labelled
`Loaded Bin Teach | <filename>`, projected using the current station's camera
calibration and color intrinsics/distortion, with no marker detection required.
An on-hand camera also requires fresh robot TF for the RGB projection. Missing
or stale projection inputs hide the border with an explanation. The original
station's robot configuration and
calibration files are not required to load the template. Loaded templates are
preview-only; Retake clears them before a fresh capture, and Save YAML writes
only fresh captures. Older platform/bin schemas 1–2 are preserved but rejected;
re-teach them to produce platform schema 4 and bin schema 3. Camera calibration remains schema 7.

Teaching previews do not define the future item detector's behavior. Item
detection and picking are outside this change, and will consume the files
independently of these teaching nodes and their RViz previews.
See [`src/item_perception_yolo/README.md`](src/item_perception_yolo/README.md).

For first-time USB setup, install the udev rules supplied by Orbbec:

```bash
sudo bash src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh
```

Refer to the upstream READMEs for complete robot safety, networking, camera, and launch instructions. Pick-and-place and YOLO application packages will be added in subsequent steps.
