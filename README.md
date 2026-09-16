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

The root build contains 15 ROS packages below `src`: seven packages grouped under the official vendor snapshots and the project-level `motion_debug`, `gripper_control`, `orbbec_camera_launcher`, `camera_calibration`, `item_perception_yolo`, `robot_controller`, `robot_controller_interfaces`, and `item_perception_interfaces` packages. Imported `item_pick` is reference-only and excluded by `COLCON_IGNORE`. Gazebo/robot simulation, MoveIt, vendor demonstration nodes, `servo_action`, and the Dobot `ServoJ`/`ServoP` streaming interfaces are deliberately excluded. Each retained package has a package-local README describing its role and safe entry points. See [`src/README.md`](src/README.md) for the complete package index. The vendor grouping is intentional and must remain intact for offline provenance and refreshes.

## Robot Controller v2

`robot_controller` is now the production hardware authority for deterministic
Home and Pick operations. Normal launch separates it into a headless controller,
a TF-only preview process with no Dobot clients, and an API-only GUI. Headless
launch starts only the controller and strictly loads the flat `runtime_teach/`
catalog. Crucially, neither launch mode enables or moves the robot: both require
an explicit typed `/robot_controller/startup` call.

Home and Pick are native ROS actions, and each goal carries the exact active
configuration SHA-256 so stale clients cannot execute replaced teach files.
Pick requests one fresh hash-matched batch from the sole canonical provider:
headless `item_detect`, or explicitly Armed `item_teach` during attended use.
They share `/item_detect/get_item_poses`; running both providers is rejected.
Candidate count always comes from Item Teach `pose_candidates`. Typed Startup,
Recover, Pause, Continue, direct Stop, Configure and global-speed services support
those actions, while reliable transient-local typed status reports the state and
operation phase. The old Trigger/JSON/Live/Enable/validation/pose-proxy/debug-image
endpoints are removed.

```bash
ros2 launch robot_controller robot_controller.launch.py
ros2 launch robot_controller robot_controller.launch.py headless:=true
```

Startup performs strict Stop/queue confirmation, DI1 protection,
disable/conditional-clear/enable, SpeedFactor 100/User 0/Tool 0/Tool-1-zero/CP
100, unheld output reset, and coherent READY confirmation. Recover performs the
same guarded recovery without moving Home. Pause preserves the current queue and
active Home/Pick generation; Continue resumes only that confirmed paused state.
Direct Stop and native cancellation preserve all gripper outputs, discard queued
motion, and finish in `RECOVERY_REQUIRED`; Stop never requires Pause first and
never automatically Homes, releases, or resumes. Trusted held-item DI/output
feedback is checked throughout recovery and Stop confirmation. If idle
supervision sees an unexpected running/nonempty queue, it pre-empts that motion
with the independent Stop path before requiring recovery.

The Dobot `isPauseCmdFlag` bit is not a general readiness gate. Hardware evidence
shows that `EnableRobot()` may latch it to one while mode 5 is enabled and the
queue is empty/not running; `Continue()` is rejected in that condition. The
controller recognizes resumable Pause only from its own confirmed Pause request
and retained operation context.

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

Home uses fresh GetPose only to decide whether an upward current-XY rise is
needed, then sends exact taught joints through joint-mode MovL. Pick runs Home,
transforms platform-relative poses, applies schema-6 vertical geometry and
timed gripper behavior, and returns Home after every attempt. Only missed suction
advances to another candidate. No-I/O moves use MovL, real timed-output moves use
non-empty MovLIO, and the conditional rise uses RelMovLUser. Continue is used only
by the explicit paused-queue service; the controller never uses InverseKin. See the
[controller README](src/robot_controller/README.md) for its typed APIs, state
machine, raw CLI examples, timing policy and commissioning requirements.

Home arrival means every actual joint is within ±1° of its taught value for
300 ms with enabled, fault-free, stationary, empty-queue feedback. Cartesian
waypoints use 5 mm Euclidean translation and 1° orientation with the same final
feedback gates. Hardware Home and Pick's initial shared-Home step first apply
that exact joint gate; if the robot is already Home, they log the skip and send
no GetPose or Home motion. Queued return-to-Home paths after a pick attempt are
not skipped.

## Item Teach and controller

In separate terminals after building and sourcing the workspace:

```bash
ros2 launch item_perception_yolo item_teach.launch.py
ros2 launch robot_controller robot_controller.launch.py
```

Item Teach selects `.pt` from any directory, edits grouped item/YOLO settings,
and records all six actual home joints from fresh canonical bringup feedback.
Save creates a strict schema-6 YAML and SHA-256-bound `.pt` copy under
`offline_teach/item_teach/`, with matching timestamped names and a confirmation
dialog. Transfer both files together; the original model path is not needed.
Home joints are portable between the user's identical robots: source IP/node
are provenance, not a station restriction. Loading never replays joint positions.

**Load Item Teach** also loads its paired `.pt` and reads the model classes after
one combined replacement/trust confirmation. Saved class selection, geometry and
settings are preserved; no separate Load Model click is needed. Missing, changed
or incompatible pairs are rejected. YOLO and Armed remain OFF, and startup
prefill still does not execute model weights. Armed ON is highlighted red so
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
complete the form, and Save a valid schema-6 YAML/.pt pair before simulating,
arming or sending it to the controller. The same known item name updates the
loaded file with a previous-version backup; an unknown original name creates a
new pair. Loading alone never rewrites files. Detector/controller loaders
accept only complete schema-6 profiles; they never recover old files.
The removed zheight_offset is not recovered. Old retract_height is blank in GUI
drafts because it now means extra clearance above pre-pick, not above pick.

Item Teach also edits per-motion speed and acceleration percentages (integers
1–100). New profiles explicitly start with travel/Home speed 100%, final-approach
speed 6% and pick-to-prepick retract speed 6%; remaining clearance/Home moves
use travel speed. Acceleration starts at 100%
for all three phases. Save records separate `speed` and `acceleration` groups.
The controller passes each target's `v=`/`a=` to MovL, MovLIO or the Home-height
RelMovLUser exception, independently of the controller's global SpeedFactor
(100% at initialization, adjustable explicitly while idle). Loaded rates are
preserved; missing/invalid rates in old GUI recovery drafts remain blank,
never silently defaulted. Production rejects schemas 1–5.
Motion saves only standoff_height, prepick_height and retract_height:
pick Z = item Z + standoff; pre-pick Z = pick Z + prepick;
clearance Z = pre-pick Z + retract. Offsets are millimetres in robot base Z.
Queued commands use cp=0 to preserve these corners and rate boundaries;
startup/global CP remains 100%. See the controller README for feedback/Stop
confirmation and deployment safety requirements.

For a standalone `.pt`, select **Load Model / Read Classes** and confirm it is trusted. You can
load while the automatic bin border is updating: the confirmed
load takes the next worker slot, with queued/loading progress and no retry.
YOLO and Armed remain OFF after loading. Use **Connect RGB**, then **YOLO Detect**.
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
The view retains all model classes and size failures. Item borders are green
within the taught size tolerance, red outside it, and gray if dimensions or
plane measurement are unavailable. Green means size-valid, not a validated pose.
Select the station platform/bin and mask/OBB to measure long-X/height and
short-Y/width on platform Z=0. Registered depth is displayed alongside RGB;
missing/mismatched depth leaves RGB detections visible but blocks pose calculation.
Registered depth and RGB must share their optical frame, dimensions and K.
Their lens-distortion coefficients may differ: the worker uses both CameraInfo
models to map the physical sampling circle and RGB mask onto native depth pixels.
Depth is not resized or interpolated; the original RGB pick center is unchanged.
Click an item to freeze the exact RGB/depth observation and calculate only its pose.
Class, size, ROI, freshness and MAD depth checks must pass. No newer image/depth
or TF is substituted, and an expired snapshot is rejected. The top-left shows
platform-relative XYZ/yaw and dimensions. A valid click publishes teaching-only
`base_link -> item_teach_selected_item` at 10 Hz, composed with the platform's
full tilt/height. This frozen TF is not a live tracked item or a robot command;
it remains until resuming, changing settings/sources/arming, YOLO OFF, failure or exit.
Click the image again to resume. Item Teach never launches RViz; use its TF
display to inspect that frame. It does not publish another `platform_reference`
authority. Rejected clicks show a reason and publish no selected pose.
The cyan selection ring follows `pickdepth_radius` (circle diameter in mm),
projected from platform Z=0 with the same geometry used for depth sampling.
It may appear elliptical under perspective; no fixed-pixel ring is substituted
when calibration is unavailable. Edit the diameter, then click an item again.
Depth mirrors RGB mask shading, size borders, long-X/short-Y axes, pick dots
and bin ROI through its own calibrated pixel model. Both views freeze together
on a click; only the selected pose is calculated from that exact pair, while
all item outlines remain visible. Both panes show selection/pose feedback;
accepted depth points are black and rejected points red. The redundant help
paragraph above the views is removed. Platform/Bin Teach use the same compact
setup/large-video presentation, with Save/capture visible and extra calibration
details expandable; their explicit Apply and capture/save workflow is unchanged.
RGB keeps mask shading and one mask-derived rectangle (or the native oriented
rectangle for OBB), with centered long-X/short-Y lines and a pick-point dot.
No extra axis-aligned YOLO box is drawn. The green loaded bin ROI appears on the
same image, including with YOLO OFF. Item Teach automatically selects the latest
platform for this robot and latest calibration for that platform's camera from
root `calibration/`; filenames' UTC timestamps define newest, not modification
times. Select only the portable bin file to validate and connect its calibrated
camera; there is no platform picker or Apply button. Read-only paths show the
chosen pair; **Reload Latest Calibration** reselects it, stopping YOLO/disarming
and clearing old previews first. Calibration selection is not a live watcher.
The newest camera must match the platform's recorded hash: after recalibrating
that camera, re-teach the platform. Invalid/missing/ambiguous files fail visibly;
there is no older-file fallback or mixing of calibration transforms.
The bin file records its teaching platform's filename, SHA-256 and transform.
If the selected platform's SHA-256 differs, Item Teach shows an amber warning
under the file selectors with both filenames (full hashes in its tooltip and
Activity event). Intentional cross-station reuse stays allowed: verify the
same physical origin, X/Y directions, bin size and placement. This checks file
identity, not physical alignment; it never substitutes the original transform.
The saved bin selection reconnects this preview using the latest station pair at
startup, not an older platform prefill. The border appears when fresh RGB,
CameraInfo and required TF arrive; this never launches cameras, executes a model
or arms the pose service. The
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
Both views freeze with only the returned ranked P1…Pn overlays, capped by
`pose_candidates`, and the green bin border. Every returned pose also publishes
a teaching-only TF at 10 Hz: `base_link -> item_teach_candidate_1` through
`item_teach_candidate_N`, matching the image priorities. Use RViz's TF display;
Item Teach does not launch RViz. These frozen poses preserve the platform's full
tilt/height, replace any clicked-item/batch preview, and stop publishing on resume,
edits/source changes, arming changes, YOLO OFF or failure/exit. Empty batches
publish no candidate frames. TF/RViz may briefly retain old frames after stopping.
Pose/depth/count feedback and ages
are in the top bands; additional pose details are in Activity. A shortage or zero
valid items is explicit. Click RGB again to resume; edits/source changes or YOLO
OFF cancel old results. The armed service continues acquiring independent fresh
observations—it never returns this frozen teaching batch.
Unsaved text-box edits are not autosaved; restart prefills the selected saved
teach YAML and camera prefix. Station/bin selections are the narrow automatic
read-only-preview exception; item settings, model execution and arming remain unapplied.

`item_detect.launch.py` uses the same automatic station selection, with explicit
item/bin artifact paths, `trusted_model:=true` and `armed:=true`; its
`platform_teach_file` argument is removed. Platform/Bin Teach retain their
explicit calibration selection. The controller requests one fresh batch directly
from `/item_detect/get_item_poses`, always using the taught `pose_candidates`
limit and exact profile/station hashes. Detector/teach nodes remain read-only;
only explicit controller Startup followed by a typed Home/Pick action can issue
motion or I/O. No training is included. Private inference
wheels are pinned/verified/extracted offline; exact Torch/system dependencies
still require separate provisioning. The retired training-oriented teacher
and separate `item_detect_yolo_debug` sources/launchers have been removed.
See [Item Teach](src/item_perception_yolo/README.md) and
[robot_controller](src/robot_controller/README.md) for details.

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

Launch the read-only actual-robot and TF viewer in a second sourced terminal
after bringup is connected and publishing:

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

The file uses strict `KEY=value` lines and does not require a Python dotenv package. Do not use shell exports, alternate key names, or alternate configuration paths. Every Dobot and Orbbec key shown in `.env.example` is required. Active Orbbec serial numbers may remain empty only while opening the camera GUI for first-time configuration; camera startup remains disabled until the configured set is complete.

Runtime datalogs are isolated per package under `logs/<package-name>/events.jsonl`; the bringup launch creates the package files before starting the node. Compile the timestamped package records into a separate universal file only when needed:

```bash
python3 scripts/compile_logs.py --workspace-root . --output /tmp/picknplace-events.jsonl
```

Each package log overwrites itself before record 1,001. The compiler is a standalone script, not a ROS package; its output location is explicit.

The `gripper_control` GUI requires the separately launched bringup to already be
connected. It uses only the default Dobot V4 DO service and feedback topics; if
those interfaces are not live within five seconds, it writes a failure to its
package log and exits. Its fixed output map is DO1 suction exhaust, DO2
gripper close, DO13 finger close, and DO14 gripper open; DI1 and DI2 are
monitored as generic digital inputs with semantic names intentionally deferred.

**Wiring correction / migration pending:** the user-confirmed physical map is
DO1 exhaust, DO2 finger close, DO13 suction, DO14 finger open, DI1 suction
detection, and DI12 finger fully open. The preceding paragraph describes the
still-unmigrated GUI, not the new wiring contract. Do not use its old automatic
Grip/Release patterns with the corrected wiring. The new controller's explicit
I/O sequence and open-confirmation timing must be finalized before use.

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
See [`src/orbbec_camera_launcher/README.md`](src/orbbec_camera_launcher/README.md).

Calibrate one camera at a time with the local-only ChArUco GUI:

```bash
ros2 launch camera_calibration camera_calibration.launch.py
```

The standalone GUI follows the pinned
[MoveIt ROS 2 calibration pipeline](https://github.com/moveit/moveit_calibration/tree/3f9d48ebe843caf1de060bfafe78160585c7c26f)
without adding MoveIt dependencies or robot motion. Calibration uses only RGB
ChArUco and color CameraInfo: there is no depth subscription, synchronization,
plane fitting, fusion, or depth panel. Camera-launcher configuration is unchanged.

Enter the camera prefix, mode, dictionary and measured board geometry manually;
names are never inferred from root `.env`. Camera-to-hand references
`base_link`; camera-on-hand references `Link6`. Apply Settings saves validated
form fields as strict schema-4 `logs/camera_calibration/last_session.json`.
Startup restores unapplied prefill only, not samples or a solution.

The modern detector uses grayscale, color intrinsics/distortion, legacy board
layout, no marker-corner refinement, two adjacent markers, no marker recovery,
and iterative board PnP with at least four non-collinear ChArUco corners.
There is no editable corner minimum, stability wait or pose averaging.
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
Joint positions are recorded only; this package never replays robot motion.
RGB frames and overlays are transient memory only, never an accumulating archive.

The build verifies and extracts the existing exact vendored
`opencv-python 4.10.0.84` wheel offline into the package-private prefix.
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

The strict schema-3 result is saved beside the camera artifacts as
`calibration/platform_calibration_<UTC_TIMESTAMP>_<DOBOT_ROBOT_LAN1_IP>.yaml`.
It records the selected camera calibration and SHA-256, inherited ChArUco
settings, and exact transform chain, but no RGB frame or overlay. See
[`src/item_perception_yolo/README.md`](src/item_perception_yolo/README.md).

After saving a platform calibration, teach a bin ROI with four 5x5 ArUco
markers (IDs 0–3):

```bash
ros2 launch item_perception_yolo bin_teach.launch.py
```

Select the platform calibration, choose the exact 5x5 dictionary, enter the
measured common marker size, and apply. The node uses the same camera mode as
the selected platform artifact: fixed-camera mode uses its static calibrated
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
Platform and bin outputs now use strict schema 3. The platform file records the
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
re-teach them to produce schema 3. Camera calibration remains schema 7.

Teaching previews do not define the future item detector's behavior. Item
detection and picking are outside this change, and will consume the files
independently of these teaching nodes and their RViz previews.
See [`src/item_perception_yolo/README.md`](src/item_perception_yolo/README.md).

For first-time USB setup, install the udev rules supplied by Orbbec:

```bash
sudo bash src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh
```

Refer to the upstream READMEs for complete robot safety, networking, camera, and launch instructions. Pick-and-place and YOLO application packages will be added in subsequent steps.
