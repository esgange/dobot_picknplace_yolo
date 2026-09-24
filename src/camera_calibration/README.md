# camera_calibration

Standalone, local-only RGB ChArUco hand-eye calibration for a manually selected
Orbbec camera. The pipeline follows the pinned
[MoveIt ROS 2 implementation](https://github.com/moveit/moveit_calibration/tree/3f9d48ebe843caf1de060bfafe78160585c7c26f),
without installing MoveIt or changing this package's GUI/launch command.
See [NOTICE.md](NOTICE.md) for source attribution and the upstream BSD license.

## Modes and prerequisites

| Mode | Camera mounting | Board mounting | Output transform |
| --- | --- | --- | --- |
| Camera to hand | Fixed relative to base | Rigidly mounted on Link6 | `base_link <- <prefix>_link` |
| Camera on hand | Rigidly mounted on Link6 | Fixed relative to base | `Link6 <- <prefix>_link` |

Start bringup, the canonical robot TF publisher, and the camera separately.
Calibration needs actual `/joint_states`, fresh `base_link <- Link6`, and
the camera's color image, color CameraInfo, and internal camera TF.
It never launches cameras, bringup, RViz or the controller. Manual capture is
read-only. Explicit automatic capture is an attended maintenance exception, like Motion
Debug and Gripper Diagnostics: it sends guarded CP/MovJ/Stop requests directly
to canonical Dobot bringup. It has no Robot Controller dependency.
Depth is neither subscribed to nor required. Camera-launcher settings and
supervision remain independent and unchanged.

```bash
source ~/PicknPlace/scripts/source_ros_workspace.bash
ros2 launch camera_calibration camera_calibration.launch.py
```

The launch has no arguments and requires `ROS_LOCALHOST_ONLY=1`.
Exactly two ROS executor threads keep the tf2 listener's reentrant callback
group independent from serialized RGB/CameraInfo/joint callbacks. RViz and
calibration consume TF from `robot_state_publisher`; RViz is not its source.

## Form and capture

Choose a mode explicitly, enter the camera prefix manually, and enter the
dictionary, checker-square counts X/Y, measured square size, and measured marker
size in millimetres. No camera name is inferred from root `.env`.
Apply Settings derives exactly:

```text
/<prefix>/color/image_raw
/<prefix>/color/camera_info
<prefix>_color_optical_frame
<prefix>_link
```

Apply clears both samples and solution, restarts the C# counter, and atomically
writes validated form values to ignored
`logs/camera_calibration/last_session.json`. This store accepts only schema 4,
records `minimum_samples: 5`, and has no editable-corner setting. Startup
restores unapplied form prefill only: no auto-Apply, samples, solution, or
hardware actions. Missing state means first run with empty mode and prefix;
malformed or older state hard-fails. The approved implementation explicitly
rewrote this workstation's validated old prefill; there is no runtime migration.

The RGB-only detector uses color CameraInfo intrinsics and distortion throughout:

1. Convert the owned, contiguous RGB frame to grayscale.
2. Detect ChArUco using `CharucoDetector.detectBoard`, with
   `CORNER_REFINE_NONE`, `minMarkers=2`, and `tryRefineMarkers=False`.
3. Require at least four non-collinear checker corners. Match their IDs to
   measured board coordinates with `CharucoBoard.matchImagePoints`.
4. Estimate one complete optical-frame-from-board pose using
   `solvePnP(..., SOLVEPNP_ITERATIVE)`. Draw axes directly from this RGB pose.

The dictionary and `CharucoBoard` use `setLegacyPattern(True)` for the existing
physical boards. Missing markers, partial/collinear corners, or a normal
no-pose result block that frame; malformed native output remains terminal.

There is no target stability history, pose averaging, depth fusion,
or translation-based diversity exception. Automatic capture adds a one-second
robot stationary/idle hold before each attempt. Hold the robot stationary while
capturing. A sample stores the latest valid RGB pose (age at most 0.5 seconds),
latest robot TF (at most 1 second), and fresh ordered `joint1` through `joint6`
feedback (at most 1 second, six finite radians, non-zero timestamp).
For **every** earlier sample, both the robot orientation and camera-relative
board orientation must differ by at least **5 degrees**. A rejection names the
conflicting C# ID, failed angular check, and measured angle.
Collect rotations around multiple axes; five samples do not guarantee
informative geometry or physical accuracy.

## Solve and visualization

Sample five automatically solves using the fixed OpenCV
`CALIB_HAND_EYE_TSAI` method (upstream `Tsai1989`). Every later capture or
calibration-sample removal recomputes from the full remaining set. No solver
selector, manual Compute action, retries, or alternate method is provided.

Writing H = `base_link <- Link6`, C = `camera_optical_frame <- board`:
camera-on-hand passes H directly to hand-eye solving; camera-to-hand passes
inverse(H). Both pass the RGB C observations. The optical mounting result X
is composed with inverse(`camera_link <- camera_optical_frame`) from the
required live camera-internal TF to obtain the output camera-link transform.

A successful solve overlays reference-relative XYZ metres, RPY degrees, sample
count, and **FIT RMS** on the expanded single RGB view. Only the solved
`base_link -> <prefix>_link` or `Link6 -> <prefix>_link` is broadcast for RViz.
Board axes remain video-only; `charuco_board` is never broadcast.
Fewer than five samples clears the solution and stops calibrated-TF publication.

## Neutral consistency diagnostics

The read-only panel and sample table show:

- FIT RMS, maximum translation/rotation residuals, and responsible sample IDs.
  Residuals measure consistency of the inferred fixed `Link6 <- board` for
  camera-to-hand or `base_link <- board` for camera-on-hand.
- Changes from the previous solution in FIT RMS and solved camera TF.
- Robot coverage as maximum pairwise translation and rotation span.
- Leave-one-out camera-TF RMS, maxima and omitted-sample IDs from six samples.
- Separately labelled **AX=XB RMS**, in millimetres and degrees, over adjacent
  captured pose pairs in their original order.

AX=XB uses A = inverse(H_i) H_(i+1) for on-hand, or H_i inverse(H_(i+1))
for to-hand, and B = C_i inverse(C_(i+1)). Translation error averages the
forward and inverse AX/XB translation distances; rotation is their relative
angle. RMS is taken across adjacent pairs. This follows the upstream
`getReprojectionError` calculation, but is **not** pixel reprojection error.
Upstream returns rotation first despite its GUI's labels; our named fields
assign translation to mm and rotation to degrees explicitly.

These are consistency diagnostics, not accuracy grades or automatic rejection
thresholds. There is no holdout-validation sample set.
At five samples leave-one-out is unavailable and does not block saving.
From six onward a failed omission retains the full-solution preview, identifies
and logs the omitted ID, and disables saving. No sample is removed automatically.

Remove Selected Sample requires confirmation. Undo Last Calibration removes
only the last calibration sample. Reset Samples clears samples, solution history,
and ID counters. Within a session IDs remain non-reused after removal.

## Save and load

Save YAML requires a current solution and passing applicable leave-one-out
checks. Each save atomically creates one new timestamped file and displays
success with its exact path, or explicit failure:

```text
WORKSPACE_ROOT/calibration/camera_to_hand_calibration_<UTC_TIMESTAMP>.yaml
WORKSPACE_ROOT/calibration/camera_on_hand_calibration_<UTC_TIMESTAMP>.yaml
```

The UTC token is `YYYYMMDDTHHMMSS_microsecondsZ`; existing files are not overwritten.
Only strict **schema 7** is written/read. It records settings, output transform,
pinned pipeline commit/runtime/detection/solver provenance, all diagnostics,
stable sample IDs, one RGB optical-frame-from-board pose per sample, its robot
transform, and its exact six joint positions. No fused pose or depth fields exist.

Load Calibration is the sole explicit sample-restoration workflow. It confirms
replacement, validates schema 7, at least five samples and both all-prior angular
checks, preserves order/IDs/joints, applies artifact settings, and recomputes
using the live internal camera TF. Older schema 1–6 files remain untouched but
are explicitly rejected; there is no conversion reader or stored-result shortcut.
Loading alone never moves the robot.

## Automatic recalibration from a loaded file

1. Start the configured camera and canonical Dobot bringup (including its TF
   publisher). Close Robot Controller, Motion Debug and Gripper Diagnostics
   before replay; calibration must be the only command application. No Item/Bin
   Teach files or controller process are required.
2. Load a valid schema-7 calibration. Its settings and ordered joint positions
   become the replay recipe. Prepare an already enabled, idle robot with user
   and tool zero and DI1 LOW. Launch and loading perform no enable or motion.
3. Click **Start Automatic Capture** and confirm that the starting position and
   all connecting joint paths are clear. Replay uses exact saved joint targets
   with MovJ at 20% speed/acceleration; the existing global speed factor also
   applies. Calibration sets global CP(100), preserves all outputs, and never
   enables/disables the robot or resets the gripper during replay.
4. After validating robot readiness and all CR10 target limits, Start clears
   working observations and the solution. Each position waits for valid camera
   input before moving. The saved joints and completed queue are physically
   confirmed, then held stationary for one full second before collecting a fresh
   sample. Each position gets three automatic attempts before a Continue/Stop
   prompt. Continue starts another three attempts at that position, preserving
   all earlier samples. Progress shows the position, attempt and phase. The robot stays at the final position.
5. Review diagnostics, then click **Save as New Calibration**. Edit the filename
   in the dialog or accept the default
   `<mode>_calibration_<YYYYMMDDTHHMMSS_microsecondsZ>.yaml`. Saves stay in root
   `calibration/`; `.yaml` is appended when no extension is supplied. Cancel
   changes nothing. Existing files, including the loaded source, cannot be
   overwritten. The strict schema-7 writer and all quality gates still apply;
   an incomplete or failed replay cannot be saved. Keep the default filename
   for Item Teach/Detect's strict automatic station discovery; custom names are
   intended for explicit loading and are not an alternative discovery format.

Arrival requires advancing canonical FeedInfo after command acceptance, the
returned MovJ queue ID, idle/empty queue and RobotStatus's enabled-idle flag,
and actual joints within 1 degree of the saved joints. There is no comparison
against nominal CR10 forward kinematics or the previous calibration's TCP;
the canonical CR10 file supplies joint limits only. Live TCP is used only to
check movement between actual feedback samples.
Two distinct advancing samples must show TCP unchanged within 0.05 mm/degrees
and joints within 0.05 degree. Before **every capture attempt**, require a full
one-second stationary/idle hold with advancing feedback and these same limits.
RGB, joints and robot TF must all be newer than the end of that hold; RGB remains at most 0.5 seconds
old and joints/TF at most 1 second old. FeedInfo and RobotStatus must remain
fresh within 1 second, with an advancing controller timer. Stationarity,
enabled/idle state, DI1 LOW and unchanged outputs remain monitored during
capture and solving; only a successful capture permits the next move.

Dobot responses have a five-second deadline. Motion is bounded to 300 seconds,
with a three-second no-progress limit. Fresh-sample waiting is bounded to ten
seconds, with up to twenty additional seconds for processing. Before every
move, each camera-readiness attempt allows up to two seconds for valid RGB and
CameraInfo while supervising the actual idle pose. Missing CameraInfo or missing,
stale or future-dated RGB input consumes an attempt, including before the first
move, during a move and after a motion retry. Readiness lost during a move requires
confirmed physical Stop before retrying. Three failed attempts show Continue/Stop;
Continue waits for valid camera input before moving to the saved joints.
Stream readiness uses the last validated RGB input timestamp before OpenCV
detection; the subscription retains only the newest queued image. This does not
relax capture freshness or treat input as an accepted board observation.
A missing/rejected board observation after ten seconds retries capture at the
same pose. The next attempt starts with a new one-second hold and fresh image/
joints/TF; it does not move again or reuse earlier observations. After three
failed attempts, a nonmodal **Continue / Stop** prompt lets the operator inspect
the camera view. Continue starts a new three-attempt batch here; Stop or closing
the prompt ends the run. The main Stop button and robot monitoring stay active.

A robot-arrival timeout is reported separately with observed/expected queue ID,
idle state and joint error. It consumes an
attempt and requires confirmed physical Stop, all replies received, unchanged
outputs and fresh fault-free enabled/idle feedback before retrying the same
move. After three such failures, the prompt explains that Continue may move
the robot again. No skipped positions or automatic Home return is introduced.
An explicit operator Stop, new Stop attempt or latched feedback fault cannot
be cleared by retry. Joint changes also count as motion progress.

Malformed/stale feedback, changed outputs, a competing command application,
native camera failure, rejected/unanswered commands, failed solving, or Stop
end the run through direct
Dobot Stop and physical confirmation. Stop acknowledgement alone is insufficient:
fresh advancing feedback must show a stationary robot and empty queue. A late
accepted abandoned MovJ triggers another containment Stop. These failures do
not enter the observation/camera-readiness/arrival retry workflow.

**Stop Automatic Capture** stays available while moving, sampling or stopping.
An unconfirmed Stop blocks editing and another run; another explicit Stop makes
a new attempt. GUI shutdown requests Stop for an active run before stopping its
executor. A confirmed Stop permits explicit restart; Apply Settings can start a
separate manual calibration. Partial replay is never saved as a replacement.
The recipe stays in memory and never replays after process restart.

Exactly two ROS executor threads remain: camera/joints/capture are serialized,
while Dobot feedback, responses and Stop use an independent reentrant group;
TF retains its own reentrant group. An operation thread sequences commands and
supervises the robot while the existing private OpenCV worker solves. No new
configuration store, artifact schema or native runtime is introduced. Software
tests use an isolated fake Dobot node; live paths still require attended validation.

## Isolated runtime, failure handling, and storage

The build verifies SHA-256
`9ace140fc6d647fbe1c692bcb2abce768973491222c067c131d80957c595b71f` and extracts
the vendored `opencv-python 4.10.0.84` wheel offline into the package prefix.
The extracted runtime is always installed as ordinary copied files, including
under `colcon build --symlink-install`. Its `cv2` loader and native binary must
never be build-tree symlinks because that layout triggers OpenCV's guarded
recursive package import.
One lifetime worker created with multiprocessing `spawn` owns all OpenCV calls,
loads only that private OpenCV exactly `4.10.0`, uses one OpenCV thread, disables
OpenCL, and serializes every request. The ROS/Qt parent never imports `cv2`;
system OpenCV remains unchanged and is never a fallback.

Bin Teach also uses this worker's checked `image_rays` operation
(`cv2.undistortPoints`) to obtain normalized color-optical corner rays. Ray/plane
intersection runs in NumPy outside the child, and `project_points` draws the
same saved-plane border during teaching and loading. This does not change
camera-calibration detection, pose estimation or hand-eye mathematics.

Startup and requests have a fixed five-second response bound. Worker exit,
timeout, runtime/module/API/checksum/thread/OpenCL drift or malformed output
stops TF, logs PID/signal/frame metadata and a ROS fatal message, and terminates
the node with code 1. The worker is never restarted or replaced.

ROS color input is exact tightly packed little-endian `rgb8`. Color/CameraInfo
must have the derived optical frame, valid timestamps and finite supported
intrinsics/distortion with matching dimensions. An invalid message is logged
and skipped, clears readiness and invalid CameraInfo, and preserves an existing
solution. Only a later independently valid message can restore readiness.
No resizing, encoding conversion, alternate topic or stale-data fallback exists.

RGB/grayscale frames and overlays are transient memory only: no image archive,
raw frame, rendered image, or video path is saved in YAML, events or UI state.
Only numerical samples accumulate in memory during a long session.
UTC events use `logs/camera_calibration/events.jsonl`, overwriting before
record 1,001. The last-session file contains only the latest validated form.

## Build and synthetic verification

```bash
cd ~/PicknPlace
source scripts/source_ros_workspace.bash
colcon build --packages-select camera_calibration
colcon test --packages-select camera_calibration
colcon test-result --test-result-base build/camera_calibration --all
```

Tests use synthetic transforms/images, offscreen Qt widgets and isolated fake
Dobot ROS services; they never connect to cameras or robot hardware. The standalone
`test/opencv_worker_stress.py` runs 10,000 mixed 1920x1080 RGB frames through
one unchanged private-worker PID.
