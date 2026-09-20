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
It never launches cameras, bringup, RViz, or commands/replays robot motion.
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

There is no stability history, one-second wait, pose averaging, depth fusion,
or translation-based diversity exception. Hold the robot stationary while
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
Samples are never replayed as robot commands.

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

Tests use synthetic transforms/images and offscreen Qt widgets, never cameras,
bringup, RViz or robot commands. The standalone
`test/opencv_worker_stress.py` runs 10,000 mixed 1920x1080 RGB frames through
one unchanged private-worker PID.
