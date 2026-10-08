# item_perception_yolo

The teaching window title is **Item Teach**, without a version suffix.

`item_perception_yolo` is the project perception package. Its aligned local-only
teaching GUIs establish the robot pick-area origin and then the four-corner bin
ROI used by later item detection.

Strict automatic calibration catalog scans share a bounded cache of parsed camera
prefixes keyed by exact YAML bytes. Every scan still reads each file and checks
its filename/symlink/timestamp; every selected artifact still undergoes its full
validation and hash checks. Same-size/mtime edits cannot reuse old prefix data.
Only prefix strings are reused, never camera frames, poses, depth or model results.
This reduces repeated controller source-validation cost without changing explicit
Item Teach/Detect calibration selection or the fresh-request contract.

## Installed nodes

| Node | Purpose |
| --- | --- |
| `platform_teach` | Detect one ChArUco board pose, calculate `base_link <- platform_reference`, preview it in TF, and save a strict platform calibration YAML. |
| `bin_teach` | Detect 5x5 ArUco IDs 0–3, select their four outside corners in `platform_reference`, and save a strict four-point bin ROI YAML. |
| `item_teach` | Item profile editor, class-checkbox selection, RGB/depth YOLO preview and explicitly armed pose service; no robot commands. |
| `item_detect` | Headless version of the same calibrated, request-driven pose generator; no robot commands. |

The retired training-oriented `item_teach_yolo` and separate
`item_detect_yolo_debug` sources/launchers are removed. The remaining imported
detector prototype is reference-only and is not installed as a runtime node or
launch file. Canonical teach names have no `_yolo` suffix; retired names are
not compatibility aliases.

## Item Teach — detection and calibrated pose requests

```bash
ros2 launch item_perception_yolo item_teach.launch.py
```

1. Browse for a local pretrained `.pt` anywhere on the PC; it loads automatically
   and reads classes without another Load button or model confirmation. Available
   model paths restored from an Item Teach profile also load automatically, with
   the saved pair's hashes checked. Selecting/restoring that path authorizes local
   model loading. One package-private CPU worker loads the model,
   reports its actual task and lists **one checkbox per class ID/name**. Check
   only production classes to accept; no implicit production all-classes selection. A new model starts
   with unchecked classes. A saved profile restores selected IDs as unverified
   prefill until the automatic model load verifies them.
   Model loading takes priority over the automatic bin-border/inference preview:
   the current operation finishes, then the selected load runs once in the
   same worker. A status label reports queued/loading progress; no extra clicks
   or YOLO activation are needed. Cancelling Browse retains the previous selection.
   Successful loading automatically starts the 1 Hz YOLO
   preview when ready; Armed stays OFF. Worker failures remain terminal.
   The locked native runtime is copied into the install prefix as ordinary
   files even for a workspace `--symlink-install`. Its `cv2` loader and binary
   must never be symlinks into `build/`; that layout recursively imports the
   Python package and terminates the worker before the first overlay.
2. Select mask or OBB from the model's available geometry outputs. Segmentation
   uses a minimum-area mask rectangle; OBB uses the model's oriented rectangle.
   If both outputs are observed, choose one explicitly. Box-only `detect` models
   can be previewed but cannot be armed as mask/OBB pose generators. There is no
   task conversion or alternate-model fallback.
3. Enter a prefix and **Connect RGB**. The loaded model's 1 Hz preview
   starts when ready; **YOLO Detect** stops/resumes it. There is one view;
   the Detect All/Filtered controls, dropdown and Resume Live button are removed.
   Camera panes stay passive; background diagnostics still evaluate every model class. Home and a saved item
   profile are not needed for preview. Visible confidence, IoU and detection cap
   apply immediately after a typing pause. First-run fields start at 0.25,
   0.70 and 100; new profiles use internal size 448, loaded ones retain their size.
   CPU inference uses square 448 × 448 input for new profiles, independently of
   camera resolution. Existing saved/deployed profiles must also have
   `yolo.image_size: 448` to use that size; loading preserves their explicit value.
   Reload Teach and restart headless Detect/controller after editing deployed
   profiles so their profile hashes match. Weights, camera resolution and original
   RGB/depth pose sampling are unchanged.
   Displaying all means detections under those settings, not every raw proposal.
   No platform/bin/depth, home or saved item file is required to see detections.
   OFF stops inference and shows raw RGB; it also removes the pose service.
4. Use **Browse…** beside **Platform calibration**, **Bin camera calibration**
   and **Robot camera calibration** to choose files directly inside root
   `calibration/`. Changing the platform preserves the independent active camera choice.
   Browsing automatically validates the complete set and atomically saves only
   their basenames to root `.env` as `ITEM_TEACH_PLATFORM_CALIBRATION`,
   `ITEM_TEACH_BIN_CAMERA_CALIBRATION` and `ITEM_TEACH_ROBOT_CAMERA_CALIBRATION`.
   Every key is required; all empty is first run, otherwise all must be set.
   Descriptive camera filenames such as `camera_to_hand_calibration_station_2.yaml`
   work. Existing mode prefixes and strict schemas still apply, but Item Teach
   does not scan for newer files or reject unrelated catalog entries.
   The platform must match the current robot identity. Its original camera is
   historical teaching evidence, not a live filename/hash/mounting dependency.
   Select a new bin-camera calibration after camera movement; retain the taught
   platform and bin when the robot base, platform and bin stayed fixed. The robot-camera file
   must be strict schema 7, `camera_on_hand`, and exactly
   `Link6 <- robot_camera_link`. It supplies the rigid camera-body placement for
   pick clearance; no robot-camera RGB/depth/CameraInfo/TF subscription is added.
   Invalid choices leave `.env` unchanged and show the reason. Canceling a file
   dialog changes nothing. Missing or changed files never select a replacement.
   Selecting the portable bin ROI also validates and saves the chosen calibration
   set, then connects the camera preview; no additional Connect RGB click is
   needed. The prefix comes from the independently selected active camera calibration.
   Saved `.env` choices and the saved bin reconnect this read-only preview on
   startup. This narrow restoration exception never launches a camera, loads
   weights, enables YOLO or arms the service. Platform supports schema 3/4; camera schema 7,
   bin schema 3 and package UI-state schema 6 remain unchanged. Incomplete or
   invalid files leave the ROI hidden with a status reason. Requests recheck the
   exact selected files and hashes, regardless of other newer calibrations.
   Selection changes stop YOLO, disarm and clear old overlays/TF. Use **Load
   Calibration** after correcting a file, or reselect the bin, to revalidate.
5. Use **Simulate Trigger** with a complete saved profile to inspect the exact
   returned RGB/depth result for five seconds. Real controller requests served by
   Armed Item Teach use the same captured image buffers and hold. Both pane
   updates are complete before display, including size/depth/nearby overlays.
   New results replace older captures and start a new five-second hold; click
   inside RGB to resume passive video sooner. Passive images contain no overlays
   and are never hit-tested against an older inference result. Detailed rejection
   evidence remains in tooltips, the activity log and request diagnostics.
   Size assumes each item is parallel to the floor and projects its rectangle
   onto the plane through the measured center. Registered depth must pass range,
   mask, MAD and coverage checks; no floor-plane size is substituted. Missing
   geometry/depth blocks poses with an explicit reason. No form fields are
   overwritten. The separate Bin Teach ROI editor is unchanged.

6. Enter physical item length in `height`, short side in `width`, and choose the
   ±millimetre `tolerance`. A green item rectangle means size within tolerance;
   red means outside tolerance, gray means size not checked (missing measurements
   or dimensions). Green alone does not mean a valid 3D pose. Detections remain
   in background diagnostics in all three cases. Complete and save the settings,
   then trigger a fresh capture to inspect the accepted poses and any rejections.
   The request applies strict class, size, ROI overlap, center-in-item and MAD checks. Accepted depth points are
   black, rejected points red inside the sampling circle. Blank/invalid required
   fields or failed checks show a reason and publish no selected pose.
   Confidence/IoU are 0–1 (e.g. 0.40/0.35, not 40/35);
   the editable `image_size` field has been removed. New profiles use 448 internally;
   loading a profile retains its exact validated saved inference size.
   Confidence/IoU/cap, class, size, quality and geometry-output edits update an already
   enabled preview after 300 ms without further typing, without an OFF/ON click.
   Pending or invalid values pause inference and show a nonmodal reason, never
   reuse old settings. Correcting them resumes preview automatically. Incomplete
   size fields explicitly disable size checking (gray), never supply guessed values.
   Edits clear old/frozen detections and discard in-flight/queued old-setting
   replies; they always disarm and invalidate saved-profile eligibility. Preview
   uses all classes; triggered poses require checked classes.
   Other form edits clear the selection/TF and disarm without stopping detection.

   Section **4  Item size / pick depth — mm** also contains **Nearby depth radius
   filter (mm)** (default **150**) and **Maximum nearby floor-height difference (mm)**
   (default **60**). Save them as `geometry.nearby_depth_radius_mm` and
   `geometry.nearby_depth_height_mm`; both must be finite and greater than zero.
   A candidate fails if even one usable depth point is within/on that horizontal
   radius and at least that high above the detected item surface. Exclude
   `motion.standoff_height` from this obstacle reference; it only compensates robot
   motion. Radius uses camera optical XY. Height is floor-relative along camera Z,
   evaluating the platform Z=0 plane beneath each measured point. Include the outer
   bin border and inset margin, but exclude points outside the outer bin.
   **Depth frames (1, 3 or 5)** defaults to **3** (`geometry.depth_frame_count`).
   The effective item minimum is max(500 mm, saved minimum); tray limits are unchanged.
   Rank geometrically eligible poses first, then check
   nearby height one candidate at a time. Skip each blocked candidate and stop
   when `pose_candidates` (or the request's smaller count) have passed. Stop with
   SHORTAGE/NO_VALID_ITEMS if exhausted. Leave remaining poses unchecked.
   Background teaching diagnostics, Simulate Trigger and real acquisition share
   the ranked batch logic.
   Save/load the exact complete
   item profile, then **Armed ON**
   to advertise `/item_detect/get_item_poses`. OFF removes the service. GUI and
   headless detector must not advertise it simultaneously. No robot motion is
   performed by either mode.

If Armed unexpectedly switches OFF, inspect the Armed tooltip/activity log and
`logs/item_perception_yolo/events.jsonl`. Each `item_disarmed` event records its
reason: a named teaching-field edit, explicit toggle/Save/load, source-validation
error, CameraInfo change, worker failure or shutdown. CameraInfo changes include
the changed fields and old/new values. The original reason survives subsequent
GUI status refreshes. Ordinary image timestamps, temporary image freshness/sync
gaps and production preview suspension do not themselves disarm the service.
Correct the reported cause and explicitly re-arm; there is no automatic re-arm.

### Schema 13 capture and request scheduling

Production and Simulate Trigger select 1, 3 or 5 distinct advancing depth frames
captured after the request. RGB is selected near the depth-window midpoint; every
frame must satisfy the existing freshness, RGB synchronization and CameraInfo
checks. Bounded histories are invalidated on camera/source changes. For on-hand
cameras every pair of RGB/depth-time transforms must remain within 0.05 mm / 0.05°;
a moving bundle is reacquired within the same request deadline.

Every pose request uses RGB and all contributing raw depth frames with timestamps
strictly newer than that request. Equal-boundary and previous-request frames are
rejected; an immediate controller retry waits for new publications within the
existing deadline. Zero controller settling delay cannot reuse the earlier capture.

The shared acquisition layer masks readings outside `[max(500, depth_min_mm),
depth_max_mm]` before aggregation. A per-pixel median needs a strict majority
(two of three); missing support becomes NaN. Immutable float32 millimetres preserve
fractional values. Pose sampling, clearance, clicked inspection, teaching geometry
and debug images use that exact median; local spatial median/MAD remains in place.
The response depth timestamp is the newest contributing frame. Diagnostics record
all contributing timestamps, capture span and stage timings. Shared depth settings
and tray capture/processing keep their original behavior.

The existing `inference_ms` is aggregate native processing, including YOLO,
geometry, clearance and optional rendering. `timings_ms` separates those stages,
source validation, preview wait, capture (including median), median preparation,
planning validation, native round-trip/transport and total detector duration.
Item Teach adds `capture_voxel_ms` for the request's cloud-only worker processing.
Measured item surfaces below the calibrated platform-Z=0 bin floor are rejected
before candidate ranking/count limits and before gripper standoff. The shared
guard compares base Z with the floor at the measured base X/Y, supporting tilted
planes and either normal direction; 0.001 mm is the sole arithmetic tolerance.
Reject without clamping, record source index plus measured/plane Z and deficit,
and continue searching for eligible candidates. Invalid reference geometry is an
error even for an empty batch. Item Teach, Simulate Trigger and headless detection
share this path; the controller independently checks received surfaces. Existing
nearby-height, size, camera-clearance and fresh-frame gates remain active.

Controller `candidate_timing` records validation, service wait and total latency.
Native transport is the round-trip residual outside measured native processing;
it also includes dispatch/scheduling and worker preparation.
Overlapping/nested stage durations must not be summed as independent costs.

Debug-disabled headless production skips annotated-image generation and transfer.
Teaching requests, simulation and requested debug capture render completed geometry
once. Camera-only
rays/mappings are cached by CameraInfo/dimensions, while per-capture original scene
geometry remains separate from display voxels. Parsed artifact caches use verified
file bytes; files and model hashes are still read at validation boundaries. File
size/mtime alone never authorizes reuse of changed artifacts.

Item Teach subscribes read-only to `/robot_controller/status`. Fresh active Pick,
Place or Auto Run suspends background YOLO/voxel refresh. Every successful real
or simulated pose capture, including `NO_VALID_ITEMS`, instead refreshes
`/item_teach/voxel_cloud` once using that request's exact RGB, temporal-median
depth and calibrated transform. The already-owned native worker calculates only
cloud geometry, with no second YOLO prediction, new frames or candidate filtering.
Cloud processing stays within the request's existing deadline and source checks;
publication follows final request validation and is independent of debug saving.
Each capture bypasses the idle 1 Hz publication limit and retains its original
timestamps. Request clouds add no candidate TFs/markers; existing simulated TFs
remain separate. Diagnostics identify `source=pose_capture` or `background_preview`.
Idle restores the live 1 Hz background stream. Already-running background work
finishes but cannot publish during active production or overwrite a newer capture.
The previous cloud remains visible and greys after five seconds without replacement;
showing/dismissing/expiring the image capture does not clear it. Missing/stale status
uses existing request-priority scheduling; headless Item Detect still has no voxel
publisher. No extra executor, hardware client, worker kill or tray behavior change.

Older schemas open only as unarmed recovery drafts. Review changed floor-relative
measurement, radius/height, proposed three-frame median and effective minimum, then
explicitly Save the schema-13 pair. Manually deploy matching YAML/.pt to the strict
flat `runtime_teach/` catalog. Rebuild source packages while applications are stopped,
then manually restart Item Teach/Item Detect and controller consumers and reload
matching profiles. Do not run GUI/headless pose providers simultaneously. Code
installation never rewrites saved profiles, calibration, models or runtime deployment.
ROS service/message layouts and YOLO's 448 × 448 default are unchanged.

Manual rebuild, after stopping the perception/controller applications:

```bash
source scripts/source_ros_workspace.bash
colcon build --symlink-install --packages-select \
  camera_calibration item_perception_yolo tray_perception robot_controller
source install/setup.bash
```

Then open the old Item Teach file in the updated GUI, review recovery fields,
floor/radius/height and frame count, and explicitly Save. For headless use, copy
the saved same-stem YAML/.pt pair into `runtime_teach/`, preserving its one-item-pair
catalog rule and existing Bin/Tray artifacts. Finish deployment before restarting
headless Item Detect/controller. Reload the matching profile in controller GUI
mode. Calibration/model selection remains explicit; this change performs none of
these operator actions automatically.

### Acquisition benchmark (2026-10-06)

The reproducible `scripts/benchmark_item_acquisition.py` uses the same synthetic
640 × 480, 700 mm depth scene, six fixed detections and three returned poses.
Compare baseline `02e139d` with this change using the pinned CPU runtime, one
OpenCV thread, three warmups and 30 measured runs. The benchmark includes CPU
median preparation in the new acquisition time; it excludes YOLO, artifact
validation, ROS/native transport and real camera waiting. All returned positions
and rankings matched. These are synthetic processing measurements, not a robot
cycle-time or full service-latency claim.

| Processing from supplied captures | Median ms | p95 ms |
| --- | ---: | ---: |
| Baseline, single frame, mandatory images | 127.8 | 130.2 |
| Schema 13, three-frame median, debug off | 100.1 | 102.7 |
| Schema 13, three-frame median, debug on | 123.3 | 127.2 |

With debug off, median stage durations were median preparation 24.4 ms,
ordinary geometry 29.3 ms and clearance 46.0 ms; image rendering was below
0.01 ms. Debug rendering took 23.2 ms median. Nested timings are not additive
percentile estimates. The strict three-frame window spans another 66.7 ms at
30 FPS, plus the first-frame arrival phase; that deliberate acquisition wait
can outweigh the measured CPU savings. Use request `timings_ms` and controller
`candidate_timing` to measure actual deployment latency before drawing a speed
conclusion. No recorded capture archive or live hardware benchmark was used.

After sourcing the isolated build, reproduce current measurements with:

```bash
OPENBLAS_NUM_THREADS=1 python3 scripts/benchmark_item_acquisition.py \
  --runtime install/nearby_depth_check/item_perception_yolo/lib/item_perception_yolo/yolo_runtime
```

Add `--render` for debug rendering. `--python-root` accepts an exported older
`item_perception_yolo/python` tree for the identical baseline fixture; exporting
source must not replace the active installation or operator files.

### Default 1 Hz RViz preview

After an automatic model load, Item Teach enables YOLO preview when its
camera/settings are ready. Available model selections restored at startup also
load, while arming remains explicit. At most once per second, one worker job runs YOLO,
then reuses that exact RGB/depth/TF observation for a calibrated colored 10 mm
voxel cloud and the validated candidate batch. Slow processing lowers the rate; jobs
never accumulate or catch up. There is no second YOLO prediction, additional
executor or automatic image saving.

| Output | Topic or frame | Contents |
| --- | --- | --- |
| `PointCloud2` | `/item_teach/voxel_cloud` | Valid depth view including bin surroundings, 10 mm centroids with averaged original RGB, `base_link`, source stamp |
| `MarkerArray` | `/item_teach/valid_items` | All valid pose XYZ axes, without text/number overlays |
| TF | `base_link -> item_teach_live_candidate_N` | Frame-local ranked item poses; 1 Hz, not tracked identities |
| `String` JSON | `/item_teach/rviz_diagnostics` | Source timestamps/age, voxel count, checked candidates, rejection reasons and unchecked source IDs |

The canonical `dobot_rviz` configuration enables the cloud and marker displays.
Cloud transport is reliable, transient-local, depth one; markers/diagnostics
are reliable, volatile, depth one. Colors use both registered-depth and RGB
distortion models. Complete platform/camera transforms preserve station tilt
and height. Only visualization
points are voxelized: pose depth sampling stays full resolution and retains
class, dimensions, green/light-blue borders, MAD/quality and robot-camera
clearance checks. Nearby scans stop once `pose_candidates` valid poses are found;
only these fully checked poses receive TF/markers. Remaining geometric candidates
appear as `unchecked` source IDs in diagnostics and a count in the UI, never as
validated poses or rejections. All-class RGB size annotations remain visible.
Selected classes, a valid `pose_candidates` count,
dimensions, recorded Home, pick rotation and standoff are required for valid
poses. With incomplete pose settings or YOLO OFF, the calibrated cloud can still
run and the view explains which pose prerequisites are missing.

These are snapshots, also retained while the teaching view is frozen. Item rank,
class, confidence and snapshot age are not overlaid as RViz text; candidate
details and age remain available in diagnostics and the teaching view.
The marker displays show unchanged red X/green Y and a blue **UP** guide: the
reference-plane normal on the side with positive `base_link` Z. An exactly
horizontal normal remains horizontal. This blue arrow is a visual guide, not
necessarily the pose's +Z. Actual pose quaternions, TF, service responses and pick
planning remain unchanged. Clicked and simulated poses use the same convention
on `/item_teach/selected_pose_guides`; live poses retain `/item_teach/valid_items`.
All pose-guide arrows are 200 mm long with 20 mm shafts and 40 mm heads (diameter
and length), matching the robot-joint TF axis length/width at Marker Scale 1.
The selected guides follow their existing frozen-preview lifetime and clear with
the selection. Every marker expires after 2.5 seconds without publication.
Canonical RViz enables both guide displays and sets **TF → Show Axes = false**
to hide overlapping raw axes, including robot-frame axes. The real TF data and
RobotModel remain unchanged; enable Show Axes explicitly for frame inspection.
Reload the configuration or uncheck Show Axes in an existing viewer. No numeric
RViz labels, additional inference or headless publishers are introduced.
RViz retains the latest cloud indefinitely. After five seconds without a new
validated cloud, Item Teach replaces its colors with grey while keeping the
geometry and source timestamps. A fresh snapshot replaces it and restores its
colors; repeated/frozen snapshots cannot reset the refresh age. RGB/depth
loss, synchronization gaps, a busy worker or an empty/unavailable next cloud
never publish an intermediate empty cloud. During these gaps, diagnostics report
`retained_cloud`, then `stale_grey`, with original source timestamps, source and
refresh ages, and the reason. Candidate markers/TF pause during gaps and while
grey until a newly validated snapshot arrives. Changed CameraInfo, source/
settings changes, terminal failure or orderly exit grey the cloud immediately.
RViz uses **Decay Time = 0** and **Size (m) = 0.010**. Its reliable transient-local
depth-one subscription receives the cached cloud even when opened after the
last publication, while Item Teach runs. This cache is in memory, not on disk.
The canonical RViz TF timeout
is 2.5 seconds; other TF consumers may keep historical transforms. No cloud or
pose preview can satisfy a production request. Headless Item Detect publishes
none of these topics, remains request-driven, and writes images only for an
explicit `GetItemPoses.save_debug_images=true` request.

Use the triggered result to inspect returned poses and dimensions. Passive
camera pixels have no selectable detection geometry; old inference cannot be
hit-tested on a new camera frame. Simulated batch TFs retain the captured platform
transform and expire with the five-second image hold. Only bounded current,
in-flight and held observations remain in memory, and only an explicitly checked
debug request writes images to disk.

### Simulate Trigger

The main row is **YOLO Detect ON/OFF | Simulate Trigger | Armed ON/OFF**.
Armed ON is highlighted red so advertised production pose-service state cannot
be mistaken for the unarmed teaching state; the color does not bypass validation.
Simulate Trigger is a one-shot action, available with Armed OFF or ON. It needs
a complete saved/loaded schema-13 profile, its verified model, matching current
settings, YOLO ON and the applied station/bin. Correct and save recovery drafts
first. It neither advertises/calls the pose service nor issues robot commands.

It uses the very same acquisition, inference, class/size/ROI/depth filtering,
center-first ranking and typed response builder as a real service request.
Acquire one new synchronized RGB/depth pair after the click, resolve RGB-time TF,
and enforce the same input freshness, request deadline and hash/generation
checks. One action queues behind the current GUI job, with progress shown on the
button; no repeated clicks or automatic retry. The deadline includes queue time.
Production/simulated requests are mutually exclusive and report BUSY on overlap.

The successful pair freezes on both views for **5 seconds after display**, then
returns to live automatically. Queue/inference time does not consume this hold;
the headings show a countdown. Click RGB to resume sooner. Each replacement
capture starts a new hold; the timer also applies to empty results.
The frozen images give pick annotations to **only returned candidates**,
ranked P1…Pn and capped by `retry.pose_candidates`. Retain their mask shading,
one green rectangle, X/Y axes, center dot, cyan metric sampling rings, black/red
accepted/rejected depth pixels, the green loaded bin ROI and any configured
light-blue pick clearance. Nearby-blocked candidates retain their diagnostic
circles/points/labels; unchecked later candidates have no pick or nearby overlays.
Null depth is excluded before MAD; it
never contributes to the pose. Top bands report counts, source age and the first
three poses in platform_reference (XYZ, yaw, size and depth counts); all returned
poses have on-image priority labels and full details in the bounded Activity log.
Every returned pose also appears in the teaching-only RViz TF preview at 10 Hz:
`base_link -> item_teach_candidate_1` through `item_teach_candidate_N`, matching
P1…Pn priority order on the images (not limited to the three expanded text lines).
The same transform composition as clicking an item preserves the selected
platform's full rotation/translation. No extra `platform_reference` authority,
robot-TCP compensation, live tracking, motion or service change is introduced.
Use an already-running RViz TF display; Item Teach never launches RViz.

These labels retain Item Detect's center-first priorities. The controller
independently orders the returned poses by 3D distance from taught Home, so its
attempt/Preview numbers may differ. Controller logs preserve each pose ID and
both priorities; Item Detect's filtering and returned count remain unchanged.

The complete batch replaces any previous clicked-item or simulated preview.
Validate the response frame, priorities, IDs, pose values, source/profile identity
and snapshot identity before installing all TFs atomically. While frozen, only their
broadcast timestamps refresh; their positions/orientations do not follow newer
images or robot TF. The timer independently checks source/profile, arming epoch,
YOLO and native/fatal state so invalidation stops publication even if Qt is busy.
It also stops simulated TF/pose-guide publication after the same five-second hold
using monotonic time independently of Qt or ROS clock changes. Expiry clears only
the teaching visualization, leaving armed services and real candidate batches valid.
SHORTAGE and NO_VALID_ITEMS are explicit successful outcomes; zero items freezes
the pair/bin ROI plus any measured nearby rejection and publishes no candidate frames.
ROS/RViz can retain old
TF frames in their buffers until timeout/reset after publication stops.

Click RGB to cancel/resume; image margins and status bands do nothing. Settings,
station/profile/model/arming changes, YOLO OFF or failure invalidate pending and
frozen results and stop all teaching TFs. A failed request never displays or
publishes a previous batch as its result.
An armed real service always obtains new observations, even while a capture is
frozen. Its completed result replaces the captured pair for five seconds, labelled
Controller. The same buffers are saved when debug is checked; unchecked requests
still render for the teaching window but create no files. Simulate Trigger never
saves images. One bounded mailbox hands the newest complete pair to Qt; expiry
never disarms, delays a service response or changes controller targets. Real
captures do not install simulated TFs. Headless requests retain their debug-only
rendering and do not send images to a separately running teaching process.

Arming always validates and uses the production profile. Its service acquires a
new observation; it cannot
return teaching-preview detections or a frozen selection. Headless behavior,
strict production item schema 13, class filters and quality gates remain enforced.

### Pick-oriented RGB overlays

Segmentation shows mask shading and exactly one size-colored minimum-area rectangle
derived from the mask. OBB shows its native oriented rectangle instead. Do not
add the axis-aligned YOLO box or a second rectangle/per-box class label. Keep
rectangle geometry for measurement/click selection: show red **X** along its
short direction and green
**Y** along its long direction, spanning opposite edge midpoints through the
same center. A white dot with a black rim marks that exact pick pixel; selecting
it adds a cyan sampling ring, never another box. Its physical diameter is the
current `pickdepth_radius` in millimetres (30 means diameter 30 mm, radius 15 mm),
not a fixed pixel count. The worker projects the same 96-point platform-plane
circle used for depth sampling with the frozen observation's camera transform,
intrinsics and distortion. Perspective can make it appear elliptical. It remains
available with blank size filters or rejected depth/poses if calibrated geometry
exists; missing/invalid projection hides it with a reason, never a guessed ring.
Changing the diameter clears the frozen selection; click again for its new size.
The image and outline scale together when resizing/letterboxing the video.
The teaching view shows all geometric
pick pixels; green/red borders report only the size check, gray is unchecked.
A preview dot is not a validated 3D pick pose. Production service calculations
continue to retain only validated candidates.
Metric dimensions, depth sampling and pose-generation mathematics are unchanged.

With the selected station and bin validated, a green
unfilled border labelled **Loaded Bin ROI** projects the saved bin XY points at
platform Z=0 into the RGB
view when its valid camera inputs arrive. No Apply click or model loading is needed.
It works with **YOLO ON or OFF**, with no detections required. The
YOLO-off path uses pure projection in the same isolated worker; it does not load
model weights, call prediction, or require depth. The projection uses the exact
current station camera/platform/bin evidence, RGB-time TF, color intrinsics and
distortion (32 samples per edge), not the source station's placement or a guessed
rectangle. The 32-samples-per-edge construction and
platform-to-optical conversion are shared with Bin Teach, so its yellow capture
and green loaded borders match Item Teach under identical inputs. Tilt and height
come only from the current destination platform. The portable bin's source robot,
camera and platform are provenance, not its destination placement: no source
files, marker detection, plane flattening, depth-based displacement or auto-sizing
participates in loading. Use the same physical origin/axis directions, bin size
and bin offset at each station. Previously saved XY remains unchanged; select the
newly re-taught bin file explicitly to replace an older ROI selection.

Item Teach compares the selected platform's SHA-256 with
`teaching_provenance.platform_calibration.sha256` in the bin file. Different hashes
with identical source/selected robot identity, reference convention and recorded
base/platform transform (absolute tolerance 1e-9, no relative tolerance) produce
no warning: a metadata-only schema migration preserves geometry. Otherwise a
difference shows a persistent amber **Bin/platform mismatch** warning below the selectors,
including the original and selected filenames; hover for the full hashes.
It also records one bounded warning event per selected/restored binding, not
per video frame. The warning clears when matching files are selected or the
binding is cleared/invalid. It is nonmodal and does not block portable reuse,
change ROI geometry, auto-enable YOLO/Armed, or require the original source file.
Check the physical origin, X/Y directions, bin size and placement before reuse.
A matching checksum confirms the same platform artifact, not physical alignment;
different checksums do not prove the destination setup is wrong. Destination
camera/platform validation remains strict; this notice does not relax hash errors
or make source-station transforms a deployment binding. Artifact schemas remain
unchanged; no existing files are rewritten.

Missing/changed inputs, behind-camera or offscreen geometry have an
explicit `Bin ROI hidden` reason. YOLO-OFF live projections older than 0.5 seconds
are replaced by raw RGB. Completed teaching inference is a **result snapshot**:
retain its mask/rectangle/axes and bin border on its exact source RGB until a new
result arrives, even if CPU inference takes longer than 0.5 seconds. Display the
source-frame age and inference time; at over 0.5 seconds explicitly label
**RESULT SNAPSHOT**, **STALE**, and the ROI as historical, not a live projection.
Never transfer old annotated pixels onto a newer raw image, show a past detection
count over raw RGB, or pass display snapshots into production service requests. Incoming-input
and pose-service freshness limits are unchanged. Frozen selections are also
age-labelled. Station and model selections load independently; neither arms the service.

The ROS/Qt parent never imports cv2/Torch/Ultralytics. Build verifies and extracts
the exact private inference wheels without internet/global installation. Fixed
CPU inference uses Ultralytics 8.4.150, OpenCV 4.10.0 and NumPy 1.26.4, with
four Torch CPU threads, one OpenCV thread and OpenCL disabled. The exact
preprovisioned Torch/torchvision and other dependencies are recorded in
`third_party/wheels/item-yolo-runtime-lock.json`; they are not bundled OS
dependencies. See [NOTICE.md](NOTICE.md) for attribution/offline limitations.
No worker restart, GPU selection, downloads or runtime fallback is permitted.

**Record Current Joints as Home** reads the latest actual `/joint_states` from
the sole configured root-namespace Dobot publisher. Both message timestamp and
local receipt must be no older than one second. It requires exactly six finite
positions, reordered to `joint1` through `joint6`, and saves them in radians
(the GUI displays degrees). No robot is moved or enabled. Source robot IP and
publisher are provenance only: home joints are portable across the user's
identical robots, with no station-identity rejection. Loading a home is not
execution or a claim that the destination travel path is safe.

**Save Item Teach** requires confirmation and updates the loaded same-stem pair
in `offline_teach/item_teach/` when the item name is unchanged. Edits invalidate
saved eligibility but retain that loaded save target. A changed item name, or
a new document without a known original name, creates a new timestamped pair;
later saves target that new pair. Unchanged weights remain untouched; a selected
replacement is copied and SHA-256 verified before publication. Reject external
YAML/model changes since loading instead of overwriting someone else's edits.
Keep one hidden `.<stem>.previous.zip` with the previous YAML and paired weights
(if present), replaced on the next update. The backup is for manual recovery,
not an alternate loader. Staging/backup errors leave the original pair intact;
if weight replacement precedes a YAML write failure, restore the original model.
YAML is the commit marker: interrupted mixed pairs fail strict hash validation,
never silently load. A success dialog names both files; the original external
model remains untouched and the pair works without it.
**Load Item Teach** accepts only that directory. Complete schema-13 files load
normally and immediately count as saved, including startup named-file restoration.
No redundant Save is required before Simulate Trigger or manual Armed, but model
verification, YOLO ON and fresh station inputs remain mandatory. Loading
does not arm, simulate or command anything. Older/partially invalid files open as
labelled GUI-only recovery drafts: keep independently valid fields, blank unclear/missing values,
and show unknown booleans as partial checkboxes requiring an explicit choice.
Unreadable/ambiguous YAML clears all fields; no partial parser guesses. Clear old
`retry_limit` counts are recovered as `pose_candidates` for editing only. Invalid
home records are cleared as a whole, never filled with zero joints. Unverified
paired weights leave the model field empty; browse a trusted model explicitly.
The warning/Activity log explains every cleared field. Missing internal
`image_size` requires explicitly browsing a model to establish new-profile 448.
Recovery also applies to named-file startup prefill; any independently verified
paired model then loads automatically. Missing/changed pairs never execute.
No recovered draft can simulate, arm or be validated in the controller until
reviewed and saved as a strict schema-13 pair. Same known item name overwrites
the loaded file with its previous-version backup; changed/unknown original name
creates a new pair. Loading alone leaves files untouched. Shared
UI-state schema 6 remains strict; no recovered field autosave. Headless and
controller readers stay strict and never call the GUI recovery reader.
Its single confirmation covers replacing the form/home. Loading the YAML queues
that exact model and reads its classes—no second Load Model click. The existing
worker finishes its current preview and gives the load the next slot. Overlapping
loads are disabled. The pair's hash is checked before queuing/loading and after
inspection, along with saved task, class IDs and geometry support. Keep the saved
selection and fields; do not substitute classes, a task or another output.
Failures are visible and never retried. Successful verified loading starts the
1 Hz teaching preview when ready; Armed stays OFF.
Startup prefill and browsing a standalone model both load available selected weights
automatically. Missing files are reported without blocking independent camera work;
they can load once available. Invalid existing files do not trigger repeated loads;
Browse the same corrected file to retry. Already loaded files are not watched/replaced.

The YAML groups `item`, `model`, `units`, `home`, `pick_rotation`, `motion`, `speed`, `acceleration`, `timing`, `gripper`,
`retry`, `yolo`, `geometry`, `geometry_source`, `bin_clearance`, `quality`, and the non-executing
`controller_contract`. See
[`offline_teach/item_teach/README.md`](../../offline_teach/item_teach/README.md)
for field details. `retry.pose_candidates=3` requests up to three ranked poses
for the controller to use for retries; it does not execute any retry itself.
`yolo.max_detections=20` is the separate
per-frame detection cap before geometric filtering.
The two gripper checkboxes are independent. `grip_onpick=true` closes fingers
immediately after confirmed suction pickup, even with `use_grip=false`.
At 50% of the first upward lift, `use_grip=false` relaxes DO2 and DO14 (both OFF);
`use_grip=true, grip_onpick=false` closes them, and both true keeps them closed.
After a valid tray pose/depth, `use_grip=false` reopens before placement motion.
Suction remains on until the shared 80% placement/return descent release.
These are existing boolean fields; loading/saving preserves both independently.
Controller rule 57 now defines vertical Home-attitude height equations and
DI1-monitored final descent; teaching remains non-actuating.
The **Vertical motion — mm** form saves `standoff_height`, `prepick_height`,
`retract_height` and `trayplace_height`. The last field appears below
`pick_rotation` and requires an explicit finite, nonnegative value in millimetres;
zero means the detected surface. Placement drop Z = detected tray surface Z +
`trayplace_height`, independent of pick heights. Robot Controller and Preview
use this same endpoint; approach/retract remain at taught Home Z above it.
New profiles start with this height blank. Schema-9 and older files recover with
it blank too; enter the intended clearance and Save a valid schema-13 pair before
controller use or runtime deployment. Loading never invents or writes a height.
`pick_rotation` is a separate required 0–90° value. It is an unsigned offset
from the detected short-axis line; Robot Controller chooses the lower-travel
clockwise/counter-clockwise equivalent from taught Home independently for every
candidate.
`bin_clearance` contains the optional inward millimetre offsets for directed Bin
Teach edges P1→P2, P2→P3, P3→P4 and P4→P1. Blank/null leaves that edge at the
green ROI. Any configured valid inner polygon is light blue on RGB and depth.
A detection is eligible when its platform-plane footprint overlaps or touches
green and is ignored only when fully disjoint. The exact depth-derived pick XY
must remain inside/on green and additionally inside/on the blue border. The
original RGB pick pixel must also lie inside/on the corresponding projected
allowed-pick polygon, preventing item-height parallax from showing a returned
candidate outside the blue/green overlay.
The shared click, Simulate Trigger, Armed and headless candidate pipeline applies
this before ranking; the controller receives the resulting filtered list.
At the planned pick Link6 pose (`item Z + standoff_height`), the shared planner
uses taught Home and the calibrated Link6-relative camera pose. Model the Gemini
335 housing in `robot_camera_color_optical_frame`: XYZ dimensions 90/25/30 mm
(X right/width, Y down/height, Z forward/depth), center (+11, 0, −12.79) mm.
The nominal mechanical RGB origin is (+14, 0, +2.02) mm in depth optical axes.
Convert optical axes to camera-link axes (X forward, Y left, Z up), then compose
with the unchanged saved `Link6 <- robot_camera_link` calibration. The resulting
body center in camera-link axes is (−10.77, −25, 0) mm; its XYZ extent is
30/90/25 mm. No added margin is included.

Sources: [Orbbec Gemini 330 datasheet V1.6](https://www.orbbec.com/wp-content/uploads/2025/06/Gemini-330-series-Datasheet-V1.6.pdf),
§3.2.3 (size), §§4.8–4.9 (origins, RGB 2.21 mm behind glass), Appendix B
(RGB 11 mm from body center); verified against the bundled Gemini 335 URDF/mesh.
The optical-to-link bridge is a fixed nominal **housing model**, never a
replacement for factory TF used to measure pixels or solve calibration.
RGB-aligned depth keeps its RGB optical frame; alignment does not move the
physical camera-link origin to the RGB lens. No new live TF dependency is added.

Transform all eight corners, including camera/platform tilt, and project their
convex outline onto platform XY. If the normal shortest attitude's entire body
does not fit inside/on green, check the equivalent 180° tool-Z mirror; if neither fits, the item
is rejected before ranking/capping and the next safe pose can take its place.
Green is the robot-camera-body constraint, independent of the blue pick-point
inset. A magenta `CAM` or `CAM 180` footprint shows the selected projected
housing outline on both bin RGB and registered depth, with camera-clearance
rejection reasons in result diagnostics. Native result validation requires the
same RGB reference frame, dimensions, center offset and normal/mirrored/selected
outlines, rejecting old centered-body evidence. The same pure planner runs in headless detection and both
controller preview and hardware planning. Restart all these processes together
after updating. Teach-file schemas/settings and pose-service interfaces are unchanged.
This models the housing at the pick pose, without mount/cable geometry or swept-path checks.
Item Teach launch starts the separate read-only `item_perception_yolo/robot_camera_box`
node, displaying the same offset housing on `/item_teach/robot_camera_body` in
RViz. Item Teach publishes its validated selected camera mount at 1 Hz on
`/item_teach/robot_camera_mount` (`geometry_msgs/PoseArray`, frame `Link6`): one
pose is `Link6 <- robot_camera_link`, an empty array clears the display.
Selection edits, failed validation and closing Item Teach clear the mount;
each heartbeat rechecks the selected camera file's hash without scanning for a
newer file. A changed file stays invalid until explicitly reloaded. The display
rejects malformed, repeated, future or older-than-2.5-second mounting messages
and expires when updates stop. Both topics use reliable transient-local depth-one
transport. The marker remains frame-locked to live Link6 with a three-second
lifetime if its own process stops. Item Teach closing shuts down its launched
display process. No trusted model, bin selection, inference or camera stream is
needed to show a validated mounting pose. Controller/headless launches do not
start this node; shared clearance checks never depend on visualization.
Pick Z=item Z+standoff, pre-pick Z=pick Z+prepick and clearance Z=pre-pick Z+retract,
in robot base Z. zheight_offset is removed. GUI-only old-file recovery leaves
old retract_height blank because its reference changed; correction/Save is
required. Existing calibration/bin/shared UI schemas are unchanged.

The scrollable routine settings include three editable speed and acceleration
percentages: travel/Home, final approach, pick-to-prepick retract. All must be
integers 1–100. New-profile speed is explicitly 100/6/6 and acceleration
100/100/100; loaded profiles retain their exact values. Speed and acceleration
edits disarm and invalidate saved eligibility without interrupting read-only
inference or automatically saving/commanding hardware. Save writes schema 13 with
percentage units and separate groups, both using `travel_percent`,
`approach_percent`, `retract_percent`. Controller supplies each motion's `v=`/`a=`;
global SpeedFactor starts at 100% and the controller can adjust it explicitly
while Live/idle, without rewriting these taught per-command rates. Production
rejects schemas 1–10; old GUI recovery drafts leave missing/invalid rates,
`pick_rotation`, `trayplace_height` and the four newer bin-clearance fields blank
until the operator explicitly reviews and saves them. Shared schema-6 named-file
UI state is unchanged.

Item Teach has no controller-validation button or controller client. It creates
profiles, inspects detections and exposes read-only poses when explicitly armed;
it never selects a controller profile or sends a controller request. Configure
`robot_controller` separately in its GUI/explicit launch parameters or headlessly
from `runtime_teach/`. Headless Item Detect uses that same deployment catalog and
the same current detection implementation as Item Teach. Teaching never requests
motion.

Item Save/Load records the selected artifact filename in shared strict schema-6
UI state, alongside explicit preview prefix and applied station/bin filenames.
Restart reads validated fields, treats a complete valid item profile as saved,
and automatically loads its verified paired model plus the selected calibrations.
It never sends a controller profile, replays joints, arms a service, restores an
external model path or keeps duplicate profile settings. Read-only station/RGB
preview restoration follows the validated automatic bin-ROI workflow above.
Unsaved text-box edits are **not** autosaved. Only fields in the selected saved
teach YAML return on restart. Live/frozen images, clicked measurements, stage,
and arming state are never restored or written as image files. Model loading is
fresh verification of the selected file, not restoration of an old worker result.

### Pose generation contract

The Item Teach window uses one scrollable settings column, ordered camera/station,
model, live YOLO, dimensions/depth, quality limits, then saved routine fields.
RGB and native registered-depth views are side by side in a horizontal draggable
splitter. Both carry mask shading, green/red/gray size borders, centered axes/dots,
the loaded green bin ROI, and any configured light-blue pick clearance. Depth
geometry is projected through its own CameraInfo;
straight RGB edges are sampled before projection to handle differing distortion.
Both panes normally show raw camera video, including unannotated registered
depth colored over 200–1000 mm (black outside that display range). This display
range is independent of pose acceptance. Passive drawing uses NumPy and the
native Turbo palette without importing OpenCV into ROS/Qt or consuming the worker.
The background 1 Hz diagnostic/voxel pipeline remains independent of these pixels.
A completed Simulate Trigger or real request served by this teaching node shows
the exact annotated RGB/depth pair for five seconds after GUI acceptance. All
native overlays finish before the pair becomes visible; newer results replace
both panes together. RGB clicks resume passive viewing, including during controller
operation. No old background detection is selectable on a newer passive image.
Item Teach and Tray Teach share fixed three-line bands: source/result/pose count,
age/processing/countdown, then a rejection summary or the first rejection reason
for an empty result. Full pose, batch, settings and rejection details stay in
band tooltips and diagnostics. Only geometric overlays and short labels remain
on the camera pixels, matching optional debug saves.
The redundant above-video help/settings text is removed. Missing plane calibration
does not hide pixel-space depth overlays, but still blocks metric poses/circles/ROI.
Load/Save stay visible in the header; Activity log expands the bounded
read-only log. No controls or saved variables are removed, and layout does not
enable inference, arming, motion or autosaving.
Platform and Bin Teach share a compact single-column setup, large RGB area,
persistent Save/capture controls and expandable calibration/status details.
Essential physical setup guidance remains visible; full guidance/output paths
remain in Details. Their explicit Apply, capture, load, retake and save rules
and teaching TF behavior are unchanged. Invalid capture reasons remain visible
on the video or waiting view and in the compact status line.

The controller sends
`GetItemPoses(max_candidates, profile_sha256, save_debug_images, pose_convention)`.
The required convention is `item_short_x_long_y_v1`. One request
at a time is accepted; a concurrent request returns BUSY. Every request acquires
a new RGB/depth pair after its arrival, not a cached prior result. Source frames
must be tightly packed `rgb8` and registered little-endian `16UC1` millimetres,
with matching dimensions, optical frame and color/depth CameraInfo intrinsic K.
Distortion coefficients are independently validated, not required to be equal:
[Orbbec documents rectified D2C depth with distorted raw RGB for Gemini 335](https://github.com/orbbec/OrbbecSDK_v2/blob/main/docs/tutorial/orbbec_camera_distortion.md).
Keep both CameraInfo models in the observation. The worker projects the same
metric sampling circle separately into RGB and depth. Native depth-pixel rays
determine circle membership on platform Z=0 and are projected into RGB to check
mask membership. Each original depth pixel is counted once; accepted/rejected
points and the circle are drawn in native depth coordinates. Median accepted
depth still back-projects the unchanged RGB center through the RGB model.
Missing metadata, different K/frame/dimensions and stale/unsynchronized data
still block poses. No resizing, depth interpolation, silent registration,
unit guessing, replacement distortion or old-frame substitution. When the
explicit debug flag is true, the detector atomically saves this request's exact
already-rendered RGB and registered-depth result pair as PNG under root
`debug/pick_img/` and returns the absolute paths in diagnostics. It never creates
a second camera subscription or continuous archive. A persistence error is a
reported warning and does not change an otherwise valid candidate response.
An on-hand camera additionally uses live `base_link <- Link6` at the RGB
timestamp. Once a valid pair is selected, keep that pair while waiting for its
TF; never chase newer camera frames. The pair must remain within the taught
input-age limit. Arming checks latest TF availability only, not a generated
target; every actual target uses the observation's timestamp. Three executor
threads keep TF/sensor reception independent of a
blocking service request. All native operations are serialized in one worker.

- Confidence and checked classes filter YOLO detections. IoU controls NMS;
  inference explicitly requests NMS rather than silently ignoring IoU for an
  end-to-end model.
- Keep a detection when its selected footprint projected onto platform Z=0
  overlaps or touches the green bin ROI; ignore it only when the polygons are
  fully disjoint. This floor-plane ROI test is independent of size measurement.
- Require the final depth-derived pick XY inside/on the green bin ROI.
- Require the final depth-derived pick XY inside/on the optional light-blue
  `bin_clearance` polygon. Do not apply this inner polygon to the footprint.
- Independently require the unchanged RGB pick pixel inside/on that same
  allowed-pick polygon as projected at platform Z=0 (light blue when configured,
  otherwise green). This conservative visual gate handles item-height parallax;
  never relocate the pixel to its vertical projection merely to make it pass.
- The original pixel rectangle center is the pick ray; never relocate it.
  A mask whose rectangle center lies outside its own polygon is rejected.
- `pickdepth_radius` is deliberately the sampling **diameter**, initially 30 mm.
  Define the circle on the fixed reference plane centered at that ray-plane
  intersection, then project it to the registered image. Perspective can make
  the displayed boundary non-circular. Process only its bounded pixel window,
  clipping eligibility to the item mask/OBB. A clipped image-edge circle is
  rejected instead of silently shrinking the sampling area.
- Remove invalid/non-positive/out-of-range depths, calculate median and MAD,
  and retain `abs(depth - median) <= 3 * 1.4826 * MAD`. When MAD=0 only readings
  equal to the median pass. Minimum valid-pixel percentage still applies. Median
  retained camera depth back-projects the exact center pixel, then the complete
  3D point is transformed into `platform_reference`. Never append optical depth
  to platform-plane XY.
- Use that same measured center's signed platform Z for the size plane, parallel
  to platform Z=0. Assume flat items; do not fit individual surface tilts or use
  gripper standoff/base-Z/camera-Z height as the plane offset. Undistort the selected
  rectangle's RGB corners, intersect their rays with this plane and fit its
  minimum-area metric enclosing rectangle. Long side is Y/`height`, short side
  X/`width`; check both against taught physical millimetres plus/minus `tolerance`
  only after valid depth. In parallel-plane geometry this corrects the floor size
  by `(camera_platform_z - item_platform_z) / camera_platform_z`, with signed
  coordinates. Reject non-finite/behind-camera projections; never fall back to Z=0.
  Mask/OBB preview, clicked inspection, RViz candidates, Simulate Trigger and
  headless detection use the same depth sampling and measurement functions.
  The bin border, inset, sampling circle and nearby-obstacle reference stay at
  their existing definitions. No additional YOLO pass, schema or setting is added.
  Restart Item Teach/Item Detect and explicitly re-arm after this Python update;
  review saved size/tolerance values against physical items. Operator artifacts
  are never rewritten automatically.
- RGB shows the loaded ROI, mask shading (mask source only), a single mask-derived
  rectangle or native OBB, centered pick axes/dots, size, rejection reasons and
  priority labels. There is no extra axis-aligned YOLO box. In Filtered mode,
  only valid pick candidates receive the rectangle and pick axes/dot.
  Depth shows **accepted pixels black**, **rejected pixels red**, sampling
  boundaries, counts, filtered camera depth, platform Z and frame age.
- Clearance uses the candidate's measured optical surface position before standoff.
  Express platform Z=0 as camera plane `nx X + ny Y + nz Z + d = 0` and evaluate
  `floor_depth(X,Y) = -(nx X + ny Y + d)/nz` at each point's own physical XY.
  Height is `floor_depth - measured_Z`, parallel to camera Z, including floor slope.
  Back-project each original median-depth pixel with registered-depth CameraInfo
  and its own depth. Transform its physical point into platform XY for inclusive
  outer-bin membership. Include depth outside masks and between the outer border
  and inset pick boundary; ignore outside-bin depth. Candidate containment and
  camera-body checks remain unchanged. No percentile, clustering or voxel reduction.
  Inside/on the saved camera-XY radius, reject if maximum height minus candidate
  height is at/above the saved height threshold; one qualifying point is sufficient.
  Invalid calibration, degenerate floor, missing boundaries, invalid candidate
  depth or no usable neighborhood rejects. Build scene geometry once per capture,
  then scan ranked candidates until the requested batch is full. Never retain
  another candidate's result or a previous capture's eligibility.
  Native evidence is explicitly `platform_floor_camera_z_v1`, carrying candidate
  height, maximum nearby height, their difference and point counts. Blocked checks
  retain this evidence in rejection diagnostics. This replaces base-Z measurement
  and outside-bin obstacle inclusion. It does not model the complete swept path.
- Nearby-check overlays use the exact checked scene in RGB and registered depth:
  solid yellow is the camera-XY radius on the candidate floor-relative
  height surface; dashed orange is the rejection-height surface. Short orange lines
  connect the planes. These are metric 3D projections, not fixed pixel circles;
  RGB and depth use their own distortion models. Red points are the offending
  usable depth samples; a red/white X marks the highest blocker.
  Small labels identify only the detection source index, colored red when blocked
  and yellow otherwise. Full floor-relative heights and rejection reasons remain
  in diagnostics. These colors describe only this check, not overall pick eligibility.
  Candidates rejected before depth/geometry validation have no nearby result.
  The cyan sampling circle and its black/red MAD samples retain their meaning.
  Passive camera panes have no overlays. Simulate Trigger, real GUI-served
  requests and their optional debug PNG pairs share the same captured diagnostics. Capped production
  images retain blocked nearby checks, including empty batches, while
  unchecked later candidates receive no annotations of their own. The worker
  explicitly reports their source IDs in `unchecked`; they are neither valid
  poses nor rejected candidates. `valid_count` is the checked returned count.
  The native worker reuses its existing 1 Hz teaching pose calculation; there is
  no additional inference, cloud resampling, pose request, thread or hardware
  command. Invalidated worker results cannot update the displayed images. Restart
  Item Teach/Item Detect after updating the native/parent visualization protocol.
- Rank valid positions by XY distance to the bin polygon's area centroid;
  confidence descending then source index break exact distance ties. Return up
  to the requested number, explicitly reporting SHORTAGE or NO_VALID_ITEMS.
  The right-handed frame has short X, long Y and platform-normal Z with a
  deterministic sign; it is not estimated surface tilt or a TCP command.
- Replies carry metre poses/dimensions, timestamps, batch-local IDs, confidence,
  depth counts/spread and artifact/transform evidence. Disarm/config changes,
  source tampering, invalidated observations and malformed worker output never
  return usable targets. The controller checks frame/profile and timestamp
  validity again.

The item frame stays centered on the same pick point. Short X is the former
short Y; long Y is the negative of the former long X, retaining right-handedness
and the same Z. The shared Link6 planner follows short X, preserving physical
gripper attitudes, camera-clearance choices and motion routes. Schema-9 dimensions
are unchanged: `geometry.height`/response `length` remain the long side, and
`width` remains the short side. Existing teach files need no conversion.
After upgrading, rebuild and restart Item Teach/Detect and Robot Controller
(including TF preview) together. `GetItemPoses.pose_convention` and response
evidence must both equal `item_short_x_long_y_v1`; older clients/providers are
rejected before their candidate poses can be used for picking.

Initial form values are explicit and saved in `quality`: input age 0.5 s,
RGB/depth separation 0.1 s, robot TF age 1 s, request deadline 10 s, at least
50% valid circle pixels, and depth 200–1000 mm. **Minimum valid depth (%)** is
displayed on a 0–100 scale (greater than zero) and stored as
`quality.minimum_depth_fraction` in (0, 1]. The numerator is the pixels retained
after range/MAD filtering and item-mask inclusion; the denominator is every
original depth pixel in the physical sampling circle. Empty footprints and
zero valid pixels fail. There is no additional fixed pixel-count minimum.
Tray placement uses a separate fixed 20% requirement, with tray containment
instead of the item mask, while retaining the saved diameter and other quality
settings. Item picking keeps its saved percentage. Plane-based tray pose
measurement itself needs no live depth.
Schema 11 removed `minimum_depth_samples`; current schema 13 additionally requires
the nearby depth radius and height. Open older profiles in Item Teach, review the
retained percentage and the proposed 150/60 mm defaults for absent new fields,
then Save, reload the controller and manually deploy the updated pair before
headless startup. Missing/invalid schema-13 fields remain blank in recovery;
production readers reject incomplete or older profiles. Recovery never saves or arms.
There is no
result-age field: an accepted batch remains valid until invalidated or replaced.
Edit and save these fields deliberately; a file missing them is rejected.
Frames/overlays are bounded transient memory, never saved as an accumulating
archive. Events remain timestamped and bounded at 1,000 package records.

### Headless detection and controller request

Headless Item Detect loads the same exact calibration files as Item Teach from
these existing root `.env` keys:

```text
ITEM_TEACH_PLATFORM_CALIBRATION
ITEM_TEACH_BIN_CAMERA_CALIBRATION
ITEM_TEACH_ROBOT_CAMERA_CALIBRATION
```

Use Item Teach's Browse selectors to automatically save a complete validated
set before starting the detector. Values are filenames directly inside root
`calibration/`. All empty is allowed for Item Teach's first run, but headless
startup fails with an instruction to make a selection. Partial/missing keys,
missing active files, invalid schemas or a platform/robot identity mismatch fail
without a latest-file scan or fallback. Descriptive mode-prefixed camera names
are supported, and unrelated or newer files cannot replace the chosen set.
The detector reads `.env` once for selection and never writes it. Runtime source
hash checks continue to protect the exact loaded files; changing `.env` choices
requires a restart. The camera prefix comes from the selected active camera calibration.
Original teaching-camera files are not opened or compared to that active calibration.

Populate flat root `runtime_teach/` with exactly these ordinary files:

```text
item_teach_<name>_<timestamp>.yaml
item_teach_<name>_<timestamp>.pt
bin_teach_<timestamp>_<source>.yaml
```

The Item YAML and model must have the same stem. The shared headless catalog
classifies by `item_teach_` and `bin_teach_` filename prefixes, then applies the
current strict schema/hash readers. Missing, duplicate, mismatched, symlinked,
nested, unknown-prefix or unsupported-extension entries fail startup. Hidden
dot-prefixed atomic-write entries are ignored. A complete optional
`tray_teach_*.yaml` / same-stem `.pt` pair may coexist with these files; an
incomplete or duplicate tray pair fails explicitly.
Each missing Item YAML, Item model or Bin YAML has a distinct error. Duplicate
errors list the conflicting filenames. Item Detect writes the exact message as
a bounded `FATAL` `item_detector_failed` event, logs it through ROS and exits.

Deployment is manual for now; a third-party program or remote node may later
manage the same directory contract. Finish the complete catalog before launch.
For a replacement, stop the process, stage incomplete transfers under hidden
dot-prefixed names, expose exactly one complete visible set, and restart. The
runtime does not copy files, watch the directory, retry selection or switch
catalogs while running.

GUI/headless pose parity requires copying the exact saved Item YAML/model and
selected Bin YAML into that catalog after teaching changes. Identical filenames
alone do not establish parity: compare file SHA-256 hashes. Both modes use
`ItemDetectNode` for strict profile validation, fresh request snapshots, inference,
filtering, candidate limits and pose responses. GUI preview can show additional
candidates; compare Simulate Trigger or the Armed service with headless requests.
Unsaved GUI edits and recovery drafts are not deployed production settings.

Launch with no arguments:

```bash
ros2 launch item_perception_yolo item_detect.launch.py
```

Starting this dedicated process is the explicit decision to load the trusted
deployed `.pt` and advertise `/item_detect/get_item_poses`. It validates the
saved station/robot-camera selection, current Item Teach model/settings and Bin Teach,
then waits only the taught request deadline for fresh inputs. The model stays
loaded and YOLO/arming enable automatically after validation. Inference runs
only for a service request and every request acquires
a new RGB/depth pair after arrival. It never returns a cached batch. Restart is
required to select replacement runtime or calibration artifacts. Item Teach and
an armed headless detector must not advertise the service simultaneously.

The separate Robot Controller must load the same runtime snapshot in headless
mode, or the matching explicit files in GUI mode. Its typed Pick action requests
the batch; Item Detect remains read-only and cannot issue motion or I/O.
Robot Controller still selects calibrations using its strict canonical latest
catalog, independently of these `.env` choices. Its source-hash checks reject
detector results from different calibrations. Custom catalog filenames can also
block controller loading; this detector change does not alter that controller
contract. Configure matching artifacts when using both processes together.

## Transform contract

`platform_teach` accepts one strict schema-7 `camera_to_hand` or
`camera_on_hand` artifact from the root `calibration/` directory. It reads the
camera prefix and measured ChArUco settings from that file and subscribes to
its exact color image and CameraInfo topics. Fixed-camera mode calculates:

```text
base_link <- platform_reference
  = base_link <- camera_link
  * camera_link <- color_optical_frame
  * color_optical_frame <- ChArUco board
```

On-hand mode calculates:

```text
base_link <- platform_reference
  = base_link <- Link6                 (live, at most one second old)
  * Link6 <- camera_link               (camera calibration)
  * camera_link <- color_optical_frame
  * color_optical_frame <- ChArUco board
```

The ChArUco board origin is the platform origin. Place that origin at the bin
mount corner chosen as the origin of the whole robot pick area. The workflow
does not use depth, robot joints, hand-eye solving, pose averaging, or robot
motion. Robot TF is required only for camera-on-hand mode.

Detection uses the exact package-private OpenCV 4.10.0 runtime supplied by
`camera_calibration`, including the saved dictionary, board dimensions,
measured square/marker sizes, and legacy board layout. The ROS/Qt parent does
not import system OpenCV. RGB frames and overlays exist only in transient
memory and are never saved.

## Build and launch

Build from the repository root so `camera_calibration` is built first:

```bash
cd ~/PicknPlace
source /opt/ros/humble/setup.bash
colcon build --packages-up-to item_perception_yolo
source scripts/source_ros_workspace.bash
ros2 launch item_perception_yolo platform_teach.launch.py
ros2 launch item_perception_yolo bin_teach.launch.py
```

Both launches have no arguments. They require `ROS_LOCALHOST_ONLY=1`, the
strict root `.env`, the selected active camera, and its camera-internal TF.
Camera-on-hand mode additionally requires the separately published live robot
TF chain through `Link6`. Neither launch starts a camera, Dobot bringup, RViz,
robot-state publisher, or robot motion.

## Operator workflow

1. Select a `camera_to_hand_calibration_<timestamp>.yaml` or
   `camera_on_hand_calibration_<timestamp>.yaml` directly from root
   `calibration/`.
2. Select **Apply Calibration**. This validates schema 7, calibration mode,
   pipeline provenance, complete sample data, saved transform, and ChArUco
   settings before subscribing.
3. Place the board origin at the selected bin-mount corner. Use the live RGB
   marker/corner/axis overlay to confirm detection.
4. Select **Capture Platform** while the displayed board pose is fresh. The
   GUI composes the saved camera calibration, live camera-internal TF, and
   newest RGB board pose without averaging. Camera-on-hand mode also validates
   and records fresh `base_link <- Link6` TF.
5. Inspect the displayed `base_link <- platform_reference` XYZ/RPY and its
   dynamic TF preview in RViz. **Retake** clears the capture and stops further
   preview broadcasts.
6. Select **Save YAML** and confirm. The success dialog shows the exact path.

The package uses one atomically replaced
`logs/item_perception_yolo/last_session.json` with strict schema 6. Its
`platform_teach`, `bin_teach`, and `item_teach` sections preserve each other. Restart only
restores validated fields as unapplied prefill; it never reloads a transform,
captures a target, starts a process, or publishes TF automatically.

## Output

Platform calibrations share the root camera-calibration directory:

```text
calibration/platform_calibration_<UTC_TIMESTAMP>_<DOBOT_ROBOT_LAN1_IP>.yaml
```

LAN1 is the stable robot identity; LAN2 remains diagnostic failover only. The
strict schema-4 artifact stores:

- `base_link <- platform_reference`;
- the exact `charuco_pick_corner_xy_v1` reference convention: ChArUco board
  origin at the shared pick-area corner, unchanged board axes, metre units,
  and platform-plane Z=0 for the bin footprint;
- historical `teaching_provenance`: the selected schema-7 camera filename/hash,
  camera prefix/frames/topics, ChArUco settings, detector, mounting mode, original
  transform chain, optional robot TF, source timestamp and detected corner count.

Platform Teach shows a startup reminder to freshly calibrate the bin camera in
its current position before teaching/re-teaching. Source calibration is validated
during capture/save. Once saved, the platform is fixed geometry in robot-base
coordinates; future camera movement does not move it. Runtime uses the independent
active camera calibration for projection and detection. The recorded camera file
need not exist, and its hash/mounting transform need not match the active camera.
The embedded historical chain is still checked for internal consistency. Schema-3
platforms are also read this way. Explicit schema-3→4 migration only relocates
the historical fields under `teaching_provenance`; no pose is recalculated.

For reusable bin templates, every station must reproduce the same board origin,
axis directions, bin size, and bin offset from that reference. Keep the board
and bin-corner markers approximately coplanar within each station. Their common
height relative to that station's robot base may differ; each station's saved
platform transform captures that difference. The GUI explains this placement
contract and asks for confirmation when saving. It does not infer physical
alignment from a matching frame name.

It stores no RGB frame or overlay. Saving hard-fails if the selected camera
calibration changed after it was applied or if the output path already exists.

Package events are written to `logs/item_perception_yolo/events.jsonl` with UTC
timestamps and the project-wide 1,000-record overwrite limit.

## Bin-teach contract

`bin_teach` accepts a strict schema-3 or schema-4
`platform_calibration_<timestamp>_<robot_ip>.yaml` selected directly from root
`calibration/` plus an independently selected **Active camera calibration**.
The camera selector prefills `ITEM_TEACH_BIN_CAMERA_CALIBRATION` from root `.env`;
Browse permits an explicit teaching camera without rewriting the saved selection.
It validates the platform, current camera and LAN1 robot identity, then pins the
two active source hashes for the session. It inherits the active camera prefix, RGB topic,
CameraInfo topic, frames, and calibrated mounting transform. Fixed-camera mode
uses `base_link <- camera_link` directly. On-hand mode requires live
`base_link <- Link6` no older than one second and composes it with calibrated
`Link6 <- camera_link`. The operator separately selects an exact 5x5 ArUco
dictionary and enters the measured common marker size in millimetres.

All four markers—IDs 0, 1, 2 and 3—must be visible simultaneously, with no
additional IDs from the selected dictionary in view. Their spatial placement,
rotation, and ID order do not define ROI order. The exact private OpenCV 4.10.0
worker detects marker corners and obtains each square pose with fixed
`SOLVEPNP_IPPE_SQUARE` for marker-axis visualization, not ROI coordinates.
The node composes:

```text
platform_reference <- color_optical_frame
  = inverse(base_link <- platform_reference)
  * base_link <- camera_link
  * camera_link <- color_optical_frame
```

In the same lifetime worker, `cv2.undistortPoints` converts all 16 detected image
corners to color-optical rays using the current CameraInfo intrinsics/distortion.
The node transforms each ray through that chain and intersects it with platform
Z=0. The platform's full taught rotation/translation remains unchanged: no
flattening to robot-base XY, marker-PnP XYZ/drop-Z step, depth input or averaging.
The measured marker size remains required for the marker-axis poses; ROI metric
scale instead comes from the calibrated camera/platform plane. Parallel rays,
non-finite results and intersections at/behind the camera block that frame;
native worker failures remain terminal without retries or fallbacks.
For each marker, the selected outside corner is the unique corner farthest from
the centroid of the four plane-projected marker centres. Those four selected points are sorted
clockwise in platform XY, starting from the lexicographically smallest `(x,y)`
point. A tied outside corner, duplicate point, non-convex polygon, or degenerate
area blocks capture rather than choosing an unexplained alternative.

The live RGB view draws detected markers, their selected outside corners, and
the yellow saved-plane ROI border. It uses the same distorted 32-samples-per-edge
projection as the green loaded border, rather than drawing independent raw
detected pixels. Readiness logs record the planar method and corner round-trip
pixel differences (a consistency check, not independent calibration accuracy).
Keep physical corner markers on the taught platform plane; an incorrect plane
can still produce incorrect metric geometry even with a perfect pixel round trip.
Select **Capture Bin ROI** while the result is no more
than 0.5 seconds old. A successful capture starts a 10 Hz RViz preview with the
complete chain:

```text
base_link -> platform_reference -> bin_corner_1
                                -> bin_corner_2
                                -> bin_corner_3
                                -> bin_corner_4
```

The numbered corner frames follow the same clockwise P1–P4 order as the YAML.
Each uses the saved `(x, y, 0)` position and the `platform_reference` orientation;
zero Z is deliberate because the bin artifact is strictly a 2D XY ROI. The node
broadcasts `base_link -> platform_reference` from the selected platform artifact,
so `platform_teach` does not need to remain open. **Retake**, applying different
settings, a terminal detection failure, or closing Bin Teach stops the preview.
Bin Teach does not launch RViz.

This planar teaching correction keeps bin schema 3, camera schema 7,
shared UI schema 6 and all existing artifacts unchanged. Re-capture and save a
new bin file to obtain corrected geometry; old XY is loaded as written, never
automatically repaired. Station transfer retains the same local XY on the
destination's own platform plane, including its tilt and height.

Then select **Save YAML** and confirm. Output is:

```text
offline_teach/bin_teach/bin_teach_<UTC_TIMESTAMP>_<DOBOT_ROBOT_LAN1_IP>.yaml
```

Strict schema 3 separates the four metric XY `roi.points` and canonical
`reference` contract from `teaching_provenance`. Provenance retains the original
robot IP, camera settings/mode, platform/camera filenames and SHA-256 values,
capture timestamp, source platform transform, and resolved transform chain.
On-hand teaching records the exact robot TF and timestamp used. Source marker
IDs/corner indices remain attached to the points for traceability. Saving a
fresh capture still rejects changed source calibration files or an existing
output path. No RGB frames, overlays, marker poses/origins, depth, or joint
states are saved.

## Reusing a bin ROI at another station

Copy the schema-3 bin file, retaining its filename, into the destination's root
`offline_teach/bin_teach/`. Bin Teach saves/loads ROI files only in this
directory; it never searches `calibration/` for them. Camera and platform files
remain in `calibration/`. The IP in that filename is the teaching station's identity;
it does not restrict reuse. The bin reader validates the complete schema,
polygon, reference convention and internally consistent teaching transform
chain without requiring the original robot `.env` or source calibration files.
Recorded source hashes are provenance, not authentication of an absent file.

In **Bin Teach**, explicitly select and Apply the destination station's own
schema-3/4 platform calibration, active camera and current marker settings, then select
**Load Bin ROI**. The destination platform and its referenced camera file must
still match the destination robot and their saved hashes. Confirm the same
physical bin size, board origin/axes, and placement. This replaces the current
capture or loaded preview; it does not create or save a new capture. Use
**Retake** to clear it before teaching new geometry. Loading never restores the
source station's camera settings, mounting mode, or platform transform.

The loaded points remain unchanged in platform XY. RViz places them using only
the destination `base_link <- platform_reference`, so a change in station
height or rotation moves the entire preview correctly. Source and destination
camera mounting modes may differ. The RViz preview needs no marker detection or
live robot TF once the destination platform has been applied.

The live RGB view shows only the loaded ROI's **green border**, without filling
the polygon, and a green **Loaded Bin Teach | &lt;filename&gt;** label at top left.
Marker detection is skipped while a template is loaded, so the four markers
do not need to remain visible. Projection uses the destination platform plane
at Z=0, destination camera calibration, live camera-internal TF and current
color CameraInfo intrinsics/distortion. It never uses the original teaching
station's transform to place the border. The same isolated OpenCV worker
projects 32 samples per edge to account for lens distortion; the ROS/Qt parent
does not import OpenCV. The RGB distortion model must be `plumb_bob` or
`rational_polynomial`; unsupported models are explicitly blocked, not guessed.

RGB must be no older than 0.5 seconds. Camera-on-hand additionally requires
`base_link <- Link6` no older than one second. Missing/invalid CameraInfo or TF,
stale RGB/robot TF, or geometry behind the camera hides the border and displays
the reason, without discarding the template. A native worker failure remains
terminal. Fresh detections in normal teaching mode retain their yellow border.
Saving remains disabled for loaded templates. Retake, re-Apply, terminal failure
and exit clear the loaded preview and stop publication. Loaded templates are
not restored from `last_session.json`.

New platform artifacts use schema 4, and Bin Teach remains schema 3. Bin teaching
provenance may name a schema-3 or schema-4 source platform; neither that platform
file nor its teaching camera is required to load the portable bin geometry.
Existing schema-1/2 files remain untouched and are rejected; re-teach them.
Camera calibration remains strict schema 7. Shared UI prefill is now
schema 6 to include Item Teach's profile, preview prefix and station/bin
filenames; the existing platform
and bin form fields are unchanged. Schema-3 platform loading is supported without
rewriting the file; migration to schema 4 is explicit. This workstation's validated schema-5 prefill was explicitly
rewritten once for this update, preserving both existing teaching sections.

Platform/bin nodes create teaching artifacts and provide operator previews only.
Item detection now explicitly loads those artifacts under the separate contract
above; physical picking remains unimplemented. It must not depend on either
platform/bin teaching GUI or its preview TF staying
alive. To avoid duplicate platform previews, close Platform Teach after saving
before previewing the selected platform through Bin Teach.
