# Tray Perception

The teaching window title is **Tray Teach**, without a version suffix.

`tray_teach` is a read-only GUI for teaching one tray profile and inspecting the
best detected tray pose in `base_link`. Headless `tray_detect` shares its fresh
pose-request pipeline, including controller-requested placement depth. No camera/robot process is launched and no
motion, enable, gripper or controller command is sent.

Build and launch from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to tray_perception
source scripts/source_ros_workspace.bash
ros2 launch tray_perception tray_teach.launch.py
```

Bring up the configured camera separately. Enter its exact prefix and click
**Connect RGB**. The UI shows the connected RGB/depth topics; its status tooltip
lists both CameraInfo topics too. RGB/YOLO preview needs no calibration, depth,
robot TF, Item Teach, name, dimensions or selected classes. The RGB and registered
depth panes share a horizontal layout and show unavailable-input reasons.
The Item Teach-style sidebar groups Camera/Calibration, Tray/Model, Detection
Settings, Tray Size Filter, Teaching Position and Reference Plane. Load/Save stay at
the top right, with YOLO/Simulate/Armed above the live views. Reference-plane
controls edit the teach file's geometry, independently of camera setup.

Browse the camera's schema-7 calibration from `calibration/` to enable metric
geometry; this also fills/connects its recorded prefix. A different manually
connected prefix permits detection but cannot reuse that calibration. Reconnecting
to another prefix clears observations and disarms requests, while preserving the
saved base-frame plane. Metric detection still requires the connected prefix to
match the active calibration.
Fixed and on-hand cameras are supported. On-hand measurements require fresh
RGB-time `base_link <- Link6` TF. A shared 100 ms background TF wait preserves
the exact image timestamp and rechecks freshness. Missing TF blocks geometry,
not RGB detection. No platform/bin artifacts are needed. Loading a profile with
no camera explicitly loaded uses the robot-camera selection saved by Item Teach
in `ITEM_TEACH_ROBOT_CAMERA_CALIBRATION`. Headless Tray Detect and controller tray
configuration use that same selection. No new `.env` key is introduced. For
controller placement, select the same active file in Tray Teach, then reload the
controller configuration after changing that choice. GUI Browse also supports
standalone inspection with another explicitly selected calibrated camera.

Drag the divider between RGB and registered depth to adjust their widths, just
like Item Teach. Each pane keeps its own status heading and scales the image to
fit; live updates preserve your chosen split.

1. Browse a local YOLO `.pt` model to automatically load it and enable 1 Hz preview when RGB is
   ready. **YOLO Detect ON/OFF** controls inference; OFF retains RGB/depth/voxel
   preview. Show all model classes under the current confidence, IoU and detection
   cap; checked classes determine pose eligibility. Defaults are 0.25 / 0.70 / 100
   and internal inference size 640; loaded profiles retain their inference size.
   Edits apply after 300 ms without typing. Invalid inference values pause YOLO
   with an inline reason; correction resumes it, with no old-value fallback.
   Segmentation masks and OBB models support metric poses; box-only models remain
   detection previews and can be saved in drafts but cannot produce a detection profile.
   **Detection Mask Clean** is always on for tray segmentation. Read each model
   instance's raw binary mask, apply a 3×3 opening in native mask pixels to remove
   thin bridges, then keep its largest connected region by pixel area. That region
   must retain at least 80% of the original foreground; empty or ambiguous splits
   are rejected before fitting a rectangle. This also avoids artificial lines
   added when YOLO converts disconnected mask regions into one polygon. RGB/depth
   outlines, dimensions and poses all use the cleaned region. Opening can adjust
   fine corners; expected-size/tolerance checks still apply. A source connected
   region touching the image edge remains clipped even if its thin tail is removed.
   A detached small satellite can be discarded without marking the main tray clipped.
   Live inspection, frozen simulation and request logs show retained/removed mask
   evidence; rejection tooltips explain failures. Live preview, Simulate Trigger,
   armed Tray Teach and headless Tray Detect use this same stage. OBB detection,
   Item Teach, placement depth and file schemas are unchanged. Restart the tray
   node after rebuilding to activate it; no teach-file edit is required.
2. Leave **Length**, **Width** and **Tolerance** blank to measure first. Neither
   these filters nor a robot observation pose is needed to capture the plane or
   inspect measured trays. **Tray Detect Pose** → **Record Current Joints as Tray
   Detect Pose** records the six current joint angles, like Item Teach Home.
   Requires a sole configured Dobot `/joint_states` publisher, receipt and nonzero
   timestamp at most one second old, and exactly six finite canonical joints.
   The display shows robot identity and J1–J6 in degrees; the file stores radians
   ordered joint1 through joint6, with timestamp and publisher evidence. No
   Cartesian TCP pose or zero-joint default is created. Replacing a recorded pose
   requires confirmation; a failed reading or cancellation preserves the old one.
   **Save Tray Teach** persists it in the same file. Record Tray Detect Pose
   independently of Item Teach Home; no Item Teach file is loaded or copied.
   Recording never moves the robot.
   This is the observation position for future controller travel before tray
   detection; controller motion integration remains separate and controller Home
   still comes from Item Teach. The pose is optional for saving a complete
   detection profile, arming, simulation and pose requests.
3. With the tray uncovered, select **Capture 4-corner snapshot…** in the sidebar's
   **Reference Plane — teach file** section. The existing RGB/depth panes hold
   that captured observation, labelled **CAPTURED** with its age; no new window
   opens. Click four distinct corners on RGB in any order, use **Undo corner**
   if needed, then **Create reference plane**. **Cancel corner capture** discards
   the draft. Both Create and Cancel return the panes to live automatically.
   Click coordinates account for scaling and letterboxing. All clicks use the
   same initially fresh synchronized RGB/depth/TF observation; live frames never
   replace that corner source. Streams, background preview and RViz keep updating,
   while inspection clicks in the captured pane select corners. Capturing leaves
   the existing plane active until Create succeeds; draft lines/corners are cyan.
   Source/settings changes discard the draft and require another capture.
   A 7×7 registered-depth patch per corner uses the median of whatever valid
   samples remain between 200 and 1000 mm after MAD filtering, including only
   one sample. There is no 30-sample minimum; zero valid samples still fail.
   Separate RGB/depth distortion is
   preserved. Four convex, noncollinear base-frame points must fit a plane with
   maximum residual at most 5 mm; bad samples are refused, never filled in.
   Numbered corner locations, accepted-sample counts and median depth appear in
   both captured panes; sampled pixels mark accepted values black and rejected values red,
   mapped with each pane's distortion model.
   Once created or loaded, the plane's green border and P1–P4 corners remain on
   both live panes, independently of YOLO or tray acceptance. Their projection
   follows the current timestamped calibrated view. Visible border segments are
   clipped to the image; only on-screen corners receive markers. Behind-camera,
   off-screen or numerically unsafe outlines are omitted without stopping the
   preview or changing the saved plane or tray-pose calculations. RGB preview
   reports why the outline is unavailable. Green means a plane is
   available in this session; the sidebar explicitly says **not saved — Save
   Tray Teach** until the plane is written to a profile or loaded from one.
4. Click a displayed tray to inspect its measured size and acceptance reason.
   Once the reference plane exists, every fully measurable mask/OBB has a cyan
   nearest-base corner, red **X / short-edge** arrow, green **Y / long-edge** arrow
   and edge lengths in mm. Measurements and axes work before dimensions are
   entered, for unchecked classes, and outside size tolerance; those detections
   remain ineligible for a production pose. Clicks report that tray's X/width,
   Y/length and corner XYZ in `base_link`, without changing your expected dimensions.
   Depth shows the same geometry projected with its own distortion model when
   available. Missing live depth does not block plane-based RGB measurements.
   Preview status reports **measured / size filter inactive** when filters are
   incomplete. If measurements are unavailable, it names the missing calibration,
   timestamped TF or reference plane instead of only saying no tray was accepted.
   Ordinary inspection stays live; the last clicked observation is labelled with its age.
   Its highlight disappears on the next frame, and settings changes invalidate
   the summary. Corner capture and Simulate Trigger hold the displayed observation;
   there is no Resume Live button for ordinary inspection.
   Enter long-side **Length**, short-side **Width**, and one **Tolerance ± (mm)**
   manually. Clicking never overwrites those fields. Missing/invalid dimensions
   keep detections visible with grey unchecked borders but prevent accepted poses.
   Green/red means size pass/fail, independently of class acceptance. Measurements intersect model
   polygons with the saved reference plane, fit a metric enclosing rectangle,
   and check both dimensions against the tolerance. Live depth is not used for
   subsequent measurements, so items on top cannot change the reference height.
   A clipped tray is rejected. Rank valid trays by image-center distance,
   confidence and source index; select exactly one or report no valid tray.
   A changed tray support height or tilt requires teaching the plane again.
   Without usable metric geometry, mask/OBB detections still show a grey fitted
   rectangle and red short/green long image axes labelled **X 2D / Y 2D**. These
   are pixel-only orientation guides; they have no nearest-base corner, mm size,
   TF or production pose. Valid plane-based corner axes replace them once ready.
5. **Save Tray Teach…** needs only a valid tray name (1–64 letters, digits,
   underscores or hyphens, starting with a letter or digit). The first save writes
   `offline_teach/tray_teach/tray_teach_<name>_<UTC_TIMESTAMP>.yaml`. Any loaded
   model is copied byte-for-byte to a same-stem `.pt`, with SHA-256 verification;
   saving never executes or re-exports the model. Without a model only YAML is needed.
   Blank/unfinished fields are preserved as draft text alongside any created
   reference plane, recorded joint pose and bound calibration. Only created planes
   are persisted; four corner clicks must still be committed with Create.
   Complete validated detection data saves the existing production schema-1 profile.
   The existing `tray_teach_position` key stores Tray Detect Pose as joint angles,
   or null when unrecorded; its `units` entry remains `rad`. No schema change is needed.
   The status distinguishes **Saved draft** with the remaining requirement from
   **Saved complete profile**. Calibration stays in root `calibration/` and is
   referenced by filename/hash, as before; no camera file or live image is copied.
6. Like Item Teach, subsequent Save clicks update the loaded/saved YAML and model
   in place while the tray name is unchanged. One hidden
   `.<stem>.previous.zip` retains the previous YAML and model (when present).
   Settings-only updates leave unchanged weights untouched. Source/target hashes,
   staged copies, YAML-last publication and rollback protect against changed files
   and failed writes. A renamed tray creates a new timestamped file/pair and leaves
   the previous one unchanged. Saved drafts become complete profiles at the same
   path when all required data validates; incomplete edits can save them as drafts
   again. External YAML/model modifications require reloading before overwriting.
7. **Load Tray Teach…** restores complete profiles or partial drafts for further
   teaching. Model-containing documents verify and load the paired model plus any
   bound calibration automatically; there is no additional model trust prompt.
   A saved plane or joint pose needs no source Item Teach file/model to reopen.
   Loading complete detection data allows Arm/Simulate immediately without another
   Save, even for an older draft saved without a teaching position. Missing detection
   data still blocks requests with the specific requirement. Headless accepts only
   explicitly saved production profiles, not drafts.
   Loading a tray is optional when starting a new profile; Save does not deploy files
   into `runtime_teach/` or alter controller configuration.

The origin is the detected rectangle corner nearest the robot `base_link`
origin by **3D Euclidean distance**. Both +X and +Y point along its adjacent
edges into the tray: **X is always the short edge; Y is the long edge**. Z is
their right-handed cross product and may point to either side of the plane;
its direction cannot also be forced toward the camera for every nearest corner.
The saved reference-plane normal still faces its teaching camera; it is separate
from the detected tray attitude. Image left/right is irrelevant. Exact distance
ties use base XYZ ordering; equal-length edges use their adjacent endpoints'
base XYZ ordering. A symmetric unmarked tray has no tracked physical-corner identity;
the origin can switch when another corner becomes nearest the base.

RViz also shows a separate **XY and blue UP guides** marker display on
`/tray_teach/pose_guides`. Red X and green Y follow that same actual pose. The blue
arrow uses the reference-plane normal facing positive `base_link` Z; it may be
opposite the actual pose's +Z. An exactly horizontal normal stays horizontal.
Like Item Teach, all guides are 200 mm long with 20 mm shafts and 40 mm heads
(diameter and length), matching robot-joint TF axis length/width at Marker Scale 1.
Only the displayed arrow changes: TF, returned tray quaternions, nearest-base
origin, X/Y extents, saved plane and controller placement coordinates are untouched.
The guides clear on invalidation/no selection, expire after 2.5 seconds and share
the selected observation's timestamp. Missing depth or grey voxels do not clear
fresh plane-based pose guides. Reload canonical RViz to add the display.
Canonical **TF → Show Axes = false** hides the overlapping raw axes, including
robot-frame axes, so the blue-UP guides are the visible pose axes. TF data and
RobotModel remain unchanged. Uncheck Show Axes in an existing viewer or reload
the configuration; enable it explicitly to inspect real frames. Headless Tray Detect
retains its existing voxel-only visualization, with no added pose publisher or
background inference.

The preview measures all complete tray polygons, while only the single eligible
tray nearest image center supplies TF/service output. Dimensions, base XYZ and
observation age appear below the image; rejection reasons remain visible.
`base_link -> tray_teach_selected_tray` is a teaching-only TF published once per
selected RGB observation with its original timestamp. Changes, failures or
expiry stop publication; TF viewers can retain history until their timeout. It is not a
controller target or production detection service. There are no placement
areas, placement targets, controller Home, motion rates or I/O settings here.

Strict tray schema 1 stores detection settings, same-stem model hash, camera
calibration filename/hash as historical plane-capture provenance, optional Tray Detect Pose joints, the four base-frame
corner observations, plane transform/fit evidence, and `nearest_base_corner_v1`
origin convention. Distances are mm for dimension settings, metres for plane/
pose geometry and radians for taught joints. No images or live depth are saved.
Camera-only recalibration or changed live intrinsics preserves the saved plane
and joint pose. Detection uses current intrinsics and image-time calibrated TF.
The original camera file may be absent; Save preserves its recorded filename/hash
while the plane is reused, and a newly captured plane records its current camera.
Neither loading nor recalibration rewrites existing teach files. The startup
calibration reminder belongs only to Platform Teach; Tray Teach has no such dialog.
Re-teach if the robot base or physical reference surface moves; after moving the camera, verify
that the saved observation pose still sees the tray. Active file hashes, exact-time
TF, synchronized depth and model validation remain enforced.

On launch, Tray Teach automatically reopens the exact last loaded or saved YAML
remembered in `logs/tray_perception/last_session.json`, just like Item Teach.
This restores the saved settings, plane, optional joint pose, independently selected
active calibration and verified paired model through the same background Load workflow. Complete
profiles and partial drafts both reopen; Armed stays OFF. The restored file remains
the Save target, with the existing overwrite/backup and rename behavior. There is
no newest-file search or automatic teach-file rewrite. Missing/invalid files report
the error once; use Load Tray Teach or browse sources to continue after correcting
it. Failed restoration never bypasses paired-file validation using remembered paths.

GUI form drafts are remembered automatically after a 300 ms typing pause and
flushed on orderly close to the single ignored `logs/tray_perception/last_session.json`.
This includes camera prefix, file choices, tray name, dimensions/tolerance,
confidence/IoU/detection cap, selected classes and inference size. Saving the draft
works with YOLO OFF, no loaded inputs, incomplete dimensions and invalid text;
the restored text must still pass normal validation before use. File dialogs
preselect remembered choices, and saved class IDs are checked against the model
on loading. With no remembered tray file, available calibration/model paths restore through the
existing worker, calibration first and model next. Both source selectors use
**Browse…**, and choosing a file also loads it without another click or model
confirmation. Selecting/restoring a model path authorizes that local load.
Missing or invalid calibration cannot block RGB/model preview. Missing files can
load once available; an invalid existing file is attempted once until browsed again.
Already loaded files are never silently replaced. YOLO preview starts when inputs
are ready; Armed stays OFF. A remembered tray file takes precedence over unsaved
session form edits. Its saved plane and joint pose are restored from that file.
Session schema 2 explicitly imports validated schema-1 sessions and replaces
them on the next draft save. Malformed or unknown formats fail explicitly;
write failures are reported and preserve the previous session.
Events are timestamped and capped at 1000 in the package's `events.jsonl`.
An arbitrary preview prefix is session-only; loading calibration or a saved tray
restores its calibrated prefix. Named incomplete saves use a distinct
`tray_teach_draft` artifact (draft schema 1) in the same teaching directory.
This is separate from automatic session prefill. Headless readers reject drafts
until detection data is complete and saved again as production `tray_teach` schema 1.
The GUI can validate a detection-complete older draft in memory from its saved form,
plane/calibration and verified model task, keeping the original file/hash unchanged.
Unsaved UI fields never fill missing saved data. Model pairing remains same-stem.

## RViz voxels

`/tray_teach/voxel_cloud` publishes `PointCloud2` snapshots in `base_link`, using
10 mm occupied-cell centroids and averaged original RGB. The canonical RViz
configuration includes a separate Tray Teach display with 10 mm boxes, zero decay,
and reliable/transient-local depth-one transport. `/tray_teach/rviz_diagnostics`
reports cloud state, source timestamps, refresh age, counts and detection reasons.
The cloud covers the calibrated valid depth scene (200–1000 mm), without a tray
ROI, and requires synchronized RGB/depth/CameraInfo and timestamped calibrated TF.
It needs neither a plane nor YOLO/classes/dimensions. Pose measurements still use
full-resolution polygons and the taught plane, independently of live depth.

Headless Tray Detect publishes the same scene voxels on `/tray_detect/voxel_cloud`
with `/tray_detect/rviz_diagnostics`. Its separate enabled canonical RViz display
is **Tray Detect - 10 mm colored voxels**. Both nodes use the same cloud geometry,
timestamps, QoS and retention rules; their topics remain independent when an
unarmed teaching GUI runs alongside headless detection.

If Item Teach voxels are visible but Tray Teach voxels are absent, check that
RViz has the enabled **Tray Teach - 10 mm colored voxels** display on
`/tray_teach/voxel_cloud`. An already-running RViz does not reload changed files
automatically. Use **File → Open Config** with
`install/dobot_rviz/share/dobot_rviz/rviz/urdf.rviz` to load the canonical display.
Tray Teach must be running with matching calibration and synchronized inputs to
publish its first cloud. Its RGB-pane RViz status and diagnostics report input gaps.

Reuse the exact displayed observation, at most once per second, without a second
YOLO prediction. Retain the latest cloud indefinitely; after five seconds without
a new validated frame its geometry stays but becomes grey. Fresh data replaces it
and restores color; duplicate/frozen observations never reset the refresh age.
Input gaps and empty results never publish intermediate empty clouds. Source or
settings changes, CameraInfo changes, terminal failure and orderly exit grey the
retained cloud immediately. A late RViz subscriber receives the current cached
cloud while the node runs. Live preview publishes the single selected-tray TF;
Simulate Trigger instead holds its separately named historical response frame.

The package reuses Item Perception's existing pinned private CPU runtime via
one lifetime native worker, without another wheel extraction, global install,
network access, worker restart or alternate model/runtime. ROS/Qt never imports
OpenCV, Torch or Ultralytics. A single GUI job slot prevents queued inference;
two ROS executor threads keep TF and camera reception independent of native work.
Automatic preview leaves fields, typing focus and controls available, including
when YOLO is off. Edits discard the in-flight preview; detection settings update
after the typing pause. Load, Save, Freeze and plane creation
reserve at most one action after the current preview; controls lock only for
that explicit operation. Freeze acquisition and save validation also run in the
background. Obsolete observations cannot restore a pose or overwrite the edit
status; native/protocol failures remain terminal even if the preview was
invalidated. See [NOTICE.md](NOTICE.md) for attribution.

## Trigger, arming and controller requests

The top row follows Item Teach: **YOLO Detect**, **Simulate Trigger**, and
**Armed**. Preview and arming are separate. GUI startup and profile loading leave
Armed OFF; Armed ON is red and advertises `/tray_detect/get_tray_pose_v2` using
[`tray_perception_interfaces/srv/GetTrayPose`](../tray_perception_interfaces/README.md).
The versioned endpoint separates this depth-capable layout from the earlier
pose-only service. No legacy endpoint is advertised or tried. Rebuild and restart
Tray Teach/Detect and Robot Controller together; an older process can remain open
with old Python/typesupport code even after an on-disk rebuild. A missing v2
provider keeps Place disabled before any observation travel.

The GUI and headless node supervise their ROS executor thread. An unexpected
exit or callback exception sets a terminal error, revokes arming and records
`tray_executor_failed` with the Python traceback. The GUI displays the failure;
headless exits. There is no silent thread restart or inference retry. Ordinary
no-tray/depth failures remain request results, keeping reception and preview live.
Each request logs its start, completion duration or failing phase and traceback
in `logs/tray_perception/events.jsonl`. Repeated identical GUI messages update the
display but are logged at most every 30 seconds, preserving useful history within
the existing 1,000-event limit. Normal shutdown does not report an executor fault.

Arming requires YOLO ON with valid settings, an explicitly saved/loaded unchanged
profile whose detection settings/plane match the form, its verified mask/OBB model,
an unchanged active calibration and fresh RGB/CameraInfo/exact-time TF. It needs no live
depth, a visible tray, Tray Detect Pose, Item Teach, controller process or
motion. Arming does not run inference. Settings/source changes and YOLO
OFF disarm immediately; changed files require explicit reload. Only one provider
may advertise the canonical service. Logical revocation is immediate; retiring
the ROS handle waits for an active callback's reply handoff to avoid destroying
the service while rclpy sends its cancellation response. Re-arming retires the old
endpoint before creating its replacement; old callbacks cannot use the new binding.

**Simulate Trigger** runs the same acquisition, inference, size/class filtering,
single-tray selection and typed response logic locally, even while Armed OFF.
It never enables the robot or requests controller motion. It requires the exact
saved profile and YOLO ON. Like Item Teach, the exact RGB/depth observation freezes
for **10 seconds after the result appears**, then returns to live automatically.
Queue/inference time does not reduce this hold. The headings show a countdown;
**click RGB to resume** sooner. Draw only the returned tray, plus the reference
plane; rejected or nonselected detection axes never masquerade as returned poses.
Both pane headings show SIMULATED/FROZEN, original frame age, inference time,
returned/valid/detected counts, base XYZ in mm, quaternion XYZW, dimensions,
confidence, batch ID and rejection reasons. An empty successful response freezes
its empty result for the same 10 seconds and clears the preceding pose. Missing
optional depth is labelled. Each replacement simulation starts its own hold.

The teaching-only `base_link -> tray_teach_simulated_tray` TF and existing pose
guides hold exactly the response geometry. They refresh their display timestamp
at 10 Hz; this never recomputes geometry from live robot/camera TF or changes the
observation timestamp shown in the UI/log. The normal live selected-tray TF stops
while the result is held. Ten-second expiry, RGB click, another simulation, edits, source/model/hash
changes, YOLO/arming changes, failure and exit clear the simulated pose. The ROS
timer independently validates its identity and monotonic expiry even if Qt is busy.
Expiry clears only teaching visualization, preserving arming and request handling. Old TF history can
remain briefly in RViz after publication stops.

Camera callbacks and armed service requests remain active. A service call always
uses independent fresh frames and cannot reuse or replace the frozen simulation.
Automatic preview inference resumes on expiry or RGB click; retained scene voxels may grey
while the historical result is held. Ordinary inspection/corner teaching retain
their existing live scheduling. No extra worker, camera subscription, robot command
or teach schema is added. Pose-only simulation does not test placement depth at a
controller X/Y; those requests still carry Item Teach sampling settings separately.
Request event logs include source timestamp, returned pose and per-detection
rejections, so a past response can be compared with a displayed simulated batch.

See [Item/Tray feature comparison](../../docs/TEACH_UI_COMPARISON.md) for the
remaining inspection, depth, logging and task-specific differences.

Each request supplies `profile_sha256`, the SHA-256 of the saved tray YAML. It
waits for RGB both captured and received after the trigger, resolves TF at that
image's timestamp, and runs inference once against the unchanged saved plane.
It never returns the continuous preview's cached pose or retained RViz cloud.
The ten-second deadline includes waiting for preview work and fresh inputs;
concurrent requests return BUSY without accumulating. Keep two ROS executor
threads and one shared native worker, with request work serialized against GUI
load/save/preview actions. Disarm, changes and shutdown cancel in-flight results.

A successful request reports `OK` with `found=true` and one tray, or
`NO_VALID_TRAY` with `found=false`. The response includes the source RGB timestamp,
base_link pose, detected/valid counts, class/confidence, sorted long/short lengths,
and positive-axis `extent_x = width` / `extent_y = length`, all in metres.
The controller chooses X/Y placement inside those extents; tray attitude is a
reference frame, not a commanded robot TCP orientation. Diagnostics include the
profile/model/calibration hashes, reference-plane evidence and detection reasons.

`save_debug_images=true` writes only that request's annotated RGB and available
depth image under ignored `debug/tray_img/`. Default requests and previews save
no images. Missing depth or image-save failure is reported in diagnostics without
discarding an otherwise valid tray pose. Simulate Trigger does not archive images.

## Headless Tray Detect

Manually deploy exactly one `tray_teach_*.yaml` and its same-stem `.pt` into flat
root `runtime_teach/`. Keep the active eye-on-hand calibration selected by
`ITEM_TEACH_ROBOT_CAMERA_CALIBRATION` under root `calibration/`. Its strict binding
is `Link6 <- robot_camera_link`; the YAML's teaching-camera filename/hash is history. No Item/Bin Teach file is needed by Tray Detect;
its complete pair may coexist with their files in the shared deployment folder.
The Item/controller catalog still requires its own Item pair and Bin YAML, and
now allows a complete optional tray pair. Missing/duplicate tray files, mismatched
stems, unknown prefixes, unsupported extensions and symlinks fail visibly.

For GUI/headless parity, deploy the exact saved production YAML/model pair and
use the same active robot-camera calibration. Tray Teach may explicitly load a
different camera; headless always uses the `.env` selection above. Both modes
share `TrayTeachNode` and `TrayRequests`: saved-plane geometry, mask cleanup,
size filtering, tray selection and optional fresh placement-depth sampling are
identical for identical inputs. GUI drafts and unsaved edits are not deployable
production profiles. Saving in Tray Teach does not update `runtime_teach/`.

```bash
source scripts/source_ros_workspace.bash
ros2 launch tray_perception tray_detect.launch.py
```

Starting this dedicated read-only process explicitly trusts the deployed model.
It loads the pair once, connects the active camera’s calibrated prefix, waits boundedly
for fresh calibrated RGB/TF, and arms automatically. There are no file/trust/arming
launch arguments, latest-calibration search, new `.env` keys or automatic deployment.
Headless YOLO inference is request-driven only; there is no GUI, selected-TF
publication or continuous image saving. Scene voxels refresh at up to 1 Hz from
fresh calibrated synchronized RGB/depth/exact-time TF, independently of pose
requests. The main loop shares the existing single native worker and gives
pending pose requests priority; no cloud backlog or extra executor thread is
created. A request arriving during an active cloud calculation waits for that
bounded calculation. Cloud processing never invokes YOLO or returns a cached
pose. Missing depth/TF retains the old cloud and greys it after five seconds;
missing depth alone does not block plane-based tray requests. Diagnostics report
the blocking input, and advancing valid RGB/depth restore color. Files are pinned
until restart; source
changes/fatal worker failure stop the process, with no reload/retry/restart loop.
Do not run it while Tray Teach is Armed. An unarmed teaching GUI may still preview.


## Placement depth requests

`GetTrayPose.sample_placement_depth=true` enables a `PlacementDepthRequest` containing
positive tray-local X/Y in mm, the physical Item Teach depth sampling **diameter**,
and its exact quality settings. The provider takes fresh synchronized RGB/depth
after the trigger, performs the usual single tray inference, and samples the
requested target using original registered-depth pixels and their own intrinsics.
It uses the same median/MAD acceptance as Item Pick, excludes outside-tray samples,
and rejects invalid targets, clipped footprints or insufficient depth. The saved
plane remains unchanged. A depth failure returns ERROR without killing/disarming
the native worker/provider. Ordinary pose requests keep depth optional.

`PlacementDepthResult` contains validity, base-frame target X/Y with measured
surface Z, the depth timestamp, accepted/total counts, median and MAD sigma.
Diagnostics echo `placement_sampling` alongside bound profile/model/camera/plane
evidence. The controller checks every requested setting, source identity, result
frame, synchronization and after-trigger timestamps before motion. An invalid
response never becomes a placement target. Perception issues no robot or I/O calls.
Rebuild these interfaces and restart all tray providers and controller clients
together; `GetTrayPose`'s wire definition changed.
