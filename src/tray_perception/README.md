# Tray Perception

`tray_teach` is a read-only GUI for teaching one tray profile and inspecting the
best detected tray pose in `base_link`. Headless `tray_detect` shares its fresh
pose-request pipeline. Controller placement and placement GUI integration remain
subsequent work. No camera/robot process is launched and no
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

Load the camera's schema-7 calibration from `calibration/` to enable metric
geometry; this also fills/connects its recorded prefix. A different manually
connected prefix permits detection but cannot reuse that calibration. Reconnecting
to another prefix invalidates the plane and pose; no calibration remapping occurs.
Fixed and on-hand cameras are supported. On-hand measurements require fresh
RGB-time `base_link <- Link6` TF. A shared 100 ms background TF wait preserves
the exact image timestamp and rechecks freshness. Missing TF blocks geometry,
not RGB detection. No platform/bin artifacts or `.env` changes are needed.

1. Load a trusted local YOLO `.pt` model to enable up to 1 Hz preview when RGB is
   ready. **YOLO Detect ON/OFF** controls inference; OFF retains RGB/depth/voxel
   preview. Show all model classes under the current confidence, IoU and detection
   cap; checked classes determine pose eligibility. Defaults are 0.25 / 0.70 / 100
   and internal inference size 640; loaded profiles retain their inference size.
   Edits apply after 300 ms without typing. Invalid inference values pause YOLO
   with an inline reason; correction resumes it, with no old-value fallback.
   Segmentation masks and OBB models support metric poses; box-only models remain
   detection previews and cannot produce a saved tray profile.
2. Leave **Length**, **Width** and **Tolerance** blank to measure first. Neither
   these filters nor a copied teaching position is needed to capture the plane
   or inspect measured trays. Before saving the complete profile, load an Item
   Teach YAML from `offline_teach/item_teach/` to copy its six Home joints into
   **Tray Teach Position**. The paired Item model is verified but
   never executed. Position copying does not move the robot. Position the robot
   with the existing authorized motion workflow if needed. Controller Home will
   continue to come from the controller's own Item Teach file.
3. With the tray uncovered, select **Capture 4-corner snapshot…** in the sidebar's
   **Reference Plane — teach file** section. A nonmodal captured-image editor
   opens while the main RGB/depth views continue updating. Click four distinct
   corners on its RGB image in any order, use **Undo corner** if needed, then
   **Create reference plane**. Both actions are also available inside the editor.
   Click coordinates account for scaling and letterboxing. All clicks use the
   same initially fresh synchronized RGB/depth/TF observation; live frames never
   replace that corner source. Capturing leaves the existing plane active until
   Create succeeds. Closing the editor discards its unfinished corner selection.
   Source/settings changes discard the draft and require another capture.
   A 7×7 registered-depth patch per corner requires at least 30 valid samples
   between 200 and 1000 mm after MAD filtering. Separate RGB/depth distortion is
   preserved. Four convex, noncollinear base-frame points must fit a plane with
   maximum residual at most 5 mm; bad samples are refused, never filled in.
   Numbered corner locations, accepted-sample counts and median depth appear in
   both captured panes; sampled pixels mark accepted values black and rejected values red,
   mapped with each pane's distortion model.
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
   Main views stay live; the last clicked observation is labelled with its age.
   Its highlight disappears on the next frame, and settings changes invalidate
   the summary. There is no Resume Live button or persistent main-view freeze.
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
5. **Save Tray Teach…** at the top right writes a new same-stem YAML/model pair to
   `offline_teach/tray_teach/tray_teach_<name>_<UTC_TIMESTAMP>.yaml` and `.pt`.
   Existing pairs are never overwritten. The adjacent **Load Tray Teach…**
   reopens an existing profile for preview or further teaching; loading a tray
   is optional when creating a new profile. Explicit loading verifies schema,
   model hash and the bound camera calibration. The stored Tray Teach Position
   and reference plane are sufficient: the original Item Teach file/model is
   not needed to reopen, preview or re-teach the tray.

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

The preview measures all complete tray polygons, while only the single eligible
tray nearest image center supplies TF/service output. Dimensions, base XYZ and
observation age appear below the image; rejection reasons remain visible.
`base_link -> tray_teach_selected_tray` is a teaching-only TF published once per
selected RGB observation with its original timestamp. Changes, failures or
expiry stop publication; TF viewers can retain history until their timeout. It is not a
controller target or production detection service. There are no placement
areas, placement targets, controller Home, motion rates or I/O settings here.

Strict tray schema 1 stores detection settings, same-stem model hash, camera
calibration filename/hash, copied Tray Teach Position, the four base-frame
corner observations, plane transform/fit evidence, and `nearest_base_corner_v1`
origin convention. Distances are mm for dimension settings, metres for plane/
pose geometry and radians for taught joints. No images or live depth are saved.
Changing the camera calibration or intrinsics requires re-teaching the plane.

GUI form drafts are remembered automatically after a 300 ms typing pause and
flushed on orderly close to the single ignored `logs/tray_perception/last_session.json`.
This includes camera prefix, file choices, tray name, dimensions/tolerance,
confidence/IoU/detection cap, selected classes and inference size. Saving the draft
works with YOLO OFF, no loaded inputs, incomplete dimensions and invalid text;
the restored text must still pass normal validation before use. File dialogs
preselect remembered choices, and saved class IDs are checked against the model
on explicit load. Startup restores only the form: YOLO and Armed remain OFF,
with no automatic model, camera, teaching position or reference-plane loading.
Use **Load Tray Teach…** to restore a saved plane and teaching position.
Session schema 2 explicitly imports validated schema-1 sessions and replaces
them on the next draft save. Malformed or unknown formats fail explicitly;
write failures are reported and preserve the previous session.
Events are timestamped and capped at 1000 in the package's `events.jsonl`.
An arbitrary preview prefix is session-only; loading calibration or a saved tray
restores its calibrated prefix. Incomplete drafts never change the strict
schema-1 saved profile or its unchanged YAML/model pairing and save prerequisites.

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
cloud while the node runs. The existing single selected-tray TF remains; no
additional tray targets or numeric RViz overlays are published.

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
Armed OFF; Armed ON is red and advertises `/tray_detect/get_tray_pose` using
[`tray_perception_interfaces/srv/GetTrayPose`](../tray_perception_interfaces/README.md).
Arming requires YOLO ON with valid settings, an explicitly saved/loaded unchanged
profile whose geometry/plane/position match the form, its verified mask/OBB model,
matching calibration and fresh RGB/CameraInfo/exact-time TF. It needs no live
depth, Item Teach, controller process or motion. Settings/source changes and YOLO
OFF disarm immediately; changed files require explicit reload. Only one provider
may advertise the canonical service. Logical revocation is immediate; retiring
the ROS handle waits for an active callback's reply handoff to avoid destroying
the service while rclpy sends its cancellation response. Re-arming retires the old
endpoint before creating its replacement; old callbacks cannot use the new binding.

**Simulate Trigger** runs the same acquisition, inference, size/class filtering,
single-tray selection and typed response logic locally, even while Armed OFF.
It never enables the robot or requests controller motion. It requires the exact
saved profile and YOLO ON. The exact request result is displayed and preview
continues automatically, with a separate age-labelled last-request summary.
Invalidated simulations cannot restore or retain old targets. Inspection and
corner teaching also leave the main preview scheduled. The existing single
worker serializes inference and explicit actions, so processing can briefly
delay a refresh; no extra inference worker or camera subscription is added.

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
root `runtime_teach/`. Keep the exact camera calibration named/hash-bound by the
YAML under root `calibration/`. No Item/Bin Teach file is needed by Tray Detect;
its complete pair may coexist with their files in the shared deployment folder.
The Item/controller catalog still requires its own Item pair and Bin YAML, and
now allows a complete optional tray pair. Missing/duplicate tray files, mismatched
stems, unknown prefixes, unsupported extensions and symlinks fail visibly.

```bash
source scripts/source_ros_workspace.bash
ros2 launch tray_perception tray_detect.launch.py
```

Starting this dedicated read-only process explicitly trusts the deployed model.
It loads the pair once, connects the profile's calibrated prefix, waits boundedly
for fresh calibrated RGB/TF, and arms automatically. There are no file/trust/arming
launch arguments, latest-calibration search, `.env` keys or automatic deployment.
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
