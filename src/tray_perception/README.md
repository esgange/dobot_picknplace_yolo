# Tray Perception

`tray_teach` is a read-only GUI for teaching one tray profile and inspecting the
best detected tray pose in `base_link`. This phase implements teaching only;
headless `tray_detect`, its controller service, placement and controller GUI
integration are subsequent work. No camera/robot process is launched and no
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
2. Load an Item Teach YAML from `offline_teach/item_teach/` to copy its six Home
   joints into **Tray Teach Position**. The paired Item model is verified but
   never executed. Position copying does not move the robot. Position the robot
   with the existing authorized motion workflow if needed. Controller Home will
   continue to come from the controller's own Item Teach file.
3. With the tray uncovered, select **Freeze for 4 corners**. Click four distinct
   corners on the tray reference surface in any order and create the reference
   plane. Click coordinates account for scaling and letterboxing. All clicks use
   the same frozen, initially fresh synchronized RGB/depth/TF observation.
   A 7×7 registered-depth patch per corner requires at least 30 valid samples
   between 200 and 1000 mm after MAD filtering. Separate RGB/depth distortion is
   preserved. Four convex, noncollinear base-frame points must fit a plane with
   maximum residual at most 5 mm; bad samples are refused, never filled in.
   Numbered corner locations, accepted-sample counts and median depth appear in
   both panes; sampled pixels mark accepted values black and rejected values red,
   mapped with each pane's distortion model.
4. Click a displayed tray to freeze and inspect its measured size and acceptance
   reason; click again to resume. This is separate from four-corner teaching.
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
edges into the tray. Z follows the taught plane normal toward the teaching
camera; X/Y edge order preserves a right-handed frame. X is not required to be
the long edge. Image left/right is irrelevant. Exact distance ties use base XYZ
ordering. A symmetric unmarked tray has no tracked physical-corner identity;
the origin can switch when another corner becomes nearest the base.

The preview marks the selected origin in cyan with red +X and green +Y. Dimensions, base XYZ and
snapshot age appear below the image; rejection reasons are in its tooltip.
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

Validated GUI fields are restored as unapplied prefill from the single ignored
`logs/tray_perception/last_session.json`. Startup never loads weights, connects
cameras or restores a plane automatically. A malformed state fails explicitly.
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
