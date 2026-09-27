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

Bring up the configured camera and calibrated TF sources separately. Load its
schema-7 camera calibration from `calibration/`; its prefix determines the RGB,
registered-depth and CameraInfo subscriptions. Fixed and on-hand cameras are
supported. On-hand observations require fresh RGB-time `base_link <- Link6` TF.
No platform/bin/item station calibration selection or `.env` edit is needed.

1. Load a trusted local YOLO `.pt` model. Select tray classes and enter a name,
   expected long-side length, short-side width, and absolute tolerance in mm.
   Segmentation masks and OBB models support metric tray poses. Box-only models
   are preview-only and cannot produce a saved tray profile.
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
4. **Apply & Preview** detects trays at up to 1 Hz. Measurements intersect model
   polygons with the saved reference plane, fit a metric enclosing rectangle,
   and check both dimensions against the tolerance. Live depth is not used for
   subsequent measurements, so items on top cannot change the reference height.
   A clipped tray is rejected. Rank valid trays by image-center distance,
   confidence and source index; select exactly one or report no valid tray.
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

The preview marks the selected origin in cyan with red +X and green +Y. Green
outlines pass dimensions; red outlines are rejected. Dimensions, base XYZ and
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

The package reuses Item Perception's existing pinned private CPU runtime via
one lifetime native worker, without another wheel extraction, global install,
network access, worker restart or alternate model/runtime. ROS/Qt never imports
OpenCV, Torch or Ultralytics. A single GUI job slot prevents queued inference;
two ROS executor threads keep TF and camera reception independent of native work.
Native/protocol failures are terminal. See [NOTICE.md](NOTICE.md) for attribution.
