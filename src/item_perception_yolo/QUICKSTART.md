# Quick start

The three installed teaching runtimes are:

```bash
ros2 launch item_perception_yolo platform_teach.launch.py
ros2 launch item_perception_yolo bin_teach.launch.py
ros2 launch item_perception_yolo item_teach.launch.py
```

Run `platform_teach` first and save its calibration artifact. `bin_teach`
requires that artifact before it can use four 5x5 ArUco markers to capture the
bin ROI. Both workflows accept the platform artifact's explicit fixed-camera
or on-hand camera mode; on-hand use requires fresh live `base_link <- Link6`
TF. See [`README.md`](README.md) for the strict input, visualization, logging,
and output contracts.

Item Teach is a non-actuating profile editor and pose detector. Select `.pt` anywhere,
enter the grouped settings, record six actual home joints from live bringup,
and save a YAML/model copy pair under `offline_teach/item_teach/`. Home joints
are portable; source IP is provenance only. Launch `robot_controller` separately
to validate the saved pair. Neither profile validation nor loading moves the
robot. For initial testing, Load Model and Connect RGB, then toggle YOLO ON.
There is one view, without Detect All/Filtered controls or Resume Live button.
Select station/bin files and mask/OBB, then click a detection to freeze it and
read long X / height and short Y / width in mm at top-left. Enter dimensions and
tolerance: green borders pass size, red fail size, gray means not checked.
Clicking calculates only that item's RGB/depth pose with the strict class/ROI/size
and MAD depth checks. A valid pose shows platform XYZ/yaw and publishes teaching-only
`base_link -> item_teach_selected_item` for RViz's TF display. No RViz launch or
robot movement occurs. Failed or invalidated clicks show a reason without a new TF.
Click the image again to resume and clear the frozen pose/TF. The depth view shows
accepted points black and rejected red. Missing registered depth blocks poses,
not RGB detection. Preview uses visible confidence/IoU/cap and all model classes.
YOLO/size/quality edits update an enabled preview after a 300 ms typing pause;
invalid inputs pause it until corrected. Edits discard old results and disarm;
YOLO/model loading and re-arming remain explicit. Unsaved fields are not autosaved.
Item overlays keep mask shading, one mask-derived rectangle (or native OBB),
long-X/short-Y centered axes and a pick dot, with no extra axis-aligned YOLO box.
The green bin border coexists with detections and also works with YOLO OFF.
Station calibration is selected automatically: newest platform for the configured
robot, then newest camera calibration for its prefix, by filename UTC timestamps.
Their hashes/mode/settings must match; recalibrating the camera requires platform
reteaching, not mixing transforms or falling back. Select the bin file to connect
preview; the border appears when RGB/CameraInfo/TF are available. There is no
platform picker or Apply button. **Reload Latest Calibration** reselects files
and clears old overlays/disarms; no periodic watcher. The restored bin selection
uses the latest station pair on startup. YOLO/model loading and arming stay manual,
and no camera process is launched.
Item Teach shares Bin Teach's saved-plane border geometry. On another station,
provide that station's own latest platform/camera files and select the copied bin
YAML: the unchanged metric XY follows its platform origin, axes, tilt and height. No markers or depth
are needed for the border. Keep the same physical bin size/offset/reference axes;
loading does not detect, reposition or resize the bin.
Slow inference results stay visible on their source RGB, explicitly labelled
as stale result snapshots with frame age and inference time; service freshness
checks stay strict. `image_size`
is no longer an editable field; new profiles use 640 internally and loading
preserves the exact saved value.
Armed exposes the pose service only for an exact saved profile with valid inputs.
The main row now includes **Simulate Trigger** between YOLO Detect and Armed.
After saving a complete profile and enabling YOLO, use it to freeze a fresh
RGB/depth pair showing only the ranked candidates a request would return, capped
by `pose_candidates`, with the bin border. It uses production filters, works
with Armed OFF and never commands the robot. SHORTAGE/NO_VALID_ITEMS are explicit;
click RGB again to resume. The armed service never returns a frozen preview batch.
See the README for headless `item_detect.launch.py`, quality limits and the
read-only controller request. Physical pick execution remains pending.

Platform and bin outputs require schema 3; camera calibration remains schema 7.
Preserve existing older files and re-teach to create the new format.

Bin Teach saves/loads teaching files only in root `offline_teach/bin_teach/`.
Camera and platform calibration files remain in `calibration/`.

To reuse bin geometry at another station, copy its bin YAML into root
`offline_teach/bin_teach/`, Apply that station's own platform calibration in Bin Teach,
and choose **Load Bin ROI**. Confirm matching physical origin, board axes, bin
size and bin offset. A different absolute station height is handled by the
station's platform transform. The original station's configuration and camera
files are not required. Loading previews existing geometry and does not create
a fresh capture or another file; select **Retake** before teaching new points.
The live camera view shows a green border labelled **Loaded Bin Teach** with
the filename; no visible markers are required. RGB/CameraInfo and camera TF
must be available, with fresh robot TF also required for an on-hand camera.
Missing or stale projection inputs hide the border with an explanation.

RViz is only an operator check for the teach nodes. Item detection independently
loads their files and does not use teaching-preview TF. Close
Platform Teach after saving before using Bin Teach's platform/corner preview.
