# item_perception_yolo

`item_perception_yolo` is the project perception package. Its aligned local-only
teaching GUIs establish the robot pick-area origin and then the four-corner bin
ROI used by later item detection.

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

1. Browse for a trusted pretrained `.pt` anywhere on the PC, then explicitly
   select **Load Model / Read Classes**. PyTorch weights can execute code; the
   confirmation is mandatory. One package-private CPU worker loads the model,
   reports its actual task and lists **one checkbox per class ID/name**. Check
   only production classes to accept; no implicit production all-classes selection. A new model starts
   with unchecked classes. A saved profile restores selected IDs as unverified
   prefill until the model is explicitly loaded.
   Model loading takes priority over the automatic bin-border/inference preview:
   the current operation finishes, then the confirmed load runs once in the
   same worker. The button reports queued/loading progress; no repeated clicks
   or YOLO activation are needed. Cancelling the trust question resumes previews
   without loading. Successful loading restores the bin preview but leaves YOLO
   and Armed OFF until explicitly enabled. Worker failures remain terminal.
2. Select mask or OBB from the model's available geometry outputs. Segmentation
   uses a minimum-area mask rectangle; OBB uses the model's oriented rectangle.
   If both outputs are observed, choose one explicitly. Box-only `detect` models
   can be previewed but cannot be armed as mask/OBB pose generators. There is no
   task conversion or alternate-model fallback.
3. Enter a prefix, **Connect RGB**, then **YOLO Detect ON**. There is one view;
   the Detect All/Filtered controls, dropdown and Resume Live button are removed.
   It displays every model class, including size failures. Home and a saved item
   profile are not needed for preview. Visible confidence, IoU and detection cap
   apply immediately after a typing pause. First-run fields start at 0.25,
   0.70 and 100; new profiles use internal size 640, loaded ones retain their size.
   Displaying all means detections under those settings, not every raw proposal.
   No platform/bin/depth, home or saved item file is required to see detections.
   OFF stops inference and shows raw RGB; it also removes the pose service.
4. Station calibration loads automatically from root `calibration/`: newest
   platform for `.env`'s `DOBOT_ROBOT_LAN1_IP`, then newest camera calibration
   matching that platform's camera prefix. Canonical filename UTC timestamps,
   not file modification times, define newest. Read-only **Latest platform**
   and **Camera calibration** fields show the selected files; no platform picker.
   The camera must match the platform's recorded SHA-256/mode/settings/transform.
   A newer calibration for that camera requires a newly taught platform. A
   newer file for another camera is not substituted. Invalid/missing/ambiguous
   selected files or unidentifiable catalog entries fail without an older-pair
   fallback. For dimensions/depth, select only the portable bin ROI. The GUI
   automatically validates the station/bin and connects the camera preview:
   **no Apply button and no additional Connect RGB click are needed**. The camera
   calibration/prefix comes only from that platform artifact and its hash-bound
   camera file. Existing platform schema 3, camera schema 7 and bin schema 3 are
   unchanged. The saved bin selection reconnects using the latest station pair
   on startup; an old platform prefill is not authoritative. This narrow
   read-only-preview exception to unapplied prefill never launches a camera,
   loads model weights, enables YOLO or arms the service. Incomplete/invalid files
   leave the ROI hidden with a status reason; validation runs on selection/startup,
   not repeatedly each timer tick or during pose requests. Use **Reload Latest
   Calibration** after teaching/correcting files, or reselect the bin, to revalidate.
   Reloading/changing selection stops YOLO, disarms and clears old overlays/TF.
5. **Click an item** to freeze that exact displayed RGB/depth result, highlight
   its pick dot with a cyan ring and show its measured **X / height (long side)** and
   **Y / width (short side)** in millimetres at the top-left. Mask uses its
   minimum-area pixel rectangle; OBB uses its oriented rectangle. Both use the
   same platform-Z=0 metric enclosing-rectangle calculation as production. No
   ROI, class, taught-size or depth filter excludes displayed measurements. The
   station's hash-validated camera, color CameraInfo and RGB-time internal TF
   are required; on-hand also needs RGB-time robot TF no older than one second.
   Unavailable geometry has an explicit reason, never guessed dimensions.
   A box-only model remains preview-only without mask/OBB metric measurements.
   The sole verified mask/OBB output is selected on Load Model; choose explicitly
   if both are available. Overlapping hits choose the smallest enclosing pixel
   rectangle, then confidence and frame-local index. Clicks in letterbox margins
   do nothing. Click the image again to release the frozen selection.
   No form values are overwritten. Registered depth is displayed alongside RGB;
   missing or mismatched depth has an explicit reason and blocks the pose, not RGB.
6. Enter measured length in `height`, short side in `width`, and choose the
   ±millimetre `tolerance`. A green item rectangle means size within tolerance;
   red means outside tolerance, gray means size not checked (missing measurements
   or dimensions). Green alone does not mean a valid 3D pose. Detections remain
   visible in all three cases. Fill class/geometry/quality settings to calculate
   a pose on click. Only the clicked item runs the strict class, size, complete
   ROI footprint, center-in-item and MAD depth checks. Accepted depth points are
   black, rejected points red inside the sampling circle. Blank/invalid required
   fields or failed checks show a reason and publish no selected pose.
   Confidence/IoU are 0–1 (e.g. 0.40/0.35, not 40/35);
   the editable `image_size` field has been removed. New profiles use 640 internally;
   loading a profile retains its exact validated saved inference size.
   Confidence/IoU/cap, class, size, quality and geometry-output edits update an already
   enabled preview after 300 ms without further typing, without an OFF/ON click.
   Pending or invalid values pause inference and show a nonmodal reason, never
   reuse old settings. Correcting them resumes preview automatically. Incomplete
   size fields explicitly disable size checking (gray), never supply guessed values.
   Edits clear old/frozen detections and discard in-flight/queued old-setting
   replies; they always disarm and invalidate saved-profile eligibility. Preview
   uses all classes; manual poses and production require checked classes.
   Other form edits clear the selection/TF and disarm without stopping detection.
   Save/load the exact complete
   item profile, then **Armed ON**
   to advertise `/item_detect/get_item_poses`. OFF removes the service. GUI and
   headless detector must not advertise it simultaneously. No robot motion is
   performed by either mode.

For a valid clicked pose, the top-left shows dimensions and platform-relative
XYZ/yaw. Item Teach broadcasts `base_link -> item_teach_selected_item` at 10 Hz,
composing the destination platform's full transform; it never flattens that plane
or creates a competing `platform_reference` TF publisher. In an independently
running RViz, add/use the TF display to inspect that child. This is a frozen
teaching snapshot, not live tracking or a commanded pick target. Its source age
remains visible; it persists until the image is clicked again, settings/source or
arming changes, YOLO OFF, native/source failure or exit. ROS TF clients may retain
previous transforms briefly in their buffers after publication stops.
Click calculation reuses the exact displayed geometry and synchronized raw
RGB/depth/TF snapshot, without re-running YOLO or substituting newer sensor data.
Input freshness checks apply when acquiring the snapshot. Once the exact pair is
accepted, it remains valid for that frozen click calculation; settings/source
changes still invalidate it, and the request deadline still bounds native work.
Only a bounded current/in-flight/selected snapshot is held in memory. The sole
persistence exception is an explicitly requested controller troubleshooting pair
described below; there is no continuous image archive.

### Simulate Trigger

The main row is **YOLO Detect ON/OFF | Simulate Trigger | Armed ON/OFF**.
Armed ON is highlighted red so advertised production pose-service state cannot
be mistaken for the unarmed teaching state; the color does not bypass validation.
Simulate Trigger is a one-shot action, available with Armed OFF or ON. It needs
a complete saved/loaded schema-7 profile, its verified model, matching current
settings, YOLO ON and the applied station/bin. Correct and save recovery drafts
first. It neither advertises/calls the pose service nor issues robot commands.

It uses the very same acquisition, inference, class/size/ROI/depth filtering,
center-first ranking and typed response builder as a real service request.
Acquire one new synchronized RGB/depth pair after the click, resolve RGB-time TF,
and enforce the same input freshness, request deadline and hash/generation
checks. One action queues behind the current GUI job, with progress shown on the
button; no repeated clicks or automatic retry. The deadline includes queue time.
Production/simulated requests are mutually exclusive and report BUSY on overlap.

The successful pair freezes on both views with **only returned candidates**,
ranked P1…Pn and capped by `retry.pose_candidates`. Retain their mask shading,
one green rectangle, X/Y axes, center dot, cyan metric sampling rings, black/red
accepted/rejected depth pixels, and the green loaded bin ROI. Rejected or excess
valid objects leave no overlays behind. Null depth is excluded before MAD; it
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

The complete batch replaces any previous clicked-item or simulated preview.
Validate the response frame, priorities, IDs, pose values, source/profile identity
and snapshot identity before installing all TFs atomically. While frozen, only their
broadcast timestamps refresh; their positions/orientations do not follow newer
images or robot TF. The timer independently checks source/profile, arming epoch,
YOLO and native/fatal state so invalidation stops publication even if Qt is busy.
SHORTAGE and NO_VALID_ITEMS are explicit successful outcomes; zero items freezes
just the pair/bin ROI and publishes no candidate frames. ROS/RViz can retain old
TF frames in their buffers until timeout/reset after publication stops.

Click RGB to cancel/resume; image margins and status bands do nothing. Settings,
station/profile/model/arming changes, YOLO OFF or failure invalidate pending and
frozen results and stop all teaching TFs. A failed request never displays or
publishes a previous batch as its result.
An armed real service remains independent and always obtains new observations,
even while a simulation is frozen. No images are persisted; batch metadata is
recorded only in the existing bounded package events.

Arming always validates and uses the production profile. Its service acquires a
new observation; it cannot
return teaching-preview detections or a frozen selection. Headless behavior,
strict production item schema 7, class filters and quality gates remain enforced.

### Pick-oriented RGB overlays

Segmentation shows mask shading and exactly one size-colored minimum-area rectangle
derived from the mask. OBB shows its native oriented rectangle instead. Do not
add the axis-aligned YOLO box or a second rectangle/per-box class label. Keep
rectangle geometry for measurement/click selection: show red **X** along its
long direction and green
**Y** along its short direction, spanning opposite edge midpoints through the
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

With the automatically selected station and selected bin validated, a green
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
`teaching_provenance.platform_calibration.sha256` in the bin file. A difference
shows a persistent amber **Bin/platform mismatch** warning below the selectors,
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
age-labelled. Loading a station automatically never starts a model.

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
**Load Item Teach** accepts only that directory. Complete schema-7 files load
normally and immediately count as saved, including startup named-file restoration.
No redundant Save is required before Simulate Trigger or manual Armed, but model
trust/verification, YOLO ON and fresh station inputs remain mandatory. Loading
does not arm, simulate or command anything. Older/partially invalid files open as
labelled GUI-only recovery drafts: keep independently valid fields, blank unclear/missing values,
and show unknown booleans as partial checkboxes requiring an explicit choice.
Unreadable/ambiguous YAML clears all fields; no partial parser guesses. Clear old
`retry_limit` counts are recovered as `pose_candidates` for editing only. Invalid
home records are cleared as a whole, never filled with zero joints. Unverified
paired weights leave the model field empty; browse a trusted model explicitly.
The warning/Activity log explains every cleared field. Missing internal
`image_size` requires explicitly browsing a model to establish new-profile 640.
Recovery also applies to named-file startup prefill, without executing weights.
No recovered draft can simulate, arm or be validated in the controller until
reviewed and saved as a strict schema-7 pair. Same known item name overwrites
the loaded file with its previous-version backup; changed/unknown original name
creates a new pair. Loading alone leaves files untouched. Shared
UI-state schema 6 remains strict; no recovered field autosave. Headless and
controller readers stay strict and never call the GUI recovery reader.
Its single confirmation now covers replacing the form/home and trusting the
paired `.pt` (weights can execute code). Loading the YAML automatically queues
that exact model and reads its classes—no second Load Model click. The existing
worker finishes its current preview and gives the load the next slot. Overlapping
loads are disabled. The pair's hash is checked before queuing/loading and after
inspection, along with saved task, class IDs and geometry support. Keep the saved
selection and fields; do not substitute classes, a task or another output.
Failures are visible and never retried. YOLO Detect and Armed stay OFF.
Startup form prefill remains weight-free; manually browsing a standalone model
still requires the separate explicit Load Model/trust action.

The YAML groups `item`, `model`, `units`, `home`, `motion`, `speed`, `acceleration`, `timing`, `gripper`,
`retry`, `yolo`, `geometry`, `geometry_source`, `quality`, and the non-executing
`controller_contract`. See
[`offline_teach/item_teach/README.md`](../../offline_teach/item_teach/README.md)
for field details. `retry.pose_candidates=3` requests up to three ranked poses
for the controller to use for retries; it does not execute any retry itself.
`yolo.max_detections=20` is the separate
per-frame detection cap before geometric filtering.
`use_grip=false` disables `grip_onpick` behavior regardless of its saved value.
Controller rule 57 now defines vertical Home-attitude height equations and
DI1-monitored final descent; teaching remains non-actuating.
The motion form has only standoff_height, prepick_height and retract_height.
Pick Z=item Z+standoff, pre-pick Z=pick Z+prepick and clearance Z=pre-pick Z+retract,
in robot base Z. zheight_offset is removed. GUI-only old-file recovery leaves
old retract_height blank because its reference changed; correction/Save is
required. Existing calibration/bin/shared UI schemas are unchanged.

The scrollable routine settings include three editable speed and acceleration
percentages: travel/Home, final approach, pick-to-prepick retract. All must be
integers 1–100. New-profile speed is explicitly 100/6/6 and acceleration
100/100/100; loaded profiles retain their exact values. Speed and acceleration
edits disarm and invalidate saved eligibility without interrupting read-only
inference or automatically saving/commanding hardware. Save writes schema 7 with
percentage units and separate groups, both using `travel_percent`,
`approach_percent`, `retract_percent`. Controller supplies each motion's `v=`/`a=`;
global SpeedFactor starts at 100% and the controller can adjust it explicitly
while Live/idle, without rewriting these taught per-command rates. Production
rejects schemas 1–6; old GUI recovery drafts leave missing/invalid rates blank
until the operator explicitly fills and saves them. Shared schema-6 named-file
UI state is unchanged.

Item Teach has no controller-validation button or controller client. It creates
profiles, inspects detections and exposes read-only poses when explicitly armed;
it never selects a controller profile or sends a controller request. Configure
`robot_controller` separately in its GUI/explicit launch parameters or headlessly
from `runtime_teach/`. It validates its selected pair itself; default mode is
TF-only debug and explicit real mode implements Home/pick/I/O. See its README
for initialization and safety requirements; teaching never requests motion.

Item Save/Load records the selected artifact filename in shared strict schema-6
UI state, alongside explicit preview prefix and applied station/bin filenames.
Restart reads validated fields and treats a complete valid item profile as saved
without enabling execution. It never sends a controller profile, replays joints,
loads native weights, arms a service, restores an external model path or keeps
duplicate profile settings. Read-only station/RGB preview restoration follows
the validated automatic bin-ROI workflow described above.
Unsaved text-box edits are **not** autosaved. Only fields in the selected saved
teach YAML return on restart. Live/frozen images, clicked measurements, stage,
model execution and arming state are never restored or written as image files.

### Pose generation contract

The Item Teach window uses one scrollable settings column, ordered camera/station,
model, live YOLO, dimensions/depth, quality limits, then saved routine fields.
RGB and native registered-depth views are side by side in a horizontal draggable
splitter. Both carry mask shading, green/red/gray size borders, centered axes/dots,
and loaded bin ROI. Depth geometry is projected through its own CameraInfo;
straight RGB edges are sampled before projection to handle differing distortion.
Both views freeze on the exact displayed pair when clicked, and only that item
gets a pose calculation (no second inference or newer depth). All other outlines
stay visible on frozen depth; the sampling circle is cyan, accepted samples black,
rejected red. Result/frozen status, inference settings, dimensions/pose and source
ages appear in a wrapping black status band at the top of each pane, below its
heading. They no longer obscure camera pixels or scale down with the image.
Masks, axes, bin borders and sampling circles stay on the images. Only image
clicks select/resume, not status-band clicks; letterbox mapping remains unchanged.
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
`GetItemPoses(max_candidates, profile_sha256, save_debug_images)`. One request
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
- Project the selected rectangle onto the fixed platform XY plane, Z=0. Under
  perspective its projection can be a quadrilateral: measure its minimum-area
  metric enclosing rectangle. Long side is X/`height`, short side Y/`width`,
  each within taught value plus/minus `tolerance` in mm. Do not move this plane
  to the item's depth. Require the selected footprint inside the bin ROI.
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
  equal to the median pass. Minimum sample count/fraction still apply. Median
  retained camera depth back-projects the exact center pixel, then the complete
  3D point is transformed into `platform_reference`. Never append optical depth
  to platform-plane XY. Depth Z does not alter projected dimensions.
- RGB shows the loaded ROI, mask shading (mask source only), a single mask-derived
  rectangle or native OBB, centered pick axes/dots, size, rejection reasons and
  priority labels. There is no extra axis-aligned YOLO box. In Filtered mode,
  only valid pick candidates receive the rectangle and pick axes/dot.
  Depth shows **accepted pixels black**, **rejected pixels red**, sampling
  boundaries, counts, filtered camera depth, platform Z and frame age.
- Rank valid positions by XY distance to the bin polygon's area centroid;
  confidence descending then source index break exact distance ties. Return up
  to the requested number, explicitly reporting SHORTAGE or NO_VALID_ITEMS.
  Heading is platform-normal and follows the rectangle's long axis with a
  deterministic sign; it is not estimated surface tilt or a TCP command.
- Replies carry metre poses/dimensions, timestamps, batch-local IDs, confidence,
  depth counts/spread and artifact/transform evidence. Disarm/config changes,
  source tampering, invalidated observations and malformed worker output never
  return usable targets. The controller checks frame/profile and timestamp
  validity again.

Initial form values are explicit and saved in `quality`: input age 0.5 s,
RGB/depth separation 0.1 s, robot TF age 1 s, request deadline 10 s, at least
30 accepted samples and 50% of circle pixels, and depth 200–1000 mm. There is no
result-age field: an accepted batch remains valid until invalidated or replaced.
Edit and save these fields deliberately; a file missing them is rejected.
Frames/overlays are bounded transient memory, never saved as an accumulating
archive. Events remain timestamped and bounded at 1,000 package records.

### Headless detection and controller request

Station files are selected automatically by the same strict rule as Item Teach.
Use absolute item/bin artifact paths on the destination machine:

```bash
ros2 launch item_perception_yolo item_detect.launch.py \
  item_teach_file:=/path/to/workspace/offline_teach/item_teach/item_teach_NAME_TIMESTAMP.yaml \
  bin_teach_file:=/path/to/workspace/offline_teach/bin_teach/bin_teach_TIMESTAMP_IP.yaml \
  trusted_model:=true armed:=true
```

Headless has no `platform_teach_file` argument. Startup requires explicit
item/bin paths, trust and arming, validates the latest station pair and same
model/settings/sources, and waits only the taught request deadline for inputs.
It advertises the same service and executes inference only on a request. No
independent detector algorithm or UI-state auto-application is used.
Restart to select newly taught calibrations; it never switches station files in
flight. Platform/Bin Teach keep their explicit calibration-selection workflows.
Automatic flat `runtime_teach/` loading and runtime/debug modes are not yet
implemented by this launch command.

After explicitly loading the same item profile in the separate controller:

```bash
ros2 service call /robot_controller/request_item_poses std_srvs/srv/Trigger '{}'
```

This read-only diagnostic request uses the profile's `pose_candidates` as pose count,
logs/returns the ranked batch and **does not move the robot**. Motion/gripper
execution belongs only to explicit controller real-mode actions; migration of
existing legacy command clients remains pending. Shared strict item/bin readers
accept flat `runtime_teach/` only via the controller's explicit deployment path;
GUI teaching save/load directories, detector explicit-path workflow and artifact
schemas remain unchanged.

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
strict schema-3 artifact stores:

- `base_link <- platform_reference`;
- the exact `charuco_pick_corner_xy_v1` reference convention: ChArUco board
  origin at the shared pick-area corner, unchanged board axes, metre units,
  and platform-plane Z=0 for the bin footprint;
- the selected schema-7 camera calibration filename and SHA-256;
- the camera prefix, frames, and RGB topics;
- the inherited ChArUco geometry and detector provenance;
- the selected mounting mode and complete resolved transform chain;
- for camera-on-hand, the exact `base_link <- Link6` transform/timestamp used;
- the source frame sequence/timestamp and detected corner count.

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

`bin_teach` accepts only a strict schema-3
`platform_calibration_<timestamp>_<robot_ip>.yaml` selected directly from root
`calibration/`. It validates the platform file, the camera calibration named
inside it, their matching mode, both SHA-256 values, and the LAN1 robot
identity from root `.env`. It inherits the camera prefix, RGB topic,
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

This planar teaching correction keeps platform/bin schema 3, camera schema 7,
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
schema-3 platform calibration and current marker settings, then select
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

Both teaching artifacts now require schema 3. Existing schema-1/2 files remain
untouched and are rejected; re-teach the platform and bin to produce the new
files. Camera calibration remains strict schema 7. Shared UI prefill is now
schema 6 to include Item Teach's profile, preview prefix and station/bin
filenames; the existing platform
and bin form fields are unchanged. No runtime migration or compatibility reader
is provided. This workstation's validated schema-5 prefill was explicitly
rewritten once for this update, preserving both existing teaching sections.

Platform/bin nodes create teaching artifacts and provide operator previews only.
Item detection now explicitly loads those artifacts under the separate contract
above; physical picking remains unimplemented. It must not depend on either
platform/bin teaching GUI or its preview TF staying
alive. To avoid duplicate platform previews, close Platform Teach after saving
before previewing the selected platform through Bin Teach.
