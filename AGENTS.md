# Project agent instructions

Item Teach and headless Item Detect provide read-only pose generation. Rule 64 defines the deterministic typed-action `robot_controller` v2 hardware authority and supersedes the historical combined GUI/controller lifecycle; migration of maintenance tools to controller clients remains pending.

Before making changes, read [`docs/WORKFLOW_RULES_BLUEPRINT_DIARY.md`](docs/WORKFLOW_RULES_BLUEPRINT_DIARY.md) and the root `README.md`. Treat the diary as the project handoff document when an agent or contributor changes.

For controller behavior changes, also read and update
[`docs/ROBOT_CONTROLLER_FSM.md`](docs/ROBOT_CONTROLLER_FSM.md) in the same change.
Regenerate its adjacent HTML/PDF visual exports whenever that document changes.

## Non-negotiable repository rules

Rule 32 is the superseding item-detection contract; historical stage-one text
in rule 31 describes the original scaffold, not the current inference feature.
Rule 37 supersedes rule 23's marker-PnP corner geometry for Bin Teach.
Rule 39 supersedes the two teaching modes with one RGB/depth view and clicked-pose TF preview.
Rule 40 ties the cyan selection ring to the metric depth-sampling diameter.
Rule 41 defines the visual-first layout and separate registered-depth distortion model.
Rule 42 combines explicit Item Teach loading with its verified paired model load.
Rule 43 makes teaching views horizontal, mirrors calibrated overlays onto depth, and puts status text in the panes' top black bands.
Rule 44 supersedes retry_limit with pose_candidates and permits GUI-only recovery drafts; production item schema is 4.
Rule 45 adds local Simulate Trigger using the real service's fresh candidate-batch pipeline.
Rule 46 warns about different bin-source and selected platform artifacts without blocking portability.
Rule 47 supersedes rule 45's no-batch-TF restriction with simulated teaching-only candidate frames.
Rule 48 treats valid loaded Item Teach profiles as saved and updates the loaded pair unless renamed.
Rule 49 removes Item Teach's controller-validation action and keeps controller configuration independent.
Rule 50 automatically selects the latest strictly bound station calibration for Item Teach/Detect only.
Rule 51 defines controller GUI/headless, immutable TF-only/real modes and the confirmed vertical pick contract.
Rule 52 adds strict schema-5 item speed/acceleration percentages and per-command motion rates.
Rule 53 makes Home's preliminary vertical-clearance segment conditional on current Link6 Z.
Rule 54 makes controller actions service-driven, replaces launch-time real mode with Live, and
returns every completed pick attempt through Home.
Rule 54 additionally requires another Stop or shutdown to cancel return recovery
without resuming motion/release; optional image saving never relaxes target freshness.
Rule 55 serializes controller command responses, names startup/readiness failures,
and stops on unanswered calls even when the startup step is otherwise best-effort.
Rule 56 adds explicit controller Enable Robot and bounded coherent final-feedback confirmation.
Rule 57 defines queued forward/return motion and motion-timed I/O, removes zheight_offset,
and advances production item profiles to schema 6.
Rule 57 also guards initial/missed vacuum OFF on DI1 remaining clear, including
late suction after the completed Home return.
Rule 58 adds a Live/idle-only global SpeedFactor slider/service without changing
taught per-command rates or the 100% initialization setting.
Rule 59 makes Live and Stop/Clear perform guarded Stop/conditional ClearError/
EnableRobot recovery and removes the separate GUI Enable Robot button; blocked
Home/Pick requests remain clickable only to show precise refusal and a prompt.
Rule 60 parses Dobot bringup's brace-only GetPose/InverseKin ROS result field,
not the full raw TCP reply; Home remains MovLIO joint mode and pick points
remain MovLIO Cartesian mode.
Rule 61 removes controller InverseKin preflight, keeps strict GetPose/FK and
taught-joint checks, and explicitly retains empty-I/O MovLIO requests despite
the vendor manual's unverified minimum-one-event wording.
Rule 62 caches taught-Home FK when the profile is loaded, removes live
GetPose/current-joint-FK equality, and defines separate joint/Cartesian arrival tolerances.
Rule 63 uses MovL for no-I/O linear targets and reserves MovLIO for targets
containing at least one real timed DO tuple, as required by the V4 protocol.
Rule 64 replaces controller Trigger/JSON/Live APIs with explicit typed lifecycle
services, native Home/Pick actions, a typed status topic, and separate hardware,
TF-preview, and GUI processes. Launch never starts or enables the robot.
Rule 65 adds explicit typed Pause and Continue queue controls while preserving
an unconditional direct Stop service; the GUI dynamically presents
Start/Continue and Pause/Stop without weakening external Stop pre-emption.
Rule 66 treats `isPauseCmdFlag` as contextual telemetry, not a general READY
gate; only a controller-confirmed Pause with retained context is resumable.
Rule 67 makes the global-speed GUI send the live slider position on release and
debounced keyboard edits without status refresh overwriting an in-progress edit.
Rule 73 removes `quality.result_max_age_sec`, advances production Item Teach to
schema 7, and keeps each accepted candidate batch valid for its owning Pick.
Rule 74 permits explicit GUI teach-configuration reload from idle, unheld READY,
then returns the controller to INACTIVE and requires another Startup.
Rule 75 aligns Link6 green/Y to each candidate's short-axis line using a
shortest modulo-180-degree spin around the unchanged taught Home tool Z.
Rule 76 makes each candidate attempt two explicit queues: Home-to-pick, then
pick-to-Home after terminal confirmation, with no intermediate arrival waits.
Rule 77 removes every per-motion `cp`/`r` override so all controller motion
inherits the strict Startup/Recover global `CP(100)` setting.
Rule 78 adds schema-8 `pick_rotation`, its legal offset choices, and removes
intermediate retry returns to Home.
Rule 79 makes every candidate orientation independently minimize from taught
Home; direct retry travel never makes rotation selection cumulative.
Rule 80 advances Item Teach to schema 9 with four optional inward bin-wall
clearances and applies the resulting light-blue polygon only to final pick points.
Rule 81 replaces complete-footprint containment with green-ROI overlap-or-touch
eligibility while retaining exact pick-point containment in green and light blue.
Rule 82 adds conservative projected-pixel containment so item-height parallax
cannot display or return a candidate pick point outside the allowed border.
Rule 83 adds Link6 robot-camera-origin clearance with a 180° tool-Z mirror
fallback; both unsafe attitudes exclude that pose before ranking and retries.
Rule 84 shortens every controller Dobot service-response deadline to two seconds
without changing five-second discovery or output-feedback deadlines.
Rule 85 dispatches each complete controller motion group before validating all
group replies and removes the additional final-pick suction-settling delay.
Rule 86 spaces adjacent motion-group service dispatches by at least 50 ms and
tracks commanded timed-output transitions without weakening held-item checks.
Rule 87 restores taught final-pick suction settling and queues each missed-pick
retract directly through the next pre-pick/final pick with timed release and
exhaust events; no per-motion CP override is permitted, and a Stop/DI1 during
group dispatch prevents later group commands.
Rule 88 makes every Home path confirm above-item clearance when applicable,
then a vertical rise to Home Z before joint Home; missed-pick retries rise to
the old item's approach height and transfer via the next approach. Motion
requests must receive acceptance in order before the next is sent, because
independent ROS services can reverse dashboard queue order.
Rule 89 supersedes historical GetPose-as-motion-origin requirements: the
controller uses the same fresh, stationary, advancing FeedInfo sample's
`tool_vector_actual` for current Link6 pose, keeps user/tool zero and all
existing safety gates, and has no GetPose client or separate ToolVectorActual
subscription.
Rule 90 makes the explicit Hardware Home action use two separately confirmed
Cartesian MovL targets: current XY with taught Home Z/attitude, then full
taught Home Cartesian pose. Pick's shared joint-Home return rule is unchanged.
Rule 91 adds the old item's pre-pick waypoint before its clearance on a missed
non-final Pick retry. Both old-item rises and the next item's clearance,
pre-pick, and final pick stay in one CP-blended group; final exhaustion is unchanged.
Rule 92 removes rule 86's 50 ms motion-service dispatch floor. Within each
motion group, send the next request immediately after the previous service
returns `res=0`, while retaining ordered admission, feedback and Stop gates.
Rule 93 queues explicit Hardware Home's alignment and final Cartesian targets
in one CP(100)-blended group and physically confirms only final Home. Pick's
shared clearance/Home-Z/joint-Home barriers remain separate.
Rule 94 defines mutually exclusive three-state vacuum/finger outputs and the
current pick/retry I/O sequence. OPEN/CLOSE and SUCK/EXHAUST always turn their
opposite output off first; a miss latches only after final-pose settling, and
later DI1 from that missed attempt cannot become a success or block its retry.
Rule 95 requires the extracted Item Perception YOLO/OpenCV runtime and Camera
Calibration OpenCV runtime to be copied as ordinary installed files even under
`colcon --symlink-install`. Build-tree symlinks inside either private runtime
are forbidden because OpenCV's binary loader resolves them into a recursive
`cv2` package import.
Rule 96 restores a five-second controller Dobot service-response deadline and
keeps motion-group completion callbacks outside the group response lock so
accepted multi-command groups cannot starve feedback or later replies.
Rule 97 makes the profile's `timing.pick_settling` the one final-pick
stationary/queue-idle and DI1 observation interval, with no preceding 300 ms
pick gate or later suction wait.
Rule 98 removes the fixed 300 ms duration from every non-pick motion endpoint,
motion origin, Home skip, Stop and Pause confirmation; only final pick uses a
timed settling interval.
Rule 99 queues a final exhausted miss through retract, clearance, conditional
Home-Z and exact joint Home as one ordered group with only exact Home physically
confirmed. Explicit Hardware Home reuses its planning pose as the group origin.
Rule 100 makes headless Item Detect and headless Robot Controller share the
strict filename-prefix catalog in flat `runtime_teach/`; Item Detect has no
artifact, trust or arming launch arguments and advertises its read-only service
after the deployed sources and fresh inputs validate.
Rule 101 gives each missing or duplicate runtime Item YAML, Item model and Bin
YAML its own fatal startup error, with duplicate filenames listed; deployment
must finish before process start and remains manual or externally managed.
Rule 102 replaces retained-queue Pause/Continue with Stop, candidate-ledger
parking and replanning. Unheld Pick parks at the next pending pre-pick; held
Pause rises vertically to Home Z. A paused drop still returns to the original
pick's +50 mm release pose, pulses exhaust for 50 ms, retreats neutral and Homes,
then remains paused. Explicit return ends READY; direct Stop always pre-empts.
Pre-pick must be at least 50 mm above pick and clearance above the release pose;
neither source context nor candidate states survive process restart.
Rule 103 moves unheld Pick's Pause endpoint to `park_transit` above the next
pending candidate at safety Z (max of stopped Z and taught Home Z). Continue
opens fingers, then descends through pre-pick to final pick.
Rule 104 skips the shared joint-Home preliminary rise when current Z is within
5 mm below taught Home Z or higher. Planning and first dispatch reuse one fresh
confirmed pose; larger deficits still require the unchanged-XY/attitude rise.
Rule 105 keeps an unheld paused candidate INTERRUPTED and eligible. Pause parks
above that same candidate at `park_transit`; Continue retries its saved approach
before later PENDING candidates and marks it ACTIVE on command acceptance.
Confirmed failed, dropped and returned candidates stay excluded.
Rule 106 adds the next candidate's `park_transit` position to each blended
non-final missed-pick retry, between old clearance and next clearance. OPEN
occurs at 50% of travel to that transit; only the next final pick is confirmed.
Rule 107 debounces DI1 HIGH-to-LOW for 50 ms across held-item checks, using one
shared feedback filter. HIGH resets the timer immediately; confirmed paused
loss stays latched. Acquisition, release/reset checks and all DO checks stay raw.
Rule 108 makes successful Pick returns use the exhausted-miss route, rates and
single group through joint Home. Held outputs and suction monitoring remain;
deferred finger CLOSE occurs at the end of the clearance rise.
Rule 109 latches held suction loss with its source, confirms physical Stop
independently of DI1, and makes explicit Recovery put back the uncertain item
before continuing eligible saved candidates, or Home when none remain. Outputs
remain protected; cold unknown items never receive inferred source context.
Rule 110 requires an entry and an explicit exit `park_transit` for each pick
and put-back route. Queue the old item's exit before the next item's entry at
a common safety Z, including retries and Recovery continuation; every item
return queues its exit before Home even at/near Home Z. CP(100) may blend both
transits, with no intermediate arrival wait. Preserve timed I/O and held checks.
Rule 111 handles confirmed held suction loss inside an active Pick automatically:
Stop and confirm containment, put back the saved item using the +50 mm release
and 50 ms exhaust, then continue eligible saved candidates or Home if exhausted.
The Pick action remains active; no Recovery click is needed for suction loss alone.
Keep both transits, the 50 ms loss debounce, strict output/feedback/response gates,
and direct Stop pre-emption. Paused loss still puts back and remains paused.
Rule 112 makes the complete item put-back route use speed 100% with taught
travel acceleration, including release approach, neutral retreat and Home.
Only a successful pick's first lift to pre-pick uses taught retract speed and
acceleration; empty retract and the clearance rise use speed 100% with travel
acceleration. Final pick retains taught approach rates; global SpeedFactor applies.
Rule 113 uses the exact saved pre-pick pose for every put-back release, removing
the fixed +50 mm offset and minimum pre-pick check. With zero retract height,
neutralize on the upward exit transit; clearance or safety Z must still provide
a real upward retreat. Preserve full-speed return, both transits and 50 ms exhaust.
Rule 114 starts the canonical read-only RViz viewer alongside Dobot bringup in
an owned child launch session. Viewer exit stops only that viewer and its TF
publisher; parent shutdown or driver exit stops the owned viewer. No automatic
restart, enable, motion, gripper command, or extra configuration is added.
Rule 115 places Robot status and Gripper status / Live I/O at the GUI's top
left, with compact teach-file loading at top right. Publish observed robot flags
and raw DI/DO through typed controller status; stale data displays UNKNOWN.
The GUI remains a controller client, with no direct Dobot or camera subscription.
Rule 116 simplifies that header to side-by-side robot-state and two-input LED
panels (DI1 suction, DI12 finger open), with compact teach loading on the right.
LEDs show HIGH/LOW/UNKNOWN; detailed messages remain in tooltips. Collapse the
command log by default while retaining its full bounded stream and Copy Log.
Rule 117 allows explicit Recover from HELD_UNKNOWN: unknown HIGH suction still
blocks enable/output reset and prompts item clearing or obstruction checks;
retry after fresh DI1 LOW requires no extra Stop click. Successful held recovery
explains Pause/Return Item. Pending managed requests immediately show STOP NOW,
matching their direct-Stop click behavior even when PAUSED status arrives first.
Rule 118 maintains docs/ROBOT_CONTROLLER_FSM.md as the visual description of
the implemented controller. Update its diagrams, state/guard tables and behavior
review baseline in the same change as affected controller behavior, alongside
the diary. Describe current code and superseding rules, not historical behavior.
Rule 119 keeps adjacent offline HTML/PDF visual FSM exports generated from the
Markdown document's diagrams and headings. Regenerate both with the documented
local renderer whenever the source document changes; never maintain a separate
behavior definition in the exports. No renderer is needed to view the exports.
Rule 120 requires advancing feedback after complete motion-group acceptance,
idle/empty queue and the actual terminal pose/I/O. MovL binds its returned queue
ID to FeedInfo currentCommandId; acceptance-only MovLIO/RelMovLUser require
latched live execution evidence. Preserve CP(100), both transits and no midpoint
waits; only final pick has taught settling. Optional safety rises within 5 mm
are skipped before dispatch using the actual pose.
Rule 121 retains put-back source, destination and APPROACH/RELEASING/RELEASED
progress across Stop. Explicit Recovery finishes interrupted return; confirmed
release is never repeated. Reconcile only issued output transitions, resume
retreat upward from actual pose, and retain strict DI1/output checks.
Rule 122 scopes Stop responses and physical confirmation to one attempt.
Concurrent callers may share an ongoing attempt; a later explicit Stop/cancel
gets a new attempt even after failure. Old results cannot finish a newer Stop
or operation, and operation startup cannot erase an in-progress Stop.
Rule 123 permits explicitly confirmed automatic camera recalibration through
the controller's ReplayCalibration action only, superseding rules 18/20's
no-replay restriction for that workflow. Load retains an ordered joint recipe;
Start collects fresh samples after guarded MovJ arrival and a timestamped
capture handshake. Require attended UNCONFIGURED/INACTIVE/READY, already enabled
idle user/tool zero, DI1 LOW and no held/retained item context. Use 20% joint
speed/acceleration, preserve outputs, keep direct Stop and bounded failures,
and leave the final pose unchanged. Save as New Calibration uses the existing
schema-7 timestamped writer; incomplete replay cannot save. No launch/load
motion, automatic enable, saved observation reuse, new config store or schema,
gripper reset, extra executor thread, auto retry or Home return is permitted.

Rule 124 supersedes rule 123's controller authority: automatic calibration is an
explicitly confirmed maintenance exception like Motion Debug/Gripper Diagnostics.
Camera Calibration sends CP/MovJ/Stop directly to canonical Dobot bringup; remove
the calibration action, capture service and CALIBRATING state from Robot Controller.
Require sole command ownership, already enabled/idle user/tool zero, DI1 LOW,
fresh advancing feedback, modeled joint/TCP arrival and post-arrival observations.
Preserve outputs, 20% joint rates, bounded failure/Stop confirmation and exactly
two ROS executor threads. Save as New Calibration prompts for an editable filename
inside calibration/, defaulting to the existing mode/timestamp rule. Never overwrite
existing files; cancellation changes nothing. Custom names are for explicit loading;
strict automatic station discovery retains its canonical filename contract.

Rule 125 supersedes calibration rules 123/124's immediate capture and no-retry
policy: every capture attempt requires a one-second stationary/idle hold and
post-hold RGB/joints/TF. Retry an unavailable observation automatically up to
three attempts per position; only after three failures show Continue/Stop.
Continue explicitly starts another three-attempt batch at that position and
preserves prior samples. Capture retries do not move again. Arrival-timeout
retries require confirmed Stop, all replies and fresh unchanged safe robot
state; prompts distinguish robot arrival from camera visibility. Faults,
unanswered/rejected commands, unconfirmed Stop and solver/runtime failures
remain terminal. Operator Stop always pre-empts and cannot be cleared by retry.

Rule 126 removes automatic calibration's nominal-model TCP arrival comparison.
Replay saved joints, confirm their returned queue ID, actual joint positions
and idle state, then retain the full one-second live TCP/joint stationary hold
before each fresh capture. Use the canonical CR10 model only for joint limits.
Transient RGB/CameraInfo readiness failures consume the same three-attempt
budget before/during motion and after retries; confirm Stop before retrying an
interrupted move. Continue appears only after three failures at that position.
Track validated RGB input before detection with a depth-one image subscription;
keep strict observation age, fatal worker handling and all robot safety gates.

Rule 127 lets confirmed Start Automatic Capture prepare the robot directly:
with fresh canonical joints/FeedInfo/RobotStatus, no competing command client,
fault-free user/tool zero, DI1 LOW and an empty queue, call StopDrag once if in
drag mode, then EnableRobot once if disabled. Require accepted responses and
advancing stationary enabled/idle feedback before CP or saved-joint motion.
Bound each response and feedback confirmation to five seconds; setup failure
ends the run through independent Stop, without retries or error/output resets.
Launch, loading and manual capture remain read-only. Never automatically
re-enable or leave drag again during an existing replay; a new Start is needed.

Rule 128 makes calibration hold loss recoverable within the current position's
three-attempt budget. Preserve strict feedback/I/O/ownership gates, confirm Stop
and all replies, discard only that attempt's unconfirmed sample/solution after
its callback finishes, then revisit the same saved joints and repeat the full
one-second hold before fresh capture. Preserve earlier samples and never reuse
sample IDs. Suppress unconfirmed solution TF/overlays. Three failures prompt
Continue/Stop; Stop, robot faults, solver/runtime failures and unconfirmed Stop
remain terminal. Log actual TCP/joint changes and idle state on hold loss.

Rule 129 makes explicit calibration Start send EnableRobot then StopDrag once
each after fresh canonical robot/joint feedback and command ownership checks,
regardless of initial mode. Nonzero setup replies are warnings; missing replies
remain terminal. Supersede rule 127's conditional order and initial state gate.
Confirm advancing stationary enabled/idle, fault-free empty-queue user/tool-zero
feedback, DI1 LOW and legal unchanged outputs before CP or saved-joint motion.
Report exact remaining blockers after five seconds. Keep Stop pre-emption,
feedback/output monitoring, no error resets and no setup retries mid-replay.

Rule 130 replaces calibration rule 129's EnableRobot/StopDrag pair with Motion
Debug's ordered startup on confirmed automatic Start: best-effort StopMoveJog,
best-effort DisableRobot/disabled confirmation, strict EnableRobot/enabled
confirmation, SpeedFactor 50%, Tool 0, Tool 1 TCP zero and CP 100%. Only those
first two commands may warn and continue on absence, rejection, exception or
timeout; missing Disabled confirmation also warns. All later steps are strict,
with five-second response/confirmation bounds and no automatic setup retry.
Keep fresh feedback, ownership, direct Stop and final stationary safe-state
confirmation before MovJ. Abandoned optional replies cannot block progression,
but late accepted setup commands require Stop containment. No launch-time
initialization, StopDrag, error/output reset or controller dependency is added.

Rule 131 replaces automatic latest-calibration discovery in Item Teach only
with explicit platform, bin-camera and robot-camera selectors. Load Calibration
validates the complete set, then atomically saves root-calibration basenames in
the mandatory ITEM_TEACH_PLATFORM_CALIBRATION, ITEM_TEACH_BIN_CAMERA_CALIBRATION
and ITEM_TEACH_ROBOT_CAMERA_CALIBRATION root .env keys. All three empty is the
first-run state; partial selections are invalid. Platform selection fills its
exact hash-bound camera; keep schema, robot identity and robot-camera mounting
checks. Custom mode-prefixed camera filenames are allowed for explicit loading.
Restore saved choices for the existing read-only station/bin preview on startup;
no catalog scan, newest-file substitution, automatic model execution or arming.
Changing a selection clears old preview and disarms; failed validation or dialog
cancellation never replaces saved choices. Recheck the exact selected files on
use. Headless Item Detect and Robot Controller keep strict latest discovery.

Rule 132 makes headless Item Detect load the same three saved calibration
filenames from root .env as Item Teach, superseding rules 50/83/131's latest
discovery for the detector. Require a complete valid selection at startup;
empty, missing, malformed or mismatched files fail visibly without scanning or
falling back. Reuse Item Teach's exact source/hash/mounting validation, including
custom mode-prefixed camera names, and pin those files until restart. Keep the
strict runtime_teach/ Item YAML/model/Bin YAML catalog and automatic YOLO/arming
after fresh-input validation. Inference remains request-driven and debug images
remain per-request. No config writes, watcher, hardware command or new key is
added. Robot Controller's independent calibration selection and hash checks
remain unchanged; controller and detector must still use matching artifacts.

Rule 133 adds default 1 Hz read-only Item Teach colored 5 mm scene voxels and
all valid detected-item poses/TF/markers with complete rejection diagnostics.
Explicit trusted model loading starts preview when ready; startup prefill never
executes weights and arming remains explicit. Use the same RGB/depth/TF snapshot
and existing private worker, one YOLO prediction per tick, no backlog or extra
executor. Keep original-resolution pose sampling and every existing pose gate;
only cloud visualization is voxelized. Include all candidates up to the YOLO
detection cap, without production pose_candidates truncation. Label snapshots
and age; source/settings/CameraInfo invalidation or stale inputs clear displays
and stop TF. Canonical RViz includes the optional cloud/marker displays and a
2.5-second TF timeout. Headless remains request-driven, creates no visualization
publishers, and saves images only on save_debug_images=true requests. No new
configuration key, schema, hardware command or automatic image archive is added.

Rule 134 increases Item Teach's display voxels and canonical RViz boxes to
10 mm. Keep the last valid cloud until a valid replacement arrives, including
input/synchronization gaps, busy processing and empty/unavailable observations;
never publish a transient empty cloud. Retain its original timestamp and report
retained_cloud age/reason while suspending candidate markers/TF until a newly
validated snapshot arrives. Source/settings/CameraInfo changes, terminal failures
and exit still clear the cloud. Keep 1 Hz, full-resolution pose sampling, all
existing acquisition/pose checks and headless behavior unchanged. This supersedes
rule 133's 5 mm size and transient-input cloud clearing only.

Rule 135 removes Item Teach's RViz item text markers (rank, class, confidence
and snapshot age). Keep colored voxels, pose-axis markers and TF; retain all
candidate details and ages in diagnostics. No detection, pose, snapshot-retention
or headless behavior changes. This supersedes rule 133's RViz text labels only.

Rule 136 keeps the latest RViz voxel cloud indefinitely with zero decay and a
reliable, transient-local, depth-one publisher/subscriber. After five seconds
without a new validated cloud, retain its geometry but turn it grey until a new
snapshot replaces it. Source/settings invalidation, terminal failure and orderly
exit grey it immediately instead of clearing it. Suspend candidate markers/TF
while waiting or grey; retain original source timestamps and report refresh age.
Repeated/frozen snapshots cannot reset that age or restore colors. Keep 10 mm
voxels, 1 Hz fresh updates and no text overlays; late viewers receive the cached
cloud while Item Teach runs. This supersedes rule 134's hard cloud clearing only;
retained visualization cannot satisfy a production pose request.

Rule 137 adds the separate read-only `tray_perception/tray_teach` GUI. Copy
Item Teach Home once into a saved Tray Teach Position; reopening the tray needs
no Item Teach file. Controller Home remains in Item Teach. Teach a base_link
reference plane from four explicitly clicked, frozen synchronized RGB/depth/TF
corners, then measure YOLO mask/OBB polygons on that plane without live depth.
Filter expected length/width by mm tolerance and select one valid tray nearest
image center. Origin is the rectangle corner nearest base_link by 3D distance;
both +X/+Y follow adjacent edges inward, with right-handed Z toward the teaching
camera. Save strict schema-1 YAML and a same-stem hashed model under
offline_teach/tray_teach/, including copied joints and bound camera/plane
evidence. No placement areas/targets, robot command clients or controller Home
are added. Headless detection and controller placement remain subsequent work.
Reuse the pinned private Item Perception CPU runtime through a separate lifetime
tray worker; no duplicate extraction or native imports in ROS/Qt. Restore only
unapplied fields from logs/tray_perception/last_session.json, keep bounded package
events, and require explicit trusted model loading and source validation.

Rule 138 keeps Tray Teach fields and controls usable throughout automatic
preview, including YOLO-off frames. Serialize explicit load/save/freeze/plane
actions behind the active preview with at most one pending action; lock edits
only for that explicit work. Bind previews to the generation at dispatch and
discard edited observations without hiding native failures. Freeze acquisition
and save validation run in the existing single background worker. Allow one
shared 100 ms wait for exact RGB-time TF, then recheck input freshness; missing
TF still rejects the observation. Keep the 1 Hz limit, two ROS executor threads,
read-only authority, strict artifacts and explicit Apply unchanged.

Rule 139 aligns Tray Teach with Item Teach's editable-prefix Connect RGB,
independent all-class RGB/YOLO preview, manual size/tolerance entry, 300 ms live
edits and horizontal RGB/depth inspection. This supersedes mandatory Apply in
rule 138; invalid inference fields pause without old-value reuse, while missing
geometry leaves size unchecked. Calibrated measurement requires a matching
prefix and the taught plane; changing the connected prefix invalidates geometry.
Add default 1 Hz 10 mm colored scene voxels on /tray_teach/voxel_cloud and
diagnostics, using the displayed RGB/depth/TF observation without another YOLO
prediction. Retain clouds indefinitely, grey after five seconds without fresh
data or immediately on invalidation/exit, and use reliable transient-local
depth-one transport with zero RViz decay. Depth loss cannot block plane-based
tray measurement. Keep one accepted tray TF, schema-1 paired saves, unapplied
session prefill, no new .env keys and no hardware or controller commands.

Rule 140 adds Tray Teach's Simulate Trigger and explicit red Armed control, plus
request-driven headless tray_detect. Both use /tray_detect/get_tray_pose with the
typed tray_perception_interfaces/GetTrayPose contract and the exact saved YAML
SHA-256. Use a fresh post-trigger RGB/exact-time TF observation and the saved plane,
never cached preview/voxel targets. Return one center-prioritized valid tray with
base pose, sorted size and inward X/Y extents in metres, or explicit no-tray/error.
Disarm edits/source changes; cancel obsolete requests and reject concurrent ones.
Keep a ten-second bound, one native worker and two ROS executor threads. Simulation
is local and read-only, may run disarmed, and freezes the exact result. Headless
loads one strict deployed runtime_teach/ tray pair and its bound calibration,
requires no Item/Bin input, and arms after readiness without continuous inference
or RViz publishers. The shared Item/controller catalog permits a complete optional
tray pair without loading it. Save images only on an explicit request flag; keep
schema 1, manual deployment, no new .env keys, no motion or placement integration.

Rule 141 makes Tray Teach remember form drafts independently of YOLO, model
loading or complete-profile validation. Atomically save after a 300 ms edit pause
and flush on orderly close to logs/tray_perception/last_session.json. Session
schema 2 stores unapplied text, including incomplete/invalid edits, file choices,
camera prefix, selected class IDs and inference size; it is not runtime authority.
Explicitly support validated schema-1 session import, writing schema 2 on the next
save; reject malformed/unknown formats. Restore fields only, with YOLO/Armed OFF
and no camera/model/plane/position loading. File dialogs preselect remembered
choices and explicit model loading verifies saved classes. Keep strict schema-1
tray artifacts, manual deployment and no new .env keys. This supersedes rules
19/137/139's complete validated-field requirement only for Tray Teach draft text.

Rule 142 keeps Tray Teach's main RGB/depth preview continuously scheduled,
including after inspection, simulation and during reference-plane teaching.
Remove Resume Live and the main-view freeze state. Group camera, tray/model,
detection, dimensions, teaching position and reference-plane controls in the
Item Teach-style sidebar; plane creation belongs to the teach artifact. Capture
one fresh synchronized RGB/depth/TF observation in a separate nonmodal corner
editor; its four corners and depth evidence never follow changing live frames.
Discard obsolete drafts/results on source/settings changes, retain the current
plane until successful Create and require Save Tray Teach to persist it. Keep
inspection/simulation summaries labelled as past observations with age, never
hold old highlights/TF instead of new preview data. Preserve the single worker,
bounded pending action, two executor threads, 1 Hz processing and strict schemas.
This supersedes rules 139/140's frozen main-view inspection/simulation only.

Rule 143 fixes detected tray axes to inward short-edge X and long-edge Y at the
rectangle corner nearest base_link in 3D. Z follows X cross Y and can face either
side of the reference plane, superseding rules 137/140's camera-facing detected
tray Z; keep the taught plane's existing normal and schema 1. Show plane-based
dimensions and red-X/green-Y edge overlays for all fully measurable mask/OBB
detections, including missing expected sizes, unchecked classes and tolerance
failures. Mirror geometry using each RGB/depth distortion model; live depth is
optional after plane teaching. Clicks report that detection's X/width, Y/length
and base corner without overwriting fields or freezing preview. Production gates
and single-tray ranking remain strict; service extents must be X=width/Y=length.
Exact equal-edge ties use adjacent endpoint base XYZ. No additional TF targets,
RViz numeric overlays, model executions, schema or hardware changes are added.

Rule 144 adds default up-to-1-Hz colored scene voxels to headless Tray Detect on
/tray_detect/voxel_cloud and status on /tray_detect/rviz_diagnostics, superseding
rule 140's no-headless-RViz-publishers restriction only. Use calibrated synchronized
RGB/depth/exact-time TF, the existing native worker and main loop; yield to pose
requests without queuing work or adding executor threads. YOLO remains exclusively
request-driven, with no background pose/TF or image saving. Retain 10 mm clouds
indefinitely, grey after five seconds without fresh data or immediately on
invalidation/failure/exit, and restore colors only with advancing RGB/depth stamps.
Missing depth blocks only cloud refresh, not plane-based pose requests. Add a
separate enabled canonical RViz display with reliable/transient-local depth-one
QoS and zero decay. Keep strict deployment, schemas and controller behavior.

Rule 145 permits reference-plane capture and tray measurement before copying
Item Teach Home or entering size filters. Plane capture still requires matching
calibration and one fresh synchronized RGB/depth/exact-time TF observation;
there is no motion or assumed plane. Explain missing calibration/TF/plane in
the preview and distinguish measured-but-unfiltered trays from accepted poses.
Blank/invalid dimensions keep measurement and click inspection available on
the taught plane. The copied Tray Teach Position and complete filters remain
mandatory for Save, arming and deployed profiles. This supersedes the previous
position-before-plane teaching order only; keep schema 1 and production gates.

Rule 146 uses Tray Teach's existing RGB/depth panes for four-corner capture,
superseding rule 142's separate editor. Hold one captured observation in those
panes during selection, label its age and return to live on Create/Cancel or
invalidation; background acquisition/preview/RViz continue through the same
worker. Draft corners are cyan. A created/loaded plane remains green with P1–P4
on live RGB and depth, even with YOLO off; label unsaved versus saved explicitly.
Use all valid depth samples remaining in each 7x7 patch, including one; remove
the 30-sample minimum. Zero valid samples still fail, and range/MAD/synchronization/
TF/plane-fit gates remain. Draw labelled 2D-only mask/OBB rectangle axes before
metric geometry is ready; never infer mm, a base origin or an accepted pose from
them. Preserve metric short-X/long-Y corner frames, save requirements, schema 1,
single native worker, two executor threads and read-only hardware authority.

Rule 147 permits explicit Tray Teach Save with only a valid tray name. Save named
GUI-only tray_teach_draft documents under offline_teach/tray_teach/ with unfinished
form text and any existing plane, copied position and bound calibration. Copy a
loaded model byte-for-byte to a same-stem hashed .pt; name-only drafts need no model.
Complete data saves the unchanged production tray_teach schema 1. Save updates the
loaded/saved file while its name is unchanged; renaming creates a new timestamped
file/pair. Match Item Teach's one hidden previous-version ZIP, source/target hash
checks, staged writes, YAML-last commit and rollback. Drafts reopen for editing but
cannot arm, simulate production requests or deploy. Save never runs model code,
writes runtime_teach or changes controller behavior. Preserve explicit model trust,
session-only startup restoration and existing worker/executor limits. This supersedes
rules 137/139/145/146's complete-profile save gate and new-pair-only saving; corner
edits still require Create before they are persisted as a plane.

Rule 148 automatically loads available selected calibration/model files in Item
Teach and Tray Teach at startup and after Browse. Remove Item Teach's separate
Load Calibration and Load Model buttons; Tray Teach's source selectors say Browse.
The selected/restored model path authorizes loading in the existing native worker,
without a separate model trust prompt. Retain paired hash/task/class validation,
saved class selections, bounded work, stale-result rejection and terminal native
failures. Missing sources do not block independent inputs; load once they become
available. Invalid existing files are not retried until reselected. Item calibration
choices validate together and only valid complete sets update existing .env keys;
startup restoration does not rewrite .env. Preview starts when ready; Armed remains
manual. Preserve teach-file save/load actions, strict artifacts and headless behavior.
No hardware commands, additional workers/executors or configuration stores are added.
This supersedes the explicit model-load/trust and unapplied calibration/model
restoration requirements in rules 19/42/131/133/137/139/141/147 only.

Rule 149 renders Item/Tray teaching RViz pose guides as unchanged red X/green Y
and a blue upward reference-plane normal. Choose the sign of the pose's local Z
whose base_link Z component is positive; an exactly horizontal normal stays
horizontal. These independent marker arrows are not a replacement coordinate
frame. Never modify candidate/service poses, quaternions, TF, origins, dimensions,
calibration, controller planning or saved artifacts for this display convention.
Cover live, clicked and simulated Item Teach poses and the selected Tray Teach
pose, with existing validation, invalidation and 2.5-second marker lifetime.
Canonical RViz enables separate blue-UP guide displays; its actual TF display
remains unchanged and can still show downward pose Z. Keep headless publishers
and request-driven inference unchanged; add no extra inference or numeric overlays.
This supersedes the blue marker direction only, not rules 133/137/143's real frames.

Rule 150 makes Tray Teach Position optional metadata, never an arming, simulation
or pose-request prerequisite. Complete schema-1 profiles retain that key with null
or a validated copied position. Loading saved detection data needs no extra Save
to arm. Explicitly loaded older GUI drafts may serve in Tray Teach when their
saved numeric settings, selected classes, verified paired mask/OBB model, camera
binding and reference plane validate fully. Build that detection view only from
saved fields and the verified model task; keep the original YAML/hash unchanged.
Never fill missing detection data from unsaved UI values. Require exact saved
settings/plane and sources, YOLO ON and fresh RGB/TF; no visible detection or robot
is needed. Headless still requires an explicitly saved production profile and
rejects draft artifacts. Preserve hashes, request freshness, single-provider and
disarm checks. This supersedes rules 137/140/145/147's required position and blanket
GUI draft rejection only; no controller behavior or hardware command changes.

Rule 151 names the tray robot observation position Tray Detect Pose and adds
Record Current Joints in Tray Teach, using Item Teach Home's feedback contract:
sole configured canonical /joint_states publisher, fresh receipt and ROS stamp
within one second, exactly six finite joints canonicalized to joint1 through
joint6. Display degrees/robot identity; save joint angles in radians and original
feedback/publisher evidence in schema 1's existing tray_teach_position field.
Keep Copy Home as an optional shortcut, confirm replacing recorded joints and
preserve the old pose on failure/cancel. Explicit Save persists the pose; loading
restores it. This describes the robot position for future controller travel before
tray detection, not a Cartesian tray/TCP pose. Keep it optional for detection and
arming. Headless reads saved metadata but never records joints or commands motion;
no controller motion, new schema/key/configuration or automatic file write is added.

Rule 152 removes Item Teach loading and Copy Home from Tray Teach. Tray Detect
Pose is recorded independently from current joints or restored from its tray file;
Home is a separate controller/Item Teach position. Remove the copy implementation
as well as the controls. Existing session/draft item_filename fields remain inert
for schema readability and new form saves leave them empty. Preserve recorded
joint angles, saved artifacts, pose requests and read-only authority. This
supersedes rules 137/151's Item Teach copy workflow only.

Rule 153 makes Tray Teach's RGB/depth panes independently resizable with a
horizontal splitter, matching Item Teach. Retain each pane's heading, aspect-fit
rendering and correct click mapping; live updates must not reset the divider.
Preserve rule 147's same-name Load/edit/Save overwrite and previous-version backup
for both drafts and complete profiles; a renamed tray creates a new pair.

Rule 154 sets canonical RViz TF Show Axes false so original downward item/tray
axes do not overlap the blue-UP marker guides. This hides all raw TF axes,
including robot-frame axes, but preserves RobotModel, TF data/display, timeout,
guide topics and every controller-facing pose. Raw axes remain an explicit viewer
toggle; already-open viewers must uncheck Show Axes or reload the config. Record
the intentional vendored viewer integration patch. This supersedes rule 149's
default raw-axis visibility only; never reflect or rotate real frames for display.

Rule 155 makes Tray Teach reopen its exact last loaded/saved tray filename on
startup, matching Item Teach. Use the existing session key and background Load
pipeline, including paired model/calibration verification, saved plane, optional
joint pose, settings and overwrite target. Support complete profiles and drafts;
saved file data takes precedence over unsaved session fields. Keep Armed OFF.
With no remembered file retain form/source restoration. Missing/invalid files
report once without newest-file substitution or unchecked source fallback; explicit
Load or Browse permits recovery. No artifact rewrite, headless change, new store,
worker, executor or hardware command. Supersede prefill-only tray-file restoration
in rules 137/139/141/148 only; retain all detection/request freshness checks.

Rule 156 sizes every Item/Tray RViz pose guide to robot-joint TF dimensions:
200 mm total length, 20 mm shaft diameter and 40 mm head diameter/length. Use
the shared marker helper for live, clicked, simulated items and the selected
tray; canonical TF Marker Scale is explicitly 1. Preserve blue-UP display
direction, actual poses/TF, marker lifetime and rule 154's raw-axis visibility.
This is display sizing only; no inference, controller or hardware change.

Rule 157 makes actual Item Teach/Detect poses and all item overlays short X /
long Y at the unchanged pick point, matching Tray's axis lengths. Rotate the
former item frame +90 degrees about its unchanged Z: X becomes old Y and Y
becomes negative old X. The shared controller/preview Link6 planner follows
short X, preserving physical tool attitudes, pick_rotation, camera clearance
and routes. Treat travel differences within 1e-12 radians as ties and keep the
CCW-before-CW offset preference. Schema-9 height/length and width remain unchanged. Require
GetItemPoses request pose_convention and response diagnostics to match
item_short_x_long_y_v1; reject missing/old conventions before candidate use.
Rebuild and restart perception/controller clients together. Preserve Tray's
corner origin, blue-UP guide policy and all robot gates; no hardware testing.


Rule 158 adds controller-owned Tray Detect Position and depth-based Place Item.
Configure binds an optional complete Tray Teach/model/camera and saved detect
joints. Positive tray-local X/Y and Rotation −180°…+180° are controller inputs;
zero attitude is the saved detect-pose tool frame with a local Z offset, independent
of item axes/pick rotation. One armed tray_teach or headless tray_detect provider
returns fresh after-trigger synchronized RGB/depth, bound sources and Item Teach
sampling diameter/quality. After observation travel, approach only pre-place →
release at sampled base Z + standoff; use taught rates/settling and retract heights.
Confirm OPEN/DI12, a single 50 ms exhaust pulse and DI1 LOW before upward retract;
record PLACED and finish READY there. Placement Pause stops in place. Continue
reobserves an interrupted approach, while release/retract recovery retains progress,
never repeats an issued pulse and never descends after release or returns to the
bin. Direct Stop remains pre-emptive. UI schema 3 explicitly imports old schema
1/2 filename selections as unapplied prefill and saves validated placement inputs.
Rebuild/restart changed tray/controller interfaces and clients together. Preserve
existing Pick/bin-camera avoidance and all teach/calibration artifacts. Verify
with synthetic services/feedback and isolated launches; no physical commissioning.


Rule 159 supersedes rule 158's placement settling, stationary release/pulse,
extra retract and finish location. Reach saved Tray Detect joints using fresh
idle/empty queue, joint tolerance and execution evidence, without added dwell;
request fresh tray/depth, then queue four Cartesian commands as one group:
MovL pre-place, MovLIO release (80% DO2 OFF/DO14 ON/DO13 OFF/DO1 ON), MovLIO
pre-place (50% DO2 OFF/DO14 OFF/DO1 OFF/DO13 OFF), MovL Cartesian Home. Require
positive prepick height. Preserve taught travel/approach/retract/travel rates,
independent placement orientation, source/depth validation and global CP(100).
No extra clearance/retract-height waypoint, pick settling, stationary release
call, fixed exhaust pulse or separate Home action. Monitor issued output history,
held suction until commanded OFF, OPEN/exhaust/DI12/DI1 release evidence before
neutralization, and neutral/unheld final Home. Pause/Stop preserve outputs in
place. Before release starts, Continue can reobserve; after confirmed release,
Continue/Recover neutralizes and retreats upward then Homes without new release
or bin put-back. Unconfirmed partial release blocks continuation. Preserve Pick's
two-phase settling/retry behavior. Test the real transport/feedback contract with
synthetic services, including execution during responses and Stop boundaries.


Rule 160 makes Tray Detect Position one direct queued MovL to the saved joint
pose, for both its button/action and Place Item observation travel. Remove the
preliminary Z rise and elevated XY transit; bin safety-clearance geometry remains
in the existing bin routes only. Preserve travel rates, exact recorded-joint
arrival with fresh idle/empty queue and execution evidence, held/unheld I/O
checks, no added dwell, and the fresh perception barrier before placement.
Already-arrived stationary joints skip the command. Do not change Pick/Home/bin
motion or the four-command placement-through-Home queue.


Rule 161 gates new Pick/Place requests on the corresponding canonical perception
service being available from exactly one allowed root provider. Item Teach/Tray
Teach advertise only while Armed; their headless Detect equivalents are also
valid. Publish item_detector_ready/tray_detector_ready in ControllerStatus; GUI
buttons and direct GUI sends use them, and the controller rechecks live readiness
before reserving either action. Missing, foreign, namespaced, duplicate or
unavailable providers fail closed without triggering inference or motion. Keep
GUI controller-only; no direct detector clients, automatic arming or hidden
inference. Preserve READY/unheld Pick and HOLDING/trusted-source/tray-pose Place
guards; Armed alone never implies holding. Show the unmet prerequisite in button
tooltips. Home and Tray Detect Position do not require perception availability.
Rebuild robot_controller_interfaces/robot_controller and restart controller clients
together after the status change. Existing pose validation and motion remain.


Rule 162 supersedes rules 158–161's held-item prerequisite for attended placement
only. Normal GUI launch (`headless=false`) is real-hardware debug mode: accept
Place from idle READY or HOLDING without requiring a prior Pick, candidate ledger
or suction presence during observation/approach. Publish manual_placement_enabled
from controller mode and use it in GUI gating; attaching a GUI cannot override a
headless controller. Headless Place still requires HOLDING, a trusted HELD source
and suction until commanded OFF. Retain the operation's policy through Pause and
reobservation; no synthetic candidate or automatic suction command for empty tests.
Both modes keep Startup, exact configuration, saved tray joints, provider readiness,
fresh tray/depth/robot feedback, unchanged output monitoring, queued 80% release /
50% neutral / Home, confirmed OPEN/exhaust/DI12 and DI1 LOW release, neutral final
Home, direct Stop and retained release recovery. Mark PLACED only for an existing
held source. Other actions, startup/idle fault handling and bin avoidance are
unchanged. Rebuild controller interfaces/controller and restart clients together.
Validate with synthetic feedback and real transport; no hardware movement in tests.


Rule 163 supersedes earlier unversioned tray-service endpoint references. Use
`/tray_detect/get_tray_pose_v2` exclusively for the current GetTrayPose layout with
placement depth, sharing its endpoint constant between provider and controller.
No legacy fallback; bump the endpoint for future incompatible wire-layout changes.
An old provider cannot satisfy readiness or receive the new request. Rebuild and
restart tray/controller processes together. Supervise GUI/headless tray executor
termination: unexpected return/exception revokes arming, records a full traceback
and becomes a visible terminal error; intentional shutdown is excluded. Never
restart the thread/worker silently. Record request starts, elapsed time and failing
phase/traceback; throttle repeated identical GUI messages to 30 seconds while
preserving every status update and the existing 1,000-event package log bound.
Keep ordinary no-tray/insufficient-depth results recoverable with live camera
callbacks. Preserve the single native worker, two ROS executor threads, freshness,
source validation, controller motion/IO and teach files. Verify with real ROS in
an isolated domain, repeated depth requests/errors, and injected executor failure;
no physical robot commands or automatic live-process restarts during testing.

Rule 164 supersedes rule 142 only for Tray Teach Simulate Trigger: hold its exact
RGB/depth result until RGB click, like Item Teach. Show only the returned tray
and reference plane, with explicit empty/error outcomes and original frame age,
counts, XYZ/quaternion, dimensions, confidence and batch identity. Publish the
response pose as teaching-only tray_teach_simulated_tray at 10 Hz while held;
refresh display timestamps only, never pose geometry. Clear on resume, replacement,
edits, source/epoch changes, YOLO/arming changes, failure and exit. Independently
validate from the ROS timer and never retain images in TF state. Camera callbacks
and armed requests continue; each service still acquires independent fresh inputs.
Simulation remains pose-only; controller placement-depth inputs are unchanged.
Log exact returned geometry and rejection evidence for both request paths.
Preserve teach schemas, native isolation, worker count and motion authority.

Rule 165 adds Detection Mask Clean for tray segmentation only, before rectangle
fitting. Use each raw binary mask, never Ultralytics' merged masks.xy. Apply one
3×3 opening in native mask pixels; retain the largest 8-connected region by pixel
area only when it keeps at least 80% of the original mask's foreground. Reject
empty/ambiguous results without guessed polygons or poses. Use the pinned runtime's
letterbox inverse. Preserve image-edge clipping evidence from the winning region's
original connected component. Preview, Simulate Trigger, armed Tray Teach and
headless Tray Detect share cleaned geometry and RGB/depth overlays. Expose cleanup
status/area/rejections in UI and request diagnostics, validate worker evidence,
and keep all native processing inside the private worker. Preserve existing size,
class, confidence, freshness and source checks, OBB/item processing, teach schemas,
placement depth, controller motion and I/O. No physical robot commands in tests.

Rule 166 supersedes the indefinite simulated-preview hold in rules 45, 47 and 164:
Item Teach and Tray Teach freeze successful Simulate Trigger results, including
empty batches, for ten seconds after installation/display, then automatically
resume live RGB/depth preview and clear simulated TF/pose guides. Use monotonic
time and one shared hold constant; acquisition/inference time is excluded. Show
a countdown, allow earlier RGB-click resume, and clear/restart deadlines with
existing resume/invalidation/replacement paths. ROS teaching timers independently
enforce simulated-pose expiry if Qt is busy. Preserve ordinary clicked-item and
corner-capture lifetimes, arming, live inference settings, accepted robot-action
batch lifetimes, freshness, schemas, hardware motion and I/O.

Rule 167 requires explicit Robot Controller emergency-stop feedback for confirmed
Dobot command `res=-3` or V4.6.5 GetErrorID alarm 1537: “Emergency stop pressed —
cannot start or recover.” Include physical-release and explicit Recover guidance
in status, service failures and operator diagnostics, and visibly in the GUI FAULT
panel. Startup queries canonical GetErrorID before initialization; absent, invalid
or failed diagnostics block startup. StopMoveJog's -3 must not be swallowed as
best effort. Preserve Recover's Stop/conditional ClearError and verified clearance
before Enable; if clearing times out, query current alarms for the specific reason.
Do not block that explicit clear attempt with a cached/latched E-stop label or
disable Recover permanently. Never infer E-stop from generic mode 9, ErrorStatus,
disabled state or collision. Keep five-second response/cancellation/ownership
guards, held-item protection, Stop-unconfirmed and motion containment semantics.
No automatic retries, resets, physical tests or live node restarts.

Rule 168 supersedes earlier explicit Recover put-back, placement continuation and
next-candidate behavior. Recover cancels the interrupted action and unfinished
batch, validates sources/ownership, confirms Stop with two distinct stationary,
empty-queue samples and stable gripper outputs/raw DI1, then adopts fresh I/O.
Do not replay expired placement history or fabricate release success. Preserve
DO1/DO2/DO13/DO14 unchanged through conditional ClearError, verified alarm clearance,
Enable/settings and motion; send no release, neutralization or output reset.
Reject opposing outputs, stale feedback, unknown DI1 HIGH, HIGH after confirmed
release, and a dropped source that merely regains DI1. Retain trusted held context
only with fresh suction and active vacuum. Stable LOW permits recovery without
asserting physical item release. Below Home Z, lift vertically at unchanged XY
and attitude using RelMovLUser; physically confirm before a separate joint-target
MovL to taught Home, using taught travel rates and the existing global factor.
Already-high/at-Home skips redundant segments. Monitor grip/output integrity and
preserve direct Stop/cancellation, response/late-ack containment and source checks.
Finish READY/HOLDING at Home. Pending/active/interrupted candidates become terminal
CANCELED; an unconfirmed held release with fresh clear suction also becomes
CANCELED, never PLACED/RETURNED. Retain the source of a still-held item for later
explicit placement/Return Item. Another Recover replans from a fresh Stop/pose.
Active Pick automatic loss return and Pause/Continue remain separate and unchanged.
No physical robot commands or live-node restart during software verification.

Rule 169 raises tray placement's nominal drop target to the item pre-pick
equivalent: sampled tray surface base Z + Item Teach standoff_height +
prepick_height. Pre-place/retract remains one additional prepick_height above
release, preserving positive vertical travel for queued 80% OPEN/exhaust and
50% neutral events. Keep the same four-command queue through Home, base-Z offsets,
requested X/Y, detect-relative rotation, rates, source/depth and release-feedback
guards. No new setting, teach-file migration, item-pick change or extra waypoint.
This supersedes rules 158/159's lower release height only. Validate geometry and
real transport encoding with synthetic feedback; do not command physical hardware.

Rule 170 removes intermediate gripper/suction release-confirmation gates from the
normal tray placement queue in both modes. After valid tray/depth observation,
queue approach → pre-pick-equivalent drop → approach → Home with the existing
80% OPEN/exhaust and 50% neutral commands. Missing/late DI12/DI1 evidence, suction
changes and output-history gaps cannot interrupt that queue. Retain any observed
coherent release as diagnostic/recovery evidence, without requiring it. Keep
ordered accepted service replies, fresh enabled/fault-free robot feedback, opposing
output protection, motion watchdogs and direct Stop/Pause. Check physical final
Home before reconciling neutral outputs and DI1 LOW; a bad final grip fails there.
Only successful Home completion marks an existing HELD source PLACED; this means
queue completion, not verified physical deposition. Log release_feedback_observed.
Before queue start, retain source/depth and headless held-item checks. Once release
is issued, Pause/Continue cannot reobserve/repeat it; unconfirmed interruption
requires explicit cancel-and-Home Recover. No changes to Pick or teach artifacts.
This supersedes rules 159/162's intermediate release gates and early PLACED state.

Rule 171 matches tray approach/retract to the executed first Item Pick approach:
placement X/Y at taught Home Z, like `pN_transit` before pre-pick. The initial Pick
route skips its lower clearance/initial point. Keep drop Z = tray surface Z +
standoff_height + prepick_height and detect-relative attitude through approach,
drop and retract; final Home restores its full pose. Require Home Z above drop Z
for nonzero timed descent/retract, otherwise block before any placement command.
Do not add a preliminary safety rise before Tray Detect Position, another height
setting or artifact rewrite. Supersede rule 169's second prepick_height addition
and rules 159/169's positive-prepick requirement only. Preserve the four-command
queue, 80%/50% I/O, no intermediate release gates, final Home checks and Item Pick.

Rule 172 treats an isolated backward controller timestamp during automatic camera
calibration as a discarded feedback frame, not a terminal restart diagnosis.
Serialize FeedInfo/RobotStatus callbacks independently of RGB, service replies
and Stop within the existing two executor threads. Log previous/received timers
and the backward delta; preserve the last accepted pose, sequence, receipt age
and progress clock, then continue with the next valid feedback without a Stop,
repeated move or consumed position attempt. Check robot faults and I/O before
discarding so they stay latched. Never rebase the clock during replay or refresh
freshness from discarded frames: the existing one-second stale/nonadvancing
timeout, connection/ownership checks, physical stationarity, capture freshness
and operator Stop remain enforced. No teach/calibration artifact or vendor change.

Rule 173 separates saved platform/bin geometry from the active camera calibration.
Platform Teach shows a startup reminder to freshly calibrate the bin camera in its
current position before teaching. New platform schema 4 keeps base/platform pose
and reference definition separate from historical teaching_provenance; schema-3
platforms remain readable with the same independent-camera semantics. Neither
loader opens the original teaching-camera file. Bin schema 3 remains portable and
may record source platform schema 3 or 4. Bin Teach selects its current camera
explicitly, prefilled from the saved Item Teach camera choice. Item Teach/Detect
use their explicit active camera; choosing a platform never replaces it. Controller
latest selection combines the latest station platform with the latest camera of
its prefix, without comparing to the platform's historical source filename/hash,
board settings or mounting transform. Keep robot/frame checks, active source hashes,
live TF/freshness and immutable geometry. Camera-only movement/recalibration does
not change the platform transform, bin points or wall margins. Preserve original
capture provenance on explicit migration; never substitute a new mounting transform
into old capture evidence. Supersede historical source-camera binding and platform
schema-3-only rules 22/23/26/50/131/132/148; no vendor or motion-sequence change.


Rule 174 makes tray base-frame reference geometry independent of its teaching-camera
file. Keep tray/draft schema 1; camera_calibration filename/hash and the plane's
pixels/intrinsics are original capture history, never a required live source.
Loading and camera-only changes preserve plane corners/transform and detect joints;
new captures record their current camera, and later saves preserve old provenance
for an unchanged plane. Use current validated intrinsics and exact-image-time TF
for live, simulated and armed detection. Remove saved-intrinsics equality gates,
not active camera/frame/hash/freshness or model/plane validation. Tray GUI retains
an explicitly loaded camera; otherwise load the strict eye-on-hand file from the
existing ITEM_TEACH_ROBOT_CAMERA_CALIBRATION selection. Headless Tray Detect and
controller tray configuration use that same explicit choice, with no historical
file fallback. Keep active hashes pinned until reload and require controller/provider
camera evidence agreement. Standalone GUI camera browsing remains available.
Camera changes disarm and discard observations, never issue hardware commands.
Show the fresh-camera reminder before teaching a new reference plane; reuse assumes
an unchanged robot base/reference surface and a still-visible observation pose.
For Bin/platform warnings, differing hashes alone are insufficient when validated
saved/current base/platform transforms agree within absolute 1e-9 (rtol=0) and robot
identity/reference convention match. Preserve warnings for changed geometry/station,
original provenance and active hash guards. This supersedes rule 46's hash-only
warning and earlier tray teaching-camera/intrinsics binding rules only. No numerical
calibration conversion, automatic teach-file rewrite or motion-sequence change.


Rule 175 fixes the commanded speed of Tray Detect Position and every normal tray
placement segment (approach, drop, retract and Home) at 100%, independent of Item
Teach speeds. Retain Item Teach travel acceleration for observation travel and
travel/approach/retract/travel acceleration for placement. Keep the operator's
global SpeedFactor scaling; never issue SpeedFactor from the placement path.
Item Pick and explicit cancel-and-Home Recover retain their existing rates.
Preserve geometry, the four-command queue, 80%/50% motion I/O, CP blending,
source/depth checks and final Home confirmation. Do not rewrite teach artifacts
or add speed fields. This supersedes only earlier placement/observation speed
inheritance; no physical robot commands or automatic controller restart for tests.


Rule 176 extends explicit Recover / Clear Error with a gripper reset after
confirmed stationary Home. The user confirmed preserving grip during recovery
travel. Keep rule 168's Stop/admission/source/held-suction checks, upward lift and
separate Home arrival. Then issue DO1, DO2, DO13 and DO14 OFF in order, with normal
command cancellation/response and fresh output confirmation. No OPEN command or
exhaust pulse. Allow only the explicitly pending OFF transition; intentional
suction decay is permitted during reset. Invalidate the former held source as
CANCELED when reset starts; never claim placement/return. Require all four outputs
OFF, raw DI1 LOW and continued Home before READY. Stuck HIGH remains HELD_UNKNOWN;
unknown HIGH before travel remains blocked. Stop or command/feedback failure
prevents subsequent reset commands and success; retry adopts fresh stopped I/O.
Update GUI feedback to relaxed at Home. Supersede rule 168's no-reset/HOLDING
completion only; Direct Stop still preserves outputs, and Pick/Place/Return Item
sequences and motion rates remain unchanged. No live commands or automatic restart.


Rule 177 moves the normal tray-placement retract's neutral-output trigger from
50% to 20% of ascent at the same target X/Y back to Home Z. At 20%, command DO2,
DO14, DO1 and DO13 OFF in order to relax fingers and disable exhaust/vacuum.
This supersedes earlier placement 50% neutral timing only. Keep descent's 80%
OPEN/exhaust trigger, all geometry, 100% speeds/global scaling, accelerations,
queue blending, final Home/grip checks and interruption behavior. Pick and explicit
Recover sequences are unchanged. Verify encoded MovLIO requests with synthetic
clients; do not issue live robot commands or automatically restart the controller.


Rule 178 limits the startup calibration reminder to Platform Teach
(`ros2 launch item_perception_yolo platform_teach.launch.py`). Remove the Tray
Teach reference-plane calibration dialog and its startup callback. This supersedes
rule 174's tray reminder only; retain independent calibration selection, plane
provenance/reuse, active camera/TF validation and all teaching/detection behavior.

Rule 179 makes Robot Controller position verification use fresh canonical
RobotStatus idle plus /joint_states. Compare saved joint targets directly (1°);
derive every actual Cartesian pose, motion origin, paused-pose check and stopped
return pose through the existing CR10 FK (5 mm/1° endpoint tolerance). Never use
FeedInfo tool_vector_actual for controller position/progress or Stop stationarity.
Position waits wake on either joint/status callback and require both streams to
advance after complete group acceptance; repeated/backward joint stamps do not
refresh evidence. Keep FeedInfo freshness, faults, user/tool zero, I/O, empty-queue
and command-execution guards, including MovL queue IDs and latched execution for
acceptance-only services. Stop uses two distinct joint samples within 0.05° and
the stopped/empty-queue guard; disabled/fault Stop remains confirmable. No fixed
arrival dwell is added; final Pick alone retains taught settling. After Recover
resets the gripper at Home, wait boundedly (five seconds) for fresh Home joints,
idle RobotStatus, completed output queue, neutral outputs and raw DI1 LOW.
Report actual idle/freshness/joint-error blockers instead of claiming position
loss from one busy sample. Preserve direct Stop, ordered admission, routes,
rates and gripper timing. Supersede rule 89's pose source and rule 120's required
new FeedInfo tick for each arrival only; no new services/subscriptions/executor
threads, camera-calibration changes, artifact rewrite or automatic restart.

Rule 180 limits each Pick action to three complete candidate batches, including
the first attempt. Exhaust all eligible poses then confirm Home before requesting
a fresh batch; a valid empty batch also consumes one attempt. Stop at first held
success, otherwise finish READY/NO_PICK after the third batch. Sum attempted
candidates across batches and retain the budget through Pause/Continue. Reject
replayed batch IDs; item-service failures, invalid pose/source evidence and robot
faults remain terminal. Place may send at most three tray/depth requests total,
including the first, for no-result/error/BUSY replies or missing replies. Preserve
each request's taught timeout plus one-second response allowance and new capture
boundary. Discard timed-out/paused results and never use late replies; provider
inference remains serialized. Pause does not reset the budget. Keep source/owner,
fresh robot/held-item checks and direct Stop active; malformed successful evidence
is terminal. Failed observation never starts placement or release. No schema,
teach setting, interface field, hardware-command retry or automatic restart is added.
Supersede one-batch Pick completion and no-retry tray observation only; preserve
existing routes, joint/status verification, I/O, settling and recovery.

Rule 181 replaces separate Home/Pick preview buttons and GUI Tray Detect Position
with the four-control grid Home / Preview toggle, then Pick Item / Place Item.
Preview starts OFF and cannot be entered during active/pending hardware work.
While ON, all three motion buttons call only /robot_controller/preview_v2; disable
and guard Start/Continue, Recover, managed Pause/Return and speed dispatch. Direct
Stop remains available. The separate preview process has no Dobot command clients;
use fresh canonical joint/status/feed observations and joint FK, including when
disabled, without Startup. Share Home/Pick/Place geometry, including Home alignment,
conditional initial joint Home, all candidates and entry/exit/return/put-back
targets, and Tray Detect plus placement/Home. Preview one fresh candidate batch's
nominal branches; never fabricate future retry observations or early-stop poses.
Place preview requests actual current-camera tray/depth with the same three-request
bound; no observation-position movement or held-item requirement. Clear on OFF,
edited inputs, source/feedback loss or robot movement. CLEAR must pre-empt pending
perception and discard late results. Keep native hardware actions, routes, safety
gates and lifecycle unchanged; external Tray Detect action remains supported.
Separate preview event logging from controller authority, remove obsolete GUI
clients/handlers and document/rebuild the extended versioned Preview interface.
No hardware fallback, new teach schema, persisted mode or automatic restart.


Rule 182 makes successful Pick finish at the saved Tray Detect pose. Queue the
actual stopped pose's upward-only pre-pick and clearance lifts, then direct
joint-target MovL to Tray Detect in one CP(100) group. Remove success's Home-Z
exit transit and final Home. First lift keeps taught retract rates, clearance
uses 100%/travel acceleration, and tray travel uses taught travel rates/global
SpeedFactor. Preserve grip timing, held-output/DI1 monitoring, ordered admission,
Stop and terminal saved-joint/idle/execution confirmation. Held Continue goes
directly from its parked pose to Tray Detect. Initial Home skips immediately
from fresh RobotStatus idle and all six /joint_states within ±1°; no additional
feedback tick, FK/query or dwell. Otherwise retain conditional rise and joint
Home. Require loaded recorded tray joints for Pick and its preview before motion
or item detection, without requiring tray arming or requesting tray observations.
Finish HOLDING at Tray Detect; Place skips observation travel when already there.
Retain missed retries, exhausted/put-back Home routes, three-batch budget and
all other rates/I/O. Preview adds each candidate's success destination and keeps
miss/put-back branches. Supersede successful Home/exit requirements of rules
108/110 only. No hardware commands or application restart during implementation.

Rule 183 makes Place verify saved Tray Detect joints before observing, without
commanding observation travel. Use fresh RobotStatus idle and all six joints
within ±1°, as for Pick's Home skip: proceed immediately when matched, otherwise
allow three seconds for arrival, then fail visibly with "Not at Tray Detect
position" and no detection/placement. Recheck during observation; retain three
requests total and all source, depth and safety gates. Queue exactly MovL pre-place
at Home Z, MovLIO drop (80% OPEN/exhaust), MovLIO retract (20% neutral); omit final
Home. Keep detect-relative attitude, 100% speeds, taught travel/approach/retract
acceleration and global SpeedFactor. Place SUCCESS acknowledges ordered acceptance
of all three commands. Retain PLACING and exclusive operation ownership in one
completion worker until advancing joint-FK/idle/execution feedback confirms final
retract, neutral outputs and DI1 LOW, then mark an existing candidate PLACED and
enter READY. Later failures report through status/events and Stop containment.
Preserve motion watchdogs, direct Stop/shutdown and placement Pause. Continue
before release admission rechecks observation position; after confirmed release
it neutralizes and retreats upward only, with no Home. Never repeat uncertain
release. Explicit Recover still cancels and Homes/reset. Preview requires saved
tray joints and shows only the three placement TFs. Retain the external tray-
position action. Supersede earlier automatic observation travel, final placement
Home and physical-completion Place-result requirements; no new interface fields,
schema, settings or ROS executor thread. Regenerate FSM exports with the change.


Rule 184 adds required `motion.trayplace_height` to Item Teach schema 10, as an
explicit finite, nonnegative millimetre value below pick_rotation in Vertical
motion. Real placement and Preview set drop Z = detected tray surface Z +
trayplace_height; Pick retains its existing standoff/pre-pick/retract equations.
Keep Home-Z approach/retract above drop Z, saved detect-relative attitude, 100%
speeds/global SpeedFactor, timed 80% release/20% neutral, ordered acceptance and
background terminal supervision. Production rejects schemas 1–9; GUI recovery
keeps independently valid fields and leaves the new height blank for explicit
entry and Save. New profiles also start blank; never infer a default or sum of
pick heights, automatically migrate files or rewrite local operator artifacts.
No hardware commands or application restart. Update FSM and adjacent exports.

Rule 185 makes three unavailable tray/depth observations pause Place at saved
Tray Detect after confirmed Stop, retaining outputs, item source, action and
operation ownership. Re-enable Place Item (Retry) for an explicit new three-request
batch; Pick Item stays disabled. The paused control uses the same RETURN ITEM &
STOP behavior as a held Pick pause, with direct STOP NOW while return is pending
or executing. Reuse the saved-source bin put-back routine, taught pre-pick release,
50 ms exhaust, neutral retreat/exit and Home; finish READY and Place CANCELED.
Restore normal held-suction/output checks during that return even in GUI mode.
Return requires trusted HELD source and no issued placement release, and stays
available without tray detection. Without such source the control stays Stop.
Ordinary placement Pause keeps its remaining acquisition budget and release
progress; only exhausted acquisition permits explicit budget reset/bin return.
Keep source/feedback/ownership/pose checks, direct Stop, no automatic fourth
request, no launch/restart/hardware command during verification and no new
interface/schema/configuration. Update FSM and adjacent visual exports.

Rule 186 simplifies Robot Controller's operator display to NOT READY, READY, BUSY,
HOLDING ITEM, PAUSED, ATTENTION REQUIRED and OFFLINE, with a visible activity/reason
and prominent confirmed emergency-stop override. Retain every internal FSM state.
Separate a permanent red direct STOP from the managed Pause/Return Item control.
STOP remains reachable during pending requests, stale status and Preview; pending
managed work disables its own control. Gate buttons by state, fresh feedback,
configuration/provider readiness, service availability and local pending work;
show missing prerequisites and recheck on click. GUI Place also requires already
at saved Tray Detect (fresh idle plus all six joints ±1°) and valid target inputs;
the external action keeps its three-second read-only arrival window. Publish
read-only readiness/Continue guards in typed status. Uncertain placement release
blocks Continue without setting resume. Acquisition Pause offers Place Item (Retry),
trusted-source Return Item and STOP; never Pick or a second Continue button.
Preview routes only TF requests, remains usable with stationary disabled feedback,
and blocks Start/Recover/Pause/Return/speed. Never latch Recover disabled from old
E-stop text; explicit Recover rechecks alarms. Show raw gripper inputs as Detected,
Not detected or Unknown. Window titles are exactly Robot Controller, Item Teach
and Tray Teach. Supersede prior GUI state/button/label presentation only, preserving
hardware routes, speeds, I/O, retries, Stop authority and all execution validation.
Rebuild interfaces/controller and manually restart their clients together; never
restart applications or command hardware during software verification.


Rule 187 makes explicit non-headless Configure/Load/Reload validate teach sources
and run the existing guarded Startup sequence to confirmed READY under one
operation owner. Keep Stop cancellation across validation, installation and setup;
never clear it with a chained second Startup request. Invalid files preserve the
old configuration; preparation failure retains loaded files and reports FAULT or
HELD_UNKNOWN, requiring explicit Recover. Startup still guards ownership, alarms,
DI1, settings, neutral outputs and READY; no Home/Pick/Place is queued by loading.
Launch/prefill and headless deployment loading remain inert; headless retains
external /startup. Remove the GUI Start button/client. The three lifecycle controls
are Recover, managed Pause/Continue/Return Item, and permanent direct STOP. Empty
ordinary Pause shows Continue; trusted held Pause defaults to Return Item with
Continue in a split-button menu, each separately gated. Acquisition exhaustion
keeps Place Item (Retry), Return and Stop with no duplicate Continue. Loading is
disabled in Preview and without fresh robot feedback; toggling Preview OFF never
prepares hardware. Supersede earlier Configure-read-only/manual-GUI-Start rules
only. Update FSM/exports; test with synthetic feedback and no live commands or
automatic application restarts. No message schema, motion route or I/O change.

Rule 188 makes eligible Pick DI1 HIGH send Stop immediately and switch to the
successful lift/Tray Detect route after command acknowledgement only. Final-pick
settling is the last chance to acquire, never a delay after acquisition. Do not
wait for stationary joints, idle status or an empty-queue sample at this handoff;
use the latest fresh joint-derived pose as the upward-only return origin. If a
motion reply was pending at acquisition, resolve it and the first Stop, then
send/acknowledge a final Stop before returning so late admission is discarded.
Never let a delayed normal callback Stop the new return queue. Keep five-second
response limits, cancellation, source/ownership/feedback/output checks, suction
arming/reset and held-loss monitoring. All other Stop/Pause/Recover stationarity
checks remain physical; do not label acquisition acceptance as confirmed standstill.
Supersede only successful pickup's stopped-origin/physical-Stop wait in earlier
rules. Preserve routes, rates, grip timing, retry budget and schema. Update FSM
and exports; validate with fake services/feedback, without hardware or restart.

Rule 189 adds a native counted AutoRun action and GUI Auto Run/quantity controls.
Reserve one owner from unheld configured READY with both detectors through final
Home or failure. Reuse Pick/Place geometry, rates, timed I/O and retry budgets;
tray acquisition still requires confirmed saved joints/idle. After placement
admission, prefetch one next bin batch in a read-only worker while supervising
motion. As soon as ready, append joint Home and next Pick behind placement without
waiting for placement/Home arrival. A valid empty batch uses Home and only the
remaining Pick attempts. Bind/consume each batch once; reject repeated IDs and
discard pending results on Stop/failure. Retain the old placement/source until
advancing FeedInfo reaches/passes the appended Home queue ID with observed neutral
outputs and DI1 LOW; then count it once and activate the next source/acquisition.
Never count acceptance alone or let the old DI1 trigger the next item. Append final
Home immediately after the last placement, with no extra item request, and confirm
saved joints/idle, neutral outputs and DI1 LOW before SUCCESS. Manual controls/input
edits, including Pause/Continue/Return, are disabled during Auto Run; direct STOP
and action cancellation always pre-empt. Three exhausted Pick batches or tray
requests end the run with its partial count; no automatic fourth attempt/restart.
Add typed quantity/progress/result fields, rebuild interfaces/controller and update
FSM/exports. Keep two ROS executor threads, no launch-time run or new config/schema,
and no hardware commands/restarts during verification. Supersede midpoint arrival
barriers only for the owned Auto Run placement-to-Home/next-Pick queue extension.

Rule 190 moves delayed finger CLOSE for successful Pick with use_grip=true and
grip_onpick=false to 50% of the first held lift to pre-pick. Use MovLIO with
DO14 OFF before DO2 ON at that percentage; clearance uses MovL without finger
events. Apply through the shared Pick executor to manual Pick and Auto Run.
Preserve immediate grip_onpick closing, no-grip and missed-pick behavior, upward
geometry, taught retract rates, full-speed clearance, suction/Stop monitoring,
ordered blended queues and final Tray Detect confirmation. No schema, settings,
hardware command or application restart during validation; update FSM and exports.

Rule 191 makes explicit Return Item with a trusted held source use the shared
placement motion/release pipeline, including after failed tray acquisition.
Queue optional current-XY vertical rise, source XY/attitude at Home Z, exact saved
pre-pick drop, same-XY retract to Home Z and exact taught joint Home in one ordered
CP(100) group. Release OPEN/SUCK-OFF/EXHAUST at 80% of descent; neutralize all four
outputs at 20% of ascent. All speeds are 100%, with taught travel/approach/retract/
travel accelerations. No intermediate arrival, release/DI12 or separate pulse
wait; confirm only final Home joints/idle/execution, neutral outputs and DI1 LOW
before RETURNED/READY. Retain source/release progress on Stop; Recover cancels
without repeating release. Keep automatic suction-loss/paused-drop put-back's
existing 50 ms pulse and continuation behavior. No detector request, new setting,
schema/interface change or live validation; update FSM/exports and the diary.

Rule 192 makes the release descent in Place Item and explicit held Return Item
use Item Teach speed.approach_percent (6% when taught as 6), superseding their
100% descent rate. Apply through the shared release planner to hardware, Preview
and Auto Run placement. Keep other segments at 100%, taught accelerations,
global SpeedFactor, 80% release / 20% neutral, ordered queues and final feedback
gates. No teach/schema/settings changes, hardware commands or application restart;
update FSM/exports and the diary.

Rule 193 restores 100% descent for explicit held Return Item and Place Item,
superseding rule 192. Split each routine into two ordered CP(100) groups:
optional rise/approach/drop, then retract plus taught Home for Return Item or
retract alone for Place. Require fresh joint-FK drop arrival, idle/empty queue
and execution evidence before the second group, with no settling or extra
motion-origin wait. Preserve release at 80%, neutral at 20%, taught acceleration,
global SpeedFactor and final neutral/DI1 LOW gates. Share release history and
offset timed-command indices across both groups; never infer observed I/O merely
from drop arrival. Stop/Pause must prevent later admission and retain source.
Place succeeds on retract admission after drop arrival; Auto Run may prefetch
and append Home/next Pick behind that retract as before. No automatic-drop
put-back changes, new settings, teach edits, hardware commands or app restart.
Update FSM/exports, READMEs and the diary.

Rule 194 moves retract NEUTRAL from 20% to the 0% start in Place Item and
explicit held Return Item. Use the existing MotionIO distance-mode zero trigger
for DO2/DO14/DO1/DO13 OFF within the upward MovLIO, including Auto Run placement.
Preserve 80% descent release, full speed/global SpeedFactor, two queues with
drop arrival and no settling, final feedback gates and automatic drop put-back.
No separate DO calls, settings/teach edits or hardware commands/restarts.
Update READMEs, FSM/exports and the diary.

Rule 195 changes explicit held Return Item and Place Item release to relaxed
fingers plus exhaust: at 80% descent set DO2 OFF, DO14 OFF, DO13 OFF, DO1 ON.
Neither routine commands finger OPEN. The shared release observer recognizes
exhaust ON, both finger outputs and suction OFF, and DI1 LOW; DI12 is irrelevant.
Retain diagnostic-only intermediate release evidence and final neutral/DI1 gates,
0%-start retract neutral, two full-speed queues without settling, Auto Run and
Stop/Pause/Recover ownership. Pick and automatic suction-loss put-back retain
their existing I/O. No teach/settings/schema changes or hardware commands/restarts;
update READMEs, FSM/exports and the diary.

Rule 196 moves only exhaust ON (DO1) to 100% of descent in explicit held Return
Item and Place Item, including Auto Run placement. Retain DO2/DO14/DO13 OFF at
80%, all-neutral retract at 0%, full speed/global SpeedFactor, two queues with
confirmed drop and no settling, release diagnostics and final feedback gates.
Use one downward MovLIO with ordered 80% OFF events then the 100% exhaust ON
event; no separate DO call, pulse, setting or teach edit. Pick and automatic
drop put-back keep existing I/O. Update READMEs, FSM/exports and diary; no
hardware commands or application restart during verification.

Rule 197 restores one complete ordered motion group for explicit held Return
Item and Place Item, superseding rule 193's drop/return split. Return queues
optional vertical rise, approach, saved pre-pick drop, retract and taught joint
Home; Place queues approach, drop and retract, including Auto Run placement.
Keep ordered service acceptance without physical intermediate arrival or settling
waits. Retain speed 100%, taught accelerations, global SpeedFactor, fingers and
suction OFF at 80% descent, exhaust ON at 100%, all-neutral at 0% retract,
final feedback/I/O checks and Stop/Pause/Recover ownership. Place acknowledges
complete queue acceptance while its worker retains physical completion; Return
completes only at Home. Remove unused split-queue helpers. No Drop Retract, 6%
segment, settings/teach edits or automatic-drop changes. Update READMEs, FSM and
exports plus the diary; no hardware commands or application restart for validation.

Rule 198 moves explicit held Return Item and Place Item finger relaxation and
suction OFF from 80% to 90% of descent. The downward MovLIO sends DO2/DO14/DO13
OFF at 90%, then DO1 ON at 100%; the upward MovLIO still sends all four outputs
OFF at 0%. Apply through the shared release planner to manual/Auto Run placement
and Preview. Keep one complete ordered queue, full speed/global SpeedFactor,
taught accelerations, final feedback checks and Stop/Pause/Recover behavior.
Pick and automatic suction-loss put-back keep their existing timing. No new
motion, setting, teach edit, hardware command or application restart; update
READMEs, FSM/exports and the diary with validation.

Rule 199 changes explicit held Return Item and Place Item to finger OPEN,
suction OFF and exhaust ON together at 90% descent. The downward MovLIO sends
DO2 OFF, DO14 ON, DO13 OFF, DO1 ON in that order, all at 90%; remove the 100%
exhaust event. Keep all four outputs OFF at 0% retract, complete ordered queues,
speed/acceleration, final feedback checks and Stop/Pause/Recover behavior.
Release evidence uses DO14/DO1 ON, DO2/DO13 OFF and DI1 LOW; DI12 remains
optional and no intermediate release wait is added. Shared planning covers
Return, manual/Auto Run placement and Preview; Pick and automatic drop put-back
are unchanged. Update READMEs, FSM/exports and diary. No new setting, teach edit,
hardware command or application restart during validation.

Rule 200 starts Pick by requesting item poses before Home, including the first
Auto Run Pick. After a valid nonempty batch, ensure Home before candidate motion
with the existing joint/idle skip. The first validated empty batch grants one
Home-and-acquisition retry per Pick; a later empty batch ends NO_PICK at Home.
This single empty-result retry survives Pause and physical misses and does not
consume the existing three-nonempty-batch physical-pick budget. Keep accepted
poses while Home is interrupted, reject reused batch IDs and retain terminal
service/validation faults. Empty Auto Run prefetch finishes the owned queued
Home and counts placement before retry; nonempty prefetch still appends Home/Pick
without a midpoint wait. Stop always pre-empts. No new teach setting, pose-client
fallback, hardware command or restart during validation. Update READMEs, diary,
FSM and visual exports with tests for order, bounded requests and interruption.

Rule 201 permits bounded reuse of camera-prefix YAML parsing keyed by exact file
bytes during strict calibration catalog scans. Read every file and recheck names,
symlinks, timestamps and newest selection on every scan; never trust size/mtime
or retain a filesystem validation result. Keep all selected-artifact schema,
hash, robot and mounting validation at existing checkpoints. Cache only the
immutable prefix string for at most 16 contents, never images, poses, depth,
models or exceptions. Preserve motion, I/O, retries, ownership and Stop behavior.
Document offline timings separately from unmeasured physical-cycle improvements.

Rule 202 starts Auto Run's single next-item request immediately after successful
Pick confirms saved Tray Detect joints/idle, before tray acquisition. Overlap the
same fresh post-request bin observation with tray-pose/depth attempts and placement;
never duplicate it on tray retries or request after the final item. Tray Detect
must leave the fixed bin view clear; no automatic occlusion test or in-motion
trigger is added. Retain an early result until complete placement admission, then
use the existing handoff/error path, empty-result retry and execution/release
counting. Preserve the old held source/DI1 boundary. Stop, held loss, tray exhaustion
or other run failure cancels/discards the result; recovery/Return cannot reuse it.
Keep manual routines, motion/I/O, retry limits and the one hardware owner unchanged.
Update READMEs, diary and FSM/exports; validate without hardware commands/restarts.

Rule 203 lets explicit Place Item start from READY/HOLDING with or without a held
item in either launch mode. Remove current Tray Detect position and held-source
button/admission gates, retaining fresh safe idle feedback, valid configuration
and target, one armed tray provider and exclusive ownership. Skip observation
travel when saved joints/idle match; otherwise reuse the direct 100% joint-target
MovL with preserved outputs and confirm arrival before tray/depth acquisition.
Stop/failed travel prevents detection and placement. Continue before release may
revisit observation pose; never repeat an issued release. Auto Run keeps its
trusted Pick and both-provider prerequisites, with a visible disabled reason.
Preview away from Tray Detect publishes only the travel TF and explains that
placement geometry needs a fresh observation there; no motion or invented depth.
Keep drop/retract I/O, retry budgets, final neutral/DI1 checks and fault/unknown-item
guards. No automatic arming, launch action, hardware command or restart in tests.
Update README, diary and FSM/exports with scoped validation.

Rule 204 adds controller-owned global CP adjustment through SetGlobalCP and the
GUI slider beside Global SpeedFactor. Accept integer 0–100 only in started idle
READY/HOLDING with exclusive operation ownership, fresh enabled safe feedback,
strict serialized response/cancellation and held-output checks. Preview, Auto Run
and other active operations disable manual adjustment. Status and response report
the last accepted CP with -1 for unknown; zero is valid. Startup/Load uses 100;
Recover retains the selected value including zero (100 only if unknown). Supersede
earlier fixed CP(100) requirements only for this setting: every motion still
omits per-command cp/r and inherits global CP. Preserve geometry, rates, I/O,
queue admission and terminal checks. No config key, teach schema, persistence,
launch-time hardware call or automatic restart. Update FSM and visual exports.

Rule 205 restores the explicit vertical Safety Z exit on successful Pick before
saved Tray Detect, superseding rule 182's exit removal only. Queue pre-pick lift,
clearance, exit and Tray Detect in one ordered group for manual Pick and Auto Run.
Exit uses max(taught Home Z, current height), unchanged measured X/Y/attitude,
taught travel rates and no I/O. Retain even if coincident with clearance; no
final Home detour, intermediate arrival gate or per-command CP override. Preserve
selected global CP blending, first-lift grip timing, source/held monitoring and
final tray-joint/idle/execution checks. Held Continue from confirmed safety-height
parking stays direct. Preview places the shared exit before the success tray TF.
Update README, diary and FSM/exports; validate without hardware commands/restarts.

Rule 206 starts Auto Run's one next-bin candidate request only after confirmed
Tray Detect, successful tray-pose/depth acquisition and acceptance of the complete
placement approach/release/retract queue. Supersede rule 202's earlier trigger;
overlap placement execution only. Failed tray acquisition, incomplete/rejected
queue admission or Stop before that boundary prevents the request. Once poses
are ready, append Home/next Pick behind placement without a physical completion
wait. Keep slow-result supervision, fresh observations, final-item behavior,
retry budgets, source/DI1 ownership, cancellation and physical placement counting.
No motion/I/O/rate, interface, configuration or executor changes. Update READMEs,
diary and FSM/exports; validate without hardware commands or application restarts.

Rule 207 removes the saved reference-plane green outline and P1–P4 labels from
Tray Teach RGB/depth previews, simulation and GUI/headless request images.
Preserve explicit corner-capture markers and depth evidence, detected-tray masks,
axes/dimensions, sidebar plane status and RViz scene/pose displays. The saved
base_link plane, measurement, pose selection and placement depth are unchanged;
no artifact/schema, configuration or controller behavior changes.

Rule 208 removes quality.minimum_depth_samples and makes valid depth coverage
percentage-only across Item Teach/Detect, tray placement sampling, service reply
validation and Robot Controller. Retain minimum_depth_fraction in (0,1], displayed
as Minimum valid depth (%) with default 50. Count all original pixels in the
physical sampling circle; only in-item/in-tray, range/MAD-accepted pixels count
as valid. Empty/zero-valid footprints fail; no absolute count floor remains.
Use strict Item schema 11 and the v3 tray endpoint; GUI recovery retains older
fractions for explicit review/Save. Preserve tray plane capture/measurement,
freshness, source hashes, motion/I/O and retry gates. Update FSM and exports.

Rule 209 starts Auto Run's single read-only next-bin worker after validated tray
pose/depth and the observation-position check, before placement planning/dispatch.
Supersede rule 206's post-admission trigger; overlap placement admission and execution.
All three placement commands must receive ordered acceptance before any Home/next-Pick
motion, even if the worker finishes early. Failed tray acquisition or pre-trigger Stop
prevents the request; later admission/execution failure cancels/discards it. No worker
for the final item. Keep one hardware owner, fresh observations, no added arrival wait,
slow-result supervision, retries, source/DI1 ownership, counting and cleanup. No motion,
I/O, rate, interface, configuration or executor change. Update FSM and exports.

Rule 210 changes the shared SUCTION_LOSS_DEBOUNCE_SEC controller constant from
0.050 to 0.300 seconds, superseding rule 107's interval only. It is not a teach
field. Require advancing LOW feedback spanning 300 ms; HIGH resets immediately,
and frozen/stale inputs cannot finish the interval. Keep immediate pickup HIGH,
raw release/reset checks, DO/freshness gates and direct Stop. The separate 50 ms
exhaust pulse, existing drop-monitoring phases and recovery routes are unchanged.
No schema/configuration/interface changes. Update FSM and regenerate its exports.

Rule 211 extends continuous held DI1 monitoring through lifting, travel, idle,
tray acquisition and placement until observed commanded suction OFF. Latest user
specification restores 50 ms, superseding rule 210's 300 ms. Latch DROPPED/source,
send Stop from feedback, block normal dispatch, resolve all issued replies, then
confirm a final Stop/empty queue before put-back. Keep safety rise, saved pre-pick,
50 ms exhaust and upward exit; no Home in active automatic return. Join eligible
old-batch poses in saved order, without new detection or operator action. Auto Run
discards speculative perception/appended sessions and never counts the dropped
placement. Planned release is exempt; submission alone does not end monitoring.
Preserve direct Stop, strict feedback/output/source/reply gates and paused-drop
policy. Exhaustion finishes the return exit; ordinary Pick retry policy is separate.


Rule 212 makes paused explicit Return Item the shared reference for every item
return, superseding the separate automatic pulse/clearance route. One ordered
queue: optional current-XY rise to Home Z, source XY/attitude at Home Z, exact
saved pre-pick with OPEN/suction OFF/exhaust ON at 90%, then Home-Z retract with
all outputs neutral at 0%. Speed 100%; acceleration travel/approach/retract by
segment. Explicit Return and paused drop append joint Home; active drop appends
eligible original-batch entry/clearance/pre-pick/pick in the same group, without
Home or intermediate arrival waits. Exhaustion confirms retract above the bin.
Retain DROPPED/source until advancing execution crosses the next clearance's
MovL queue ID with neutral/raw-DI1-LOW evidence after retract issuance. Preserve
50 ms loss debounce, Stop/reply containment, strict I/O and source gates; explicit
Recover cancels interrupted returns without repeating release. Preview and
candidate validation use this same geometry; Home Z must exceed saved pre-pick.


Rule 213 changes the shared Place Item / Return Item release trigger from 90%
to 80% descent. At 80%, command DO2 OFF, DO14 ON, DO13 OFF, DO1 ON; retain all
four outputs OFF at 0% retract. Apply to manual/Auto Run tray placement, explicit
Return Item and automatic/paused drop returns through the same release planner.
Preserve geometry, motion rates, queue ordering, continuous held-loss monitoring
until observed commanded suction OFF, and every endpoint/Stop/feedback guard.
No teach file, schema, interface or configuration changes. Update FSM/exports.

Rule 214 shares manual Place's failed-acquisition Pause with Auto Run. After
three unavailable tray pose/depth requests, confirm Stop at Tray Detect and keep
the held source, outputs, target, operation owner and completed count. Offer the
normal Continue / Return Item controls and permanent STOP; Place Item (Retry)
remains a Continue alias. Continue grants three fresh tray attempts for that same
item; repeated exhaustion pauses again. Do not request another bin batch or send
placement/next-pick commands while waiting. Return uses the shared queue through
Home, finishes READY and cancels the placement/run without counting that item or
starting another Pick. Permit typed Continue/Return during Auto Run only in this
confirmed pre-release acquisition Pause; other manual controls remain blocked.
Preserve provider/source/parked-state/held-output checks and direct Stop. Faults
or held loss while awaiting the operator must not automatically resume the run.
Keep three-request batches, motion/release timing and schemas/interfaces unchanged.
Update the FSM, its generated exports and GUI status/control tests.

Rule 215 drains each saved bin batch across successful manual Pick/Place cycles
and Auto Run cycles, including later operations under the same loaded configuration.
Preserve plans, IDs, order and terminal candidate states; use the next PENDING or
eligible INTERRUPTED pose. Request fresh bin poses only when no eligible saved
pose remains. Tray pose/depth remains fresh per placement. Auto Run skips prefetch
while saved poses remain and appends Home/next Pick only after complete placement
queue acceptance, without a placement-arrival wait. Retain the old held source
until execution crosses appended Home with neutral outputs and DI1 LOW; then mark
PLACED and activate the next candidate in that same ledger. Cleanup before this
boundary must not cancel remaining saved poses needed for automatic drop return.
Exhaustion permits the existing parallel next-bin request and bounded retry policy.
Manual Pick reports attempts for that action without resetting candidate states.
Explicit Recover cancels remaining poses; reload/source checks and process restart
prevent reuse across invalid context. Return Item excludes the returned candidate
and preserves other saved poses for later explicit work. No disk pose cache,
new configuration/schema/interface, motion/I/O change or executor thread is added.
Update the diary, FSM/exports and tests for reuse, exhaustion and source ownership.

Rule 216 aligns Tray Teach's camera diagnostic bands with Item Teach: detection
and valid-tray counts, frame/inference timing, STALE/RESULT SNAPSHOT after 0.5 s,
RViz voxel status/refresh age/reason, and the displayed request's confidence,
IoU/cap/size-color legend. Refresh ages during worker activity; show independent
depth age/unavailability and preview failures. Preserve explicit YOLO OFF/paused
states and captured-corner/simulated-result headings. This is GUI-only metadata;
keep inference, pose acquisition, saved settings, headless and controller behavior
unchanged, with no extra worker, timer, native prediction or ROS request.

Rule 217 sets new Item Teach and Tray Teach profiles and initial YOLO previews
to shared 448 × 448 CPU inference. Preserve square padding (`rect=False`),
explicit saved image sizes, original-resolution RGB/depth pose sampling and all
pose gates. Existing profiles use 448 only after their saved/deployed image_size
is explicitly updated; never silently override loaded values. Reload Teach and
restart headless consumers/controller after deployment edits to refresh pinned
profile hashes. Camera settings, weights, motion and controller FSM are unchanged.

Rule 218 supersedes rule 215's pose reuse across successful tray placements.
Mark the placed candidate PLACED and cancel unused old poses for manual and Auto
Run cycles. Auto Run starts a fresh next-bin request immediately after valid tray
pose/depth and observation-position validation whenever another item remains,
even if old eligible poses exist, in parallel with placement planning/admission.
Require all placement replies accepted before appending Home/next fresh Pick;
add no placement-arrival wait. Keep the old source/batch until execution crosses
Home with neutral outputs/DI1 LOW (or final retract completes), then invalidate
unused poses, count placement and install the fresh ledger. Manual Place waits
for the next explicit Pick to request new poses. Misses, Pause, tray failure and
drop return retain original candidates until successful placement; drop continues
eligible original poses without Home/detection. Return Item keeps other poses;
Recover/reload/restart invalidate them. Preserve slow-result supervision, retry
limits, source/Stop/I/O gates, final-item Home and the existing one-worker limit.
No schema, detector, configuration, motion rate or executor change is added.

Rule 219 makes grip_onpick independent of use_grip: after confirmed suction
pickup, grip_onpick=true closes immediately. At 50% of the first held lift to
pre-pick, use_grip=false relaxes DO2/DO14 OFF; use_grip=true closes there only if
grip_onpick=false, otherwise remains closed. Held Pause preserves outputs;
Continue restores the chosen transport state before direct Tray Detect travel
if the original lift event was interrupted. After valid tray pose/depth and
placement validation, use_grip=false reopens with confirmed DO2 OFF then DO14 ON
before placement motion, without changing vacuum or waiting for DI12. Auto Run
starts fresh bin inference before these outputs; complete placement admission
still precedes next Pick admission. Preserve continuous drop/Stop supervision,
all four flag combinations, missed-pick behavior, 80% shared release and 0%
retract neutral. Item Teach keeps both existing booleans editable independently;
no schema, saved setting, rate, geometry, runtime artifact or executor change.
Update README, diary and FSM/exports and validate without hardware commands.

Rule 220 makes the controller checkbox Save item/tray debug RGB/depth apply to
manual Pick Item, manual Place Item and all Auto Run item/tray observations.
Add save_debug_images to the typed PlaceItem goal and retain it in each placement
operation through retries and Pause/Continue. Forward it to the existing tray
pose/depth service; Auto Run reuses its one goal flag for item acquisition,
prefetch and each tray request. Use existing detector writers and directories
debug/pick_img and debug/tray_img; no extra inference, capture worker, automatic
archive, persisted setting or perception service change. Preview and unchecked
requests keep saving disabled. Preserve acquisition, motion, release and Stop
contracts. Rebuild interfaces/controller and restart controller/GUI together;
update README, diary and FSM/exports and validate without hardware commands.

Rule 221 makes failed-pick candidate logs explain the missing DI1 acquisition:
FAILED — no DI1 pickup detected before N ms settling expired. Derive N from the
loaded Item Teach pick_settling value, not a fixed 300 ms. Publish the same text
in the operator log and candidate_state event; retain the plain candidate state
in the event's candidate_state field and typed status. This is diagnostic only:
no timing, failure classification, late-DI1 handling, retract, I/O or retry change.

Rule 222 keeps a candidate ACTIVE after final-pick settling expires without DI1.
With suction and finger outputs unchanged, lift 20% of the remaining upward
distance from the actual settled pose to saved pre-pick Z, keeping measured
XY/attitude and taught final-approach speed/acceleration. Monitor eligible DI1
continuously across the transition, admission and lift; HIGH uses the existing
acquisition Stop/reply containment and held continuation from the latest pose.
Only confirmed probe endpoint/queue completion without DI1 latches FAILED; no
second settling interval. Zero available rise skips motion, never inventing a
height or descending. Log the settling duration and upward-lift check, plus
actual probe distance/rates in events. Existing failed retract/retry, late-DI1
isolation after failure, source retention, gripper-on-success behavior and
Pause/Stop gates remain. Apply to every shared Pick path without new settings,
schema or detector requests. Supersedes rules 94/97/99/221's settling-only miss
decision; update diary/FSM/exports and verify with synthetic feedback only.

Rule 223 changes held DI1-loss debounce to a fixed 500 ms and defers drop
detection until the first successful upward retract reaches saved pre-pick Z
(or the higher actual pickup origin). Arm from fresh joint-FK height feedback,
including passage through the height during CP blending, without splitting the
queued lift/clearance/Tray Detect route or waiting at a midpoint. Keep held/source
context through this lift; LOW time before arming never consumes the new 500 ms
interval. HIGH resets it immediately. A Pause parking rise crossing the same
height also arms; Stop preserves the pending height. Acquisition during Pause
uses the same deferral. Suction OFF clears pending deferral so release/reset
cannot leak it to another item. Keep raw DI/DO, output/freshness guards, pickup
acquisition/probe, intentional release and confirmed-drop Stop/return unchanged.
Log deferral and physical activation with height and threshold. No new setting,
schema, executor or robot wait; supersede rules 107/111/211's 50 ms and immediate
post-pick drop activation. Update diary/FSM/exports and test without hardware.

Rule 224 increases rule 222's last-chance pickup lift from 20% to 50% of the
remaining upward distance from actual settled Z to saved first-retract/pre-pick
Z. Keep taught final-approach rates, unchanged outputs, continuous acquisition,
Stop/reply containment and failure only after the probe endpoint. Update messages
and diagnostics to 50%. Rule 223's post-retract activation and 500 ms drop debounce
remain unchanged. No new setting/schema; update diary/FSM/exports and validate
the longer probe with synthetic feedback.

Rule 225 moves explicit Recover's stationary-joint/empty-queue confirmation
after ClearError (when needed), verified alarm clearance and EnableRobot/enabled
feedback. Initially require Stop acceptance and validated fresh gripper I/O only;
preserve those outputs through setup. Reuse the accepted Stop and confirm two
distinct stationary joint samples, empty queue and unchanged I/O/raw DI1 before
settings or Home motion. Keep bounded response/feedback waits, cancellation,
ownership, unresolved-reply, freshness and unknown-suction guards. Direct Stop,
Startup, managed Pause and drop containment keep physical Stop confirmation.
No drag command, new setting/schema, automatic retry or launch behavior change;
update diary/FSM/exports and validate collision/rejection/timeout cases synthetically.

Rule 226 replaces rule 83's camera-origin test with full projected containment
of a centered 100×30×30 mm housing in the green Bin ROI. Camera-link XYZ size
is 30/100/30 mm (Y width). Transform all eight corners with the calibrated mount,
planned Link6 and platform transforms; test the convex footprint, normal first,
then exact 180° tool-Z mirror, rejecting both-unsafe candidates before ranking.
Use one pure geometry implementation in Item Teach/headless detection/controller
preview/hardware. Keep the blue pick-point inset separate. Native diagnostics and
RGB/depth overlays include the real body outline and validated size evidence.
The optional read-only robot_camera_box node uses those same dimensions and the
controller's strict latest calibration for a CUBE Marker frame-locked to Link6;
GUI launch starts it, headless does not. No competing camera TF, robot clients,
new configuration/schema or swept-path collision guarantee. Preserve operator
RViz edits; stage only the new canonical display. Update diary/FSM/exports and
test geometry, provider/controller agreement and marker placement synthetically.

Rule 227 supersedes rule 226's camera dimensions and zero center offset. Use
Gemini 335's documented RGB-optical XYZ size 90/25/30 mm and body center
(+11, 0, −12.79) mm. Compose the nominal mechanical RGB-to-camera-link transform
with the existing saved Link6 mount: link-frame center (−10.77, −25, 0) mm.
RGB/aligned-depth still use actual factory optical TF for measurements; never
substitute the nominal housing bridge into calibration or perception projection.
Keep the shared eight-corner footprint/mirror test and RViz marker identical,
with native evidence validating reference frame, dimensions and offset. No new
live TF dependency, calibration rewrite, teach schema or configuration key.
Document Orbbec geometry sources, update FSM/exports and test independent
RGB/link bounds, rotated offsets, mirror decisions and marker/planner agreement.

Rule 228 moves robot_camera_box into item_perception_yolo and starts it only
alongside Item Teach, removing controller launch/package ownership. Item Teach
publishes its exact validated selected Link6-relative mounting as one stamped
PoseArray pose (empty means clear) on /item_teach/robot_camera_mount at 1 Hz.
Edits, failed validation and shutdown clear it; recheck the selected file hash
without independent discovery or automatic replacement. The separate read-only
node publishes /item_teach/robot_camera_body, rejects invalid/nonadvancing/stale
mounting evidence and hides it after 2.5 seconds without fresh updates. Item
Teach launch exit stops the display; its own marker retains a three-second
expiry. Keep shared geometry, controller/headless clearance and calibration
selection unchanged. No hardware clients, inference dependency or new config.

Rule 229 adds schema-12 required geometry.nearby_depth_radius_mm (default 150)
and geometry.nearby_depth_height_mm (default 60), editable in Item Teach section
4 as Nearby depth radius filter (mm) and Maximum nearby height above pick (mm).
Both are finite positive distances. From the same synchronized original depth
frame, back-project every finite positive in-range pixel with its own depth
CameraInfo and exact base transform. Reject a candidate when any point is inside
or on the base-XY radius and at least the taught base-Z height above final Link6
pick (item base Z plus standoff). Include points outside item masks/green ROI;
do not voxelize, interpolate, globally MAD-filter or require a point cluster.
Compute the scene once per candidate batch and filter before ranking/capping.
Share clicked, teaching preview, simulation and headless production geometry;
require matching native filter evidence and provide pixel/radius/height reasons.
Runtime readers require schema 12. GUI recovery of older profiles proposes
150/60 only for missing new fields, visibly requiring review and Save; malformed
current fields remain blank. Never rewrite/deploy operator artifacts automatically.
Controller uses the filtered, profile-bound batch; no new motion or tray filter.


Rule 230 sends every final controller Home target using MovJ with mode=true and
all six exact taught joint angles (no modulo wrapping). This includes explicit
Home, initial/empty-retry/exhausted Pick, Return Item, Recover and Auto Run Home.
Explicit Home and its preview now reuse the shared conditional upward-only
Home-Z clearance then joint Home, superseding rules 90/93's Cartesian alignment.
Keep necessary linear clearance/release/transit segments and their existing
barriers; all other joint-target motion, including Tray Detect, remains MovL.
Retain taught rates, ordered acceptance, CP, I/O/Stop/feedback gates and the ±1°
per-joint Home check. MovJ is a required owned motion service, participates in
late-response Stop containment and uses its returned queue ID for completion
and Auto Run handoff. Same Cartesian pose with a different wrist turn is not
Home. Keep the Home detour between placements and next picks for this change.


Rule 231 removes Home only between a successful Auto Run placement and its next
nonempty fresh Pick. After every placement command is accepted, append the next
entry/pre-pick/final-pick directly from tray retract at Home Z, without an added
arrival wait. Keep entry OPEN at 50% and final SUCK at 20%. The existing pre-pick
MovL (second appended command) supplies the handoff ID; entry MovLIO has no ID.
Keep the old source until advancing execution reaches/passes that ID and neutral
outputs/raw DI1 LOW have been observed since placement admission. Process this
handoff before old-placement output/drop interpretation on new-pick feedback.
Count once, cancel old unused poses, install the fresh ledger and arm acquisition.
Slow perception confirms/counts retract and waits unheld, then also picks directly;
retain that completion context so ordinary initial Home cannot be reintroduced.
Preserve initial Pick Home, final quantity Home, empty-result retry Home and other
return/recovery routes, all using rule-230 MovJ. Keep fresh parallel detection,
ordered admission, source validation, Stop/drop containment, I/O and retry budgets.
Supersede earlier Auto Run Home-between-cycles rules, including rule 230's retained
detour; manual Pick/Place and active-drop return geometry remain unchanged.


Rule 232 makes every saved Tray Detect destination absolute joint MovJ with
mode=true and all six exact taught angles, without modulo wrapping. Successful
Pick retains its linear lifts and Safety Z exit before this MovJ, with taught
travel rates; held Continue shares it. Explicit Tray Detect Position and Place's
observation positioning use the same target at their existing 100% speed and
taught travel acceleration. Preview shares the planner. Preserve ordered queue
acceptance, returned command-ID execution, exact ±1° joint/idle arrival, held
outputs, Stop/drop and freshness gates. Supersede prior Tray Detect MovL rules,
including rule 230's exception. Keep placement approach, release descent and
queued upward retract linear MovL/MovLIO with unchanged timed I/O. Do not add
Home, intermediate arrival waits, service calls, settings or teach-schema changes.


Rule 233 adds controller-owned monotonic elapsed seconds for Auto Run. Start
when the accepted action begins; include detection, motion, retries, pauses and
final Home, or failure/Stop/Return handling when ending early. Publish elapsed_sec
in native AutoRun feedback/results and auto_run_elapsed_sec in ControllerStatus.
Freeze at the terminal result, log the seconds and retain final/partial count and
duration in status until another run or controller restart. GUI shows one decimal
second as Elapsed while active and Total afterward; unavailable status must not
appear as a live timer. A new run starts at zero; no disk persistence, new thread,
blocking timer, motion call or changed count/queue/Stop policy is permitted.
Rebuild interfaces/controller and restart status/action clients together.


Rule 234 changes nearby-obstacle height reference to the detected item surface,
superseding rule 229's final Link6/standoff reference. Compare original depth
points to the item's base XYZ before tool/standoff compensation; retain inclusive
base-XY radius/base-Z height, saved distances and all original depth/ranking gates.
Share clicked, preview, Simulate Trigger and headless production logic. Rename
native maximum evidence to maximum_height_above_item_mm and reject old Link6
protocol evidence; no alias. UI label/help explicitly say item surface, excluding
standoff. Preserve schema 12, keys, defaults, operator values and robot targets.
No automatic artifact deployment, hardware commands or process restarts. This
observed-point filter does not certify end-effector or swept-path clearance.


Rule 235 visualizes the exact nearby-depth checks on Item Teach RGB/depth and
requested item debug PNGs. Project a solid yellow base-XY radius at item surface,
a dashed orange circle at surface Z plus height threshold, and connecting lines;
highlight blocking original depth points red, with the highest blocker marked X.
Labels give source ID, NEAR OK/BLOCKED and maximum height above item surface;
NEAR OK must not imply complete pick eligibility. Use each pane's intrinsics/
distortion and original snapshot; clipped/behind-camera drawing changes no check.
Reuse the existing 1 Hz native teaching pose calculation for live overlays, and
isolate the selected item's check on click without another prediction. Preserve
all-class size annotations and cyan depth-sampling semantics. Simulated/capped
request images retain blocked nearby diagnostics even for empty batches; valid
items excluded by the cap still receive no annotations of their own. Extend the
strict teaching-only image/cloud protocol without changing production candidate
evidence, ranking, schemas, settings, freshness gates, RViz clouds/TF or motion.
No new thread, inference, hardware command, automatic archive or restart.


Rule 236 moves nearby-height checking after ordinary geometry and final rank
selection during candidate acquisition. Check in centroid-distance/confidence/
source-index order; skip blocked candidates and stop as soon as the requested
number pass, or exhaust the list. Lazily build the original depth scene once;
do not scan leftover candidates. Return only checked poses and report remaining
geometric source IDs as unchecked, distinct from rejection. valid_count is the
checked returned count, not an exhaustive image count. Live teaching/RViz uses
the taught pose_candidates limit too, superseding rule 133's all-valid-poses
preview. Preserve all-class image annotations; only checked poses get TF/markers
and nearby overlays. Click inspection still checks that one selected item.
Keep rejection diagnostics for tested blockers, including an empty batch.
Share GUI, Simulate Trigger and headless acquisition. Preserve item-surface
height reference, radius/height inclusivity, original depth points, geometry,
ranking order, candidate evidence, schemas and controller motion/retry queues.
No extra inference, per-pick validation service, controller depth scan, new
setting, hardware command, automatic artifact deployment or process restart.


Rule 237 adds schema-13 geometry.depth_frame_count (1/3/5, default 3) and
floor-relative item clearance, superseding rules 229/234's base-XY/Z and
outside-bin interpretation. Use each original measured point's own depth and
registered optical model. Express platform Z=0 in camera coordinates; evaluate
floor depth at each point's physical camera XY and measure height along camera Z.
Within/on the saved camera-XY radius reject maximum nearby height minus candidate
surface height at/above the saved threshold (150/60 mm defaults), excluding
standoff. Include physical outer-bin boundary and inset margin; exclude outside
outer bin. Preserve candidate containment/body checks and ranked early stopping.
Invalid calibration/floor/boundaries/candidate/local depth fails closed.
Production/simulation require distinct advancing post-request frames, existing
freshness/sync/CameraInfo/source gates and at most 0.05 mm/0.05 degree camera motion
across the bundle. Reacquire moving bundles within the same deadline. Select RGB
near the depth midpoint. Apply item-only max(500 mm, configured minimum) and saved
maximum before a strict-majority per-pixel median. Preserve float32 millimetres,
invalid pixels, spatial median/MAD and identical snapshot use across pose, clearance,
teaching, clicked inspection and debug images. Newest depth stamp is returned;
record all stamps and explicit platform_floor_camera_z_v1 heights/difference/counts.
Reject incompatible native evidence; ROS layouts stay unchanged. Shared/tray depth
limits are unchanged. Older profiles require explicit unarmed GUI review/Save and
manual deployment; never rewrite operator artifacts automatically.
Measure validation, wait, capture/median, YOLO, geometry, clearance, rendering,
transport and controller stages; legacy inference_ms remains aggregate processing.
Skip production images when debug is off. Reuse per-capture original scene geometry
and bounded camera-only projections; never cache eligibility. Parsed caches key
verified content, retain boundary content reads/hash and dynamic binding checks,
and share one verified item/model snapshot within a controller validation pass.
Fresh typed Pick/Place/Auto Run status suspends new Item Teach background YOLO/voxel
jobs read-only, preserving raw RGB and age-labelled displays. Existing work finishes;
fresh idle resumes, missing/stale status falls back to request priority. Headless
stays independent; no new executor/hardware client, motion/retry change, automatic
restart or deployment. Preserve YOLO 448 and controller bounded empty-batch retry.

Rule 238 requires complete RGB/depth bundles for project registered-depth launches:
derive frame_aggregate_mode=full_frame from depth registration (ANY when disabled),
without a new .env key. Both direct GUI and supervised/headless paths must agree.
The intentional Orbbec software-D2C patch also discards depth-containing framesets
before aligned publication/point-cloud queues unless alignment completed and the
current depth dimensions/intrinsics match its RGB target. Preserve separate raw
depth diagnostics and distinct RGB/depth distortion; never substitute startup K.
Keep detector calibration-change invalidation, hardware alignment/C2D behavior,
operator artifacts and running processes unchanged during implementation. Use
isolated builds and synthetic SDK tests; activate with a manual camera rebuild
and restart. Document vendor patch provenance and validation in the diary.

Rule 239 supersedes rules 32/39's fixed-floor item sizing. Assume each item is
flat and parallel to platform Z=0. First sample its unchanged RGB pick ray with
the existing synchronized temporal median, native-depth mask/range/MAD/coverage
checks. Use that center's signed platform Z as the parallel measurement plane;
project the mask-derived rectangle/OBB there, fit its metric enclosing rectangle
and only then check taught physical length/width tolerance. Share this calculation
across RGB/depth preview, clicked/RViz poses, simulation and headless detection.
Missing/invalid depth leaves visible preview outlines with unknown size and blocks
production acceptance; never fall back to floor-projected dimensions. Keep bin
ROI/insets, the floor-based sampling circle, clearance checks, pick pixels/position,
model calls, schemas and controller motion unchanged. Do not fit item tilt, use
tool standoff as height, rewrite operator artifacts or restart running processes.

Rule 240 keeps Item Teach preview image buffers private until detection and all
available nearby overlays finish. Commit the completed RGB/depth pair together
on the GUI thread after settings and camera/source generation checks; retain the
previous completed snapshot with its original age while processing. Refresh image
pixmaps only for changed buffers, selection or pane size. Use compact fixed-height
status bands with detailed tooltips/status, source-ID-only nearby labels and no
long image legends or bin/inset captions. Preserve all geometric overlays, depth
filter decisions, 1 Hz scheduling, production priority and source invalidation.

Rule 241 makes Item Teach and Tray Teach camera panes passive live RGB/depth
between captures. Show the completed request's annotated pair for five seconds
after GUI acceptance for both Simulate Trigger and real requests served by that
teaching node, including empty results. New completed captures replace the pair
and restart the hold; RGB click resumes sooner. Use the same rendered buffers as
optional debug saves, independent of the disk-save flag, with one bounded result
handoff and no extra inference or request. Share compact three-line status bands
for source/result/count, age/processing/countdown and rejection summary; keep full
evidence in tooltips/diagnostics. No passive hit-testing against old detections.
Keep tray corner capture, background diagnostic/RViz work, source invalidation,
pose filters and controller authority unchanged. Real captures add no simulated
TFs; simulated TFs expire after five seconds too. Headless rendering still depends
on debug saving and has no GUI handoff. No cross-process image relay is introduced.

Rule 242 adds a stationary Home suction test to explicit Recover before gripper
reset. Preserve fingers; confirm exhaust OFF before suction ON without cycling
active vacuum. Any raw DI1 HIGH during the test means item/obstruction; a clear
test requires one second of fresh advancing feedback at idle taught Home. Clear
then performs the existing neutral reset/READY confirmation. Positive enters
PAUSED, retaining the existing managed worker/operation slot and gripper outputs,
while the Recover service returns. Offer the shared Return Item queue only for
an existing unreleased HELD/DROPPED source. Do not promote probe HIGH to a pickup
or infer a source after release/restart. With no source, require manual clearing.
Once DI1 is LOW, Continue (GUI: RETEST SUCTION) repeats the test, never the cancelled
job. Cancel remaining candidates; preserve uncertain source until test/return
completion. Keep pre-Home unknown-suction gates, source/feedback/output/Home checks,
response ordering and direct Stop during testing, waiting and return. No new
service, schema, setting, executor or automatic restart/retry; update diary/FSM
and regenerate its HTML/PDF. Validate with synthetic feedback only.

Rule 243 sets controller tray placement coverage to a fixed 30%, independently
of Item Teach's saved pick percentage. Place Item, Auto Run and Preview use the
shared placement sampling builder; preserve all other saved quality fields and
the physical diameter. Send the explicit fraction through the existing v3
contract so native sampling, provider validation and controller admission agree.
Keep full-circle denominator, tray containment, range/MAD filtering, freshness,
empty/zero-valid rejection and retries. No profile/interface change or operator
artifact rewrite; update diary/FSM and regenerate HTML/PDF.

Rule 244 adds independent controller pick ranking after strict detector-response
validation. Sort only the returned poses by straight-line 3D distance from taught
Home Link6 XYZ to the raw item surface XYZ transformed into base_link; exact ties
retain detector priority. Exclude offsets, orientation and live robot position.
Use the shared client for manual Pick, Auto Run/prefetch and Preview. Preserve
pose IDs, detector priorities/evidence and log their controller-order/distance
mapping. Freeze this order in the batch and ledger through retries, Pause and
drop/return recovery; preserve terminal exclusions and existing invalidation.
Do not alter Item Detect generation, filtering, cap or overlays, or request more
poses. No schema, interface, setting or motion/I/O changes; update diary/FSM and
regenerate HTML/PDF. Validate without live hardware commands.


1. This project is offline-first. `src/DOBOT_6Axis_ROS2_V4` and `src/OrbbecSDK_ROS2` are vendored source snapshots, not Git submodules. Do not recreate `.git` markers, `.gitmodules`, or submodule entries.
2. Do not silently edit vendored upstream code. Put integration and application code in separate packages and record any intentional vendor patch in the diary with its reason and verification.
3. The Dobot vendor profile is physical-CR10-only and excludes all Gazebo/robot-simulation support, MoveIt, vendor demonstration nodes, and servo control. Do not reintroduce Gazebo packages, worlds, launch/configuration files, URDF/Xacro simulation tags, simulation dependencies, any MoveIt package/configuration/plugin/dependency, `dobot_demo`, the `servo_action` package, or the Dobot `ServoJ`/`ServoP` interfaces. Do not reintroduce other robot-model URDF/XACRO files or mesh directories unless the user explicitly changes the scope and the diary is updated in the same change.
4. Preserve upstream license, notice, and attribution files. Never commit credentials, machine-specific paths, or generated `build/`, `install/`, and `log/` output.
5. Keep the repository buildable from its root with `colcon build`; test offline transfer with a Git bundle or source archive before declaring an offline milestone complete.
6. Hardware motion is safety-critical. Do not launch or command a real robot unless the user explicitly requests it and the robot/network/safety preconditions have been checked.
7. Every new or changed project rule must be written into the blueprint diary in the same change; the diary is the durable source of truth for project rules.
8. After each completed change passes relevant tests/checks, commit and push its scoped source, tests and documentation to the current branch's configured remote without asking again; the user explicitly authorized this standing workflow. Before each commit, run `git diff --check`, inspect `git status` and the staged diff, and update the diary when architecture, workflow, upstream revisions, or constraints change. Never include unrelated user edits, local configuration/logs/build output, station calibration/teaching artifacts or operator model weights in these automatic source commits. Verify the push and report the commit ID; if validation or pushing is blocked, report it honestly instead of claiming completion or discarding work. Do not force-push or rewrite shared history.
9. Project-wide runtime settings belong only in the ignored root `.env`, created from the tracked `.env.example`. Configuration is absolute: use canonical keys and strict `KEY=value` syntax; do not add compatibility aliases, fallback values, package-local configuration files, or alternate configuration workflows. The canonical shell loader and Dobot bringup launch must require and validate every key tracked in `.env.example`, including `ROS_LOCALHOST_ONLY`, all canonical `DOBOT_*` keys, and all canonical `ORBBEC_*` keys. `ROS_LOCALHOST_ONLY` must be exactly `1`, and the Dobot TCP timeout must be an explicit integer from 100 through 60,000 ms.
10. Every ROS package under `src/` must keep a package-local `README.md` beside `package.xml`. Preserve the Dobot and Orbbec vendor directory boundaries; record any package relocation in the blueprint diary.
11. Runtime datalogs belong in ignored `logs/<package>/events.jsonl` files owned by the package that emits them. Include UTC timestamps and cap each package file at 1,000 events by overwriting before the next record. Keep cross-package compilation in the standalone `scripts/compile_logs.py`; do not create a logger-only ROS package.
12. `motion_debug` must complete its documented one-time startup initialization before enabling GUI controls. The exact order is `StopMoveJog`, `DisableRobot`, `EnableRobot`, enabled-state confirmation, SpeedFactor `50%`, Tool `0`, Tool `1` TCP zero, and CP `100%`. By explicit user rule, only `StopMoveJog` and `DisableRobot` are best effort: log `WARNING` and continue to the next step on absence, failure, timeout, or missing Disabled confirmation. Enable and every setting remain strict and stop all later steps on failure; never add an automatic retry.
13. `motion_debug` scripts are saved and loaded only from root `config/debug_script/*.json`. Do not restore `config/motion_calibrate`, search alternate directories, or add a legacy-path fallback.
14. `gripper_control` requires the separately launched canonical `dobot_bringup_v4` process to already be connected and publishing when it starts. It must never start bringup, prompt for bringup, or remain in an indefinite waiting mode; missing canonical DO service, RobotStatus, or FeedInfo feedback must produce a package datalog failure and terminate startup. The user-confirmed wiring contract is `DO1` exhaust, `DO2` finger close, `DO13` suction, `DO14` finger open, `DI1` suction detection, and `DI12` finger fully open. This supersedes the earlier DO13 finger-close and generic DI1/DI2 contract. Current runtime remapping is pending: do not reuse the existing Grip/Release patterns with this wiring. The new controller must have an explicitly designed I/O sequence and full-open confirmation/failure behavior; low DI12 while intentionally closed must not be silently treated as a damage diagnosis. Do not add alternate channel mappings or configuration overrides. Application command ownership follows rule 29.
15. `orbbec_camera_launcher` uses the non-headless GUI as the only editor of the canonical root `.env` camera configuration. Camera launch is always an explicit operator action and every process requires `ROS_LOCALHOST_ONLY=1`; every Gemini 335 driver receives `enumerate_net_device=false`. Each row button may start exactly that one active, saved camera through the canonical vendor launch in a separate `/usr/bin/gnome-terminal`, with `device_num=1`, the complete saved vendor argument set, and no watchdog; this is the sole diagnostic exception to mandatory supervision, only one launch mode may run at a time, and the operator stops it with Ctrl-C in that terminal. The lower launch button starts the complete configured set headlessly through the read-only internal supervisor. In canonical `.env` slot order, each complete-set attempt scans every serial, starts Camera 1 alone, requires its color and depth streams within that camera's exact five-second deadline, and only then starts Camera 2 with a new independent five-second deadline; Camera 1 stays supervised while Camera 2 starts. Healthy operation supervises every owned process and all required streams. Failure of either camera stops the entire owned set; after shutdown completes, the supervisor rests exactly three seconds, rescans, and repeats the full ordered sequence, with exactly three total complete-set attempts. Only exact DDS endpoint GIDs observed from the current supervisor's owned processes may be ignored as retired during its retries; every unknown publisher remains a strict collision. Supervised partial operation, individual-camera retry, connection/supervision bypasses, manual budget resets, exponential backoff, and unlimited restart are forbidden. The GUI must use the supervisor's terminal event, not only the enclosing ROS launch return code, when reporting final failure. The GUI and supervisor may terminate only process groups they created; the GUI does not terminate the separate operator terminal. The GUI presents scan results and launcher activity in one chronological read-only log; detected-serial selection is informational and must not assign configuration or modify the clipboard. Selected log text is copied with `Ctrl+C`, and `Copy Log` copies the complete displayed log.
16. `dobot_rviz` is a read-only actual-robot and TF viewer. Its `robot_state_publisher` consumes only canonical `/joint_states` directly, and the configured root-namespace `DOBOT_ROBOT_NODE_NAME` must be the sole publisher. Do not restore a relay, `/joint_states_robot`, manual joint-state GUI, synthetic/zero joint positions, model/config launch overrides, or a non-hardware visualization mode. Missing, malformed, ambiguous, or stale actual joint feedback terminates the viewer and is recorded in `logs/dobot_rviz/events.jsonl`.
17. `motion_debug` keeps its in-window Scripts panel hidden by default. The `Scripts` button at the far-right edge of the full-width status header expands the main window to the right and reveals the panel; `Hide Scripts` retracts the same window without discarding loaded script state. Do not create a separate script window or restore an always-visible script panel.
18. `camera_calibration` has exactly two explicit ChArUco-only modes and no default: camera-to-hand solves `base_link <- <prefix>_link` with a fixed camera and board on `Link6`; camera-on-hand solves `Link6 <- <prefix>_link` with camera on `Link6` and board fixed relative to base. First-run mode/prefix are empty. Explicit Apply atomically saves validated fields to ignored `logs/camera_calibration/last_session.json` using only strict schema 4, minimum_samples 5, and no editable corner minimum. Restore is unapplied prefill only, never inferred from `.env`, auto-applied, or used to restore samples/solutions. Derive only color image, color CameraInfo and camera frames from the applied prefix; calibration has no depth subscription, synchronization, plane fitting, fusion or depth view. Camera-launcher configuration is independent. Keep exactly two ROS executor threads: tf2 uses its independent reentrant callback group while camera/joint callbacks remain serialized. Remove the target stability history/window; capture uses the latest valid RGB board pose without averaging, with age at most 0.5 seconds and robot TF/joints at most 1 second. Require five accepted samples for fixed Tsai automatic solving; the fifth and every later capture/removal recompute, and below five clears solution and stops TF. Overlay the solved reference-relative XYZ/RPY and FIT RMS on the single RGB view; broadcast only the calculated camera-link transform and never `charuco_board`. This package never launches cameras, bringup, RViz or robot motion. Keep existing timestamped filenames under `calibration/`, explicit save confirmation and bounded package events.
19. Every new or reworked project GUI with reusable operator setup fields must restore its last validated values from one authoritative ignored store. Project-wide fields continue to use only root `.env`; established named artifacts such as `config/debug_script/*.json` remain authoritative for their own content; otherwise use an atomically replaced `logs/<package>/last_session.json`. Restoration is prefill-only and must never auto-apply settings, start processes, issue hardware commands, replay samples/actions, or hide validation. A missing file is an explicit first-run state; a present malformed or unsupported state hard-fails without defaults or compatibility fallback. Transient status displays and one-shot command targets are not reusable setup fields.
20. `camera_calibration` follows the pinned MoveIt ROS 2 calibration logic at `3f9d48ebe843caf1de060bfafe78160585c7c26f` without MoveIt dependencies or robot motion. Stable, non-reused C# samples require both robot orientation AND camera-relative board orientation to differ by at least 5 degrees from every earlier sample; translation alone never qualifies. Name the conflicting ID and failed angular check on rejection. Remove samples only through confirmed operator removal or explicit undo/reset. Store one newest RGB board pose, robot transform and exact fresh canonical joint1 through joint6 positions in radians per sample, never frames or overlays. Fit residuals, previous-solution changes, coverage, leave-one-out and separately labelled adjacent-pair AX=XB RMS in mm/degrees are neutral consistency diagnostics; no holdout set. Leave-one-out is unavailable/non-blocking at five; from six a failed omission preserves full preview but disables saving. Strict schema-7 YAML includes pinned pipeline provenance and no depth/fused-pose fields. Load Calibration is the only sample-restoration workflow: reject schemas 1–6 explicitly without conversion, confirm replacement, validate at least five samples and all-prior angular checks, preserve sample order/IDs/joints, apply exact artifact settings and recompute using live internal camera TF. Preserve older files untouched. Never replay recorded joint positions or add a stored-result shortcut, older-schema reader or alternate writer. Hold-stationary and multi-axis-rotation guidance must be visible.
21. `camera_calibration` requires the exact vendored opencv-python 4.10.0.84 wheel, OpenCV runtime exactly 4.10.0 and recorded SHA-256. Verify/extract offline into its private install prefix, never install globally or fall back to system OpenCV. One lifetime multiprocessing-spawn child owns all OpenCV detection, drawing, pose and hand-eye calls, serializes requests, uses exactly one OpenCV thread with OpenCL disabled; the ROS/Qt parent never imports cv2. Use grayscale -> CharucoDetector.detectBoard -> CharucoBoard.matchImagePoints -> iterative solvePnP with color intrinsics/distortion, setLegacyPattern(True), CORNER_REFINE_NONE, minMarkers=2 and tryRefineMarkers=False. Require four non-collinear ChArUco corners; missing/insufficient/collinear detections or normal no-pose results block only that frame. Fixed CALIB_HAND_EYE_TSAI uses base_link<-Link6 directly for on-hand and its inverse for to-hand, with optical<-board RGB observations, then composes the camera-internal TF to output camera_link. Color input remains exact tightly packed little-endian rgb8. Invalid ROS color/CameraInfo frame ID, encoding, layout, timestamp, intrinsics or dimensions logs WARNING, clears readiness/invalid CameraInfo and skips that message without clearing an existing solution; only a later independently valid frame restores readiness, with no resizing/conversion/stale reuse. Malformed native detector output or runtime/module/checksum/thread/OpenCL/process/protocol/timeout failure remains terminal: stop TF, log fatal PID/signal/frame metadata and exit 1. Never restart/replace the worker or select an alternate runtime, detector, API or solver.
22. `item_perception_yolo/platform_teach` is the sole platform-origin workflow. It accepts one explicitly selected strict schema-7 `camera_to_hand` or `camera_on_hand` artifact directly from root `calibration/`, inherits its exact camera prefix and ChArUco settings, and uses the same private OpenCV 4.10 worker. Fixed-camera capture composes the calibrated `base_link <- camera_link`; on-hand capture strictly requires a live non-zero `base_link <- Link6` TF no older than one second and composes it with calibrated `Link6 <- camera_link`. Both then compose `camera_link <- color_optical_frame <- board` and define the board origin as `platform_reference`. There is no depth, joint subscription, averaging, hand-eye solve, or robot motion. Broadcast `base_link -> platform_reference` only after capture and stop on confirmed Retake. Save only strict schema-3 `calibration/platform_calibration_<UTC_TIMESTAMP>_<DOBOT_ROBOT_LAN1_IP>.yaml`, including camera SHA-256, selected mode, resolved transform chain, and the on-hand robot TF/timestamp when required, but no frames/overlays. `platform_teach`, `bin_teach`, and `item_teach` share strict unapplied package UI-state schema 6 at `logs/item_perception_yolo/last_session.json`; each validated write atomically changes its own section and preserves the others. Reject earlier schemas without conversion or fallback. Events share the package's bounded `events.jsonl`. Do not add automatic calibration selection or install the unaligned imported YOLO prototype nodes as current runtimes.
23. `item_perception_yolo/bin_teach` requires one explicitly selected strict schema-3 `platform_calibration` and its exact SHA-256-bound schema-7 camera calibration in the same mode. Fixed-camera geometry uses calibrated `base_link <- camera_link`; on-hand geometry strictly requires a live non-zero `base_link <- Link6` TF no older than one second and composes it with calibrated `Link6 <- camera_link`. It uses RGB/CameraInfo and live camera-internal TF and never launches a camera, robot, RViz, or motion. The operator explicitly selects one 5x5 dictionary and one measured common marker size. The private OpenCV 4.10 worker must detect the exact visible ID set `{0,1,2,3}` using `ArucoDetector.detectMarkers` and fixed `SOLVEPNP_IPPE_SQUARE`; missing or extra IDs block capture. Transform all four physical corners of each marker into `platform_reference`, select the unique corner farthest from the four-marker-center centroid, then order those four selected points clockwise from the lexicographically smallest XY point. Marker IDs never determine bin-corner order. One explicit fresh capture saves only strict schema-3 `offline_teach/bin_teach/bin_teach_<UTC_TIMESTAMP>_<DOBOT_ROBOT_LAN1_IP>.yaml` with four metric XY ROI points, source hashes, selected mode, resolved transform chain, and the on-hand robot TF/timestamp when required—never images, overlays, marker origins/poses, depth, or joints. Reject earlier schemas without conversion or fallback. Source-file/hash change, ambiguous outside corner, or non-convex/degenerate ROI blocks capture or saving; there is no fallback, automatic retry, alternate marker set, or legacy `bin_teach_yolo` launch alias.
24. The Orbbec configured set is always exactly two camera slots. `ORBBEC_CAMERA_COUNT` is forbidden in root `.env`, absent from `.env.example` and the GUI, and is not a launch override. Both canonical name/serial pairs always exist and must be complete before any launch. The complete supervised set uses internal `device_num=2`; the separate single-camera diagnostic terminal remains explicitly `device_num=1` and does not change the configured set.
25. `bin_teach` dynamically broadcasts the teaching-only preview chain `base_link -> platform_reference -> bin_corner_1..4` at a 10 Hz timer rate after a successful capture or an explicit confirmed Load Bin ROI. The platform transform comes only from the currently applied station artifact; corner frames match saved clockwise P1–P4 at `(x, y, 0)` with identity rotation relative to `platform_reference`. Never substitute detected Z because the artifact is strictly 2D XY. Retake, re-Apply, terminal failure, and node exit stop publication. Loaded geometry is not a fresh capture and cannot be saved as one. Teach nodes never launch RViz; future item detection will load the files independently of their preview TF, and its behavior remains undecided.
26. Platform/bin teaching artifacts require strict schema 3 and canonical reference convention `charuco_pick_corner_xy_v1`: the ChArUco board origin is the shared bin-mount corner, axes remain the board axes, and bin geometry consists of four metre XY points on platform Z=0. Across stations, reproduce the same physical origin, axis directions, bin size and bin offset; board/corner markers are approximately coplanar within each station, while absolute station heights may differ. Bin `teaching_provenance` records the original robot/camera/platform evidence but is not a deployment binding. Reading a portable bin file must not require its original station configuration or source files, nor use its source transform to place the ROI. The destination platform and its referenced camera remain strictly validated against the destination robot and file hashes. Explicit Load Bin ROI confirms the physical arrangement and previews unchanged XY geometry on that platform; it does not restore source settings, create a capture, or persist loaded geometry to UI state. Old platform/bin schemas 1–2 are preserved and rejected without migration; camera schema 7 remains unchanged; shared unapplied UI state uses schema 6. Keep this work limited to artifact generation, validation and teaching previews; do not implement or finalize item-detection/picking behavior.
27. Bin ROI teaching files are saved and loaded only in root `offline_teach/bin_teach/`, using `bin_teach_<UTC_TIMESTAMP>_<SOURCE_ROBOT_LAN1_IP>.yaml`. Camera and platform artifacts remain in `calibration/`. The bin writer, strict reader, GUI output label and Load Bin ROI dialog must use the canonical teaching directory; do not add an old-`calibration/` search, alternate path, fallback, or schema migration. Include the teaching directory in offline source/data transfers. This is a storage change only; item-detection/picking behavior remains undecided.
28. Explicit Load Bin ROI also shows a green unfilled border on live RGB, labelled `Loaded Bin Teach | <filename>` in green. Loaded mode skips marker detection and projects unchanged saved platform XY at Z=0 through only the current station's platform/camera chain and color CameraInfo. The existing isolated OpenCV worker projects 32 samples per edge with lens distortion; the ROS/Qt parent never imports cv2. Require RGB age at most 0.5 seconds, valid camera-internal TF and intrinsics/distortion (`plumb_bob` or `rational_polynomial`); on-hand additionally requires robot TF age at most one second. Missing/invalid/stale projection inputs or behind-camera geometry hide the border with an explicit reason, retaining the template; native worker failures remain terminal. Never substitute old projected pixels, source-station transforms, guessed intrinsics, or live detections for the loaded border. Fresh teaching keeps its yellow detection border. Artifact schemas, save/load locations, RViz preview and loaded-template save blocking remain unchanged.
29. Controller-ownership decision (implementation pending): the new project package/node `robot_controller` must be the sole application-level issuer of robot motion, stop, enable/disable, settings, and gripper/I/O commands through canonical Dobot bringup. Bringup remains the hardware transport and feedback provider, not a competing application sequencer. Teach, perception, `motion_debug`, `gripper_control`, and future applications must submit requests through this controller rather than call command services or robot TCP directly. The migration must preserve the established initialization/safety semantics while replacing direct application command authority; no controller-unavailable bypass is permitted. This rule is not yet enforced by the current runtime: existing direct clients must be migrated and verified before declaring sole ownership implemented. The newly imported `item_pick` is reference material for this work, not an approved drop-in execution path; do not inherit its I/O mappings, old teach paths, compatibility defaults, tray interfaces, or motion/retry policy without an explicit project decision. Physical emergency-stop functions remain independent of software ownership.
30. Item teach/detect/controller scope decision (implementation pending, superseding the earlier perception-scope deferral): item teaching loads a pretrained `.pt`, edits item pick/inference settings and creates the paired YAML/model under `offline_teach/item_teach/`; no training/annotation workflow or separate `item_detect_yolo_debug` runtime is to remain after the rework. `robot_controller` requests candidates from `item_detect`. The detector loads the item profile/model plus the current camera calibration, platform and bin artifacts, draws overlays, and returns valid item targets in `platform_reference`, not standoff-adjusted Link6 goals. The controller owns frame conversion and pick execution. `retry_limit` is saved per item and means total candidate attempts including the first; it also defines the requested pose count. Example: three requests up to three distinct valid ranked poses, not four attempts. A separate per-item `yolo.max_detections` caps YOLO detections per image before geometric validation/ranking; it does not guarantee that many valid items. Record all supported operator-adjustable item pick/YOLO settings and the exact model pairing in the item artifact. The user requires fixed vertical pick/place with `standoff_height` gripper compensation and no per-item tool-orientation/TCP handling. `zheight_offset` replaces `final_zheight` for the shared initial/final above-item position. `use_grip=true` enables finger control; false disables finger-control behavior and makes `grip_onpick` ineffective. With finger control enabled, `grip_onpick=true` closes on suction confirmation; false-case close timing, exact height references/fixed vertical attitude, sensor polarity/timeouts, candidate ranking/freshness/exhaustion, and supported YOLO task/runtime remain to be finalized. Do not invent these pending behaviors or import prototype defaults. A missed-suction next-candidate attempt requires confirmed completion of the previous final retract; hardware/feedback/stop/retract failures are not ordinary retryable missed picks.
31. The initial `item_perception_yolo/item_teach` GUI selects a non-empty `.pt` anywhere on the PC but writes/loads final strict schema-1 YAML and its same-stem SHA-256-verified model copy only in root `offline_teach/item_teach/`. Keep the original untouched, reject changed copies, never overwrite existing pairs and confirm successful saves. Model integrity is explicitly `file_sha256_only`: this stage never imports torch/Ultralytics, deserializes weights, runs inference or commands hardware. Store grouped item/model/units/home/motion/timing/gripper/retry/yolo fields and an exact non-executing controller contract. Home records exactly six finite canonical joint1–joint6 positions in radians from sole canonical `/joint_states` feedback, with source stamp/receipt no older than one second when recorded. Per explicit user decision, recording robot IP/publisher are provenance only, not a station binding; identical robots may reuse the same home joints without identity rejection. Loading never moves home; start/end home execution remains pending. Shared unapplied UI-state schema 4 stores only the selected item YAML filename and preserves platform/bin sections; named profiles remain authoritative. No compatibility reader or auto-application. The minimal `robot_controller` validates explicit profiles and publishes non-armed status only, has no Dobot command clients and never auto-selects a profile. Full sole-command ownership, inference, detection requests and pick/I/O execution remain pending, not implemented claims. The old training teach/debug sources are removed; imported `item_pick` remains reference-only under `COLCON_IGNORE`. Keep the existing platform/bin artifact schemas and geometry unchanged.
32. Item Teach and headless `item_detect` share one read-only pose-generation implementation. Explicitly load a trusted local model in one lifetime private CPU worker; discover actual class IDs/names and show checkboxes, default none for a new model. Save checked IDs only; no all-class or model/task fallback. Use pinned Ultralytics 8.4.150, NumPy 1.26.4, OpenCV 4.10.0 and the exact wheel/system lock; preserve upstream licenses, verify offline wheel extraction, do not install globally. The ROS/Qt parent never imports cv2/Torch/Ultralytics. Mask geometry uses its minimum-area pixel rectangle, OBB its oriented rectangle; expose only available outputs. Project that rectangle to fixed platform Z=0 and measure its metric enclosing rectangle (long X/height, short Y/width), independent of depth. Require selected classes/confidence, taught size tolerance, complete footprint inside the portable bin ROI and center inside the mask. The exact pixel rectangle center is never relocated. Project a reference-plane circle of diameter `pickdepth_radius` (initial 30 mm), sample registered 16UC1 mm depth within the circle/item, exclude invalid/range failures, apply abs(d-median)<=3*1.4826*MAD, and back-project the original center using median accepted depth before transforming the complete XYZ into platform_reference. MAD=0 retains only median-equal samples; no alternate filter. Depth overlay accepted pixels are black, rejected pixels red, with counts/filtered depth/age. Rank by bin area-centroid XY distance, confidence then source index for exact ties. Strict item artifact schema 3 stores geometry source/fields, checked classes and explicit editable quality limits (initial input 0.5s, sync 0.1s, robot TF 1s, request 10s, result age 2s, 30 accepted samples, 50% circle fraction, 200–1000mm depth). Reject earlier schemas without migration; preserve old files. Shared UI state is schema 6, with unapplied profile/prefix/station/bin filenames only; no auto-model execution, camera connection or arming. Armed ON exposes the canonical GetItemPoses service only for matching verified profile/model and applied hash-bound station/camera plus portable bin and fresh inputs; OFF removes it. Every request acquires a new synchronized observation, uses timestamped camera/robot TF, returns distinct ranked metre poses with IDs/timestamps/evidence and explicit shortage/no-items/errors, and never stale cached targets after disarm/settings changes. Three executor threads keep TF/sensors alive; native calls are serialized, bounded and never retried/replaced after terminal failure. Headless uses explicit artifact paths, trusted_model and armed flags and the same code. `robot_controller` may make a read-only pose request and log/return it, but has no Dobot command client, does not execute poses, and does not yet enforce sole command ownership. Teach/headless modes never run concurrently as the same pose service. No accumulating images/depth archive, hardware launches or automatic picks.
33. Item Teach starts in the explicit `All detections / measure` stage, followed by operator-selected `Filtered detections / pick checks`. All mode ignores production fields/class checkboxes and uses a fixed visible CPU inference profile: confidence 0.25, IoU 0.70, image_size 640, maximum 100, every verified model class. This is a teaching-only exception to rule 32, never a production fallback. Fresh RGB is required, but depth/platform/bin/home/filled production fields are not required for detection display. Optional dimensions use the applied hash-validated station/camera, matching color CameraInfo and RGB-time camera-internal TF; on-hand also needs RGB-time base_link<-Link6 no older than one second. Use exactly the same mask/OBB rectangle-to-platform-Z=0 metric enclosing-rectangle calculation as production, without size/class/ROI/depth rejection. Select the sole verified mask/OBB source on explicit model loading; when both exist require an explicit choice. Box-only models have no substituted metric geometry. Missing measurement inputs leave detections visible with an explicit measurement-unavailable reason, never guessed dimensions. Clicking a detection freezes that exact displayed frame/result, highlights its rectangle and shows X/height and Y/width millimetres at top-left; Resume Live releases the selection. Correct for actual pixmap scaling/letterboxing, reject margin clicks, and resolve overlap by smallest enclosing pixel rectangle area, descending confidence then frame-local source index. Do not track/reassign that index on a newer frame, auto-fill teach fields, infer a tolerance or persist images/selections. Edits disarm and invalidate saved-profile eligibility; All preview/selection stays available while editing production fields, whereas Filtered preview stops until explicitly re-enabled. Filtered/Armed/headless always enforce rule 32 production settings and fresh input checks. Arming from All mode must separately validate the exact saved production profile; neither all-mode detections nor frozen measurements may become service responses. Item artifact schema 3 and shared unapplied UI schema 6 remain unchanged: unsaved form edits are not autosaved, named teach YAML remains authoritative. Record changes in the diary and retain bounded logs, one isolated worker and no hardware actions.
34. Item Teach uses pick-oriented overlays, superseding rule 33's rectangle drawing: do not draw YOLO bounding/OBB boxes, enclosing-rectangle outlines or a selected rectangle. Keep mask shading; retain rectangle geometry only for measurement/clicks/pick pixel. Draw full centered red X along the pixel rectangle's long direction and green Y along its short direction, from opposite edge midpoint to midpoint; place a white dot with black rim at their common center. Selected frozen measurements use only a cyan ring at that dot. All-mode dots are geometric previews, not validated 3D poses; Filtered axes belong only to valid pick candidates. Depth sampling and pose mathematics remain unchanged. After explicit station/bin application, show the saved bin XY at platform Z=0 as a green unfilled Loaded Bin ROI border in All/Filtered and YOLO-OFF views. Use current hash-bound station/camera/bin data, exact RGB-time TF, color intrinsics/distortion and 32 samples per edge; on-hand also requires fresh RGB-time robot TF. Missing/changed/stale/behind-camera/out-of-view inputs hide the border with a reason, never stale or guessed projection. YOLO-OFF projection runs without model weights/inference/depth in the same lifetime private worker. The operator image_size field is removed: new profiles use internal 640 while existing strict schema-3 profiles retain their exact validated yolo.image_size; do not drop the artifact key, normalize loaded values, add defaults for missing keys or introduce schema conversion. Shared UI schema 6, production filters, explicit arming, no image archives, bounded logs and no hardware execution are unchanged.
35. Item Teach's corrected overlay contract supersedes rule 34 only for item outlines and completed inference display: keep segmentation mask shading plus exactly one minimum-area mask-derived rectangle; OBB uses its native oriented rectangle. Do not draw an additional axis-aligned YOLO box. Keep centered long-X/short-Y axes, the exact-center dot, cyan selection ring and the green loaded bin ROI on the same source RGB. All mode shows geometric previews; Filtered gives rectangles/axes only to valid candidates. Retain a completed All/Filtered inference image until the next result, even if CPU inference takes longer than 0.5 seconds. Display its actual source-frame age and inference time; over 0.5 seconds label RESULT SNAPSHOT and STALE and identify its ROI as historical, not a live projection. Never replace this annotated image with newer raw RGB while displaying its old detection count, nor paint cached annotations onto a new frame. YOLO-OFF live ROI projections retain the 0.5-second expiry. Input/source validation, model failure handling, source-change invalidation and all pose-service freshness/generation checks remain strict; display snapshots and frozen measurements must never supply pose-service responses. Model execution/arming remain explicit, and schemas, geometry, depth calculations, camera configuration and hardware behavior are unchanged.
36. Item Teach has no Apply Station + Bin ROI button. Selecting both named platform/bin files automatically validates the exact canonical artifacts/hash chain, derives the camera prefix from the platform's bound calibration and subscribes to that camera for read-only preview. Valid saved station/bin selections do the same on startup: this is a narrow exception to rules 19/22/32–35's unapplied/explicit-Apply requirements for Item Teach station-preview binding only. The green ROI appears automatically when fresh RGB, matching CameraInfo and required timestamped TF are available, with YOLO OFF or ON. Incomplete/invalid artifacts hide the border with a visible reason and no modal/repeated timer reload; explicit reselection can revalidate a corrected file. Changing selections stops YOLO, disarms and clears previous geometry/frozen views before validation. No automatic model execution, class/profile activation, arming, camera launch, robot command, artifact selection/search, schema fallback or changed-file hash adoption is allowed. All other GUIs and headless explicit-path behavior remain unchanged; last-session storage stays schema 6 and named item profiles stay authoritative.
37. Bin Teach uses one platform-plane geometry for teaching and loading, superseding rule 23's marker-PnP ROI coordinates. Keep exact 5x5 IDs 0–3, measured common size, marker-axis IPPE poses, freshness checks, source hashes and strict capture rules. The existing lifetime private OpenCV worker undistorts all 16 detected image corners using color CameraInfo; transform those optical rays into platform_reference and intersect the unchanged platform Z=0 plane. Select each outside corner and clockwise P1–P4 ordering from those plane intersections. Never derive ROI XY by discarding marker-PnP Z, flatten the platform relative to base_link, add depth/averaging, or fall back to another geometry method. Parallel, non-finite or non-forward intersections block the frame; native/runtime/protocol failures remain terminal. The yellow teaching border and green loaded border both project the saved-plane XY at Z=0 with 32 lens-distorted samples per edge. Keep camera schema 7, platform/bin schema 3, UI schema 6 and all artifacts unchanged; old bin files load their original XY and require explicit recapture for corrected geometry, never automatic repair/migration. The physical corner markers must lie on the taught plane. A pixel round trip is a consistency diagnostic, not independent metric accuracy; correct plane calibration and the same local bin arrangement are still required for station transfer.
38. Item Teach's loaded-bin edge uses the same native-import-free 32-samples-per-edge platform-Z=0 construction and platform-to-optical conversion as Bin Teach's fresh/loaded borders. Preserve the destination platform's full tilt/height and saved metric XY unchanged; never place a portable template with its source-station transform or require source calibration files, live markers or depth for the border. Keep automatic selected station/bin validation and read-only camera preview, simultaneous green ROI and YOLO annotations, strict fresh projection inputs and age-labelled completed-result snapshots. The shared native renderer also serves headless detection without changing pick/depth/filter/service policy. Keep existing artifact/UI schemas and files, require explicit selection of a newly re-taught bin YAML, and do not auto-resize, flatten, search for a replacement artifact or migrate geometry.
39. Item Teach has one RGB/registered-depth detection view: remove Detect All, Filtered / Pick Checks, the mode dropdown and Resume Live button; retain explicit YOLO ON/OFF and Armed controls. Show all verified model classes under the visible confidence/IoU/max_detections, including size failures. Initial form values are explicitly 0.25/0.70/100, never runtime fallbacks; preserve internal image_size 640 for new profiles and the exact loaded size. Keep mask shading and exactly one mask-derived rectangle or native OBB with centered X/Y axes and dot. Item borders are green within the taught platform-Z=0 length/width tolerance, red outside, gray when dimensions or plane measurements are unavailable; green is only a size check, not a pose-valid claim. Missing registered depth leaves RGB detections visible with a reason but cannot supply a pose. Clicking freezes the exact displayed geometry and synchronized RGB/depth/RGB-time-TF snapshot and calculates only that item using the same class, size, ROI, center and MAD depth checks as production. Never predict again or substitute newer frames/depth/TF for that selection. Enforce input limits at acquisition and result_max_age_sec at click/completion; expired selections require resuming live, not retries. Show measured dimensions and platform-relative XYZ/yaw; accepted depth pixels are black, rejected red. Only a successful click publishes the teaching-only frozen base_link -> item_teach_selected_item TF at 10 Hz, composing the destination platform's full tilt/height without a duplicate platform_reference authority. It is not live tracking, a robot command or a service response. Click the image again to resume; selection/settings/source/arming changes, YOLO OFF, source/native failure and exit stop selected-TF publication. YOLO, class, geometry and quality edits pause an ON preview and apply after 300 ms without further typing; invalid inputs pause with a nonmodal reason, and incomplete dimensions are explicitly unchecked, never guessed or reused. Edits clear selections/old previews, discard queued/in-flight old-generation results, disarm and invalidate saved-profile eligibility without auto-save, model reload, worker restart, auto-start or auto-arm. Other form edits clear selection/TF and disarm without stopping detection. Preserve green bin ROI, model-load priority, bounded transient snapshots/events, strict native handling, artifact/UI schemas and named-profile persistence. Armed/headless production still uses fresh independent observations and never returns clicked/frozen targets. No hardware or RViz launch or motion.
40. Item Teach's cyan selection ring is the projected depth-sampling boundary, not a fixed pixel-radius highlight. Use pickdepth_radius as the existing millimetre DIAMETER (30 mm means 15 mm radius), centered at the immutable rectangle-center ray's intersection with platform Z=0. Share the native 96-point circle projection with actual depth sampling, including full platform/camera tilt and lens distortion; the ROS/Qt parent only draws returned points and scales them with the source image. Project for teaching display even when size fields/depth/pose checks are unavailable or rejected, provided valid calibrated geometry exists. Missing/invalid projection hides the ring with a reason; no fixed-size or guessed fallback. Diameter edits retain rule 39's clear-selection/re-inference behavior; click again to see the updated circle. Preserve schemas, accepted/rejected depth colors, sampling/pose math, TF/service safety and no hardware actions.
41. Item Teach is visual-first: one scrollable settings column, camera/model/live-inference/size settings before routine fields, larger resizable RGB/depth panes, persistent Load/Save actions and a collapsed optional bounded activity log. Layout changes never auto-enable inference/arming or persist unsaved fields. Registered depth must retain the same selected color optical frame, image dimensions and intrinsic K; its validated distortion coefficients may differ from raw RGB (Gemini 335 D2C depth is rectified). Keep both CameraInfo models in the exact observation snapshot. Project the same platform-plane sampling circle into each camera model; sample original depth pixels once, intersect their rays with the plane for circle membership, and reproject those rays into RGB for mask membership. Do not resize/interpolate depth, copy RGB distortion over depth, ignore differing distortion, infer a missing model, or support unregistered/different-K inputs. Back-project the unchanged RGB pick center with RGB intrinsics/distortion and the accepted median depth. Depth dots/outline remain in native depth coordinates, RGB overlays in RGB coordinates. This supersedes the earlier exact color/depth distortion-equality assumption only; retain freshness, MAD, counts, filters, TF/service safety, schemas and private-worker isolation. No vendor patch, camera-setting change or hardware launch.
42. Explicit Load Item Teach uses one combined form-replacement/trusted-model confirmation, then automatically queues its exact same-stem SHA-256-bound .pt and reads classes in the existing lifetime worker. No second Load Model click is required. Validate the YAML/model pair before queuing, again before native loading and at completion; require the recorded task, checked class IDs and geometry output to exist without substitution. Retain loaded settings/class selection and saved-profile eligibility unless edited or invalidated. Missing/changed/incompatible pairs fail visibly without a retry, alternate model or worker replacement. Paired loads take the same reserved next worker slot as manual model loads; disable overlapping load actions while queued/loading. YOLO and Armed remain OFF. Startup restoration must never deserialize weights; rule 48 preserves saved eligibility without activating execution. Direct Browse .pt / Load Model retains its explicit trust confirmation. Preserve schemas, artifact paths, camera-preview behavior and no hardware actions.
43. Item Teach shows RGB and registered depth side by side in a horizontal, resizable split. Remove the duplicate above-video help/settings paragraph. Put result/frozen status, inference settings, size/selection/pose feedback and source age in a readable, wrapping black band at the top of each image pane, below its heading, not painted across camera pixels. Clear obsolete bands when their view is unavailable. Status-band clicks must not select/release items; preserve image-only click mapping through the actual centered pixmap after resizing. Mirror mask shading, size-colored mask/OBB outline, long-X/short-Y axes, exact pick dot and loaded bin ROI onto depth using its own CameraInfo model. Project sampled edges/rays in the existing private worker, never copy RGB pixel indices or resize depth; missing station calibration still allows registered mask/axis visualization but not guessed metric geometry. On a click, keep BOTH views frozen on the displayed RGB/depth observation until another image click; calculate only the selected pose from those exact data, with existing filters and freshness limits. Keep all item outlines in the frozen depth view; selected sampling circle is cyan, accepted points black/rejected red, and report the same selected pose/dimensions in both panes. No fresh-depth substitution, new YOLO prediction, tracking or automatic pose requests. Platform/Bin Teach use a matching visual-first single setup column, larger RGB area, persistent Save/capture actions, compact guidance and expandable calibration/status details. Preserve explicit Apply/capture/load/retake/save gating, preview TF semantics, schemas, settings persistence and all hardware prohibitions.
44. Item Teach saves strict schema-4 artifacts using retry.pose_candidates, the maximum number of ranked poses requested for controller retries, not implemented robot retry execution. Keep the retry group, integer 1–1000 and pose_candidates <= yolo.max_detections; controller requests this count and detector enforces it, returning up to that many valid poses. Production loaders reject schemas 1–3 and retry_limit without aliases. By explicit user exception to rules 19/31/32/33/42, only the GUI may recover old/partially invalid item files into an unarmed draft: retain independently valid known-format fields; map an unambiguous schema-1–3 retry_limit for editing only; blank missing/invalid/conflicting/unknown-unit values, mark unknown flags partial and require explicit choices. Never reuse previous form values, infer missing dimensions/units/home joints or silently default corrupted fields. Malformed/ambiguous YAML or unknown formats yields a blank draft with reasons. Clear invalid home records as a whole and clear missing/tampered/ambiguous paired models; only SHA-256-verified same-stem models may auto-load after the explicit trust confirmation, with queue/completion hash checks. Native metadata governs recovered task/classes/geometry. Startup recovery is prefill-only with no weights executed. Display recovery status and cleared-field reasons; recovered files never set saved-profile/controller/arming eligibility. Require correction and explicit Save of a new strictly validated schema-4 pair; preserve originals, no automatic conversion or overwrite. Shared UI schema 6 stays strict and stores only named selections, never recovered field values. Platform/bin/camera formats, worker isolation, fresh-request semantics and no hardware execution remain unchanged.
45. Item Teach has exactly the main control row YOLO Detect ON/OFF, Simulate Trigger, Armed ON/OFF. Simulate Trigger is a one-shot local action, not a service client, automatic arming or robot request. Require YOLO ON, the exact saved/loaded valid schema-4 profile/model/settings, applied hash-bound station/bin and fresh synchronized inputs; recovery drafts must first be corrected and saved. It works with Armed OFF or ON without creating/removing the service. Real and simulated triggers share one serialized acquisition/inference/filter/ranking/typed-response implementation, taking a new RGB/depth pair after request arrival and timestamped TF, with the same deadlines, generation/hash checks, candidate cap and shortage/no-items/failure outcomes. Queue only one simulation behind current GUI work with visible progress and a deadline starting at the button click; model loading retains priority, and a concurrent production request returns BUSY rather than interleaving native work. Freeze the successful simulated RGB/depth pair and show only returned P1..Pn candidates (up to retry.pose_candidates), their masks, one rectangle/axes/dot and diameter-based cyan rings on both views, plus the green bin ROI. Excluded candidates leave no annotations; null/invalid/MAD-rejected pixels remain red and accepted samples black. Put pose/count/age feedback in the top bands, limiting expanded details there to the first three poses to preserve image space; all returned poses are labelled on-image and listed in the bounded Activity log. Empty successful batches freeze the pair with ROI and NO_VALID_ITEMS, never old poses. Click RGB again to resume/cancel; setting/source/arming/model changes, YOLO OFF, native failure or exit cancel pending results and clear frozen batches. Frozen views are historical only, never production responses; no batch TF, tracking, image archive, retry/replacement of native work, robot commands, new schema or artifact rewriting.
46. Item Teach compares the selected validated platform artifact's SHA-256 with the bin file's recorded teaching_provenance.platform_calibration.sha256. If different, show a persistent, nonmodal amber warning beside the station/bin selectors with original and selected filenames and full hashes in its tooltip; record one bounded warning event per applied/restored selection, never per image or timer tick. Clear the notice on matching selections or cleared/rejected binding. This is file-identity evidence only, not automatic physical-alignment validation or a source-station deployment restriction. Keep portable reuse allowed, place unchanged XY using only the destination platform and remind the operator to verify origin, X/Y directions, bin size and placement. Never load missing source files to compare, use source transforms, auto-correct geometry, auto-enable inference/arming, relax destination hash checks, change schemas or rewrite existing artifacts.
47. A successful Item Teach Simulate Trigger publishes every returned candidate as a teaching-only dynamic TF at 10 Hz under base_link: item_teach_candidate_1 through item_teach_candidate_N in the same P1..Pn priority order as the frozen images, capped by pose_candidates and not by the three expanded text entries. This supersedes rule 45's no-batch-TF restriction only. Compose each response's platform-relative pose with the selected platform's full transform exactly as for item_teach_selected_item; never publish another platform_reference authority, use live tracking/robot TF to move frozen poses, add standoff/TCP handling, launch RViz or command hardware. Validate response frame, distinct IDs, ordered priorities, pose values, snapshot age and source/profile/generation before atomic installation. Clicked and simulated previews are mutually exclusive; replacement/empty batches never keep publishing surplus or old candidates. Resume, settings/source/profile/model/arming changes, YOLO OFF, native/fatal failure and exit stop all preview publication. The existing timer independently checks source/profile identity and epoch even while Qt is busy; only broadcast timestamps refresh while a valid frozen preview is held. State holds transforms and identity evidence, not another image archive. Explain frame names in both top bands and log the batch/frame mapping in bounded events. TF consumers may retain old cached frames briefly after publication stops. Preserve real/headless service behavior, candidate/pose mathematics, schemas and operator artifacts.
48. A complete strictly validated loaded Item Teach profile, including the named startup-restored profile, counts as saved without a redundant Save before Simulate Trigger or Armed. This supersedes only the saved-eligibility aspect of rule 19/42 prefill; model trust/loading, YOLO activation, arming and fresh production inputs remain explicit and strict. Edits invalidate eligibility and disarm but retain the loaded document's filename, original known item name and observed YAML/model hashes as its save target. Confirmed Save updates that same YAML/.pt pair when the item name is unchanged; a changed name or new document with unknown original name creates a new timestamped pair, then becomes the save target. This supersedes rule 31/44's no-overwrite/new-pair-only rule for explicit saves. GUI recovery drafts still require correction and a strict schema-4 Save before pose generation; same known name may replace the old artifact, but loading never writes. Before overwrite, verify no external YAML/model changes, stage/verify the new data and retain one hidden .<stem>.previous.zip with the previous YAML and model when present. Unchanged weights remain untouched; replacement weights publish before the YAML commit marker, with rollback on YAML write errors and strict hash rejection of any crash-interrupted mixed pair. Backups are manual recovery data, never alternate runtime inputs. Do not overwrite external source weights, follow target symlinks, autosave edits, auto-execute a model/robot or add production compatibility readers. Preserve item schema 4, shared UI schema 6, canonical directories and all other teach-node behavior.
49. Item Teach has no Validate Saved Profile in Controller button, controller parameter client or controller-request polling. Teaching remains independent: Save/Load, local simulation, read-only armed pose service and teaching TF retain their validation and behavior. Configure robot_controller explicitly through its own item_teach_file launch argument/standard ROS parameter interface; its strict profile validation and separate read-only pose-request trigger remain available. This supersedes the historical GUI-to-controller validation workflow only, not controller command-ownership or production safety rules. Before implementing robot execution, summarize the current non-actuating controller and resolve outstanding fixed-vertical height/attitude, home/travel/place safety, I/O/sensor confirmation/deadline and retry/retract decisions; do not invent pending policies or claim sole-command ownership is enforced. No automatic picks, hardware launches/commands, schema changes, artifact rewrites or native-worker changes belong to this UI removal.
50. Item Teach and headless Item Detect automatically select station calibrations only from root calibration/: choose the newest canonical UTC-timestamped platform file for the root .env DOBOT_ROBOT_LAN1_IP, derive its camera prefix, then choose the newest canonical UTC-timestamped camera calibration for that prefix across both hand-eye modes. Filename timestamps, never mtimes, define newest. Require the chosen schema-3 platform and schema-7 camera to remain the same exact hash-bound pair, including mode, settings and mounting transform; a newer same-prefix camera not referenced by the newest platform requires platform reteaching, never mixing or older-compatible fallback. Ignore newer platforms for other robot identities and identifiable cameras of other prefixes; missing, ambiguous, noncanonical, symlinked, malformed/unidentifiable catalog entries, invalid selected artifacts or changed selected hashes fail visibly without fallback. Select once at startup and on an explicit GUI Reload Latest Calibration/bin selection, never periodically or during a pose request. The GUI displays read-only platform/camera paths, ignores older platform prefill as an authority and retains explicit portable bin selection; successful automatic station/bin binding may reconnect read-only preview but never loads weights, enables YOLO, arms, launches hardware or rewrites artifacts. Reload/source failure clears old bindings, views and teaching TF and disarms. Headless no longer accepts platform_teach_file; explicit item_teach_file, bin_teach_file, trusted_model and armed controls remain unchanged. This supersedes only Item Teach/Detect's earlier explicit/no-search calibration-selection requirements, not Platform/Bin Teach's explicit calibration workflow or other GUI prefill rules. Preserve current schemas, hashes, portable bin placement, pose/service mathematics, private workers, bounded logs and transient image storage. Flat runtime_teach catalog/runtime-debug workflows remain a separate pending change.
51. robot_controller shares explicit Home/pick actions across GUI and headless, with immutable debug=true by default (TF-only, no Dobot command clients or initialization). Real mode requires explicit debug=false and one-time StopMoveJog, DisableRobot, EnableRobot/enabled confirmation, SpeedFactor 100%, Tool 0, Tool 1 TCP zero, CP 100%; only the first two are best effort, all later failures terminate startup, no retries/automatic Home/pick/bringup. GUI selects strict offline item/bin artifacts and restores only unapplied schema-1 filenames in logs/robot_controller/last_session.json; Item Teach alone supplies Home. Headless strictly loads exactly one schema-4 item, its verified same-stem .pt and one portable schema-3 bin from flat root runtime_teach/ without file overrides, partitions, ambiguous selection or compatibility readers. This stage does not finalize multi-item/tray deployment or Item Detect's pending flat runtime catalog/debug archival workflow. Controller uses rule 50's latest strictly bound station selector, full platform-to-base transform and independently armed detector's fresh ranked batch with matching profile/model/camera/platform/bin hashes. Never deserialize weights in controller. Derive Home Cartesian pose only from canonical static CR10 FK and taught joints; real current GetPose(user=0,tool=0) and every IK result must match that model before motion. Every Home first preserves actual XY/attitude and reaches Home Z using GetPose plus RelMovLUser in user/tool 0, then MovLIO joint-mode to exact taught Home joints; all pick/transit/retract segments use MovLIO. Hold Home attitude and base Z: pick=itemZ+standoff_height, initial/final=pick+zheight_offset, prepick=pick+prepick_height, retract=pick+retract_height. Require zheight_offset >= both prepick/retract and Home Z >= all candidate clearances; never repair settings or fabricate a collision-free path. Suction stays off until final approach; turn DO1 off and DO13 on, monitor active-high DI1 during final descent and stop/confirm fresh stationary feedback on acquisition, including while awaiting command acknowledgement. If nominal pick completes without DI1, wait the saved pick_settling interval then classify a miss. Retract only upward from actual stopped Z, confirming intermediate/final completion before advancing to another distinct still-fresh batch candidate. Only missed suction is retryable; stale feedback/candidate, stop/motion/I/O/sensor/retract faults cancel and block later commands, never automatically reacquire. Success holds at final retract with suction on, no automatic Home/place; explicit Home preserves suction. use_grip=false leaves DO2/DO14 untouched; true opens initially, grip_onpick=false stays open, true closes only after suction confirmation. DI12 full-open gating/damage diagnosis is explicitly deferred by user for this stage, superseding rule 14's full-open requirement for controller only; do not reuse old Grip/Release patterns. Require sole canonical fresh joint/status/FeedInfo publishers, advancing controller_timer, enabled fault-free feedback and user/tool zero for real actions. Reject duplicate controller/command providers and known legacy motion/gripper applications, but do not claim complete command-ownership migration/enforcement. Debug broadcasts only uniquely named robot_controller_debug_* targets at 10 Hz after explicit actions, not actual robot/platform TF; source changes/cancel/exit clear publication. Never launch hardware/RViz for verification. Preserve item/bin/platform/camera schemas, artifacts, private workers and bounded package events. Safety Stop requests and command acknowledgements are not confirmed motion completion; hardware validation/collision safety and legacy-client migration remain follow-up.

52. Item Teach saves strict schema-5 item artifacts with separate speed and acceleration groups, each containing exactly travel_percent, approach_percent, retract_percent as integers 1–100 and explicit percent units. New-profile speed fields start at 100/6/6, acceleration at 100/100/100; these are explicit editable form values, never defaults for loading missing fields. Travel covers Home (both relative-Z and joint MovLIO), XY transit, initial positioning and pre-pick positioning; approach covers only pre-pick-to-pick descent, retract covers intermediate and final retract including early-contact adjusted targets and missed pickups. Controller targets carry validated rates into every MovLIO and Home-height RelMovLUser param_value as v=/a= with global SpeedFactor 100%; do not add global per-phase setting calls, mm/s aliases, inferred rates or motion retries. Edits disarm/invalidate saved eligibility without issuing commands, auto-save or interrupting read-only inference. Preserve loaded rates, named overwrite/previous-version backup and paired weights. Production readers require schema 5 and reject schemas 1–4 without migration; GUI-only recovery may retain known independent fields but must blank missing/invalid/unknown-unit rates until explicit correction/Save. Shared UI schema 6 and camera/platform/bin schemas stay unchanged. User-authorized workstation teach/runtime updates change only schema, percentage units and rates, retaining other settings and recoverable originals; never commit operator artifacts/weights. Existing height, freshness, feedback, cancellation/Stop and no-hardware-test rules remain strict; slow speeds do not expand deadlines or make expired candidates retryable.

53. Every robot_controller Home compares the validated current Link6 base-Z with the canonical FK-derived taught Home Z. When current Z is below Home Z, retain the existing GetPose/RelMovLUser vertical rise at current XY/attitude followed by exact joint-mode MovLIO Home. When current Z is equal to or above Home Z, the user declares direct Home safe: omit RelMovLUser/home_height entirely and issue only the exact joint-mode MovLIO Home target. Debug and real modes must use the same branch; debug publishes robot_controller_debug_home_height only for the below-Home branch and always publishes the resulting Home target. Do not add tolerance, descend to Home Z before the joint target, change Home orientation/joints/rates, skip feedback/FK checks, infer a clearance path or weaken suction/cancellation/fault handling. This supersedes rule 51's unconditional preliminary Home-Z segment only. No hardware/RViz launch for verification.

54. robot_controller GUI and headless expose the same `/robot_controller/go_home`, `/pick_item`, `/stop` Trigger services plus `/set_live` and `/set_debug_images` SetBool services; GUI buttons must be clients of these services, never a private direct-action path. GUI starts Live OFF/TF-only with no Dobot command clients; its visibly red Live ON performs rule-51 initialization before READY, Live OFF is accepted only idle/not-holding, destroys controller command clients without sending DisableRobot, and every later ON initializes again. Headless has no `debug` launch override: it automatically starts permanently Live, performs initialization, rejects Live OFF, but never automatically Homes or picks. Home/Pick/Stop replies acknowledge asynchronous acceptance; `/robot_controller/status` is authoritative for completion and includes Live/headless plus `debug_images` and `debug_capture_status`. Debug Images is independent of Live, defaults OFF in both modes, and snapshots its value per pose request. When ON, `GetItemPoses.save_debug_images` directs the detector to atomically save exactly that request's already-rendered RGB and registered-depth overlay pair as PNGs under ignored root `debug/pick_img/`; never open another camera path or continuously archive frames. Absolute paths or a non-blocking save warning return in diagnostics/status; save failure must not change candidates or robot motion. Pick is a reusable function that validates the detector before travel, completes Home, requests exactly one fresh ranked batch, and attempts only its returned candidates in order. A suction miss completes retract and Home before the next candidate; success completes retract and Home with suction/fingers held; exhaustion completes Home and reports failure. Only missed suction retries; no automatic new batch. Stop is a reusable function: cancel remaining commands, send canonical Stop while Live, require acknowledgement and fresh stationary feedback, then if DI1 is ON require the remembered last pre-pick target, preserve suction while returning there, and only after confirmed arrival set DO13 OFF/DO1 exhaust ON plus DO2 OFF/DO14 ON when use_grip is true. DI1 OFF performs no recovery motion. Missing target, lost suction or Stop/motion/I/O/feedback failure never releases and fails closed; physical emergency stop remains independent. Item Teach's Armed ON button is also red as a conspicuous service-state indicator only. This supersedes rule 51's immutable debug/real launch mode and final-retract success policy, but preserves conditional Home rule 53, all geometry/rates/freshness/FK/feedback/ownership rules, schemas and no-hardware verification.

55. robot_controller follows Motion Debug's ordered startup preconditioning/enable/settings logic, retaining SpeedFactor 100%. Wait boundedly for strict services; only StopMoveJog and DisableRobot may continue on missing services, returned failure or absent Disabled confirmation. Every normal command waits for its predecessor's response before dispatch; an unanswered response timeout stops startup with the exact service named and forbids later commands, including best-effort steps. A late response never automatically resumes initialization; safety Stop stays independent to interrupt pending motion. This supersedes only the controller's previous best-effort unanswered-timeout continuation, not Motion Debug's separate rule 12. Show/log the current startup service and distinguish failed calls from post-startup readiness. READY requires final enabled/fault/pause/user/tool feedback validation; report all exact blocking fields/values (including isPauseCmdFlag) instead of the generic fault/pause/disabled message. Do not ignore paused/error/collision/disabled/nonzero user/tool feedback, automatically Continue a paused queue, change initialization settings, retry hardware commands, or issue hardware commands during verification.

56. robot_controller GUI Enable Robot must call the shared headless `/robot_controller/enable_robot` Trigger service. It is explicit and Live-only, requires completed startup settings, no active action/recovery/held item/ambiguous moving state/pending Stop, sole canonical command ownership and fresh fault-free idle Disabled/Enabled feedback with DI1 OFF. No teach file is required. Send only EnableRobot once, wait for its response, then boundedly confirm fresh idle enabled/fault/pause/user/tool readiness before READY; do not repeat DisableRobot/settings, resume paused queues or Home/pick. Persistent blocked feedback fails without retries, and incomplete/fatal startup cannot be bypassed. Final startup readiness also waits up to five seconds for coherent asynchronous feedback after settings responses, with no new command or safety bypass. Publish startup_settings_applied and asynchronous ENABLING/completion/failure status. Serialize Enable acceptance with Stop; all existing Live/response-order/freshness/cancellation/ownership and no-hardware-test rules remain.

57. Item Teach saves strict production schema 6 with exactly standoff_height, prepick_height and retract_height in motion; zheight_offset is removed, not an alias. Keep Home attitude/base Z: P=itemZ+standoff, pre=P+prepick, clearance=pre+retract. Complete the exact shared Home function before acquiring one fresh batch. Forward queue is item XY at Home Z, clearance, pre-pick at travel rates, then P at approach rates; return queue is pre-pick at retract rates, clearance and shared conditional Home at travel rates. Preserve Home Z >= clearance and actual-stop upward-only recovery. Preflight all endpoint IK/FK while idle, submit one motion at a time awaiting its response but not each waypoint arrival, then require fresh queue-idle/stationary tail pose/exact Home joints and motion-I/O feedback. Per-command cp=0 preserves corners and phase boundaries without changing startup CP 100%; no automatic Continue. use_grip=true opens DO2 OFF/DO14 ON at 50% of clearance motion. Final descent starts suction DO13 ON at its hardware start event with exhaust OFF. DI1 anywhere after suction starts interrupts the owned forward queue, requires acknowledged Stop/stationary feedback and no later descent, then recovers upward from the actual stopped pose. grip_onpick=true closes after this confirmed acquisition before retract; false closes DO14 OFF/DO2 ON at 100% of retract-to-prepick ONLY on confirmed DI1 success. use_grip=false never writes DO2/DO14. A miss never closes fingers, completes retract/Home before vacuum OFF/next still-fresh candidate; late DI1 during a missed return is a fault, not a retry. Suction/feedback/response/motion/I/O/Stop faults do not advance candidates. Canonical FeedInfo must include EnableStatus; enabled actions require it exactly 1. Vendor RobotStatus.is_enable means idle mode 5, so False during modes 7/8 is not a disable indication when EnableStatus is 1; idle readiness still requires RobotStatus enabled, and faults/pause/freshness/user/tool checks remain strict. Production rejects schemas 1–5 without conversion; GUI recovery blanks old retract_height because its reference changed, and requires explicit correction/Save. User-authorized workstation offline/runtime updates retain backups and weights, change schema/remove zheight_offset only, and are excluded from source commits. Preserve shared UI schema 6, calibration/bin schemas, service-driven Live/Stop, bounded events and existing motion/result deadlines; test only synthetic services/streams, never hardware.

58. robot_controller GUI has a global speed slider, integer 1–100, using the shared GUI/headless `/robot_controller/set_global_speed` (dobot_msgs_v4/srv/SpeedFactor) service. Every Live initialization still sets SpeedFactor 100%; explicit idle changes supersede only the historical always-100 runtime restriction, never modify taught per-command v/a or artifacts. Slider release/debounced keyboard changes call the controller, not vendor bringup directly; disable while Live OFF, initialization, active action/recovery or pending setting. Require completed non-fatal startup, sole command ownership, fresh enabled/fault-free/unpaused user/tool-0 idle feedback and no unresolved preceding normal/Stop response. An idle held item must retain DI1 throughout. Serialize with the action lock and normal response ordering; res=0 only after actual successful robot response, otherwise res=-1 with precise status/events. A failed/ambiguous accepted setting fails closed without automatic retry or later commands; late responses never restore READY or a known factor. Preserve independent safety Stop and all motion/result deadlines. Publish global_speed_percent (null if unknown/Live OFF) and global_speed_message. The slider is a transient robot-command target, not reusable setup or an item-teach field: no autosaved/auto-applied speed, new .env key or UI/profile schema. Test synthetic services/offscreen UI only; no hardware commands during verification.
59. robot_controller Live ON must itself execute the ordered startup EnableRobot and strict settings, with no separate GUI Enable Robot button; the existing headless `/robot_controller/enable_robot` service remains an optional explicit enable-only interface. If post-settings readiness stays blocked after its bounded feedback wait, Live makes exactly one guarded recovery attempt: canonical Stop, acknowledged stationary/empty queue, conditional ClearError for fresh fault/error mode, then EnableRobot with response and fresh idle/fault-free/unpaused/user/tool-0 confirmation. Initial error/paused mode may be cancelled/conditionally cleared before ordered startup. Explicit Stop/Clear uses the same guarded idle recovery after stopping an action; when startup settings or the global SpeedFactor confirmation are incomplete/unknown it may rerun the full ordered initialization only as that new operator action. The enable-only service cannot bypass an unknown global SpeedFactor. Once enabled with DI1 OFF and no held item, if an unchanged validated Item Teach Home is loaded, Stop/Clear must use the existing Home function from a new GetPose/FK-checked actual Link6 pose, with the rule-53 conditional vertical-clearance branch and queued Home target; unexpected DI1 blocks that transit. Without a loaded Home, re-enable only and tell the operator to load a profile. Never Continue/resume a paused queue, use stale queued EE pose, clear an alarm or re-enable before Stop confirmation, overlap unanswered normal responses, guess state from a returned service acknowledgement, or release/recover a held/DI1-active item through idle enable. The established DI1-active last-prepick return/release policy and second-Stop/shutdown cancellation remain unchanged. Ordinary startup/recovery refusal leaves GUI/headless FAILED and does not terminate the GUI; unexpected fatal errors still terminate. In Live FAILED, Home/Pick buttons stay clickable with a loaded profile/binding but their services must refuse motion and show the exact feedback blocker plus an operator prompt (including possible pressed emergency stop); GUI also prompts once per failed recovery. Only verified idle READY/HOLDING/NO_PICK permits actual motion. This supersedes rules 55–56 only for this explicit recovery and GUI-button change, retaining the five-second feedback/response limits, command-ownership/freshness/queue/suction checks, independent safety Stop and no-hardware verification. Record all attempts/failures in bounded controller events. Test synthetic paused/error/persistent alarm, stop sequencing, held-item gating, failed-action prompting and GUI/headless service equivalence.
60. The vendored Dobot bridge separates raw TCP GetPose/InverseKin replies into ROS `res` (error ID) and `robot_return` (only the brace-delimited six-value payload). `robot_controller` must validate `res=0` before strictly parsing `{v1,v2,v3,v4,v5,v6}` as six finite numbers; never require or accept a raw TCP error prefix/command echo in `robot_return`, silently fall back to a different parser, or bypass canonical GetPose/FK and InverseKin/FK checks. The taught six Home joints remain radians in Item Teach and are sent as degree values through MovLIO joint mode; pick/transit/retract points remain Cartesian mm/degrees through MovLIO pose mode. Synthetic service fixtures must model the ROS field, not the raw TCP line. Preserve all existing response ordering, motion/Stop/suction/feedback gates and hardware-free verification.
61. By the user's verified Dobot V4 wrapper/manual decision, MovLIO accepts `joint={...}` or `pose={...}` directly, so `robot_controller` must not create or call an InverseKin client as a prerequisite for Cartesian MovLIO. This supersedes rules 51, 52, 57 and 60 only where they require nearest-joint InverseKin/FK preflight or call the IK result a movement prerequisite. Keep successful-res/brace-only GetPose parsing, fresh current-joint FK agreement, exact taught-Home joint-limit/FK agreement, finite rigid target validation, relative Home-Z rise constraints, canonical user/tool zero, serial responses and full actual motion/output feedback confirmation. Cartesian reachability and joint branch are no longer prevalidated; vendor acceptance and actual completion remain mandatory, and rejection/timeout is a fault, not a retryable missed pick. Per explicit user selection, no-I/O Home/transit/retract targets continue to send MovLIO with `mdis=[]`; do not substitute MovL, insert fake/no-op DO events, or silently relax the tool-output policy. The vendored TCP/IP V4.6.5 manual says MovLIO requires at least one DO tuple, while the pinned wrapper omits empty `mdis`: physical firmware acceptance of this selected behavior remains unverified and must be checked under supervised commissioning before relying on real Home/Pick. Preserve schemas, queue timing, Stop/suction safety and no-hardware software verification.
62. When `robot_controller` successfully loads an Item Teach profile, calculate the canonical CR10 FK of its exact six taught Home joints once and cache both that Cartesian Home reference and its joint tuple in runtime memory. Clear them whenever the profile becomes unapplied/invalid, and reject an action if the freshly read unchanged profile does not contain that exact cached joint tuple. The cached FK is planning/reference geometry only (including Home Z, fixed vertical attitude and item-waypoint construction), never proof of the live Cartesian pose. A successful canonical `GetPose(user=0,tool=0)` response is the actual Cartesian pose for Home branch selection and queue origin; do not compare it with fresh-current-joint FK or reject motion because those two representations disagree. This supersedes rules 59–61 only where they require live GetPose/current-joint-FK agreement. Keep model-limit/FK validation of the taught Home target before dispatch. Joint-form Home completion requires fresh sole-publisher `/joint_states` with every actual joint within plus/minus one degree of its taught target, together with fresh enabled, fault-free, queue-idle/stationary feedback; do not additionally require Cartesian FK/GetPose agreement for that joint-form endpoint. Cartesian target completion requires actual FeedInfo `tool_vector_actual` within 5 mm translation and one degree rotation, with the same fresh enabled and queue-idle/stationary gates. Service success alone remains insufficient. Preserve exact response serialization, timeouts, user/tool zero, suction/Stop rules, debug TF-only behavior and hardware-free software verification.
63. A supervised real Home proved the V4.6.5 protocol requirement: `MovLIO(joint={...},user=0,tool=0,v=100,a=100,cp=0)` without an I/O tuple was rejected with `-20000` (parameter-count error) before motion. The vendor manual requires one or more `{Mode,Distance,Index,Status}` groups, and the unchanged ROS wrapper omits `mdis=[]`. Superseding rules 51–52, 57 and 60–62 wherever they prescribe empty-I/O MovLIO, `robot_controller` must create the canonical MovL client and dispatch every no-I/O linear target through MovL with the same joint/pose mode, coordinates and user/tool/v/a/cp values. Dispatch MovLIO only when the target contains at least one genuine saved controller motion-I/O event, and pass every such event unchanged. Never send empty `mdis`, invent a dummy/redundant DO event, split a required timed event into a host-side DO call, or patch vendor bringup around the robot protocol. Home uses joint-mode MovL; no-I/O Cartesian transit/pre-pick/retract uses pose-mode MovL; event-bearing finger-open, suction-start and confirmed finger-close segments remain pose-mode MovLIO. MovL is a motion command for cancellation, late-ack Stop containment, command-ownership checks, response serialization and failure reporting exactly like MovLIO/RelMovLUser. Preserve cached Home geometry, taught-joint and rigid-target validation, rates, queue order, completion tolerances, output confirmation, suction/Stop policies and all schemas. Synthetic transport must reject/assert against empty-I/O MovLIO so firmware-incompatible requests cannot falsely pass again. Do not modify the vendor MovL/MovLIO service, parser or handler; verification must not launch or command hardware.
64. `robot_controller` v2 is one headless hardware-control node with two primary native ROS actions, `/robot_controller/go_home` and `/robot_controller/pick_item`, whose goals carry the active immutable configuration SHA-256; Pick additionally carries one-shot debug-image capture and always takes `pose_candidates` from strict Item Teach schema 6. Supporting typed services are `/robot_controller/startup`, `/recover`, `/stop`, `/configure`, and `/set_global_speed`; reliable transient-local typed `/robot_controller/status` replaces JSON. Remove the old Trigger Home/Pick, Live, Enable Robot, profile-validation, pose-proxy, and global debug-image endpoints without compatibility wrappers. Split `robot_controller` (only Dobot client owner), `robot_controller_preview` (TF-only with no Dobot client/discovery), and `robot_controller_gui` (API client only). GUI launch starts all three; headless starts only controller and strictly loads the flat `runtime_teach/` catalog. Neither launch mode enables, recovers, homes, or picks: explicit Startup is mandatory. The single operation executor uses `UNCONFIGURED`, `INACTIVE`, `STARTING`, `READY`, `HOMING`, `PICKING`, `HOLDING`, `STOPPING`, `RECOVERY_REQUIRED`, `RECOVERING`, `HELD_UNKNOWN`, and `FAULT`. Process sole canonical RobotStatus, FeedInfo and joint feedback conditionally at its live rate; normal Dobot calls are response-serialized while independent Stop may pre-empt. Use five-second discovery/response, one-second feedback age, two-second expected-mode, three consistent pause/error samples, 200 ms READY coherence, three-second no-progress, 300-second motion cap, 300 ms stationary completion, one-degree Home-joint, and 5 mm/one-degree Cartesian limits. Startup verifies ownership/feedback, best-effort StopMoveJog, strict Stop/empty-stationary queue, rejects cold DI1 as HELD_UNKNOWN without changing outputs, disables, conditionally clears alarms, enables, performs at most one bounded Stop→Enable pause correction detected immediately after Enable or, if still unused, once more after settings/output reset before READY, applies SpeedFactor 100/User 0/Tool 0/Tool-1-zero/CP 100, resets canonical outputs only when unheld, and returns only after READY; it never moves Home. A final READY refusal must report its exact live blocker rather than only a timeout. Recover performs Stop, conditional ClearError, Enable/settings/READY and never Home, retaining the last confirmed global speed and continuously checking trusted held-item DI/output feedback. Stop and action cancellation invalidate the command generation, issue and confirm independent Stop, preserve all gripper outputs, monitor held-item integrity through confirmation, never release/Home/resume, and finish RECOVERY_REQUIRED; a late motion acknowledgement causes another confirmed Stop, and an unconfirmed Stop faults. Idle supervision that observes an unexpected running/nonempty queue must use the same independent Stop path before requiring recovery. Cold DI1 remains HELD_UNKNOWN until an explicit Stop observes manually cleared DI1, after which Recover is permitted. Home from READY or trusted HOLDING reads fresh GetPose, uses upward-only RelMovLUser when below cached taught Home Z, and finishes exact taught joints using joint-mode MovL while supervising held suction. Pick from READY/DI1-clear runs shared Home, requests one fresh hash-matched detector batch, transforms platform poses to base, tries up to taught `pose_candidates`, and uses the established schema-6 geometry/rates/timed gripper rules; each miss fully retracts and returns Home before the next pose, success retracts and returns Home holding, only missed suction is retryable, and every other failure Stops without advancing. Preserve Item schema 6, Bin schema 3, automatic calibration binding, GetItemPoses, wiring, rule-63 MovL/MovLIO distinction, and source-only software verification. `motion_debug` and `gripper_control` remain mutually exclusive maintenance applications and block production Startup until their future client migration is completed.
65. `robot_controller` owns the canonical Dobot `Pause`, `Continue`, and `Stop` clients and exposes typed `/robot_controller/pause`, `/robot_controller/continue`, and `/robot_controller/stop` services in both GUI and headless modes. `PAUSED` is an explicit state that retains the active immutable Home/Pick generation, previous state/phase/waypoint, queued Dobot path, gripper outputs and trusted holding context. Pause is accepted only after Startup from idle READY/HOLDING or an active Home/Pick; it blocks further host-side phase/waypoint/I/O dispatch, awaits the Pause response, and requires fresh enabled, fault-free, pause-flagged, stationary feedback. Intentional pause time does not consume sensor/arrival/no-progress/hard-motion deadlines. Continue is accepted only from confirmed PAUSED, awaits the Continue response and three fresh cleared-pause samples, revalidates feedback/holding/output integrity, then restores the suspended state and executor. An ambiguous/rejected-after-dispatch Pause or Continue is contained by confirmed independent Stop; never guess or resume. Superseding rule 64's prohibition on Continue, Continue is now used only by this explicit service and never by Startup, Recover, Stop, cancellation, or fault handling. `/robot_controller/stop` remains unconditional and independently pre-emptive from every state: neither the API nor external callers require Pause first. Stop invalidates the action generation and confirms acknowledgement plus stationary/empty queue even if the pause flag remains latched, then clears paused context and requires recovery as before; unknown active DI1 becomes HELD_UNKNOWN. The GUI alone provides a two-stage button: READY/HOLDING/Home/Pick shows amber Pause; its first click immediately changes the controls to red Stop and Continue while awaiting the Pause response, and a second rapid click queues (but does not concurrently dispatch) direct Stop immediately after that response. PAUSED shows Stop and Continue. In all other states the red control directly calls Stop. The Start button calls Startup only from INACTIVE and changes to Continue only for Pause. Preserve response serialization: the GUI never sends Continue or its queued second-click Stop before the preceding Pause reply. Status, bounded events, sole-command ownership, held-output monitoring, late-ack containment and source-only testing remain mandatory.
66. A supervised live audit proved that Dobot `EnableRobot()` can set `isPauseCmdFlag=1` while mode 5 is enabled, `isRunQueuedCmd=0`, `RunningStatus=0`, faults/collision are clear and no resumable queue exists; raw `Continue()` then returns `-1` and leaves the bit set. Therefore `isPauseCmdFlag` is contextual firmware telemetry and is not a general READY, command-admission, motion-completion, idle-supervision or Stop-confirmation gate. Startup and Recover must not perform a Stop→Enable pause correction, wait for this bit to clear, call Continue, or reject an otherwise coherent idle robot because it is one. READY still strictly requires fresh connected/enabled idle mode 5, EnableStatus 1, RobotStatus enabled, empty/not-running queue, no error/collision, user/tool 0, stationary coherence and all holding/output invariants. Mode 10 remains invalid outside the explicit PAUSED lifecycle. Rule 65's explicit Pause/Continue transaction remains strict: only a successful controller-issued Pause plus retained generation/context may enter PAUSED; that path may use the bit to confirm Pause and its clearing to confirm Continue. A latched bit without that context is never resumable and must not change GUI state. This supersedes rules 55, 56, 59 and 64 only where they treat the bit as a global readiness blocker or prescribe persistent-pause correction; all other lifecycle, ownership, response-ordering and safety requirements remain.
67. The Robot Controller GUI global-speed slider must track its live slider position while dragged and send exactly that integer on release; never read a stale non-tracking committed value. Keyboard/groove edits are sent after one 350 ms debounce. The 10 Hz status refresh must not overwrite the local position or label while the handle is down, keyboard debounce is active, the typed speed service is pending, or its confirmed value is awaiting the matching status sample. Disable the slider only while unavailable, outside stationary READY/HOLDING, during another operation, or while its own request is pending. Suppress a release whose selected value already equals the confirmed controller value. A successful typed response is authoritative for the displayed value; a rejected response restores its returned confirmed value when available. Preserve the controller-side 1–100 validation, serialized canonical SpeedFactor response, status publication, no profile persistence and all rule-58 safety gates.
68. The vendored bringup defines `RobotStatus.is_enable` exactly as `robot_mode == 5` and publishes it separately at the configured joint/status rate; it is an idle-mode alias, not an independent enabled latch. `robot_controller` must still require fresh connected RobotStatus, and explicit disabled/enabled plus final idle READY/arrival confirmation must require the mode-derived Boolean to converge. During active/transitioning Home, Pick, Pause and queue motion, however, command admission and supervision use fresh FeedInfo `EnableStatus`, `robot_mode`, error/collision, user/tool and queue fields and must not fault on a cross-topic `RobotStatus.is_enable=False` sample. This supersedes rule 66 only where its general enabled gate treated that Boolean as an independent runtime latch. Every actual canonical Dobot request issued by `robot_controller`—including settings, GetPose, DO, MovL/MovLIO/RelMovLUser, Pause, Continue and the independent Stop channel—must emit a console line and bounded `logs/robot_controller/events.jsonl` record at dispatch and at its terminal acknowledgement, rejection, timeout/cancellation/response error, and any late response. Correlate both sides with a process-local request ID and record the exact service endpoint, request fields, ROS `res`, available `robot_return`, outcome and elapsed milliseconds. Service acknowledgement remains acceptance only and never replaces feedback confirmation. `robot_controller` remains the sole production/runtime Dobot command issuer; `motion_debug` may retain direct Dobot calls solely as a mutually exclusive maintenance tool and its presence continues to block production Startup.
69. The hardware-authority node publishes a human-readable, reliable transient-local `/robot_controller/operator_log` (`std_msgs/msg/String`) with a 1,000-message retained depth in GUI and headless modes. It contains timestamped state transitions, operation phases, and the exact paired Dobot service audit lines required by rule 68; it is an observability topic only and never replaces typed status/actions/services or the authoritative bounded JSONL event log. The non-headless GUI replaces its lower static instruction/blank area with a dark read-only no-wrap log text box capped at 1,000 displayed lines. Live messages preserve order, follow the tail only while the operator is already at the bottom, remain normally selectable/copyable with Ctrl+C, and a `Copy Log` button copies the complete displayed contents. The GUI remains an API client with no Dobot client or command authority.
70. Before the shared initial Home step of both `/robot_controller/go_home` and `/robot_controller/pick_item`, use fresh actual `/joint_states` to test the same Home completion contract used after commanded motion: every one of the six joints within plus/minus one degree of the cached taught Home tuple, fresh `EnableStatus=1`, mode-derived RobotStatus enabled in idle mode 5, fault/collision clear, user/tool zero, queue empty/not running, trusted held-item I/O intact when applicable, and 300 ms of advancing coherent feedback. When this gate passes, report `HOME` motion skipped and do not call GetPose, RelMovLUser, MovL or MovLIO for the initial Home step. If the initial sample is outside tolerance, retain the existing GetPose-selected conditional rise and exact joint-mode MovL Home. This optimization applies only when Home begins with no already-planned preceding targets; pick-attempt retract/return queues must still append and execute exact Home because their preceding motion has not yet completed. Cartesian targets remain complete at 5 mm Euclidean translation and one degree orientation with the same enabled/idle/stationary gates; Home completion remains joint-only and does not add a Cartesian comparison.
71. `/item_detect/get_item_poses` is the one canonical candidate-service name shared by Item Teach and headless Item Detect. Hardware Pick and TF Preview must require exactly one root-namespace provider and accept only node `/item_teach` while its explicit Armed state advertises the service or node `/item_detect` in headless runtime. Reject a missing provider, either name in a non-root namespace, any unknown node identity, or simultaneous GUI/headless providers. Provider identity is an ownership guard only: both modes retain the same strict profile/model/source hashes, fresh independent request observation, response validation and no-stale-target contract. Item Teach Simulate Trigger remains a local teaching preview and is neither required nor consumed by Robot Controller Pick.
72. Every canonical `GetPose(user=0,tool=0)` used to establish the origin of a Home or Pick motion batch, or the actual stopped pose after approach, must first wait up to two seconds for 300 ms of advancing coherent idle feedback rather than rejecting one transient post-DO/post-Stop sample. The gate is exactly RobotStatus mode-derived enabled true, FeedInfo mode 5 and EnableStatus 1, queue and RunningStatus zero, error/collision zero, user/tool zero, plus trusted held-item DI/output integrity where applicable. Only after that stable gate may GetPose be dispatched. A timeout sends no GetPose and reports every presently observed blocking field; if all fields are individually valid but unstable, report failure to remain coherent for 300 ms. This wait is bounded, cancellation/Pause-aware, and does not weaken action failure Stop containment, freshness, response serialization, target validation or final arrival confirmation.
73. Item Teach production artifacts use strict schema 7 and remove `quality.result_max_age_sec` completely from the YAML, Item Teach UI, detector, simulated/clicked preview, and controller. Input RGB/depth/TF freshness and synchronization remain enforced when acquiring a request snapshot, and `request_timeout_sec` still bounds queueing, inference, optional debug-image work and response creation. A successfully returned hash/source-matched batch is latched for the entire owning Pick action and does not expire while Home, candidate motion, suction settling, retract or between-candidate recovery consumes time; it remains invalidated by cancellation, Stop, profile/model/calibration/bin source change, disarm/generation change, malformed/future timestamps or a new request. The controller never automatically reacquires a batch mid-action. Production readers reject schemas 1–6 without migration; GUI-only recovery may open them as drafts, and operator artifacts may be explicitly rewritten as schema 7 without modifying paired model weights. Shared Item Perception UI schema 6, camera schema 7, platform/bin schema 3, pose geometry, ranking and all robot safety/feedback gates are unchanged. This supersedes rules 32, 39, 52, 57, 64 and 71 only for item artifact schema/result-age expiry semantics.
74. Non-headless Robot Controller may explicitly load or reload Item/Bin Teach configuration only in `UNCONFIGURED`, `INACTIVE`, or idle unheld `READY`, and only when the single-operation lock is available. Validate the complete replacement and all bound sources before changing the installed immutable snapshot; a rejected reload preserves the prior configuration and lifecycle state. A successful reload performs no Dobot call, clears preview TFs, replaces the configuration/hash, invalidates Startup/global-speed/output assumptions, transitions to `INACTIVE`, and requires another explicit Startup before Home or Pick. The GUI exposes `Reload Teach Configuration` while configured and keeps it disabled throughout active, paused, holding, stopped/recovery, unknown-held, or faulted states. Headless `runtime_teach/` configuration remains immutable for the process lifetime. Existing schema, source-integrity, action configuration-ID and sole-command-ownership rules are unchanged.
75. Item candidates retain the existing platform-relative pose convention: local X is the measured long axis, local Y is the measured short axis, and the normalized quaternion contains platform-plane yaw only, never a TCP attitude or measured surface tilt. Robot Controller must compose that pose through the destination `base_link <- platform_reference` transform, project the resulting short-axis direction onto the plane perpendicular to the exact taught Home tool-Z vector, and rotate the taught Home attitude only about that local tool Z until Link6 green/Y is parallel to the projected short-axis line. Because the rectangle axis is undirected, select the equivalent modulo-180-degree solution requiring at most 90 degrees from Home. Preserve tool Z exactly; never copy platform tilt into TCP roll/pitch, tilt toward the item, reinterpret standoff along tool Z, or change base-Z waypoint heights. Apply one candidate-specific attitude unchanged to transit, clearance, pre-pick, pick, retract, final-clearance and stopped-pose upward recovery; the exact joint Home return restores the taught attitude. Reject nonfinite/non-normalized/non-yaw candidate quaternions, degenerate projection, or non-rigid results before motion. Hardware Pick and TF Preview use the same planner; log candidate quaternion, base short/green axes, applied tool-axis angle and target RPY. Item/response schemas and detector geometry remain unchanged. This supersedes rules 30, 51, 57, 62 and 64 only where they require fixed Home attitude at every item waypoint; all vertical motion, feedback, I/O, Stop, retry and Home contracts remain unchanged.
76. After the initial shared Home and fresh candidate request, each Pick candidate uses exactly two named owned motion queues. The forward `candidate_N_home_to_pick` queue contains item-XY transit at Home Z, clearance, pre-pick and final pick; after every command is accepted, monitor only the terminal pick/stopped pose and suction settling, never intermediate waypoint arrival. The return `candidate_N_pick_to_home` queue is created only after that terminal decision and contains stopped-pose vertical retract, clearance, any conditional rise to Home Z and exact taught-joint Home; monitor only its exact Home terminal result. Every ROS service response remains mandatory queue-admission evidence and must be awaited before dispatching the next command, because requests cross different MovL/MovLIO service endpoints and ignoring acknowledgements would make rejection/order ambiguous; an acknowledgement is never mistaken for physical arrival. Early DI1 acquisition may interrupt the forward queue through the existing confirmed Stop path. Cancellation, Pause/Continue, late acknowledgement containment, timed I/O, suction/output monitoring, missed-pick retry and terminal feedback rules remain unchanged. This clarifies and supersedes rule 57 only for explicit batch boundaries and observability.
77. Robot Controller must omit `cp` and `r` from the `param_value` of every `MovL`, `MovLIO`, and `RelMovLUser` request, including standalone Home, Home-to-pick, pick-to-Home and conditional Home-Z movement. The strict Startup/Recover `CP(100)` acknowledgement is the sole blending configuration and therefore governs all queued transitions; never reintroduce a per-command zero or alternate override. Preserve per-target `user=0`, `tool=0`, `v` and `a`, serialized queue-admission responses, early-suction Stop, and terminal-only physical completion checks. Forward motion still settles and validates the terminal pick/stopped pose before constructing return; return still validates exact taught-joint Home. Global CP smoothing means intermediate transit, clearance, pre-pick, retract and Home-height coordinates are planning control points and are not guaranteed to be physically reached; timed I/O may execute during a blended transition as documented by the V4.6.5 protocol. This explicitly supersedes rules 57 and 63 only for their per-command `cp=0`/preserved-`cp` requirements; all geometry, queue boundaries, speed/acceleration, I/O, feedback, cancellation and failure contracts remain unchanged.
78. Item Teach production artifacts use strict schema 8 and add the required top-level finite `pick_rotation` with explicit degree units and range 0 through 90. It is an unsigned angular offset from the detector's projected item short-axis line, not a replacement yaw or TCP tilt. For every candidate, Robot Controller evaluates both clockwise and counter-clockwise offsets plus each modulo-180-degree line equivalent while preserving the exact taught Home tool Z; choose the legal target requiring the least absolute rotation from the current path reference. Candidate one uses taught Home as that reference; every later candidate uses the preceding candidate's selected attitude. Apply the chosen attitude unchanged to its complete forward/retract path, and make TF Preview use the same sequential planner. Schema-7 and older files remain untouched, are rejected by production, and open only as GUI recovery drafts with `pick_rotation` blank for explicit review and schema-8 Save. After a coherent missed suction on any non-final candidate, execute only stopped-pose vertical retract and final clearance as `candidate_N_pick_to_retry`, confirm its terminal feedback and DI1 clear, turn suction off, and begin the next candidate directly without exact Home. Success and final candidate exhaustion still use `candidate_N_pick_to_home` and finish at exact taught Home; a non-suction failure never advances. The accepted candidate batch remains latched, and all response ordering, CP, I/O, settling, Stop/cancellation, source validation and final-feedback rules remain unchanged. This supersedes rules 54, 57, 64, 73, 75 and 76 only for Item Teach schema, offset-orientation selection and between-candidate return routing.
79. Every candidate's schema-8 `pick_rotation` attitude must be calculated independently from the exact taught Home orientation, never from the previous candidate, actual retry attitude, or any cumulative angular state. Before executing candidate one, precompute the complete accepted batch: for each item, evaluate both clockwise/counter-clockwise offsets and modulo-180-degree line equivalents against Home and select the least absolute Home-relative turn while preserving taught tool Z. Hardware Pick and TF Preview consume those same absolute plans. A missed candidate still retracts only to its clearance and travels directly to the next candidate without Home, but that direct physical path does not alter the next target orientation. Log the signed rotation from Home. This supersedes rule 78 only where it used the current path/preceding candidate as the angular reference; schema 8, direct retry routing, final Home, I/O, feedback and safety behavior are unchanged.
80. Item Teach production artifacts use strict schema 9 and add the exact `bin_clearance` group with nullable millimetre fields `p1_p2`, `p2_p3`, `p3_p4` and `p4_p1`, corresponding to the directed Bin Teach edges P1→P2 through P4→P1. Blank/null means no inset on that edge; all blank preserves the existing ROI behavior and draws no clearance border. Shift configured edges toward the convex ROI interior in `platform_reference`, intersect adjacent shifted lines, and reject a collapsed, inverted, non-convex or outside result. Display a valid configured polygon in light blue on both RGB and registered depth. The green Bin Teach ROI remains the hard complete-item-footprint and depth-derived-pick boundary. The light-blue inset filters only the exact depth-derived pick XY; its boundary is accepted and detection geometry may cross it. The shared Item Teach click/Simulate/Armed and headless Item Detect pipeline performs this filter before ranking/returning candidates; Robot Controller consumes those already-filtered candidates and must not recompute it. Production readers reject schemas 1–8. GUI recovery leaves all four fields blank for older profiles and requires explicit review/Save as schema 9. Invalid inset geometry blocks detection-setting application, Save, simulation and arming. Camera/platform/bin/shared-UI schemas, model pairing, orientation, depth, queue, motion and I/O behavior remain unchanged.
81. Green Bin Teach ROI eligibility uses platform-plane polygon overlap-or-touch, superseding rules 32 and 80 only where they require complete detection-footprint containment. Project the selected mask/OBB/preview geometry to platform Z=0 and keep it when either polygon contains a vertex or their edges intersect/touch; ignore it only when the detection polygon and green ROI are fully disjoint. Apply this same rule to calibrated live RGB/depth preview, clicked selection, Simulate Trigger, Armed Item Teach and headless Item Detect. A final depth-derived pick XY must independently be inside/on green and, when configured, inside/on the light-blue inset; both boundaries are accepted. A detection may cross either border, but crossing blue does not relax the exact blue pick-point test. When calibrated ROI projection is unavailable, live RGB detection remains visible with its explicit calibration reason, while pose generation remains blocked. Schema 9, candidate ranking, depth/MAD, size filtering, controller behavior and all artifact formats remain unchanged.
82. Candidate pick-point eligibility requires both metric and visual containment. Retain rule 81's depth-derived platform XY test against green and, when configured, the light-blue inset. Independently test the unchanged exact RGB rectangle-center pixel against the same allowed-pick polygon projected with current color intrinsics/distortion at platform Z=0 (light blue when configured, otherwise green); accept boundary contact and reject a pixel outside it. This deliberately conservative second gate resolves item-height parallax where a metric-safe point can appear beyond the platform-plane border. Never move the pick ray, substitute its platform-plane vertical projection, change returned XYZ, or infer a different item surface to make the overlay pass. Apply the shared gate to clicked selection, Simulate Trigger, Armed Item Teach and headless Item Detect before ranking/returning; candidate-only frozen RGB/depth overlays therefore cannot contain a returned center outside the displayed allowed border. Green detection-footprint overlap, blue's point-only meaning, schema 9, depth/MAD, orientation, controller and artifact formats remain unchanged.
83. Item Teach and Item Detect additionally auto-select the newest strict schema-7 `robot_camera` `camera_on_hand` calibration with exactly `Link6 <- robot_camera_link`. This second calibration is transform-only; subscribe to no robot-camera images, depth, CameraInfo or TF. Require its unchanged SHA-256 for pose generation, detector diagnostics, controller configuration and independent controller source validation. Use the one shared Home-relative pick-attitude planner in Item Teach click, Simulate Trigger, Armed Item Teach, headless Item Detect, controller TF preview and hardware Pick: place Link6 at the item's raw base XY/Z plus taught standoff, preserve taught tool Z, compose the calibrated camera-link origin, transform it into `platform_reference` and test its XY inside/on the green Bin ROI. Retain the normal shortest Home-relative attitude when safe; otherwise test exactly one 180° rotation about unchanged tool Z, reversing tool X/Y while retaining the undirected short-axis alignment. The mirrored solution may exceed the earlier normal 90° Home-relative limit. If both camera origins are outside green, exclude the detection before candidate ranking/capping; the next safe pose becomes eligible for the controller retry. Controller preview/hardware independently reject a detector disagreement before movement, not as a retryable missed suction. Show selected normal/mirrored camera footprint projected at platform Z=0 on bin RGB/depth and expose camera-clearance rejections explicitly. The light-blue inset remains solely a pick-point constraint and Item Teach remains schema 9. Only the camera-link origin is modelled, not camera housing; the green ROI must provide real physical margin. No hardware commands are permitted in software verification.
84. Controller Dobot service replies must arrive within two seconds for every normal command, independent Stop acknowledgement, and Pause/Continue queue control. Keep strict `res==0` verification and normal command serialization; a timeout remains ambiguous and stops later normal dispatch, with the established independent Stop/recovery containment. This supersedes historical five-second service-response limits only. Keep service discovery and DO/output-feedback confirmation at five seconds; feedback freshness, mode transition, motion watchdog/cap, and terminal arrival checks remain unchanged. Do not impose per-waypoint physical-arrival waits on blended pick queues. No physical robot is commanded during verification.
85. Robot Controller motion batches supersede rules 64, 76, 77 and 84 only for motion-request admission ordering: prepare the complete named `MovL`/`MovLIO`/`RelMovLUser` group, dispatch every request in target order without waiting between entries, then validate every group response under the two-second response deadline before accepting the batch and monitoring only its terminal physical target. Any dispatch error, nonzero/empty reply, cancellation, or group timeout immediately uses the independent Stop path; unfinished/late motion replies remain ambiguous and each late acknowledgement triggers Stop containment. Keep non-motion commands response-serialized, retain all per-request audit logs, CP(100), terminal 300 ms stability, DI1 descent monitoring and exact Home checks. After a forward group reaches its terminal pick pose with DI1 still clear, classify a miss immediately; do not consume `timing.pick_settling`. Existing schema-9 files retain that field but controller execution ignores it. No hardware commands during software verification.
86. Adjacent `MovL`, `MovLIO`, and `RelMovLUser` requests within one named Robot Controller motion group must be dispatched in target order with at least 50 ms measured by the controller's monotonic clock between sends. This pacing does not wait for or serialize service replies: after the complete paced group is sent, retain rule 85's all-response verification and terminal-only physical confirmation. Independent safety Stop is never delayed by this pacing. Non-motion calls remain response-serialized. A `MovLIO` timed output transition becomes pending only after its owning service request is dispatched: held-item monitoring then accepts only the previously confirmed value or that exact commanded value, adopts the commanded value once observed, and preserves the observed legal state if Stop interrupts the group. Any premature value change, change on a channel without a pending command, loss of DI1/DO13 while holding, wrong final timed-output state, or other feedback violation remains a fault. This supersedes rule 85 only where it said adjacent entries were dispatched without waiting; CP(100), two-second response deadlines, group containment, output-feedback confirmation and all arrival checks remain unchanged.
92. Superseding rule 86's dispatch pacing, the Robot Controller must impose no minimum time between adjacent `MovL`, `MovLIO`, or `RelMovLUser` sends in a named motion group. It must still wait for each previous ROS service response with `res=0` before sending the next command, so dashboard queue order is known; the next request may be sent immediately once that response and the existing cancellation, DI1, feedback, and output checks pass. A nonzero, missing, late, or timed-out response prevents all later sends and invokes independent Stop containment. Do not remove the two-second response deadline, terminal-only physical arrival check, global `CP(100)`, timed-output tracking, or Stop pre-emption. This change does not relax the one-second feedback-freshness gate or claim to resolve its observed failure during final descent.
93. Superseding rule 90's separate physical confirmation between explicit Hardware Home targets, `/robot_controller/go_home` must submit `home_align` followed by final Cartesian `home` in one named motion group. Require each service response with `res=0` in order and no added dispatch delay, then physically confirm only final Home within 5 mm/1° and 300 ms stationary/empty-queue feedback. Both targets inherit global `CP(100)`, so `home_align` is an approximate blended control point rather than a guaranteed reached pose. Preserve held-item monitoring, cancellation, Stop containment, the Cartesian skip gate, and all source validation. Pick's shared above-item clearance, conditional Home-Z rise, and exact joint Home remain separately confirmed safety barriers and are not changed by this rule.
96. Superseding rules 84 and 92 only for response duration, every Robot Controller Dobot service response has an exact five-second deadline: serialized normal/settings/I/O commands, each ordered motion-group admission, independent Stop, and Pause/Continue. Keep five-second service discovery and output-feedback confirmation, one-second feedback freshness, strict `res=0`, no later normal or group command after an unanswered/rejected response, and established independent Stop/late-acknowledgement containment. A motion-group response completion callback must never acquire the response lock retained by the owning ordered group; otherwise completed callbacks can consume every executor worker and starve later replies plus canonical joint/RobotStatus/FeedInfo subscriptions. Normal single-response bookkeeping may retain its lock. Do not add parallel motion dispatch, retries, per-waypoint arrival waits, a freshness fallback, or hardware commands during verification.
97. Superseding rules 72, 85, 87 and 94 only for final-pick timing, schema-9 `timing.pick_settling` is the single terminal confirmation interval for every candidate. Once the final pick target is within 5 mm/1 degree, the command queue is idle, canonical feedback is advancing, and the complete commanded final output state is observed, those conditions must remain coherent for exactly the taught duration while eligible DI1 remains monitored. Do not first apply the fixed 300 ms terminal gate and do not add a separate suction-sensor wait afterward. Eligible DI1 during descent or this interval retains the existing Stop-and-confirm acquisition path; DI1 still low when the interval completes irrevocably latches the miss. The exact final feedback sample supplies the stopped pose and may be carried directly into the immediate retract/return batch without another 300 ms motion-origin wait. Keep the 300 ms gate for Home, clearance, Home-height, all other terminal targets, and independently acquired motion origins. Preserve feedback freshness, output/reset arming, late-DI1 isolation, Stop containment and schema 9; no artifact migration or fallback is permitted.
98. Superseding every earlier Robot Controller rule that requires a fixed 300 ms stationary/coherence duration, `timing.pick_settling` at the final pick is the only timed motion-settling interval. Home, Home-height, clearance, retract and every other non-pick terminal target complete on the first fresh post-command feedback sample that is enabled, fault-free, queue-idle, within its existing joint or Cartesian tolerance, and consistent with required I/O. The initial Home skip uses the same one-sample gate. An independently acquired motion origin must still wait for fresh enabled idle feedback and an advancing `controller_timer`, but it has no additional dwell duration. Stop and Pause confirmation still require two distinct feedback samples showing an unchanged tool pose and the required stopped/paused queue state, without a timed stability window. Keep Startup/Recover's separate 200 ms READY lifecycle coherence, feedback freshness, target tolerances, output/held-item supervision, Stop containment and the final-pick rule-97 interval. No schema or teach artifact changes are required.
99. Superseding rules 76, 88, 93 and 94 only for the exhausted final-candidate return, once the final-pick settling interval irrevocably latches a miss, queue the actual stopped-pose rise through old pre-pick with EXHAUST at 80%, old clearance with finger/vacuum NEUTRAL at 0%, any conditional relative rise to taught Home Z, and exact taught-joint Home in one `candidate_N_pick_to_home` group. Derive the conditional Home-Z segment from the planned clearance endpoint. Each motion service must still return `res=0` before the next request is sent, but do not physically confirm clearance or Home Z; physically confirm only exact joint Home on the first fresh valid terminal sample under rule 98. Continue ignoring later DI1 from the latched miss. This exception does not change non-final retry groups or successful held-item returns, whose established barriers remain. Explicit `/robot_controller/go_home` continues to queue Cartesian `home_align` and final Home in one group and physically confirm only final Home; reuse the same fresh stationary pose for planning and as the confirmed group origin instead of acquiring a duplicate origin sample. Preserve source validation, target geometry, global CP(100), rates, I/O ordering, feedback supervision, cancellation and Stop containment. No hardware commands are permitted during software verification.
100. Superseding rules 32, 50 and 51 only for headless Item Detect deployment selection, `item_detect.launch.py` has no artifact-path, `trusted_model`, `armed`, or compatibility arguments. It must load exclusively from the flat root `runtime_teach/` directory through the exact same shared catalog selector used by headless Robot Controller. Classify artifacts by canonical filename prefix, then pass them through their strict current readers: require exactly one regular `item_teach_*.yaml`, its same-stem regular `item_teach_*.pt`, and exactly one regular `bin_teach_*.yaml`; reject missing/duplicate/mismatched files, symlinks, subdirectories, unknown prefixes and unsupported extensions. Hidden dot-prefixed atomic-write entries may be ignored. Reserve `tray_teach_` as the future tray artifact prefix but reject it visibly until its schema and consumer are implemented. Starting the dedicated headless launch is the explicit operator decision to deserialize the deployed model and advertise `/item_detect/get_item_poses`; it remains read-only and cannot command hardware. Load and retain the model once, use the current Item Teach schema/settings, automatic station and robot-camera selection, fresh request-arrival RGB/depth/TF acquisition, shared filtering/ranking pipeline, and request-driven inference only. Do not run continuous inference or return cached poses. The startup selection is immutable; source hash changes fail/disarm and replacement files require process restart. Runtime teach files are operator deployment artifacts and must never enter automatic source commits.
101. The shared `runtime_teach/` selector must report a distinct startup error for a missing or duplicate `item_teach_*.yaml`, `item_teach_*.pt`, or `bin_teach_*.yaml`; every duplicate error lists the conflicting filenames. Headless Item Detect records that message as a bounded `FATAL` `item_detector_failed` event, emits it through the ROS logger and exits without advertising a usable service. Runtime deployment is currently manual and may later be performed by a third-party program or remote node, but every producer must finish one complete visible catalog before starting or restarting either consumer. Producers may stage incomplete files only under hidden dot-prefixed names and then rename them into place. Do not add directory watching, automatic copying, retry, fallback, or live catalog replacement.
