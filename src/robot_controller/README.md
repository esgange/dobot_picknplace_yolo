# Robot Controller

**Waiting for operator intervention:** exhausted item acquisition/pick retries,
blocked-only poses or all unsafe trickle targets now confirm Home and Stop and
show **WAITING FOR ITEMS** (`PAUSED`, phase `ITEM_ACQUISITION_PAUSED`). Manual Pick
and Auto Run keep their action ownership; Auto Run retains its count and elapsed
timer. Refill/rearrange the bin, then press **Continue**. This explicit choice
clears repick exclusions and trickle-depth counts, discards saved poses, and grants
a new bounded acquisition/pick window using fresh observations. Entering the wait
or pressing Stop does not clear history. Ordinary Pause/Continue preserves history
and budgets. There is no automatic polling or restart. Ten failed tray requests
show **WAITING FOR TRAY** at Tray Detect with the held item preserved; Continue
retries that same item and target without resetting bin history. Return Item is
available there with a trusted held source. Both waits supervise feedback, pose,
queue and outputs; STOP and faults retain their containment behavior.

Visual guide: [Controller finite state machine](../../docs/ROBOT_CONTROLLER_FSM.md).
It covers the current lifecycle, candidate ledger and operation/recovery routes;
update it in the same change whenever those behaviors change.
Ready-to-view versions: [visual HTML](../../docs/ROBOT_CONTROLLER_FSM.html) and
[visual PDF](../../docs/ROBOT_CONTROLLER_FSM.pdf), generated from that document.

`robot_controller` is the sole production application-level authority for the
physical CR10. The GUI provides Home, Pick Item and Place Item with one Preview toggle,
plus Manual/Test Run and Local Headless Run with one quantity field, completed count and elapsed seconds.
The normal HMI opens without station nodes. Explicit Initialize Station owns Dobot
bringup, robot TF and both supervised cameras; selection of either mode can also start
missing required nodes. The HMI owns its headless detectors. The HMI can open teaching/setup
windows and the passive RViz viewer.

## Station HMI

The persistent mode selector contains **Remote Auto** and **Local / Maintenance**.
The latter combines local production/testing and teaching/maintenance; there is
no separate Local Auto mode. The three pages remain **Operate**, **Setup / Teach**
and **Tools / Debug**, with robot/gripper status, Station health and lifecycle
controls always visible. Keep the native Qt appearance and existing layout.

Opening the HMI starts no station nodes and never loads the restored file fields.
**Initialize Station** starts owned bringup/robot TF, both supervised cameras and
the controller/Preview core. Clicking either mode also starts missing owned nodes.
Wait for a dependency identity before core startup to avoid competing ownership
preflight. No foreign process is adopted or stopped; incomplete owned shutdown
blocks retry. RViz is optional. Use `start_dependencies:=false` when dependencies
are managed separately.

| Mode | Source and behavior |
| --- | --- |
| Local / Maintenance | Each explicit mode click snapshots the selected Item/Bin/Tray paths, waits for healthy nodes and fresh idle/unheld controller feedback, then sends one Configure request to load and prepare/enable. Home/Pick/Place, Preview, speed/CP, teaching windows and local runs share this mode. |
| Remote Auto | Validate the complete external runtime catalog without copying, start read-only headless Item/Tray providers, check required health and remain at DETECTORS_READY. Show WAITING FOR REMOTE. The server command connection is not implemented; no HMI Configure/enable/cycle or Prepare Robot button is provided in this mode. |

There is no separate Load/Reload Teach Configuration button. Reselect
**Local / Maintenance** to reload/prepare the controller from the selected files.
Opening **Item Teach** loads the selected Item file and selected Bin path; opening
**Tray Teach** loads the selected Tray file. Each Open snapshots those paths,
overriding the tool's previous session. Missing/invalid selections fail visibly
without loading another profile. Existing paired-model/source checks remain and
Armed stays OFF. Changing HMI selections while a teaching window is open does
not reload it: close/reopen it, or use its own Load control. Load Auto retains its
explicit deployment/headless workflow.

To deploy this teaching-selection change, close affected processes and rebuild
`item_perception_yolo`, `tray_perception` and `robot_controller` with
`--symlink-install --cmake-force-configure`, then source and relaunch the HMI.
The installed teaching launch files need the new file arguments; no interface
rebuild or teach-file upload is required.

Local preparation has a 180-second health wait and a separate 180-second
Configure deadline, followed by five seconds for fresh enabled idle READY with
the exact returned configuration ID. Fields and conflicting controls are locked
while preparing. Missing/invalid files leave a visible failure with no fallback;
correct the files and explicitly click the local mode again.
A mode click never moves Home or starts a production cycle. STOP cancels pending
preparation and independently stops issued work; a late successful reply after
cancellation/timeout requires another Stop. There is no automatic retry or resume.
Successful explicit health checks clear only the HMI station alarm, not controller
or camera recovery context.
After a failed local load, a later successful **Load Auto** replaces that old
failure once matching READY is confirmed and outstanding replies/Stop are contained.
The HMI then rechecks full station health before clearing the alarm. Saving a
corrected teach file alone does not reload the controller or dismiss a failed load.

For teaching and testing:

1. Select **Local / Maintenance**. The files shown in **Setup / Teach → 1 · Select
   teach files** are loaded automatically and the robot is prepared. If those
   fields are empty, browse the saved files and click the mode again.
   Changing fields does not silently reload them.
2. Keep **Load Auto OFF**. Open Item Teach/Tray Teach to load the selected profiles
   and verified paired models, then explicitly arm each provider. Loading uses
   the teaching windows' existing source validation; opening never arms them.
3. Use **Home**, **Pick Item**, **Place Item** or **Manual/Test Run** on Operate
   while the teaching windows remain open. Quantity/placement/debug inputs and
   the existing controller action guards are unchanged.

The same local mode also supports optional headless production through Load Auto,
explained below. No server is required for either local workflow.

**Direct robot tools:** Motion Debug, Gripper Diagnostics and Camera Calibration
need exclusive command ownership. Clicking Open in Local / Maintenance requests
a guarded release of the production controller before opening that tool. Require
fresh stationary queue-empty editable UNCONFIGURED/INACTIVE/READY feedback,
DI1 LOW, no held/retained operation, pending request or Preview. With Load Auto
active, turn it OFF first. Recheck feedback after detector shutdown, then wait
for both the owned core session and its graph/local-process identities to exit.
Only then may the requested tool launch, after its own final ownership check.
A failed/refused/timed-out handoff never opens the tool.

The mode selector remains Local / Maintenance during this release; the status is
**LOCAL DIRECT TOOLS** and production actions are unavailable. Use the active
tool's Stop. Stop its motion and close it, then click **Local / Maintenance** to
restore the controller and reload/prepare from the selected files. Closing a tool
never automatically re-enables production. Motion Debug retains its own explicit
Open-time initialization. Camera Calibration remains read-only until its separately
confirmed automatic capture. Teaching windows keep the controller running.

Tools opened elsewhere show **External** and cannot be closed by the HMI. RViz
and Remote Setup are available in either mode, including during a run. Other
tool changes require no active operation, held item, queued motion or pending
request; closing owned direct tools remains available to resolve ownership.
The bare executable `ros2 run robot_controller robot_controller_gui` remains an
external client with no mode/dependency ownership.

Source-mode changes restart the owned core only after idle/unheld admission and
complete detector/core shutdown. Remote uses the external runtime recipe; local
mode loads the selected offline files. Candidate context and in-memory repick
history do not survive that restart. No queued work resumes. HMI shutdown stops
owned detectors/tools/core while bringup is available, then cameras/bringup.
During an operation, independent Stop precedes removal of owned pose providers.
Child sessions retain parent-death cleanup; foreign applications remain running.

Station health covers the driver, feedback, robot TF, controller, Preview, camera
supervisor, both exact configured cameras and perception providers. Require fresh
coherent feedback, current TF, advancing canonical camera diagnostics and service
readiness. Perception still validates live images/depth/transforms and sources on
every request. Remote health has a 180-second bound and failures latch STATION
ALARM; explicit Remote Auto / Check Station Health retries. Startup camera/status
arrival races wait within that bound. Camera failures keep HMI/bringup alive,
retain source/grip/release evidence and never resume motion on reconnection.
Supervisor exit or incomplete owned cleanup needs explicit correction.

## Load Auto and run buttons

**Manual/Test Run** uses explicitly armed Item Teach and Tray Teach.
**Local Headless Run** uses the owned headless pair started by **Load Auto**.
Both are available in Local / Maintenance and send the same native AutoRun action
with different enforced provider types, shared Quantity/X/Y/rotation/debug inputs,
controller count and elapsed timer. Neither runs in Remote Auto or Preview.

To use Local Headless Run:

1. Remain idle and unheld in Local / Maintenance, leave Preview, and close teaching,
   calibration and direct-maintenance windows. Stop independent headless providers.
2. Choose the Item/Bin/Tray files, then explicitly turn **Load Auto ON**.
3. Load Auto validates the YAMLs, both same-stem paired model hashes and saved
   calibration choices, stages/verifies ordinary copies, then replaces the complete
   `runtime_teach/` catalog with exactly three YAMLs and two models. Validation/copy
   failure preserves the old catalog. Offline sources, calibration and `.env` are
   not rewritten.
4. The owned detector pair starts. Configure uses those exact deployed runtime
   paths, followed by its successful response and fresh matching enabled idle
   READY. Each detector startup and preparation has its existing 180-second bound;
   post-response READY confirmation has five seconds.
5. Set Quantity/placement and click **Local Headless Run**. It rechecks all required
   node health immediately before sending a goal; a failure sends no goal and
   latches a station alarm.

Load Auto starts OFF on HMI launch. It remains an explicit local deployment action,
separate from local-mode automatic loading of the selected offline files. OFF
stops owned detectors while idle/unheld and leaves deployed files intact. Close
providers before changing deployment. A child exit, lost service, source/catalog
drift or conflicting provider stops the owned pair and reports failure. Canceling
issued preparation uses independent Stop and contains late accepted responses.
No automatic restart, watcher, teaching-provider disarm or recipe adoption exists.

Remote Auto validates the external five-file runtime catalog and uses the same
ordinary-file identity fences without copying/deleting it or overwriting saved
local selections. It starts detectors after base health passes and waits at
DETECTORS_READY. Both detector services must be available; it never advances to
Configure/PREPARING from an HMI toggle. Remote server jobs, authentication and
command routing remain future work, so this waiting state cannot receive jobs yet.

## Repick history

Schema-16 Item Teach `retry` contains pose_candidates plus the exact keys
blaclist_trigger, blackist_radius (mm), blackist_cooldown and trickle_depth (mm).
New profiles default all four to zero. Schema-14 GUI recovery preserves the first
three values and proposes trickle_depth=0; older files start repick disabled.
Enabled trigger/cooldown are whole numbers 1–1000 and radius must be positive.

Record one attempt only on a confirmed HELD or FAILED pickup outcome, after the
existing suction/settling/probe logic. Queue admission, interrupted/canceled
approaches, sensing failures, captures and Preview do not count. Anchor a new
circle at the first measured platform-XY pick point, before tool offsets. Later
points inside/on that radius count despite changed detection IDs or height.
Centers never move; overlapping inactive circles assign to the oldest containing
area. The trigger-th attempt is allowed, then activates its circle. Recheck
pending/interrupted saved candidates before choosing each next motion group;
skipped poses keep their identities and do not become physical attempts.

Each other completed physical attempt ages existing active circles once. The
triggering attempt does not age its own new circle. Expiry removes both circle
and count. Confirmed PLACED clears only the original area's ID, even when Auto Run
has installed a different next session. Acquisition alone, Return Item and drops
do not clear it. History persists through fresh batches, Auto Run cycles, Pause,
Stop and recovery within the same loaded configuration/process. Reload/restart
starts empty. Bound history to 1000 areas; at capacity retire the oldest inactive
observation, never silently evict an active exclusion. Ordinary candidate-ledger
rules still exclude failed/dropped/returned poses in a retained batch.

`trickle_depth` is finite and nonnegative; positive values require blacklisting.
Immediately before each pending attempt, lower nominal final pick Z by prior
completed attempts in its oldest matching fixed circle × step / 1000. A 10 mm
step gives 0/10/20 mm on attempts 1/2/3; trigger 3 then excludes that area. Recompute
from each fresh measured candidate, never from an already-adjusted target. Keep
interrupted attempts at their selected depth. Save pre-pick/retract/clearance,
Return Item release and travel targets, orientation, rates and timed I/O unchanged.
Check the actual adjusted endpoint against the calibrated floor at its base XY:
20 mm downward allowance plus 0.001 mm numeric tolerance. Recheck the selected
camera body's footprint at that endpoint without changing attitude. Reject, never
clamp. Geometry/configuration errors remain errors. Skip unsafe candidates without
counting an attempt; if none remain, confirm Home and enter WAITING FOR ITEMS without
spending empty acquisition retries. After a dropped pickup, this also completes
the shared put-back through Home. Preserve survivor IDs and order.

The queued placement-to-pick bridge plans as though only its old source area has
been cleared, because next pick executes after the confirmed release boundary.
It never clears real history early; PLACED remains the only success reset. A
failed/interrupted placement retains ownership and containment. Other areas keep
their counts. Preview uses the same endpoint guard with zero prior attempts and
reports that limitation. Structured `trickle_target` / `trickle_rejected` events
include candidate/batch, area, prior count, extra descent and target/floor heights.

Send immutable, revisioned exclusion snapshots with bin/platform source hashes to
Item Detect. Validate the exact echo and independently filter returned poses;
retain nearest-Home ordering among survivors. The provider filters before the
requested count. All-blocked results confirm Home and enter the item wait rather than
spending the nine empty-acquisition retries. Ordinary empty results retain that
budget. For empty prefetched results, confirm any owned placement first; if its
successful release changed history, reacquire against the new revision before
concluding no unblocked poses remain. After Home confirms READY, check cancellation
and restore PICKING before that fresh request and its pickup. The existing action
retains ownership; successful acquisition can reach HOLDING and suction loss can
enter automatic RETURNING_ITEM. READY does not permit either transition directly.
Preserve placement count/source handoff,
physical request budgets, fresh inputs, motion geometry/rates/blending and I/O.

The GUI's Clear Repick History calls the typed Command service
`/robot_controller/clear_repick_history`. Require configured INACTIVE/READY,
unheld state and the exclusive operation slot; recheck state after admission.
It clears history and cancels/discards saved candidate poses so rearranged bins
require a fresh capture. No hardware command is sent. Preview disables this
control. Repick events and detector rejection diagnostics report area/count/cooldown.

Rebuild item_perception_interfaces, item_perception_yolo and robot_controller in
an isolated install while processes run; canonical rebuild and coordinated relaunch
are operator tasks. Review/save/deploy schema-16 Item Teach files explicitly.
Local Simulate Trigger and controller TF Preview request no exclusions and never
mutate real pick history. Hardware independently enforces current history.

## Pick priority

After validating an Item Detect response in its original priority order, the
controller sorts the received poses by straight-line **3D distance from taught
Home**, nearest first. Transform each item's platform-relative XYZ into
`base_link` and compare it with the taught Home Link6 XYZ. Use the received item
surface position before standoff/pre-pick offsets; orientation, live robot
position and queue travel are not ranking inputs. Exact distance ties preserve
detector priority. No extra poses are requested, and detector selection, filters,
ranking, count limit and images remain unchanged.

Manual Pick, Auto Run (including prefetched batches) and Preview share this
controller order. Freeze it when accepting a batch; misses, Pause/Continue and
drop/return recovery keep its plans, IDs, order and terminal-state exclusions.
Successful placement still discards unused candidates. Controller attempt/preview
numbers refer to execution order and can differ from Item Teach's P1…Pn labels.
The `candidate_response` event logs Home XYZ and each pose's controller priority,
original detector priority, ID and distance in metres. Restart controller and
Preview to load the change; no artifact, interface or configuration migration.

## Measured-surface height guards

Before ranking, independently reject received item surfaces more than 20 mm below
the calibrated platform-Z=0 bin floor. Before placement planning, independently
reject measured placement surfaces more than 20 mm below the saved Tray Teach
reference plane. Both use one shared base-Z calculation at the actual target X/Y,
supporting tilted planes and either normal direction. Keep 0.001 mm arithmetic
tolerance at the fixed 20 mm downward allowance.
Check raw surfaces before standoff or trayplace_height; never clamp or repair them.
Perception applies the same guard before returning usable targets.

Skip bad bin candidates without renumbering their detector IDs/priorities; empty
batches use the existing nine Home retries. Tray observations beyond the allowance
consume the existing ten-request budget, then pause with Continue / Return Item
available.
Validate source and response evidence before treating a plane rejection as retryable.
Invalid reference geometry remains terminal, including provider INVALID_GEOMETRY.
Hardware, Auto Run/prefetch and Preview share admission. Taught travel, offsets,
rates, blending and I/O are unchanged; this is not full-tool or trajectory checking.
Rebuild item_perception_yolo to install the shared helper, then manually restart
affected perception/controller/Preview processes when safe. No interface migration.

## Auto Run

Start from configured, unheld READY with recorded tray joints and both pose
providers available. Select a whole quantity from 1 to 10000; the run freezes the
current placement X/Y/rotation and debug-image choice. One native `AutoRun` action
owns all hardware commands until completion/failure/cancellation. The GUI disables
manual motion, Preview, teach loading, speed/CP, Pause/Continue/Return and input edits;
permanent STOP remains enabled. External manual actions cannot acquire the operation
slot. After ten unavailable tray observations, Auto Run instead enters the
normal acquisition Pause and permits Continue or Return Item, as described below.
Other manual queue-control requests reject while Auto Run is active.
Preview cannot start Auto Run. Launch/prefill never starts it.

Successful tray placement discards unused poses; the next manual or Auto Run
Pick uses a fresh bin batch. Original poses remain eligible for misses,
Pause/Continue and drop/return recovery until a successful placement. Preserve
initial Pick Home skip/arrival, candidate plans, timed I/O, acquisition Stop
acknowledgement, lifts and saved Tray Detect destination.
Tray pose acquisition begins only after Pick confirms saved tray joints and idle
feedback. The shared tray provider now confirms two consecutive processed observations
with advancing RGB timestamps, each after the request and using exact image-time TF.
Origins, centers and the requested placement point must agree within 5% of taught
tray width; rotation within 5 degrees. Each frame uses taught dimension tolerance.
The controller recomputes version-1 `diagnostics_json.stability` evidence and rejects
old single-frame providers. It uses only the second pose and its registered depth.
No fixed inter-frame wait or averaging is added. One request deadline includes all
comparisons; ten exhausted requests still pause at WAITING FOR TRAY. Keep rebuilding
and relaunching tray provider and controller together; no teach upload is required.

Auto Run always requires the trusted picked item for placement, including
in GUI mode. Placement retains its existing approach/release/retract queue and rates.

After Pick confirms Tray Detect joints/idle, acquire the tray pose and placement
depth first, including any of the ten allowed observation attempts. After the
validated result and observation-position check, start one read-only worker requesting
a fresh next-bin candidate batch whenever another item is needed, even if the
old batch still contains eligible poses (rule 218 supersedes rule 215).
It runs while the owning action plans and sends the placement approach/release/retract
queue, overlapping command admission and execution. The worker never sends hardware
commands. Require ordered acceptance of all three placement commands before consuming
even an already-ready result or appending next-Pick motion. Tray failure or Stop
before the worker starts prevents the request; a subsequent placement admission or
execution failure cancels/discards it. Capture remains a fresh post-request
RGB/depth/TF observation.
Keep the fixed bin camera's view clear during placement; no automatic occlusion
test is added. Manual Pick/Place do not launch this lookahead.

Keep the original batch and held source until placement execution/release is
confirmed, so tray failure or an interrupted drop can still use its saved poses.
At successful placement, mark the old item PLACED and cancel unused candidates.
Never select an old remaining candidate for the next normal placement cycle.
The prefetched result gets its own ledger; its candidates cannot acquire ownership
until the old placement crosses the execution/release boundary.

Consume the result using the existing handoff/error path. When valid poses are ready,
append taught Tray Detect joint return → next entry → pre-pick → final pick **without Home or a placement
arrival wait**. The dashboard executes these requests after every accepted
placement command, with the selected global CP. The new joint return uses
`MovJIO(mode=true)` with all six saved Tray Detect angles, taught travel speed/
acceleration, and `{0,50,2,0}` then `{0,50,14,1}` to open fingers at 50%.
Move this transfer's OPEN event off the next item transit, which uses plain MovL
to item X/Y at Home Z. Final descent starts with SUCK. There is no intermediate
arrival wait or new perception request at Tray Detect; CP may blend the waypoint.
If detection is slower than placement, confirm/count retract and wait unheld,
then use the same joint return and next Pick without an initial Home.
The fresh batch is retained for retries/recovery until the first successful placement. Manual
Pick → Place cycles discard the old batch at placement completion and acquire
fresh poses on the next explicit Pick.
Source/hash/attitude checks and three nonempty batches per Pick remain in force.
Each Pick has nine retries for empty pose results: confirm Home before each
new request (ten requests total if all are empty). An empty prefetch uses the
owned placement-to-Home route and counts the
completed placement before this retry. A tenth empty result enters WAITING FOR ITEMS at Home
with the completed quantity retained. Empty observations do not consume physical-pick batches.

The handoff retains the previous placement and source until advancing FeedInfo
reports the next pre-pick MovL's returned queue ID (or a later ID), with observed neutral
gripper outputs and DI1 LOW since placement admission. This proves execution passed
the placement queue without inventing a MovJIO queue ID or requiring a midpoint
stop. The joint return's reply has no ID; pre-pick is the third appended
command. Apply this handoff before interpreting any new-pick suction as old-item
feedback, including when feedback jumps directly to final descent. Only then count
that placement, mark the old candidate PLACED, activate the
next candidate and permit its suction acquisition. Cancel the old batch's unused
poses and install the fresh ledger. Old held DI1 cannot
trigger the next Pick. Missing boundary/release evidence faults and Stops the run.
An interrupted handoff retains the correct source for explicit Recover. Counted
placement is robot execution/release evidence, not camera proof of item delivery.

Initial Pick and empty-result recovery retain Home. For the final item, append
MovJ Home immediately after placement admission without
requesting another item batch. Confirm final Home, neutral outputs and DI1 LOW
before Auto Run SUCCESS/READY. Status publishes `auto_run_active`, requested and
completed quantities; the action result preserves the final/partial count.

The controller starts a monotonic elapsed timer when the accepted run begins.
It includes detection, retries, pauses, all movement and final Home, or failure/
Stop handling if the run ends early. The GUI shows one decimal second beside the
count, using periodic status with no local motion timer, extra thread or wait.
`AutoRun` feedback/result carry `elapsed_sec`; status carries
`auto_run_elapsed_sec`. On any terminal outcome the duration freezes and the GUI
shows **Total**. The last count/time remain in controller memory for reconnecting
clients until a new run replaces them or the controller restarts. Rebuild both
interfaces and controller, then restart controller/GUI/status/action clients together.

Three exhausted nonempty Pick batches, or an empty result after all nine Home
acquisition retries are used, enter WAITING FOR ITEMS at Home. Ten unavailable tray
requests confirm Stop and enter **PAUSED at Tray Detect**, retaining the item,
source, target, operation owner and completed count. The same paused controls
offer **Continue**, **Return Item** and permanent **STOP**; Place Item (Retry) is
an alias for Continue. Continue grants ten fresh tray requests for that same
item and resumes the run after a valid result. Another exhausted batch pauses
again. No new Pick or next-bin request starts while waiting. Return Item uses
the shared return queue, ends Home/READY and cancels Auto Run with its unchanged
partial count. Return requires no tray provider; Continue requires the provider
and unchanged safe parked state. A fault or held loss while awaiting the operator
cannot resume counted production automatically; normal Stop containment applies.
Malformed successful detector evidence and robot/transport faults still require
explicit recovery. Only explicit item-wait Continue grants a new bounded retry
window; it clears repick/trickle history and saved poses. No automatic restart is granted. STOP
cancels the run and outstanding prefetch, discarding its result. Tray exhaustion
prevents prefetch; held loss or another fault during placement discards it.
Explicit Recover cancels remaining poses; reloading configuration and process
restart invalidate reuse. Successful Return Item excludes the returned candidate
but retains remaining poses for a later explicit Pick/Auto Run. Manual Place shares this
acquisition-pause/retry workflow. Rebuild and manually restart controller/GUI
after this change; no teach-file or interface changes are required.

Launching the package never enables, recovers, homes, or moves the robot. An
operator explicitly loads teach configuration to prepare the robot to READY.
Headless supervisors retain the explicit Startup service after deployment loading.

## Cycle audit and acquisition performance

Rule 268 supplies daily archives and automatic Auto Run JSON/CSV time studies.
Daily files append across restarts and midnight without a count limit; keep today
plus the previous 29 Dubai dates. UTC timestamps correlate packages; monotonic
nanoseconds measure durations within a process. Session/event IDs, run IDs,
cycle context, candidate IDs and motion-group IDs disambiguate repeated commands.
Next-item prefetch retains its own cycle context while placement is executing.

`auto_run_started` records the profiles, debug setting, global speed and CP.
`auto_run_result` closes a report on success, no-pick, Return Item, Stop or failure.
Reports are `logs/robot_controller/run-YYYY-MM-DD-<run_id>.json` and `.csv` and have
30-day retention. An abrupt process kill leaves a partial daily trace without a
completed report; no success is inferred. The JSON includes per-cycle phases,
mean/median/p95/min/max, per-attempt tray latency, failed-request time, first-request
usable counts and worker timing distributions. CSV contains exclusive phase
seconds per cycle plus final Home. Retry/return and pause time remain visible.

Phases use observed group dispatch, pickup, tray arrival, acquisition and placement
handoff events. They include dispatch/feedback latency; they do not claim exact
intermediate physical waypoint times. The existing placement handoff can be the
next pre-pick or Home command ID, so its interval includes some departure travel.
Acquisition work overlaps placement; request durations are separate from exclusive
wall-clock phase totals. An idle wait for the next item is counted as `item_wait`.
`tray_request_timing` joins the provider through request/batch identity, includes
controller validation and service wait, and reports failures as well as successes.
A usable tray rate is not model accuracy; review the provider's image evidence.
Logging/report I/O failures are visible diagnostics, without changing control gates.
The GUI and operator-log topic still retain 1,000 entries. Existing legacy files
are preserved. Rebuild all three controller/perception packages before relaunch.


The 2026-10-01 audit before rule 202 checked manual Pick/Place, missed candidates, held-loss
put-back, Return Item, Auto Run handoff and final Home against their shared
planners/executors. The latest complete recorded Auto Run finished 3/3 in 53.65 s
(12:52:05.898–12:52:59.552 Dubai time), with first-candidate picks and first-attempt
tray/depth results. The two next-item requests started after placement admission
and overlapped execution; their validated results arrived 1.69 s and 1.08 s after
retract completion. Final Home was appended immediately behind the last placement.
Rule 209 now starts the worker after validated tray/depth acquisition, overlapping
placement admission as well as execution while retaining the complete placement
admission gate before any next-Pick motion. No new physical cycle timing has been measured.

Repeated source validation was a measured CPU cost. Strict catalog scans now
share one implementation and reuse camera-prefix parsing only for identical
file bytes. Every scan still reads files and checks their names, symlinks and
selection; selected calibration parsing, schemas, hashes and validation checkpoints
remain unchanged. No perception result is cached. An alternating seven-pair
offline benchmark on the local configured artifacts reduced median validation
from 425.09 ms to 248.66 ms (41.5%). This measures validation only, not a new robot
cycle time; first-time parsing still runs normally. Rebuild Item Perception and
Robot Controller, then manually restart the controller to use this optimization.

Further work suggested by the audit: measure per-request validation, preview-lock
wait, inference, geometry and reply validation separately; avoid overlay generation
in production requests when no display/debug image consumes it; record failed depth
sample counts/rejection reasons; retain a complete run trace across the former
1,000-event log reset. Rule 268 implements those timing/retention diagnostics;
production overlay optimization remains separate.

## Return Item compared with tray placement

Explicit Return Item requires a trusted held source, including after failed tray
acquisition. It never requests another bin/tray pose. The original saved pre-pick
matrix supplies its exact drop position and attitude; `trayplace_height` is unused.

| Motion | Return Item | Place Item |
| --- | --- | --- |
| MovL approach | Saved item X/Y and pick attitude at Home Z | Tray target X/Y and item-X/tray-X alignment plus rotation, preserving taught tilt, at Home Z |
| MovLIO descent | Exact saved pre-pick target at 100% | Detected surface Z + trayplace_height at 100% |
| At 80% descent | DO2 OFF, DO14 ON (open), DO13 OFF, DO1 ON (exhaust) | DO2 OFF, DO14 ON only; suction remains ON |
| At 90% descent | No new event | DO13 OFF then DO1 ON (exhaust) |
| Queue admission | One complete ordered group; no drop-arrival wait | Same |
| MovLIO retract | Same item X/Y/attitude back to Home Z | Same tray X/Y/attitude back to Home Z |
| At 0% ascent (start) | DO2/DO14/DO1/DO13 OFF | Same |
| Final MovJ | Exact taught Home joints | None for manual Place |
| Physical completion | Home joints/idle/execution, neutral outputs and DI1 LOW | Final retract/idle/execution, neutral outputs and DI1 LOW |

Return uses one ordered CP (default 100%) group: approach/drop/retract/Home. When
starting more than 5 mm below Home Z, prepend a vertical MovL at current X/Y/attitude
to the same group. Confirm only final Home from fresh joints, idle and execution
feedback. No intermediate arrival, settling, release-I/O,
DI12 or exhaust-pulse wait is added. All motions use speed 100%, scaled by global
SpeedFactor. Approach/Home
use taught travel acceleration, descent uses approach acceleration, and retract
uses retract acceleration. Finger OPEN, suction OFF and exhaust ON occur at 80%;
exhaust ends as retract begins. No separate 100% event is sent.
The shared planner/observer keeps release supervision identical; rule 267 moves
only tray placement's vacuum event to 90%, leaving Return Item at 80%.
Observed release means finger-open/exhaust outputs ON, finger-close
and suction OFF, and DI1 LOW; DI12 can be HIGH or LOW.
Only final Home completion marks the candidate RETURNED and clears its
source; queue acceptance alone cannot report READY or successful return.

Direct Stop pre-empts admission/execution and retains observed release/source
progress. Explicit Recover cancels the interrupted queue, preserves current grip
through its existing lift/Home, and resets at Home; it never repeats the drop.
Rule 212 makes this paused Return Item route the shared reference for automatic
suction-loss returns too. Paused drop appends Home and remains PAUSED; an active
drop appends the next saved candidate instead. No configuration or interface
changes are required; restart the controller after rebuilding to apply.

## Emergency-stop feedback

The controller translates a Dobot command response `res=-3` or confirmed robot
alarm **1537** into **“Emergency stop pressed — cannot start or recover. Release
the emergency-stop button, then click Recover / Clear Error.”** The failed service
response, operator log and FAULT status carry this explanation; the GUI shows
EMERGENCY STOP / PRESSED / Cannot start / recover in its status panel.
Generic mode 9, ErrorStatus, collision, disabled state and unrelated alarms are
not enough to label an emergency stop.

Startup first makes a read-only canonical `GetErrorID` query before any hardware
initialization commands. Its V4.6.5 payload must be `{[integer alarm IDs]}` (`{[]}`
when clear); missing/malformed/failed responses block startup. An emergency stop
reported by StopMoveJog is fatal even though ordinary StopMoveJog rejections are
best effort. Stop rejection still means Stop is unconfirmed.

Recover retains its explicit Stop → conditional ClearError → verified alarm
clearance → Enable sequence. Only Stop acceptance is required before clearing;
stationary joints and an empty queue are confirmed after enabled feedback,
before settings or recovery motion. An acknowledged ClearError alone is not success.
If the alarm persists, GetErrorID distinguishes an emergency stop from the old
generic clearance timeout. Recover remains clickable in FAULT so it can clear a
latched alarm after physical release; no stale UI flag blocks that attempt. It
cannot reach Enable, settings, output reset or motion unless clearance passes.
All queries use canonical ownership, feedback/cancellation and five-second
response checks, with no retries. Held-item and independent Stop guards remain.
Restart the controller and GUI after rebuilding `robot_controller` to load this
change; detection/teach files and ROS interfaces are unchanged.


## Tray placement

Load **Item Teach**, **Bin Teach** and a complete **Tray Teach** in the controller,
which automatically prepares the robot to READY. Explicit Place Item accepts an
empty or held robot in either launch mode. Pick finishes at the recorded Tray Detect joints.
**Place Item commands travel to the observation pose when needed.** It checks fresh
RobotStatus idle and all six `/joint_states` within ±1° of saved Tray Detect,
just like Pick's immediate Home skip. If matched, observe immediately; otherwise
queue joint-target MovL then MovJ to the same saved angles at 100% with taught
travel acceleration and preserved outputs. Queue the second command immediately
after the first is accepted, without an intermediate arrival check. Confirm only
final MovJ execution/idle/exact joints before requesting the tray.
Failure or interruption performs existing containment and blocks detection and
placement. There is no separate observation-position button; the typed
Tray Detect Position action remains available to external clients with its direct
MovL/MovJ pair. The bin routes retain their existing clearance logic.
After confirmation, request fresh tray/depth from Armed Tray Teach or headless
Tray Detect, with
**ten requests per acquisition batch**. Retry a missing pose/depth result,
provider no-result/error/BUSY reply or response timeout at the same observation
position. One attempt is **fresh tray pose → fresh placement depth**. A depth
failure consumes that attempt even when a tray was found; the next attempt repeats
tray detection and depth sampling from a new observation, with no saved-pose reuse.
The first attempt plus nine retries share one budget, not separate pose/depth budgets.
Each request keeps its taught request timeout plus one second for the
reply and requires RGB/depth captured after that request. A timed-out local future
is discarded; a late reply cannot supply a later attempt. The provider serializes
inference. Ordinary Pause/Continue keeps the request count and discards interrupted
results after completion or their original deadline. After ten unavailable
observations, confirm Stop and enter **PAUSED at the saved Tray Detect position**,
with the original Place/Auto Run action and command ownership retained. Preserve outputs
and any held source; queue no placement, release or parking rise. Report the final
reason in status, action feedback and operator log. **Continue / Place Item (Retry)** starts
another batch of up to ten fresh requests, after the usual position/source checks.
There is no automatic eleventh request. Another exhausted batch pauses again.
Local source/ownership/feedback faults and invalid successful pose
evidence remain terminal. Run exactly one provider; commands remain controller-owned.

During this acquisition pause, Pick Item stays disabled. The existing Pause
control shows **RETURN ITEM**, with **CONTINUE** in the same split-button menu
as an ordinary held pause. Place Item is also labelled **Place Item (Retry)**.
Return stays available if tray detection becomes unavailable; retry requires its
provider. The ongoing Place/Auto Run action does not block these choices. Auto Run
shows PAUSED with its retained completed/requested count. Pending return disables
Return and Retry; permanent red STOP always stops/cancels. Without a trusted held
source, the primary action is Continue and Return stays disabled.

A trusted held item may be returned to its saved
bin source through one queued approach at Home Z → saved pre-pick drop →
retract to Home Z → taught joint Home, without intermediate arrival waits or
settling. Keep Return Item's 80%-descent finger OPEN/suction OFF/exhaust ON
and 0%-ascent neutral events,
with a preliminary vertical rise in the same queue
when needed. Return finishes READY
and ends the waiting Place/Auto Run action as CANCELED, without claiming placement
success or incrementing Auto Run's completed count.
The gripper is preserved until the motion-timed release begins.
GUI placement's relaxed item-presence policy does not apply to the bin return;
trusted held-item checks apply before dispatch; queued release supervision matches placement.
Return is blocked during ordinary placement Pause and after release admission.
Direct Stop always pre-empts; stopped/faulted operations still require Recover.

Tray configuration uses the active eye-on-hand calibration saved in the shared
`ITEM_TEACH_ROBOT_CAMERA_CALIBRATION` selection, independently of the camera
recorded when the tray plane was taught. Use the same active camera in Tray Teach
or headless Tray Detect; returned camera hashes must still match. Recalibration
preserves the saved plane and detect joints. Reload controller configuration and
restart headless detection after changing calibration; GUI Tray Teach can load
the replacement explicitly and must be armed again. Loading never moves the robot.

Pick Item is enabled only in idle READY with its Item Teach/Detect service
available and a loaded Tray Teach with recorded Tray Detect joints. Pick does
not require tray detection to be armed. Place Item needs a recorded Tray Detect Pose and its Tray Teach/Detect
service. Explicit Place, including through a headless controller,
is available from idle READY or HOLDING with an empty gripper or a trusted
controller pickup. A held item requires its measured pickup orientation and bound
candidate; DI1 HIGH without that context rejects before observation travel.
An empty placement has no suction-presence prerequisite.
The sequence still commands real hardware; final neutral I/O and DI1 are checked
at final retract. Auto Run retains its trusted picked-item source and held-suction
guards. Status reports `manual_placement_enabled=true` for explicit Place in both
launch modes; it does not permit placement from a fault or unknown-item state.

The controller reports `item_detector_ready` and `tray_detector_ready` in typed
status at 5 Hz. It checks service availability and exactly one allowed provider:
root Item Teach or Item Detect for Pick; root Tray Teach or Tray Detect for Place.
Teaching disarm removes the service. Missing, foreign, namespaced or duplicate
providers disable the corresponding button. The controller checks again at action
admission, so stale GUI readiness cannot bypass it. These are read-only checks;
no detector trigger, model inference or automatic arming occurs. Fresh request-time
source/pose/depth validation still applies. Home and Tray Detect Position remain
available under their usual guards even when detection is disarmed.
Auto Run also displays its blocking reason beside the controls. It can start away
from Tray Detect, but both detectors must be available. CameraInfo changes disarm
Tray Teach; re-arm its validated setup before using Place or Auto Run.

Rebuild `robot_controller_interfaces` and `robot_controller`, then restart all
controller/GUI clients together for these status fields.

Tray requests use `/tray_detect/get_tray_pose_v3`, the depth-capable contract.
The earlier endpoint is never used as a fallback. Restart Tray Teach/Detect and
Robot Controller together after rebuilding; an old provider leaves Place disabled
instead of accepting an incompatible request. No robot motion is sent by detection.

### Tray placement alignment

Enter positive **X (mm)** and **Y (mm)** from the detected tray origin along its
inward short-X and long-Y axes, and **Rotation (−180° to +180°)**. At 0°, align the
held item's X axis line with the detected tray's X; any requested rotation is an additional
offset about the unchanged saved Tray Detect tool Z. The long axes therefore
follow each other at zero offset. A 90° tray turn rotates the item heading and
transforms the same tray-local X/Y point into its new base-frame position.
Both directions along that line are valid: after applying the offset, choose the
attitude with the smaller angular difference from fresh measured Link6 at Tray
Detect. For example, +170° and −10° are equivalent; choose −10°. On a numerical
tie keep the directed item-+X/tray-+X solution. Hardware reuses that measured pose
as the queue origin; Preview uses the same selector with fresh joint FK. This
minimizes Cartesian attitude change, not an IK-derived joint-path cost; the robot's
existing Cartesian planner still determines actual joint travel.

Preserve the taught tilt: alignment compares the X directions projected into the
plane perpendicular to tool Z; it does not make the item parallel to the tray
plane. Never copy the tray's Z sign, which can flip when the nearest corner changes.
Save the measured tool attitude at acquisition, transform the bound candidate's
item X into that tool frame, and retain it through held travel and Pause/Continue.
This compensates for pickup rotation, nearest-angle selection and camera mirrors.
An item without trusted pickup context cannot promise this alignment and is rejected
before travel. Empty placement uses tool Y as its reference axis.

Keep each validated tray quaternion paired with its sampled surface point. The
same planner serves hardware and Preview. Held Preview requires a matching
configuration and fresh `held_item_alignment_known` / `held_item_x_tool` status;
both source and receipt ages must be within one second. Empty Preview labels its
tool-Y reference. Record `placement_alignment` with tray/item directions, offset
and preserved tool Z. Rebuild controller interfaces and controller and relaunch
controller/GUI/Preview and other status clients together. Operator files stay unchanged.

Placement keeps requested X/Y and measures surface Z using the Item Teach depth
sampling diameter and quality settings, except placement has its own fixed
**20%** minimum depth coverage. This applies to Place Item, Auto Run and Preview;
item picking keeps its saved percentage. Valid original pixels after range/MAD
filtering must cover at least 20% of the full sampling circle;
no fixed sample-count threshold remains. Empty samples fail. The detector,
service and controller apply the same rule. Release Z is surface Z + the explicit
Item Teach `motion.trayplace_height` in millimetres (rule 184). This placement
clearance is independent of standoff, pre-pick and retract heights; Preview uses
the same formula.
Pre-place/retract uses the placement X/Y and orientation at taught Home Z,
matching the first Item Pick approach (`pN_transit`) before pre-pick (rule 171).
That initial Pick route skips its lower clearance/initial point. For surface Z
250 mm, trayplace_height 40 mm and Home Z 800 mm, release is Z 290 mm and
approach/retract Z 800 mm. The final pose stays above the tray at this height
with the tray-aligned attitude and taught tilt; there is no final Home command.
Home Z must be above drop Z for percentage I/O on both legs; invalid geometry
blocks the placement queue. Require a complete schema-16 Item Teach profile with
a finite, nonnegative `trayplace_height`. To use older profiles, open them in
Item Teach, explicitly fill this blank recovery field, Save and reload/redeploy
the updated pair. Production readers do not supply a fallback height.
Schema-10 profiles retain that height during GUI recovery; review the retained
**Minimum valid depth (%)** and Save as schema 16 to remove the fixed-count field.
The arrival check adds no fixed settling interval, new FeedInfo tick or pose query.
Recheck idle and saved joints during observation and before using its result.
Then send three Cartesian commands in one queue:

| Command | Target | Timed outputs |
| --- | --- | --- |
| MovLIO when Use Grip OFF; otherwise MovL | Pre-place | Use Grip OFF: at 80%, DO2 OFF then DO14 ON; preserve vacuum |
| MovLIO | Release height | At 80%: DO2 OFF, DO14 ON (open); at 90%: DO13 OFF then DO1 ON |
| MovLIO | Back to pre-place | At 0% (start): DO2 OFF, DO14 OFF, DO1 OFF, DO13 OFF |

All placement segments and Tray Detect Position use **speed 100%**, scaled by
global SpeedFactor; placement never changes that slider. Acceleration remains
Item Teach travel for Tray Detect Position, then travel / approach / retract
for placement. Item Pick retains its taught speeds. No teach-file edit is required.
There is no placement settling, separate release call, fixed-duration exhaust
pulse, extra retract-height waypoint or separate Home action. At 80% of descent,
open fingers; at 90%, turn suction OFF then exhaust ON. Exhaust lasts
until the upward command starts. Zero uses the existing distance-mode
start trigger (`{1,0,channel,0}`), carried by MovLIO. All commands inherit CP (default 100%); control points
can blend through the complete group. There is no drop-arrival, settling or
release-I/O wait between descent and retract.
Return **PlaceItem SUCCESS after all three commands are accepted**,
normally with `final_state=PLACING`. The action result acknowledges the final queue.
A completion worker retains the operation slot and PLACING status while the robot
moves, so another motion cannot overlap. It uses the existing joint-FK/idle,
execution, freshness, output and watchdog checks. Direct Stop, Pause and shutdown
still pre-empt. This worker adds no ROS executor thread.

Rule 170 removes intermediate release-confirmation gates in both modes. Send the
entire approach → pre-pick-equivalent drop → retract queue without waiting
for finger-open/exhaust, DI12 or DI1 transitions. Missing intermediate release evidence and history gaps alone do not interrupt
the queue. Rule 211 interrupts confirmed held loss before observed commanded
suction OFF; release admission and finger transitions do not disable monitoring. Any observed
coherent release feedback is retained as diagnostic/recovery evidence only.
Keep command acceptance/order, live enabled/error/collision/freshness checks,
opposing-output protection, motion watchdogs and direct Stop/Pause. These are
hardware/transport checks, without intermediate arrival or release-I/O confirmation.

Only at physically confirmed idle retract, require neutral DO1/DO2/DO13/DO14 and
DI1 LOW before READY; DI12 need not be HIGH. A bad final grip reports a
specific retract-reached fault without trying to release again. Successful retract
completion records PLACED for an existing held candidate and clears holding;
the `placement_retract_completed` event records `release_feedback_observed`.
Failures after action acceptance use Stop containment and report through typed
status/events; they cannot change the already returned action result.
PLACED now describes the completed queue and clear final grip, not proof that
the item was physically deposited on the tray. An empty test creates no candidate.

Placement Pause stops in place and preserves outputs. Continue reobserves when
the release command has not been issued, after rechecking saved Tray Detect
position within the same three-second window. It never moves back there. Once
issued, never repeat descent or release, even when its feedback was missed.
After confirmed release, Continue neutralizes outputs, retracts upward
from actual position to at least pre-place height if needed, then ends there.
An interrupted partial release with insufficient confirmation remains blocked;
no automatic release or bin put-back is inferred. Direct Stop requires explicit
Recover, which cancels the placement and preserves current outputs while lifting
to Home height and returning Home, then relaxes all four gripper outputs. It never
replays the expired placement history
or waits for continuous exhaust to switch itself OFF. A failed
tray/depth observation performs no placement or release. Pick settling,
within-batch retries, camera/bin avoidance, rotation and Home paths are unchanged.

The controller UI's strict schema-3 last-session store preserves Item/Bin/Tray
filenames and the last validated X/Y/Rotation as unapplied prefill. Explicit
in-memory import of schema 1/2 preserves their selections; disk changes only on
configuration save or a valid Place request. First-use X/Y remain empty.

New APIs are `GoTrayDetectPosition` at `/robot_controller/go_tray_detect_position`
and `PlaceItem` at `/robot_controller/place_item`, both with the active configuration
ID. Configure accepts `tray_teach_file`; status includes `tray_configured`,
`tray_position_recorded`, `manual_placement_enabled`, `TRAY_POSITIONING` and `PLACING`.
Auto Run placement requires a trusted HELD candidate; explicit Place does not.

Rebuild `tray_perception_interfaces`, `robot_controller_interfaces`,
`tray_perception` and `robot_controller`, then restart the tray provider and
controller/GUI together. Launch alone still sends no robot commands. Software
validation uses synthetic feedback and isolated ROS; physical placement requires
separate commissioning.

## Processes

- `robot_controller` is the background hardware authority. It alone creates Dobot
  motion, Pause/Continue/Stop, robot-setting, and gripper-output clients.
- `robot_controller_preview` calculates and broadcasts TF-only Home/Pick/Place plans.
  It has no Dobot command client and cannot actuate the robot.
- `robot_controller_gui` is a client of the controller and preview APIs. It has
  no Dobot command client.

The normal launch opens only the HMI. After Initialize Station (or explicit
selection of either mode), the HMI owns the controller
and Preview through `robot_controller_core.launch.py`, separately from its own
lifetime so direct local tools can release them. The station also owns canonical Dobot
bringup (driver, actual-joint monitor and robot TF)
and the canonical headless camera supervisor (both cameras, in saved slot order):

```bash
ros2 launch robot_controller robot_controller.launch.py
```

The `robot_controller_station` supervisor starts canonical installed launch files
in private sessions. It has no robot-command clients. The camera supervisor keeps
its strict stream/collision checks and three-attempt startup batches. Exhaustion
keeps a connection alarm alive and scans until both configured cameras return,
then retries the complete set. The station adds no outer process restart loop.
RViz remains optional and is opened from Tools.
Station health lists driver, feedback monitor, TF publisher, controller, preview,
camera supervisor, both cameras and both perception providers. Discovery alone is
UNVERIFIED. Driver/feedback require fresh coherent controller telemetry; TF must
be current; cameras require advancing canonical diagnostics with exact serials.
Preview/perception report service readiness, with source/input eligibility still
validated for each request. No cached pose is a readiness proof. RViz is optional.

Before spawning, check every installed launch and refuse detected existing local
driver/TF/camera/controller/detector or direct-maintenance processes, including
other ROS domains and startup phases. A workspace lock excludes another station
supervisor. Nothing external is adopted or stopped. This is a startup check, not
a replacement for existing runtime command/publisher ownership checks. For an
installation whose dependencies are already managed separately, use:

```bash
ros2 launch robot_controller robot_controller.launch.py start_dependencies:=false
```

HMI exit or terminal Ctrl-C closes the owned station. Bringup launch exit
(including exit code zero) closes owned dependencies and core, retaining the HMI alarm. Intentional Manual core exit keeps HMI/dependencies;
unexpected core/Preview exit leaves HMI mode attention and stops owned detectors,
without restarting. Graceful station shutdown closes HMI-owned detectors/tools/
core while bringup remains available, then cameras and finally bringup/robot TF.
The core has a 40-second graceful session wait (30 seconds for its controller
node); HMI allows 45 seconds for core cleanup after detectors/tools. The station
allows cameras/bringup 40 seconds each, then escalates with SIGTERM for five
seconds and SIGKILL for two seconds. The HMI waits up to 110 seconds for dependency
cleanup after its core/tools; its launch allows 200 seconds before escalation. Hardware Stop cannot be confirmed
if bringup itself has failed; existing containment diagnostics still apply.
Parent-death signals clean up children after an unexpected supervisor death;
forced kills cannot promise orderly Stop confirmation. No foreign process is
signalled. Root configuration, calibration, teach/models and runtime files are
unchanged merely by launching or closing the station.

**Camera connection alarm (rule 295):** ordinary camera failure and even an
unexpected camera-supervisor exit keep HMI/bringup open. The normal owned station
passes `require_camera_watchdog:=true` through HMI to each production core. A
standalone/external controller can opt in with that same launch argument; otherwise
it begins monitoring once it sees supervisor diagnostics. Production requires
fresh, advancing `/camera_watchdog/status` from the sole `/camera_watchdog/supervisor`
with both exact configured names/serials healthy. Missing health for five seconds,
an unhealthy report or invalid ownership produces **CAMERA ALARM** with connection,
cable and power guidance. Existing STOP remains available; no controls move.

An active/prepared controller uses one shared Stop attempt per outage and retains
outputs, held source and release progress. Initial unconfigured camera startup
does not command the robot. Reconnection shows **Camera streams restored** but
keeps production blocked until explicit **Recover** (or configuration/startup when
unprepared). Recover retains its guarded Home/suction procedure and cancels the
interrupted action; reconnection never resumes it. Stop confirmation failure remains
visible alongside the camera alarm. A supervisor crash needs explicit supervisor
relaunch; unconfirmed camera-process shutdown needs operator correction, not
another competing process. No teach-file change is required. Rebuild
`orbbec_camera_launcher` and `robot_controller` with their affected processes closed,
then relaunch the station; deployment is operator-controlled.

Opening the HMI leaves station nodes and Load Auto OFF and saved selections as
prefill. Click Local / Maintenance to load the selected files and prepare the robot.
Teaching and controller actions coexist. Optionally enable Load Auto to deploy/start
the headless pair, then press Local Headless Run. Online command handling remains deferred.

Headless mode starts only the hardware controller and loads exactly one Item
Teach YAML/paired `.pt` plus one Bin Teach YAML from the flat root
`runtime_teach/` directory. The shared detector/controller catalog selects them
by `item_teach_` and `bin_teach_` filename prefix; Item YAML/PT must have the
same stem. It still remains `INACTIVE` until Startup:

A complete optional `tray_teach_*.yaml`/same-stem `.pt` pair may coexist in the
catalog for Tray Detect and controller placement. Its profile/camera hashes and
recorded observation joints become part of the controller configuration.

Missing Item YAML, Item model and Bin YAML inputs are reported separately.
Duplicate errors list the conflicting filenames. Explicit GUI Load Auto, manual or external deployment
must finish one complete visible set before launch; replacement requires stopping
the process, replacing the catalog, and restarting it. Hidden dot-prefixed staging
files are ignored. There is no directory watcher, retry, fallback or automatic
copy during headless startup. Load Auto performs its copy only on an explicit GUI toggle.

```bash
ros2 launch robot_controller robot_controller.launch.py headless:=true
```

Canonical Dobot bringup and Item Detect are separate processes. Do not run the
maintenance `motion_debug` or `gripper_control` application alongside
production Startup; their presence is rejected as competing command ownership.
Start the detector pair with Load Auto, or launch headless Item Detect separately
with no arguments; it reads the same
immutable `runtime_teach/` selection and advertises only the read-only pose
service.

## Typed API

The **Save item/tray debug RGB/depth** checkbox is captured when Pick Item,
Place Item or Auto Run starts. Item requests save under `debug/pick_img/`;
tray requests save under `debug/tray_img/`. Each detector keeps its newest
**500 captures**, with RGB/depth retained and removed as a group; tray RGB-only
captures count as one. A successful save removes the oldest overflow, preserving
the just-saved capture and unrelated files. Cleanup errors remain diagnostic only.
Auto Run passes the same flag to
initial/next-item detection and every tray acquisition. Tray observation retries
and Pause/Continue retain the original choice. Unchecked requests and controller
Preview do not save images. Saving uses each detector's existing request snapshot
and diagnostic error reporting, without another inference or background archive.
`PlaceItem` includes `bool save_debug_images`; rebuild interfaces/controller
and restart controller and GUI together. The Item Detect request also carries
the repick exclusion contract described above.

Actions:

- `/robot_controller/go_home` — goal contains `configuration_id`.
- `/robot_controller/pick_item` — goal contains `configuration_id` and the
  one-shot `save_debug_images` flag. Candidate count cannot be supplied by the
  caller; it comes from Item Teach `retry.pose_candidates`. Each full candidate
  nonempty batch is one physical-pick attempt, with **three attempts total**.
  Successful tray placement invalidates unused poses; its next Pick requests
  fresh ones. Poses retained after interruption/return may still be used. Then
  ensure Home before candidate motion. Valid empty
  results allow nine Home-and-acquisition retries per Pick, preserved across Pause
  and later physical misses. The tenth empty result enters WAITING FOR ITEMS at Home.
  Empty results do not consume physical-pick attempts. After exhausting a
  nonempty batch, confirm Home and request a fresh batch.
  Success ends immediately in HOLDING; three exhausted nonempty batches wait
  at Home for explicit Continue. `attempted_candidates` totals actual candidate attempts
  in that Pick action across all batches, including canceled/faulted results.
  Successful placement cancels unattempted poses; the next manual/Auto Run Pick
  acquires a fresh batch.
  Ordinary Pause/Continue retains
  the current batch and attempt count; refill Continue clears old poses/history and starts a new window; direct Stop/Recover ends the action.
  Item detector errors/timeouts, source/pose validation failures and robot faults
  remain terminal. Reject reused IDs in new detector replies; intentionally
  continuing the retained batch is permitted. No teach schema, retry
  setting or action-interface field is added; progress reports attempt N/3.

- `/robot_controller/place_item` — goal contains `configuration_id`, placement
  `x_mm`, `y_mm`, `rotation_deg` and `save_debug_images`. The debug flag is passed
  to every `/tray_detect/get_tray_pose_v3` observation for the action, including
  explicit acquisition retries. It does not change motion or release timing.

Services:

- `/robot_controller/configure` loads or reloads explicit Item/Bin Teach in GUI
  mode, then runs guarded robot preparation under the same operation ownership.
  Accepted from unheld UNCONFIGURED/INACTIVE/READY; success means confirmed READY.
  Invalid files preserve the old configuration. Preparation failure retains the
  newly loaded files but reports failure, requiring explicit Recover.
- `/robot_controller/startup` performs the deterministic cold Startup sequence.
- `/robot_controller/recover` cancels the interrupted action and remaining batch,
  preserves gripper outputs, restores readiness, lifts vertically to Home height
  and moves to taught Home, then turns DO1/DO2/DO13/DO14 OFF. Confirmed neutral
  outputs and DI1 LOW finish READY. It accepts `FAULT`, `RECOVERY_REQUIRED` and
  `HELD_UNKNOWN`; unknown DI1 HIGH still blocks enabling/motion.
- `/robot_controller/pause` accepts a managed stop-and-park request. The service
  reports acceptance; status reaches `PAUSED` only after parking completes.
- `/robot_controller/continue` accepts replanning from confirmed `PAUSED`.
- `/robot_controller/return_item` accepts controlled return of a trusted held
  candidate to its source. Observe `RETURNING_ITEM` and then `READY` in status.
- `/robot_controller/stop` pre-empts and confirms Stop while preserving outputs.
  It is always direct and never requires a preceding Pause.
- `/robot_controller/set_global_speed` accepts an integer 1–100 only while
  stationary in `READY` or `HOLDING`.
- `/robot_controller/set_global_cp` (`SetGlobalCP`) accepts integer 0–100 with
  the same idle READY/HOLDING and single-operation ownership gates.
- `/robot_controller/preview_v2` belongs to the TF-only preview node. It accepts
  HOME, PICK, PLACE and CLEAR, selected Item/Bin/Tray files and placement X/Y/rotation.
  The former preview endpoint is not used as a fallback.

## Global CP control

The **Global CP** slider below Global SpeedFactor adjusts continuous-path blending
from 0 through 100%. It sends the live position on release or after 350 ms of
keyboard/groove inactivity, displays the last accepted setting, and preserves
an in-progress edit against status refresh. READY and HOLDING permit changes with
fresh enabled idle feedback and no active operation. Preview, Auto Run, motion,
Pause and faults disable it; STOP remains independent.

The controller sends one `CP(r)` through its existing serialized transport, with
strict ownership, response deadline, cancellation and held-output checks. A failed
response never claims the requested setting. Status `global_cp_percent` and the
service response's signed `confirmed_percent` use -1 for unknown and 0 for valid
zero blending. Accepted settings apply to all subsequent queues; motion requests
still omit per-command `cp`/`r`. Poses, rates, I/O timings and arrival checks stay
unchanged. Smaller CP values can reduce blending and change cycle timing.

Load/Startup starts at 100%. Recover reapplies the last accepted CP, including 0,
or 100 if none is known. No teach schema, `.env` key or disk persistence is added.
Restart the rebuilt controller, preview and GUI together for the changed status
interface. Launch alone never sends CP or another robot command.
The [official Dobot V4 API](https://github.com/Dobot-Arm/TCP-IP-Python-V4/blob/main/dobot_api.py#L584-L594)
defines CP as 0–100; the vendored ROS service's older 1–100 comment is not a
runtime limit (its integer `r` is sent unchanged).

## Operator status and buttons

Selecting Local / Maintenance explicitly requests controller preparation. After the
files validate, the controller runs its existing Startup sequence under the same
operation owner: alarm/ownership/feedback checks, Stop, cold DI1 protection,
disable/clear/enable, global settings, guarded neutral outputs and confirmed READY.
It does not queue Home, Pick or Place. The service reports success only at READY.
Invalid files preserve the old configuration; preparation failure retains the
new configuration and reports FAULT or HELD_UNKNOWN with explicit Recover guidance.
Stop during validation, installation or preparation cancels the sequence; no later
Startup request can re-enable the robot. Launch and filename prefill remain inert.
Headless launch still loads into INACTIVE and needs the existing external Startup
service. The GUI has no Startup client or button.

The lifecycle row has exactly **Recover / Clear Error**, **Pause** and permanent
red **STOP**. In an ordinary empty pause, the middle button becomes **CONTINUE**.
With a trusted held item, its main action is **RETURN ITEM**, with **CONTINUE** in
the split-button menu. If Return is unavailable but Continue is valid, Continue
becomes the main action. Each choice has its own eligibility guard. Acquisition
failure uses that same Continue/Return menu, plus the **Place Item (Retry)**
shortcut, in manual Place and Auto Run. Pending requests disable conflicting
actions; STOP remains direct.

Local preparation is blocked in Preview because it prepares real hardware.
Turning Preview OFF never starts preparation; select Local / Maintenance explicitly.
Preview motion buttons retain their read-only behavior, including while disabled.

The window title is **Pick & Place — Station HMI**. The main status is NOT READY, READY,
BUSY, HOLDING ITEM, PAUSED, WAITING FOR ITEMS, WAITING FOR TRAY, ATTENTION REQUIRED or OFFLINE. Starting, homing,
picking, placing, parking, returning and recovering appear in the activity line;
PAUSED appears only after confirmed parking/Stop. The reason is always visible.
Confirmed emergency-stop evidence overrides the main label in red.

The lifecycle row is Recover / Clear Error, a managed Pause/Continue/Return Item
control, and a **permanent red STOP**. STOP always calls the independent Stop service when
reachable, including while another request is pending, with stale status, and in
Preview. It never changes into Pause or Return. A pending Pause/Return disables
the managed button and shows Pausing…/Returning item…; STOP remains available.

| Operator status | Available controls, subject to the prerequisites below |
| --- | --- |
| NOT READY, no configuration | Select/load teach files; STOP |
| NOT READY, configured | Select Local / Maintenance to prepare; Preview; STOP |
| READY | Home; Pick Item; Place Item; Manual/Test Run / Local Headless Run by provider; Load Auto; Preview; reload; speed; CP; STOP |
| AUTO RUN | STOP; quantity/progress visible; manual controls and edits disabled |
| BUSY, Home/Pick/Place/tray travel | Pause; STOP |
| BUSY, startup/recovery/return/parking/stopping | STOP |
| HOLDING ITEM | Home preserving grip; Place Item; Pause; Preview; speed; CP; STOP |
| PAUSED, ordinary | Continue; Return Item with trusted held source; STOP |
| WAITING FOR ITEMS (PAUSED, Pick / Auto Run) | Continue clears history/poses and detects afresh; STOP |
| WAITING FOR TRAY (PAUSED, Place / Auto Run) | Continue / Place Item (Retry); Return Item with trusted held source; STOP |
| ATTENTION REQUIRED | Recover when configured with fresh feedback and no active operation; STOP |
| OFFLINE | STOP if reachable; loading/preparation waits for fresh robot feedback |
| EMERGENCY STOP PRESSED | STOP; explicit Recover rechecks alarms after physical release |

Home/Pick/Place and speed/CP require started configuration and fresh enabled,
fault-free, idle robot feedback with an empty queue. Pick also requires an unheld
READY state, configured Item/Bin, saved Tray Detect joints and exactly one armed
item provider. Place requires saved tray joints, positive finite X/Y, legal rotation
and one tray provider. It accepts READY/HOLDING with or without an item, regardless
of current Tray Detect position; the action moves there if needed and confirms
arrival before acquisition. Missing conditions, including Auto Run blockers,
appear beside the controls and in tooltips.

Continue requires retained managed Pause context, unchanged parked pose/outputs,
no pending suction loss, and no uncertain placement release. A paused observation
also needs the tray provider. Retry uses the original target and a new ten-request
budget; target fields stay locked. Return remains independent of tray perception.
An old E-stop message never permanently disables Recover: the controller rechecks
current alarms on the explicit request. Unknown suction is still checked by
Recover and never authorizes an inferred item source or output reset.

Preview ON explicitly says **No robot motion** and routes Home/Pick/Place only to
TF planning. Enable it after loading configuration. It requires fresh stationary
feedback but permits a disabled robot and no Startup. Selected files, placement inputs and detector availability gate
preview requests; the preview node validates the selected sources and position.
Preview OFF and STOP remain available; Load/Recover/Pause/Continue/Return/speed/CP stay disabled.
A stopped fault can also be inspected with read-only Preview. All buttons check
service availability and recheck their policy on click. Pending requests disable
conflicting controls immediately. These status hints do not replace execution's
source, ownership, I/O and feedback validation or guarantee a detection result.

## Unified motion preview

The operations grid contains Home (top left), Preview toggle (top right), Pick
Item (bottom left) and Place Item (bottom right). Preview defaults OFF on launch;
it is a GUI routing choice, not a hardware-controller lifecycle or persisted teach
setting. Turn it ON only with no active/pending hardware operation. Home/Pick/Place
then call only the preview service, even before Startup. No fallback to hardware
is allowed on unavailable/rejected preview. Load, Continue, Recover, managed
Pause/Return and speed/CP changes cannot dispatch while ON. The Stop control sends
direct Stop and clears preview; it never parks or puts back an item in this mode.
The separate typed hardware API retains its existing guards and behavior.

The preview process has no Dobot command clients. It reads the same three canonical
robot feedback streams, requires fresh stationary/empty-queue inputs, and obtains
the current Link6 pose from joint FK. It can preview while the robot is disabled;
it never enables it. Shared planners supply the conditional vertical clearance
and exact joint Home for both Home and Pick, every candidate's approach/retract,
successful Tray Detect, missed entry/exit transits and Home/put-back targets, and
Place's three approach/drop/retract targets when already at Tray Detect. Away from
the saved joints, Place preview shows only its observation-travel TF and explains
that placement targets need a fresh observation at that pose. It never moves the
robot, samples placement from the wrong pose or fabricates the unknown drop height.
Targets skipped by Home arrival checks are omitted. Each target is broadcast under
`base_link` as a distinct `robot_controller_preview_*` frame.

Pick preview shows one fresh candidate batch and its possible nominal branches,
not simulated suction outcomes or unknown future retry observations. Actual early
contact/Stop positions depend on hardware feedback and cannot be predicted.
Place uses fresh tray/depth from the camera's **current physical position**, with
the same ten-request bound and evidence validation. A hidden tray cannot be
observed by previewing a future camera pose; a missing observation fails visibly
without fabricated targets or movement. Preview does not require a held item.

Turning OFF, editing teach/placement inputs or clearing cancels pending preview
work and stops broadcasting. CLEAR pre-empts blocked perception and late results
cannot reinstall canceled targets. Changed sources, invalid/stale feedback or
robot movement clear installed previews; the existing RViz TF timeout retires
old display frames. No output commands, hardware state changes, new teach schema
or automatic application restart are introduced. Rebuild both controller packages
and restart controller/preview/GUI together for the versioned preview contract.

`/robot_controller/status` uses
`robot_controller_interfaces/msg/ControllerStatus`, reliable/transient-local
QoS. It reports state, phase, waypoint, candidate index, configuration ID,
holding context, Startup completion, global factor, and feedback freshness.
The same snapshot includes observed `robot_enabled` (FeedInfo EnableStatus),
`robot_running`, `robot_queue_active`, `robot_error`, `robot_collision`, and raw
64-bit `digital_input_bits` / `digital_outputs`. These fields are valid only
when `feedback_fresh` is true; zeroed fields in an unavailable snapshot mean
unknown, not confirmed disabled/OFF/LOW. These are feedback values, independent
of expected/commanded outputs and the debounced held-item state. The periodic
status rate remains 5 Hz, with additional state/progress updates.
`candidate_ids` and `candidate_states` are parallel arrays in the retained batch's
controller Home-distance order; `can_return_item` identifies trusted held source context.
The removed Trigger/JSON/Live/Enable/validation/pose-proxy/debug-image endpoints
have no compatibility wrappers.

Example CLI flow for a non-headless controller:

```bash
ros2 service call /robot_controller/configure \
  robot_controller_interfaces/srv/Configure \
  "{item_teach_file: '/absolute/item.yaml', bin_teach_file: '/absolute/bin.yaml'}"
ros2 topic echo --once /robot_controller/status \
  robot_controller_interfaces/msg/ControllerStatus
ros2 action send_goal /robot_controller/go_home \
  robot_controller_interfaces/action/GoHome \
  "{configuration_id: '<exact status configuration_id>'}" --feedback
ros2 action send_goal /robot_controller/pick_item \
  robot_controller_interfaces/action/PickItem \
  "{configuration_id: '<exact status configuration_id>', save_debug_images: false}" \
  --feedback
ros2 service call /robot_controller/pause \
  robot_controller_interfaces/srv/Command '{}'
ros2 service call /robot_controller/continue \
  robot_controller_interfaces/srv/Command '{}'
ros2 service call /robot_controller/stop \
  robot_controller_interfaces/srv/Command '{}'
ros2 service call /robot_controller/recover \
  robot_controller_interfaces/srv/Command '{}'
ros2 service call /robot_controller/set_global_speed \
  robot_controller_interfaces/srv/SetGlobalSpeed '{percent: 50}'
```

## State and safety contract

The states are `UNCONFIGURED`, `INACTIVE`, `STARTING`, `READY`, `HOMING`,
`PICKING`, `TRAY_POSITIONING`, `PLACING`, `HOLDING`, `PAUSING`, `PAUSED`, `RETURNING_ITEM`, `STOPPING`,
`RECOVERY_REQUIRED`, `RECOVERING`,
`HELD_UNKNOWN`, and `FAULT`. One immutable configuration snapshot and one
operation generation exist at a time. Action configuration IDs prevent a stale
GUI or supervisor from executing a replaced profile.

Reselecting **Local / Maintenance** in idle, unheld `READY` lets the operator
revalidate the selected files or install a different selection without restarting
the controller process. There is no separate reload button. A successful reload
clears TF preview output and repeats robot preparation, restoring SpeedFactor
100% and validated neutral outputs before reporting READY. Reload is blocked during Home/Pick, Pause, holding, Stop,
recovery, unknown-held, and fault states. Headless configuration remains fixed
to its startup `runtime_teach/` snapshot.

Startup validates sole canonical services and publishers, then performs:

1. best-effort `StopMoveJog`;
2. strict `Stop` with stationary, empty queue confirmation (a latched pause flag
   does not invalidate a confirmed Stop);
3. cold DI1 check (active DI1 preserves I/O and enters `HELD_UNKNOWN`);
4. Disable and conditional ClearError;
5. Enable and confirm fresh enabled mode. A latched `isPauseCmdFlag` does not
   block READY when mode 5 is enabled and the queue is empty/not running;
6. SpeedFactor 100, User 0, Tool 0, Tool 1 TCP zero, and CP 100;
7. DO1/DO2/DO13/DO14 reset only when no item is held;
8. 200 ms of coherent `READY` feedback.

Recover cancels the interrupted pick/place/put-back and marks unfinished
candidates CANCELED. It sends Stop and requires an accepted response, then adopts
validated fresh gripper I/O without waiting for stationary joints or an empty
queue. It does not replay old placement history. Opposing output pairs, unknown
DI1 HIGH, DI1 HIGH after confirmed release, or a prior dropped source that merely
regains suction block motion. Sustained fresh LOW permits empty recovery while
preserving the outputs: LOW is not proof that an object physically left the fingers.
A trusted HELD source with live suction and vacuum stays held throughout recovery travel.

Keep gripper and feedback checks active through conditional ClearError, verified
alarm clearance and Enable. After enabled feedback, confirm two distinct fresh
stationary joint samples, an empty queue and unchanged gripper outputs/raw DI1.
This check has the existing bounded wait and Stop cancellation; rejection,
timeout, stale feedback or I/O changes prevent settings and recovery motion.
Use the original accepted Stop; do not send another Stop just for this check.
Restore settings/readiness, then issue an upward-only RelMovLUser at unchanged
XY/attitude if below Home Z. Direct Stop, managed Pause, drop containment and
Startup retain their existing physical Stop confirmation order.
Confirm that lift before a separate queued joint-target MovJ to taught Home.
Use taught travel speed/acceleration and the last confirmed global speed and CP (each 100%
if unset, preserving CP=0). Already at/above Home height skips the lift; already at taught Home
skips its move. Current outputs and suction policy are monitored throughout travel.
After stationary Home is confirmed, test suction: command DO1 OFF before DO13 ON
when needed, preserve both finger outputs, and never cycle an already-active
vacuum OFF first. Confirm every output and continued Home/idle before observing
raw DI1 for up to one second. Any HIGH latches an item-or-obstruction indication;
it is not proof of a clog or a new pickup. A clear result requires a full second
and advancing fresh feedback. Missing feedback, changed Home position/outputs,
command failure or Stop fails the test and prevents an automatic reset.

A positive test enters PAUSED at Home, keeping suction and fingers unchanged.
The existing managed worker retains the operation slot; the Recover service
returns without waiting for the operator. RETURN ITEM is offered only for an
existing unreleased HELD/DROPPED source. It uses the normal shared return queue
to the saved pre-pick release, retract and Home. A dropped source remains DROPPED;
the test never promotes it to a new HELD success. Confirmed release, missing
source or process restart cannot invent a return destination. Without a source,
clear the item/obstruction manually. Once raw DI1 is LOW, RETEST SUCTION uses the
Continue service to run another check at Home; it never resumes old candidates.
Suction loss while awaiting the choice retains the uncertain source without
starting motion. Fresh feedback, stationary Home and preserved outputs remain
monitored, and direct Stop pre-empts the test, wait and return.

Only after a clear test, reset DO1, DO2, DO13 and DO14 to OFF in that order,
confirming each response/output. Both finger outputs OFF means relaxed;
there is no OPEN command or exhaust pulse. The intentional reset permits suction
to decay. Require all four outputs OFF, DI1 LOW and continued Home arrival before
reporting READY. This final check waits up to five seconds for newer joint/status
feedback and the output command queue to finish; it does not fail immediately
on temporary I/O queue activity. A failure reports idle/freshness conditions and
the maximum Home joint error. Cancel the former held ledger entry without claiming PLACED or
RETURNED. A still-HIGH DI1 reports HELD_UNKNOWN; clear the item/obstruction and retry.
Stop/cancellation prevents remaining commands during travel or output reset; failures
never report success. Another Recover replans from fresh stopped feedback. There is
no automatic next pick or replay of an interrupted placement/put-back release.

Recover remains available in HELD_UNKNOWN for a fresh check after the operator
secures/clears the item or obstruction. Stale feedback, active E-stop/other alarms,
changed teach sources, unexpected I/O or failed commands prevent movement.
The completion prompt reports Home reached, a clear suction test, fingers relaxed,
suction/exhaust OFF and DI1 LOW. A detected item/obstruction instead leaves the
controller paused for the explicit Return Item or Retest Suction choice.

Native action cancellation and Stop invalidate the active command generation,
use the independent Stop client, wait for acknowledgement and two distinct
joint samples showing unchanged joints and a stationary empty queue, and
preserve every gripper output. They never Home, release, disable, or resume a
discarded queue. The result is
`RECOVERY_REQUIRED`. A late motion acknowledgement causes another Stop;
unconfirmed stopping is `FAULT`. Held-item DI1 and expected-output integrity are
checked throughout Stop and Recover. Unexpected running/nonempty queue feedback
while otherwise idle is immediately routed through the same Stop confirmation
path rather than merely changing the state label.

Managed Pause uses the current operation executor. It immediately requests
independent Stop, blocks later dispatch, resolves any already-admitted command
reply, then confirms a final stationary empty-queue Stop before repositioning.
The old queue and its unexecuted output events are discarded. Unanswered/rejected
admission prevents all parking/return commands; late responses retain Stop
containment. Vendor Pause and Continue clients are removed.

Each new batch initializes `PENDING` candidates. The first accepted approach
command marks `ACTIVE`; settling followed by the full lift to settling_retract without suction marks `FAILED`;
Pause marks the active candidate `INTERRUPTED` and keeps it eligible for retry.
Continue retries that same candidate; its next accepted approach marks it
`ACTIVE` again. Repeated Pause does not consume it or increase the distinct
attempted-candidate count. Confirmed suction marks `HELD`; a paused loss marks
`DROPPED`; a completed intentional held return marks `RETURNED`, and confirmed
placement marks `PLACED`. FAILED, DROPPED, RETURNED, PLACED and CANCELED
candidates remain ineligible. Successful placement retains the other poses for
later manual/Auto Run picks under the exact same configuration object; source
validation still runs before use. Reload/recovery/restart invalidates reuse.
Parking never marks a candidate attempted or changes an interrupted state.
The ledger and source plans are in memory, never written into teach artifacts.

Without an item, active Pick Pause neutralizes all four canonical outputs,
rises vertically at measured X/Y/attitude to at least taught Home Z, then crosses
at that height to `park_transit` above the interrupted candidate, or the next
pending candidate if no interrupted approach remains. This pause endpoint has
that candidate's X/Y and planned pick orientation, with Z equal to
the higher of stopped Z and Home Z. Both the safety rise and final transit are
confirmed; no descent or fixed settling dwell is added while parking. With no
remaining candidate it parks at safety height. Continue commands finger CLOSE
OFF then OPEN ON, confirms the outputs, then queues pre-pick followed by final
pick with the existing timed SUCK and taught settling. A retained candidate
batch is never replaced during Continue. Standalone Home replans its Home goal;
idle READY Pause remains stationary.

Held Pause preserves the outputs and rises vertically at current X/Y/attitude
to Home Z, without descending if already above it. Paused feedback, actual pose,
DI1 and outputs remain supervised. Drop monitoring activates when first-retract
height is reached, including during this parking rise. DI1 LOW lasting 500 ms
after activation requests Stop; confirmed loss while parked also starts the
same put-back routine described below. A shorter LOW followed by HIGH cancels
the pending loss. Confirmed loss is latched even if DI1 rises again.
Stale feedback or output faults are not
converted into ordinary drops. An eligible acquisition during the initial Stop
can establish trusted holding; late DI1 from a latched miss cannot.

All item returns use paused **Return Item** as their reference (rule 212), with
the exact original saved pre-pick X/Y/Z/attitude. The shared queue contains:

1. Optional vertical rise at current X/Y/attitude when more than 5 mm below Home Z.
2. Approach above the saved source at Home Z.
3. Descend to saved pre-pick; at 80%, DO2 OFF, DO14 ON, DO13 OFF, DO1 ON.
4. Retract vertically to Home Z; at 0%, DO2/DO14/DO1/DO13 OFF.

Every return target uses speed 100%. Rise/approach/Home use taught travel
acceleration, descent uses approach acceleration, and retract uses retract
acceleration. Global SpeedFactor and CP apply. Home Z must exceed saved settling-retract Z;
preview and candidate validation use the same geometry. The fixed +50 mm offset,
separate 50 ms exhaust pulse, explicit release DO calls and intermediate arrival/
release-I/O waits are absent. Exhaust ends with the retract's timed neutralization.

Explicit Return and paused drop append exact joint Home in that same group,
confirming Home execution/idle/neutral outputs/DI1 LOW. Explicit Return marks the
held candidate RETURNED and ends READY; a paused drop stays DROPPED and PAUSED.
An active drop instead appends the next retained entry/clearance/pre-pick/pick in
the same ordered group, without Home or a return-arrival wait. The original
source remains DROPPED and owned until advancing feedback reaches the next
clearance's returned MovL queue ID and neutral outputs with raw DI1 LOW have
been observed after the retract command was issued. Only then can the next
candidate own acquisition. Early command acceptance or old suction cannot do so.
If no candidate remains, confirm the shared retract above the bin and neutral/
DI1 LOW. Physical item placement is not measured.

Stop pre-empts either route and retains issued release/source evidence. Explicit
Recover cancels that operation, preserves grip through its existing lift/Home,
then neutralizes at Home; it never replays the interrupted release. Unexpected
outputs, stale feedback or rejected/unanswered commands remain faults.

Explicit `/return_item` cancels the interrupted action with a CANCELED result and
finishes READY at Home. The GUI uses **RETURN ITEM** while paused with
trusted holding. A paused drop runs the separate pulse-based route described here
and stays PAUSED at Home. Continue then attempts remaining candidates (including
from the retained batch of a completed Pick), or finishes without a pick if none
remain. A dropped candidate stays `DROPPED`; completing the motions is not proof
that the physical item was placed back. A new Pick replaces the old unheld batch.
The source context is not restored after a process restart.

Put-back also retains its destination and `APPROACH`, `RELEASING` or `RELEASED`
progress until Home completes or the next-candidate route takes ownership.
Stop before/during release preserves the source even when suction is already
intentionally OFF. A normal candidate becomes RETURNED only when release is
confirmed; a candidate with latched loss stays DROPPED. Explicit Recover abandons
that return progress and cancels remaining candidates. It uses fresh stopped I/O,
never repeats the release/exhaust, and lifts to Home height before moving Home.
Then it resets the gripper to neutral. New DI1 HIGH after confirmed release still
blocks recovery travel.

Direct `/stop`, action cancellation, shutdown, and another Stop during parking
or return cancel all further host-side commands and require recovery. The
already-requested controller pulse can still switch EXHAUST OFF on its timer.
Other faults never automatically invoke put-back or release. Confirmed held DI1
loss during an active Pick uses the automatic routine below. The idle vendor
`isPauseCmdFlag` remains contextual telemetry and does not enable Continue.

Concurrent Stop callers may share the same ongoing attempt. Every later
explicit Stop/action-cancel gets a new dispatch and physical confirmation,
including after an earlier failure or unanswered Stop. Old results cannot
complete a newer attempt or change a newly started operation's state.

Confirmed held-item suction loss marks the owning candidate
`DROPPED` while preserving its source and outputs. This records loss of vacuum
confirmation, not proof that the physical item left the gripper. A subsequent
HIGH does not clear the latch. Physical Stop confirmation still requires its
accepted response and stationary empty queue; a DI1 loss is logged separately
and does not mislabel a successful Stop or poison later Stop confirmation.
Freshness and output-integrity failures retain their strict handling.

Rule 211's automatic loss interruption uses rule 223's delayed activation and
500 ms debounce: after first-retract height, monitor clearance/travel, idle
holding, tray detection and placement approach/descent. Confirmed loss latches
DROPPED and sends Stop directly in the FeedInfo callback. Command
admission shares the latch lock; no further interrupted command can be submitted.
Resolve outstanding replies within their original deadlines, acknowledge Stop,
then send a final Stop and confirm stationary joints/empty queue before returning.
Feedback/source/output failures or rejected/unanswered commands block recovery.

Monitoring ends only on observed, issued suction OFF, never on release-command
submission or finger motion. Planned release is exempt. Once loss is latched,
DI1 HIGH or a late planned release cannot erase it or mark the candidate PLACED.
Keep its original source plan. Use the shared paused Return Item approach,
80%-descent release and 0%-neutral retract to Home Z, then append the next eligible
entry/clearance/pre-pick/pick in the same group. Preserve
saved order and exclude failed/dropped/returned poses. No Home, new detection or
operator action is inserted while eligible poses remain. Exhausted automatic
return confirms the exit above the bin; active Pick may then use its existing
bounded new-batch policy and ensure Home before new candidate motion.

Auto Run cancels its speculative detector worker and a separate appended next-session
ledger; it preserves the shared saved ledger when continuing that same batch,
retains the old source/batch and leaves its placement count unchanged. It places
the replacement item after a successful retained Pick. Manual Place cancels its
interrupted placement and leaves the replacement at Tray Detect; an already
accepted Place result remains acceptance-only and status reports the recovery.
Idle HOLDING uses the same recovery under exclusive operation ownership. Direct
Stop/cancel always wins. Paused drop and explicit Return retain their existing
Home/paused endpoints. Explicit Recover cancels the batch.

The dedicated held-suction-loss condition cannot turn output/readiness faults,
invalid feedback, service failures or missing source context into automatic
motion. During a pending motion reply, request Stop immediately but still require
that response to be accepted within its original five-second deadline; keep the
loss latched, send no later old-group command, then confirm Stop before put-back.
A rejected/unanswered response, unconfirmed Stop, failed release or other fault
ends the action through normal containment and explicit Recovery. Direct Stop,
action cancellation and shutdown always pre-empt; pending Pause retains its
put-back-and-remain-paused endpoint. Idle HOLDING uses the automatic shared return;
standalone Home losses continue to require explicit Recovery.

Explicit Recover is a separate cancel-and-Home-then-relax operation under rule 176. It
never invokes the automatic put-back or next-candidate routine. A repeated
Recover after interruption preserves the cancelled state and uses a new Stop,
fresh gripper feedback and a newly measured motion origin. Output changes,
unknown suction, stale feedback or failed commands stop further motion.

The GUI SpeedFactor slider tracks the handle position and sends it once on
release. Keyboard and groove changes are debounced for 350 ms. Controller status
cannot overwrite an active or pending edit, and unchanged selections do not send
another SpeedFactor request.

The GUI has separate managed Pause/Return and permanent STOP controls. See the
operator button policy above. The two gripper LEDs show raw DI1 Suction and DI12
Finger open as Detected / Not detected / Unknown. DO commands and logical holding
do not drive these LEDs. DI12 Not detected does not prove that fingers are closed.
The held-item decision uses its 500 ms loss debounce independently of the raw
DI1 display. Full raw robot flags and DI/DO remain in typed status.

Item/Bin/Tray Teach fields, Browse buttons and Load Auto occupy the **Setup / Teach**
page. Full paths remain editable and available in field tooltips;
prefill and validated persistence remain; explicit local-mode selection reloads the controller.
Lifecycle controls and the command-log toggle stay outside the pages; motion
and speed/CP controls are on **Operate**.
The GUI consumes only controller APIs. It adds no Dobot/camera subscription or
I/O command. Missing/stale canonical feedback turns both LEDs Unknown; a
controller status older than one second by source timestamp or local receipt
also shows robot OFFLINE and disables controls that depend on status. Direct Stop remains
available whenever its service is reachable. No automatic Stop or command is
sent by this display logic. Rebuild interfaces and restart controller, GUI and
preview together for the extended message definition.

Feedback is condition-driven from the approximately 100 Hz FeedInfo stream.
Policies are: five seconds for service discovery and each Dobot
service response (including Stop), five seconds for output
feedback, one-second feedback age, two-second expected mode changes, three
consistent error/collision samples, three-second no-progress watchdog, and a
300-second physical-motion cap. Home
arrival is within one degree on every taught joint; Cartesian arrival is within
5 mm and one degree, plus enabled, queue-idle and stationary confirmation.

The bridge publishes `RobotStatus.is_enable` as `robot_mode == 5`; it is an
idle-mode alias on a separate, slower publisher rather than an independent
enable latch. Fresh RobotStatus remains mandatory for connection/liveness, and
idle READY/final-arrival checks require its Boolean to converge. Active motion
uses authoritative FeedInfo `EnableStatus`, mode, error/collision, user/tool and
queue fields so an asynchronous status sample cannot cancel an accepted move.

Every actual canonical Dobot call has paired audit output in the ROS console and
`logs/robot_controller/events-YYYY-MM-DD.jsonl`. Each `SEND` and terminal accepted,
rejected, timed-out, canceled, errored or late-response record contains a
process-local request ID, exact endpoint/request fields, ROS response `res`,
available `robot_return`, and elapsed milliseconds. This covers Startup/Recover
settings, DO, all three motion services, and the independent Stop channel. A
successful response is still only command acceptance; fresh robot feedback
remains required for completion.

The authority publishes the same timestamped human-readable state, phase and
Dobot audit lines on reliable transient-local
`/robot_controller/operator_log` (`std_msgs/msg/String`) with a retained depth of
1,000. The GUI lower panel is a read-only, no-wrap 1,000-line view of that topic,
collapsed by default to keep service request/reply traffic out of the main view.
**Show command log** expands it and exposes Copy Log; **Hide command log**
collapses it again. Messages continue accumulating while hidden, and expanding
shows the full retained buffer. The toggle is a transient diagnostic view;
no operator setup file or runtime logging policy changes.
Text is selectable with Ctrl+C, **Copy Log** copies the entire displayed buffer,
and incoming messages auto-scroll only while the operator is already following
the bottom. The topic is observability-only and does not replace typed status,
actions/services, or `events.jsonl`.

## Home and Pick

The explicit Home action is permitted from `READY` and trusted `HOLDING`.
It now uses the same conditional vertical-clearance and exact joint Home plan
as Pick. Preview shows those same endpoints. Every final Home command uses
`/dobot_bringup_ros2/srv/MovJ`, `mode=true`, with the six unwrapped taught angles
in degrees, taught travel `v`/`a`, and no timed I/O. Return routes that already
use speed 100% retain that rate. All final Home destinations, including recovery,
missed-pick exhaustion, item return and Auto Run, use this joint interpolation.
Tray Detect queues joint-target `MovL` then absolute joint `MovJ` to the same
saved angles (rule 247), after the existing linear pick lifts and Safety Z exit.
Placement approach/descent and its queued upward
retract remain linear `MovL`/`MovLIO`, including their timed I/O.

The initial step skips if all six fresh actual joints are within ±1° of the taught
tuple in fresh canonical joint feedback with RobotStatus idle,
`EnableStatus=1`, fault/collision clear, user/tool zero and held-item I/O intact
where applicable. Queue/execution confirmation remains required for sent moves.
Otherwise, more than 5 mm below taught Home Z, `RelMovLUser` first rises at current XY/attitude and confirms
5 mm/1° Cartesian arrival plus stationary/empty-queue feedback. Within 5 mm
below Home Z or anywhere above it, skip that preliminary rise and send exact
taught Home joints directly using joint-mode `MovJ` and ±1° joint confirmation.
The planner and first dispatch share one fresh confirmed joint-derived pose, so a
second origin reading cannot turn a planned upward correction into a rejected
downward move. When a rise is needed, final joint Home acquires its origin after
that rise finishes. The initial Home skip reads one fresh idle RobotStatus and
canonical joint sample immediately; it waits for no extra sample, dwell, FK or
service query. FeedInfo remains the fresh fault/frame/I/O guard. Miss returns
retain an explicit Cartesian exit transit, even at Home Z.
Item returns use their shared Home-Z retract instead.
Exhausted Pick queues pre-pick, clearance, exit transit and joint Home together,
using the confirmed stopped pose as origin and confirming only final Home.
Successful Pick uses the same vertical Safety Z exit after its two lifts, then
finishes at Tray Detect instead of Home.

Pick requires `READY`, DI1 clear and a loaded Tray Teach with recorded detect
joints. Its tray detector need not be armed; Pick only travels to the saved pose:

1. after successful tray placement, acquire fresh poses; interruption/return may
   retain eligible original poses under the same configuration. Request a fresh
   profile/model/camera/platform/bin-hash-matched
   batch from `/item_detect/get_item_poses`, advertised by exactly one root node:
   headless `/item_detect` or explicitly Armed `/item_teach`;
2. for a new batch, validate the response, transform item positions into base coordinates
   and rank by 3D distance from taught Home, using detector priority for ties;
3. run the same Home function once poses are available, skipping motion when already
   matched. If the result is empty, confirm Home before each of nine acquisition retries per
   Pick window; a tenth empty result waits at Home for Continue;
4. offset Link6 green/Y by the taught `pick_rotation` from each item's short-axis
   line while preserving taught tool Z;
5. attempt eligible saved candidates in controller Home-distance order, excluding terminal
   states; new batch size still comes from Item Teach `pose_candidates`;
6. after an intermediate miss, rise through that candidate's pre-pick and final
   clearance, queue its safety-Z exit and the next candidate's entry transit,
   then descend through the next pre-pick and final pick without returning Home;
7. after success, lift to settling_retract, then the additional pick_retract,
   rise through the Home-Z exit, then move to saved
   Tray Detect joints and finish HOLDING there with suction on;
8. after full exhaustion, confirm Home and repeat from a fresh batch, up to three
   nonempty batches per window; three exhausted batches wait at Home for Continue.

The nine empty-result retries are separate from physical-pick attempts and cannot
reset after a miss or ordinary Pause. This permits at most twelve pose requests per window
when nine empty observations occur among three nonempty batches. If all initial
requests are empty, wait after the tenth. Item-wait Continue grants a new window. Detector errors/timeouts and invalid evidence
remain terminal; no hardware command is retried after uncertain acceptance.

Item and tray retries have **zero added settling delay**. Each new request
requires RGB and every contributing raw depth frame strictly after its own
request timestamp; cached frames cannot satisfy the next attempt. The acquisition
pipeline waits for new camera frames within the existing deadline, then runs
inference. Item retries first confirm Home (skip motion if already matched);
tray retries remain at Tray Detect. Physical travel, capture and processing still
take time. Continue after tray exhaustion grants another ten-request budget.

The detector pose uses local X for the measured short axis and local Y for the
long axis. It is an in-plane heading, not a TCP attitude. The controller
composes that heading through the destination platform transform, projects the
short X axis perpendicular to the exact taught Home tool Z, then considers the
taught unsigned 0–90° `pick_rotation` on either side of that line. Since a
rectangle has no directed end, both modulo-180° directions are equivalent.
Every candidate independently chooses the legal attitude with the least CW/CCW
travel from taught Home; travel ties within 1e-12 radians prefer the CCW offset
so frame relabeling cannot magnify floating-point noise into a different turn.
All candidate poses are therefore ready before execution
and their rotations never accumulate across retries. Every candidate's
transit/descent/retract targets share its selected attitude; exact
joint Home restores the taught orientation. Platform tilt is not copied into
TCP roll/pitch, and all waypoint heights remain referenced to base Z.

Rule 157 changes item coordinates from long-X/short-Y to short-X/long-Y and
updates this planner together, so the physical gripper orientation remains the
same. Teach-file dimensions, pick points, Home and motion rates are unchanged.
Rebuild/restart perception and all controller processes together. Pose requests
send `pose_convention=item_short_x_long_y_v1`, and matching response evidence is
mandatory before candidate planning; older or unspecified conventions fail
visibly instead of being interpreted as a new item frame.

The current station also requires the latest strict schema-7 robot-camera
transform `Link6 <- robot_camera_link` (transform only). For every candidate,
preview and hardware independently compose its calibrated camera-link body
at Link6's pick height (`item Z + standoff_height`) into `platform_reference`.
Use the shared Gemini 335 housing: RGB optical XYZ size 90/25/30 mm, center
(+11, 0, −12.79) mm relative to `robot_camera_color_optical_frame`. Compose the
documented nominal RGB-to-camera-link transform with the saved mounting transform;
this yields camera-link center (−10.77, −25, 0) mm and XYZ size 30/90/25 mm.
This fixed mechanical model does not replace the factory optical TF used for
RGB/aligned-depth measurement or change any saved calibration. The geometry
sources and derivation are in the [Item Perception README](../item_perception_yolo/README.md).
Transform all eight corners and project their convex outline onto
platform XY, including mount/platform tilt. The normal Home-relative pick
attitude is used if the entire outline is inside/on the
green Bin Teach ROI; otherwise the exact 180° tool-Z mirror is used if safe.
The mirror preserves the undirected short-axis line and unchanged tool Z but
may exceed the normal 90° Home-relative travel limit. If neither body fits,
planning fails before hardware candidate motion; the detector must
have excluded that pose before ranking, allowing the next safe candidate to
take its place. Robot-camera calibration SHA-256 is part of configuration and
detector evidence. Blue inset checks still
apply solely to the item pick point.

The camera-body visualization belongs to Item Teach. Its launch starts the
read-only `item_perception_yolo/robot_camera_box` node, publishing
`/item_teach/robot_camera_body` from Item Teach's exact validated calibration
selection. Controller GUI/headless launches do not start it. The display follows
live Link6 with the shared RGB-offset housing; invalidated/missing teaching
updates hide it. Controller planning and calibration selection remain independent
of Item Teach's display. Canonical RViz includes the new topic; reload its config
if already open. No camera TF, command client or hardware action is added.
This pick-pose body check does not model mounts/cables or the swept travel path.

Item Teach schema 16 also requires the nearby depth radius/height settings
(defaults 150/60 mm). The detector rejects any candidate with a usable depth
point inside/on the outer bin and camera-XY radius with a floor-relative height
difference at/above the configured threshold, excluding standoff. The platform
Z=0 floor is evaluated at each point’s camera XY; height follows camera Z. During acquisition the detector ranks
geometrically eligible candidates, then checks nearby height one by one until the
requested number pass or all are exhausted. Blocked candidates are skipped; later
unneeded candidates remain unchecked. Hardware and Preview receive
only the filtered profile-bound batch; the controller has no depth subscription
or separate scene scan. Open older profiles in Item Teach, review the proposed
new defaults, Save and manually redeploy/reload matching profiles. Motion and
tray placement filtering are unchanged.
Schema 14 requires `geometry.depth_frame_count` (1, 3 or 5; default 3). The
controller validates all contributing post-request depth timestamps and the newest
response depth stamp without aging out an already accepted batch. Detection uses
one immutable median and an item-only effective minimum of 500 mm.
`candidate_timing` separates controller validation, detector wait and total latency.
Within a validation pass, reuse the already verified item/model pair rather than
reading it twice; all existing content/hash and dynamic binding checks remain.
After explicit GUI review/Save, manually deploy matching profiles and restart/reload
consumers. Temporal depth capture preserves motion queues and bounded ordinary
empty-result Home/reacquisition retries. Schema 14 additionally extends the item
request for repick exclusions, with separate all-blocked handling described above.

The nominal geometry uses pick Z equal to item Z plus `standoff_height`,
pre-pick independently adds `prepick_height`, first retract adds
`settling_retract` to nominal pick, and second retract adds `pick_retract` to
that settling height. These are schema-16 fields; old `retract_height` has no
alias. Older files require explicit entry of both new distances and GUI Save. Enabled
trickle lowers only final pick Z from that baseline; the other heights stay fixed. Home/travel
uses taught travel rates and final descent uses approach rates. A successful
pick's first rise to settling_retract uses taught retract speed/acceleration. A missed
pick's same rise uses `v=100` and taught travel acceleration. The additional pick_retract
rise uses `v=100` with taught travel acceleration for both outcomes. All rates
remain subject to global SpeedFactor. All item returns use `v=100`, with travel
acceleration for approach/rise/Home,
approach acceleration for release descent and retract acceleration for ascent.

The outputs are explicit mutually exclusive states. Finger OPEN is DO2 OFF then
DO14 ON, CLOSE is DO14 OFF then DO2 ON, and NEUTRAL is both OFF. Vacuum SUCK is
DO1 exhaust OFF then DO13 ON, EXHAUST is DO13 OFF then DO1 ON, and NEUTRAL is
both OFF. Timed state changes preserve that order, and feedback showing either
opposing pair ON together faults the motion. `grip_onpick` controls final-arrival
pickup closing independently of `use_grip`, which selects finger holding during
transport. Every candidate enters with OPEN and no DI12 wait. At the start of final descent,
MovLIO orders DO2 OFF, DO14 OFF, DO1 OFF, then DO13 ON. With
`grip_onpick=false`, confirm DO2 OFF then DO14 OFF at final pick arrival, before
settling/probing. An earlier suction pickup performs the same relaxation after
Stop acceptance, before the held lift. This applies independently of `use_grip`.
Keep suction ON/exhaust OFF throughout; monitor DI1 during both service and
output-feedback waits. No extra settling interval is added. Each transition
requires acceptance and fresh output feedback; faults or Stop block further work.

The first `candidate_1_home_to_pick` group contains three control points: item
X/Y at Home Z with OPEN at 50%, pre-pick with no I/O, then final pick with finger
NEUTRAL followed by SUCK at the start. It intentionally skips the item-clearance point on this initial descent.
DI1 is eligible only after the candidate's SUCK transition. The first eligible
HIGH during descent or settling immediately latches pickup and blocks later normal
command admission. Validate every already-issued service reply before sending one
pickup Stop to discard the old queue. Already-queued motion can continue while
its reply is pending; the five-second response deadline is unchanged. Operator
Stop/cancel, Pause and fault/drop containment remain independent and immediate.
After its successful command acknowledgement, mark the candidate HELD and start
the upward return from the latest fresh joint-derived pose. Do not wait for
stationary joints, idle status, an empty-queue sample or the remaining settling
time before this return. Command acceptance is not a physical-stop confirmation.
If acquisition occurs during stationary final-pick finger preparation, finish only
the DO already sent: require its accepted reply and fresh commanded output feedback
before pickup Stop. After accepted Stop, finish the remaining channel without
repeating confirmed outputs. An accepted DO is queued, so its reply alone cannot prove
the output changed. Keep the five-second output bound and vacuum/output monitoring;
rejection, unanswered calls, missing output feedback or cancellation block return
and retain fault containment. Remove the former early pickup Stop followed by a
second discard; delayed successful callbacks cannot Stop the new return queue.
The `pickup_detected` event marks the latch before the eventual pickup Stop send.
Otherwise the terminal pose must remain
within tolerance with advancing queue-idle feedback and the commanded final
outputs for the profile's `pick_settling` interval while DI1 is monitored. This
is the complete final-pick confirmation interval; there is no fixed 300 ms pick
gate before it or separate sensor wait after it. If DI1 is still low when the
interval ends, keep the attempt ACTIVE and suction ON. At taught final-approach
speed and acceleration, command one upward MovL through **the full remaining
distance from actual settled Z to saved settling-retract Z**, preserving measured XY and
attitude. A 40 mm remaining retract gives a 40 mm lift. Keep all finger/vacuum
outputs unchanged and eligible DI1 monitored throughout the transition, admission
and lift. DI1 HIGH takes the same acquisition Stop path and starts the normal held
lift/Tray Detect queue from the actual stopped pose. With Grip on Pick OFF, fingers
remain relaxed throughout the probe. Stop and Pause still pre-empt, retaining the
original source.

Only fresh feedback confirming the probe queue ID, endpoint and idle state without
DI1 latches FAILED; no second settling interval is added. With no upward distance
available, skip the probe instead of inventing a height or descending. Events
record actual lift millimetres and rates, including a zero-distance check. Existing
failed retract/retry/Home runs from the probe endpoint, retaining the settling
waypoint for its existing release events even when already at that height, then
queues the additional lift, exit and next-candidate approach. Preserve late-DI1
isolation after failure. This applies to first candidates,
retries, resumed unheld picks, return continuations and queued Auto Run picks.
No new setting, schema, detection request or attempt count is added.

The operator log and structured candidate event report `FAILED — no DI1 pickup
detected after 300 ms settling and the lift to settling_retract.` for a 0.3-second
setting; milliseconds always use the loaded `pick_settling`. The event's
`candidate_state` field and typed status remain `FAILED`.

DI1 HIGH-to-LOW uses one fixed `SUCTION_LOSS_DEBOUNCE_SEC = 0.500` filter owned
by the canonical feedback monitor, not a teach-file setting. On pickup, retain
HELD/source context but defer loss detection until a fresh joint-FK sample reaches
the settling-retract height, or the higher actual pickup origin. Observe
the upward crossing even during CP blending; do not split the settling/additional-lift/Tray
Detect queue or wait at a midpoint. LOW time before that crossing does not count.
At activation, start a new timer if DI1 is already LOW; otherwise the next LOW
starts it. Advancing LOW feedback at least 500 ms later
confirms loss; any HIGH resets the pending interval immediately. Re-reading a
snapshot or publishing the same controller timer cannot complete the debounce.
A feedback gap beyond the existing freshness limit cannot count toward it, and
stale/invalid feedback keeps its existing failure handling.

Zero remaining first lift arms on new position feedback at that height. Pause
acquisition uses the same gate, and a parking rise may complete it. Direct Stop
retains the pending height. Suction OFF clears pending deferral so release/reset
cannot carry it into another item. Raw acquisition, release and output guards
remain unchanged. Events `drop_detection_deferred` and `drop_detection_armed`
record the height and 500 ms threshold for run auditing.

The internal snapshot's `suction_present` value is used after acquisition Stop
and throughout held motion, motion-origin/Home checks, Stop/recovery, idle
holding and managed Pause/Continue/put-back. Raw `digital_input_bits` and output
history remain unchanged: first HIGH pickup detection, cold/untrusted DI1,
missed-pick suction reset, release/exhaust confirmation and unheld checks still
use raw DI1. DO13 loss, opposing outputs and other faults receive no new delay.
Direct Stop is sent immediately and its stationary confirmation never waits for
this timer. The debounce is independent of taught `pick_settling` and motion-timed
release; it introduces no teach setting, launch argument or schema change.

`grip_onpick=true` enters CLOSE at confirmed final-pick arrival, even with DI1 LOW,
regardless of `use_grip`. Send DO14 OFF then DO2 ON and confirm both responses and
output echoes before starting the one taught settling interval. Keep CLOSE through
the pickup probe. Early DI1 still stops descent; after Stop acceptance close at
that stopped pickup position. Never repeat a confirmed close at later DI1 success.
Both output calls retain DI1, Stop/Pause, vacuum and five-second response/echo guards.
Log `pickup_fingers_closed` with `final_pick_arrival` or `pickup_stop_accepted`.

| `use_grip` | `grip_onpick` | At final arrival / pickup | At 50% of settling lift | At 50% of additional pick_retract |
| --- | --- | --- | --- | --- |
| true | true | CLOSE | Keep closed | Keep closed |
| true | false | RELAX | Keep relaxed | CLOSE |
| false | true | CLOSE | RELAX | Keep relaxed |
| false | false | RELAX | Reaffirm RELAX | Keep relaxed |

The halfway event uses MovLIO: CLOSE is `{0,50,14,0}` then `{0,50,2,1}`;
RELAX is `{0,50,2,0}` then `{0,50,14,0}`. CLOSE belongs to the second lift;
RELAX remains on the first lift.
For both Grip on Pick OFF combinations, DO2/DO14 stay OFF until that halfway
event for their configured transport state. With delayed closing, fingers stay
relaxed through the settling lift and the first half of pick_retract. A missed
attempt with Grab on Pick ON can close before being declared missed; its existing
retract/exhaust/neutral sequence remains. Next entry reopens the fingers.
Held Pause preserves outputs; Continue restores CLOSE for `use_grip=true` or
RELAX for false before direct Tray Detect travel, including when Pause canceled
either lift's event. Suction remains on throughout.

After valid tray pose/depth and placement validation, `use_grip=false` uses
MovLIO for pre-place, with `{0,80,2,0}` then `{0,80,14,1}`: open fingers at 80%
of travel to the pre-place target. There are no separate DO calls or output waits
before placement. Auto Run starts fresh next-bin inference before queue admission;
all three placement motions must still be accepted before appending the next Pick.
`use_grip=true` retains MovL pre-place and stays closed until the existing 80%
descent finger event. Suction remains ON until its 90% descent release.
Track approach output changes only after that command is issued,
including feedback during its response wait. Opening preserves suction and held
monitoring; it cannot confirm release. No DI12 or intermediate arrival wait is
added. Stop/drop interruption prevents later commands. Release recovery never
reopens or repeats release. Place releases vacuum at 90%, Return at 80%; both
retain 0% retract reset and 80% descent finger opening.
Return Item's approach has no new finger event. Hardware, Auto Run and Preview
share the placement targets.

Successful Pick queues latest measured pose → settling_retract → pick_retract →
Safety Z exit → saved Tray Detect joints as one `candidate_N_pick_to_tray` group.
The vertical targets preserve measured X/Y and attitude and never descend.
Safety Z is max(taught Home Z, current height); retain the explicit exit even if
coincident with pick_retract. The first held lift uses taught retract rates;
pick_retract uses `v=100` with travel acceleration. Exit and Tray Detect use taught
travel speed/acceleration, scaled by global SpeedFactor. There is no final Home
in this success route. The Safety Z exit is a queued control point and may be
rounded by the selected global CP; it has no separate physical arrival gate.
The last two commands are joint-target MovL followed by MovJ to the identical
six saved joint angles, using the same rates. Admit MovJ as soon as MovL returns
acceptance; never wait for MovL arrival or branch on a measured joint mismatch.
The final MovJ restores the saved joint configuration, including full turns.
Confirm only its saved joints (±1°), fresh idle RobotStatus and executed/empty
queue after admission; no midpoint wait or fixed arrival dwell is added.

Selected CP remains active, so the controller may blend into MovJ before reaching
the exact linear endpoint. This pair does not validate the wrist's swept path or
guarantee against cable winding. A rejected/unanswered MovL prevents MovJ admission;
failed final MovJ blocks completion and tray acquisition through existing containment.
Both commands retain the same source, Stop/Pause, output and held-loss guards.
Preview shows both nominal targets, not an interpolated joint trajectory.
SUCK stays ON without reissuing it; no EXHAUST/NEUTRAL release events are sent.
Holding/output checks remain active throughout. DI1-loss detection begins at
measured first-retract height, then uses the shared 500 ms debounce.
Held Continue moves directly from the confirmed safety-height parked pose to
Tray Detect, without replaying the pick, lifts or a Home detour. Successful Pick
requests no tray observation and does not place; the next Place checks saved
Tray Detect joints/idle without queuing observation travel. Preview requires
the same saved tray destination and includes its TF for every candidate success.

A missed non-final candidate starts one
`candidate_N_pick_to_retry_M_pick` CP-blended group: old final to old pre-pick,
old pre-pick to old clearance, N's exit `park_transit`, M's entry `park_transit`,
then descend directly through M's pre-pick to M's final pick. The exit
preserves actual stopped X/Y/attitude; the entry uses M's X/Y/attitude. Both use
`max(stopped Z, taught Home Z)` and taught travel rates. The exit is named
`pN_transit_exit` and has no I/O. The entry's command name remains `pM_transit`
so admission marks the correct candidate
ACTIVE; a Pause there retains M for Continue. The same geometry helper supplies
Pause's `park_transit`, which neutralizes its I/O. Unheld Continue reuses that
already-confirmed entry transit. Missed retries retain both entry and exit
transits queued. Shared item return uses its Home-Z approach and retract. Both
are blended under CP (default 100%) and can be rounded without an
intermediate arrival wait or dwell; coincident coordinates still get separate
requests. This next-item descent matches the first approach: command M's selected
attitude at entry, then retain its X/Y/attitude while changing Z through pre-pick
and pick. Rule 264 omits only M's clearance from ordinary missed-pick retries;
automatic dropped-item Return Item retains its clearance/ownership handoff.
No physical orientation or J6 confirmation is added before descent, and blending
can round the transit. Direct Stop can always prevent later requests from being sent.
At 80% of the first rise the group enters EXHAUST. At 0% of the second rise it
enters both finger and vacuum NEUTRAL. At 50% of the transfer to M's transit it
enters OPEN; M's pre-pick has no I/O. At the start of M's final descent
it enters SUCK. Each service must return `res=0` before the next is sent, while
only M's final target is physically checked. A late DI1 from candidate N is
ignored throughout its latched-miss recovery; candidate M is armed only after
DO13 OFF and a clear DI1 have been observed before its new SUCK. A final
candidate miss queues its old pre-pick EXHAUST rise, old-clearance NEUTRAL rise,
explicit exit transit and exact joint Home as one
`candidate_N_pick_to_home` group. Each request still requires ordered `res=0`
acceptance, but only exact joint Home is physically confirmed; clearance and
transits are blended control points. Later DI1 cannot reclassify the latched miss
as success. Successful Pick instead uses the two lifts, Safety Z exit and Tray
Detect route above; its held-item outputs and monitoring remain active.

All `MovJ`, `MovJIO`, `MovL`, `MovLIO`, and `RelMovLUser` requests in one named batch are admitted
in target order. Each must return `res=0` before the next is sent, with no
additional inter-command delay. This is an admission barrier, not an
intermediate physical-arrival wait: it prevents separate ROS services from
reversing their dashboard TCP queue order, as observed in a failed Home return.
A response error, rejection, cancellation, or five-second response deadline invokes independent
Stop containment; an outstanding late response remains contained by another
Stop. Batch start, every dispatch/response, complete group admission,
interruption and terminal completion are recorded with the batch name.
An armed new-candidate DI1 or cancellation during admission prevents all later
targets in that group from being sent; late DI1 from a latched miss is ignored.
The independent Stop path bypasses normal admission. During a held-item return, a
timed DO2/DO14 transition requested by MovLIO/MovJIO is accepted only as the exact
old-to-commanded state change after that command has been sent, and becomes the
new expected state when observed;
uncommanded output changes, lost DI1/DO13, and wrong terminal states still fail.

Motion requests carry only `user=0`, `tool=0`, and their selected `v`/`a` rates;
they never carry a per-command `cp` or `r`. The global CP setting (default 100%)
controls all transitions; the idle slider changes it and Recover retains it.
As specified by the Dobot protocol, smoothing can bypass exact intermediate pick coordinates and timed
I/O can occur during a blended transition. Successful Pick physically confirms
only Tray Detect after its lift group; exhausted Pick confirms only exact joint Home.
Serialized service responses may let
a short pick segment decelerate even at global CP 100%; queue order takes
precedence over uninterrupted blending.

Final Home uses `MovJ(mode=true)`. Tray Detect queues `MovL(mode=true)` then
`MovJ(mode=true)` to the same taught joints for observation. Auto Run returns from
placement through those joints with `MovJIO(mode=true)`, opening fingers at 50%
before the next linear item transit. Other no-I/O targets use `MovL`.
`MovLIO`/`MovJIO` require real non-empty timed DO tuples. Initial/shared Home's conditional rise uses `RelMovLUser`; item exit
transits use Cartesian `MovL`. The controller never calls
`InverseKin` or vendor `Continue`; controller Continue rebuilds the remaining route.
Service acknowledgement is acceptance only; actual
feedback confirms every result. A coherent miss advances to the next pending
candidate; Continue retries an approach interrupted by Pause. All command,
feedback, state, cancellation, and result events
are written to ignored `logs/robot_controller/events-YYYY-MM-DD.jsonl`, retained for 30 Dubai calendar days.
Each candidate plan also records its source quaternion, transformed short axis,
commanded green axis, configured offset, selected CW/CCW side, rotation from the
taught Home reference, and target RPY.

Joint Home and Tray Detect completion use ±1° independently on every joint.
Home never wraps angles modulo 360°: a matching tool pose with J6 one turn away
must still execute the exact saved joint target. MovJ uses joint interpolation;
the final Home path is not constrained to a straight Cartesian line.
Cartesian targets use 5 mm Euclidean translation and 1° orientation, calculated
from the same canonical `/joint_states` sample using the existing CR10 model.
RobotStatus `is_enable` is the vendor's mode-5 idle indication. Home, clearance
and every other non-pick endpoint complete on the first qualifying joint/status
update after the complete group's acceptance. Require a newer joint source stamp
and a newer RobotStatus receipt; duplicate/backward joint stamps cannot refresh
position evidence. Both topic callbacks wake the wait immediately, independently
of FeedInfo updates. There is no added stability interval.
`MovJ` and `MovL` expose their queue ID in the existing reply: require that exact
`FeedInfo.currentCommandId` at completion. The fixed vendor `MovJIO`, `MovLIO` and
`RelMovLUser` response schemas expose only `res`; those endpoints instead require
live execution evidence latched during dispatch/travel (running/queued status,
changed queue ID or joint movement). Queue-empty confirmation remains a separate
execution guard. Short/zero-distance commands can
complete without ever observing a running flag when the stream shows their
execution. No new query service or midpoint wait is added. Optional managed
safety rises within the existing 5 mm tolerance are skipped before dispatch
using the actual pose. Only a final pick
uses a timed settling interval: its taught `pick_settling` duration under the
same feedback gates. Joint-mode Home remains joint-only; no position check
compares against FeedInfo `tool_vector_actual`.

Before every independently acquired motion-batch origin or stopped-pose
measurement, the controller waits up to two seconds for newer joint/status
feedback, idle RobotStatus and an empty command queue. There is no additional dwell
duration. A missed final pick supplies its settled pose directly to the return.
Successful acquisition instead uses fresh joint feedback after the Stop command
acknowledgement, without waiting for idle or stationary feedback, and carries
that pose into the immediate lift/return batch. The controller uses
FK from that validated joint sample; it never sends a separate
`GetPose` request or subscribes to the slower, unstamped `ToolVectorActual`
topic. A timeout names every current
idle, queue and joint/status advancement blockers. Fresh connected RobotStatus, joints,
FeedInfo, user/tool zero, held-item integrity, and the normal final-arrival
checks remain mandatory. Startup/Recover retains its separate 200 ms READY
lifecycle coherence. Outside successful pickup's acknowledgement-only handoff,
Stop confirms two distinct joint source samples unchanged
within 0.05° and the stopped/empty-queue state; fault/disabled stopping remains
confirmable. Paused/placement hold checks also use joint-derived pose. The FK
model is nominal geometry; this change is software-validated, not a new physical
calibration or an automatic runtime service dependency.

Software tests use synthetic services/feedback and must never commission
physical motion. Real commissioning requires separate explicit authorization,
clear workspace, functional physical emergency stop, verified wiring, and an
attentive operator.

`robot_controller` is the sole production/runtime Dobot command issuer.
`motion_debug` may issue direct commands only as a mutually exclusive maintenance
application; production Startup rejects competing maintenance command owners.

## Remote Setup (settings and connection tests only)

**Tools / Debug → Remote Setup** opens the separate `remote_setup` UI in either
Either station mode, including during a run. It requires no controller ownership
handoff and never opens automatically. Save the server/arm/belt settings, then
explicitly Test Saved Settings. The HMI owns only windows it launches.
External windows remain external; HMI exit closes its owned window/test child.
Remote Setup does not block Load Auto or acquire robot command authority.
See [Remote Setup](../remote_setup/README.md) for `.env` keys, dependencies and
headless exit codes. No remote commands, heartbeat or server-controlled cycles
are enabled by opening or successfully testing it.
