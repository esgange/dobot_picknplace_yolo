# Workflow, Rules, Blueprint, and Diary

This is the durable handoff document for the `dobot_picknplace_yolo` project. Read it before changing the workspace. It exists so a new contributor or coding agent can continue from the same assumptions without relying on chat history.

## 1. Project intent

Build a ROS 2 pick-and-place system combining:

- a Dobot CR10 arm and its official ROS 2 V4 driver;
- an Orbbec Gemini 335 depth camera and its official ROS 2 wrapper;
- YOLO-based object detection and a future grasp/planning pipeline.

The repository must be transferable to an offline PC. “Offline-ready” means the repository contains all project and vendored source files and does not require Git submodule initialization or a network fetch. It does not imply that the offline PC automatically has Ubuntu, ROS 2, GPU drivers, system packages, camera firmware, or robot firmware; those prerequisites must be provisioned separately.

## 2. Current source baseline

The following manufacturer sources were cloned online, then converted from submodules into ordinary tracked directories on 2026-09-01. The Dobot tree is a deliberate CR10-only profile derived from the upstream commit shown below; it is not an unmodified mirror.

| Source | Upstream branch | Upstream base commit | Local path | License file |
| --- | --- | --- | --- | --- |
| Dobot 6Axis ROS 2 V4 (CR10-only profile) | `main` | `def21d05149576b9aed261f38e62ea236a5d8ec5` | `src/DOBOT_6Axis_ROS2_V4` | `src/DOBOT_6Axis_ROS2_V4/LICENSE` (MIT) |
| Orbbec SDK ROS 2 wrapper | `v2-main` | `8e7cad2bfa2c4a6ac4e779be99c64e72166043af` | `src/OrbbecSDK_ROS2` | `src/OrbbecSDK_ROS2/LICENSE` (Apache 2.0) |

Official URLs:

- <https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4>
- <https://github.com/orbbec/OrbbecSDK_ROS2>

Orbbec documents Gemini 335 as part of the Gemini 330 series and recommends `v2-main` for new designs. The expected camera launch file is `orbbec_camera gemini_330_series.launch.py`.

## 3. Repository layout

```text
.
├── AGENTS.md                         # short rules automatically visible to agents
├── README.md                         # user-facing setup and transfer guide
├── docs/
│   └── WORKFLOW_RULES_BLUEPRINT_DIARY.md
└── src/
    ├── DOBOT_6Axis_ROS2_V4/          # CR10-only vendor profile
    └── OrbbecSDK_ROS2/               # immutable-by-default vendor snapshot
```

Future project packages should be added under `src/` with clear package boundaries. The current-run decomposition is:

```text
camera bringup → image/depth synchronization → YOLO detection
             → camera-to-robot TF/calibration → grasp target selection
robot bringup → feedback/status visualization → operator diagnostics
```

Motion planning and servo-action execution are outside this run's source scope.

Keep application code, launch files, calibration, and configuration separate from the two vendor directories. Do not fork or rename upstream package names unless there is a documented reason.

## 4. Operating rules

### Offline-first source policy

- The two vendor trees are regular tracked files, not submodules.
- A normal `git clone` of this repository must contain the vendor source; no `git submodule` command should be needed.
- Every new or changed project rule must be recorded in this diary in the same change. This diary is the durable source of truth when agents or contributors switch.
- The Dobot profile is intentionally limited to the physical CR10 and excludes Gazebo/robot simulation, MoveIt, and servo control. Retain the hardware CR10 URDF/Xacro and mesh assets, but do not add Gazebo packages/worlds/launch files/simulation tags/dependencies, any MoveIt package/configuration/plugin/dependency, `servo_action`, or the Dobot `ServoJ`/`ServoP` interfaces unless the user explicitly changes scope and this diary is updated in the same change.
- Never delete vendor license/notice/attribution files.
- Never commit a vendor update without recording its upstream URL, branch, commit ID, date, reason, and validation in this diary.
- Keep a clean separation between upstream snapshots and project patches. Prefer a new integration package; if a vendor patch is unavoidable, document the exact file and rationale.
- Project-wide runtime settings belong only in the ignored root `.env`, created from the tracked `.env.example`; never commit machine-specific `.env` values. Configuration is absolute: use canonical keys and strict `KEY=value` syntax; do not add compatibility aliases, fallback values, package-local configuration files, or alternate configuration workflows. The canonical shell loader and Dobot bringup launch require and validate `ROS_LOCALHOST_ONLY`, `DOBOT_ROBOT_LAN1_IP`, `DOBOT_ROBOT_LAN2_IP`, `DOBOT_CONNECTION_TIMEOUT_MS`, `DOBOT_ROBOT_TYPE`, `DOBOT_ROBOT_NUMBER`, `DOBOT_TRAJECTORY_DURATION`, and `DOBOT_ROBOT_NODE_NAME`, and must fail otherwise. `ROS_LOCALHOST_ONLY` is fixed to `1` so ROS graph discovery and DDS traffic remain on the local computer. `DOBOT_CONNECTION_TIMEOUT_MS` is explicit and must be an integer from 100 through 60,000.
- Every ROS package under `src/` must have a package-local `README.md` beside `package.xml`. Keep packages grouped under their vendor snapshot; do not flatten or relocate them without documenting the architecture change.
- Runtime event logs are isolated by package under ignored `logs/<package>/events.jsonl`, timestamped, and bounded at 1,000 records by overwrite. Cross-package compilation is a standalone script; do not add a logger-only ROS package.

### Development workflow

1. Read this diary and `README.md`.
2. Check `git status --short --branch` and avoid overwriting existing user changes.
3. Source the installed ROS 2 distribution and build from the repository root.
4. Make the smallest scoped change. Keep generated output in ignored `build/`, `install/`, and `log/` directories.
5. Run relevant tests or launch-file checks that do not move hardware. Run `git diff --check`.
6. Update this diary for a new package, architectural decision, vendor change, offline constraint, or failed validation.
7. After each completed change passes relevant tests/checks, review `git status` and the staged diff, then commit with a descriptive message and push to the current branch's configured remote. The user authorized this standing workflow on 2026-09-14; no repeated confirmation is needed. Stage only the task's source, tests and documentation, not unrelated user edits, local configuration/logs/build output, station calibration/teaching artifacts or operator model weights. Verify the push and report the commit ID. If validation or pushing is blocked, preserve the work and report the blocker, not a successful completion. Never force-push or rewrite shared history.

### Hardware safety

Real Dobot motion is opt-in, not a default test. Before any hardware command, confirm the requested robot model, IP/network, remote-control mode, workspace clearance, emergency stop, speed limits, and collision assumptions. Robot logic must use non-hardware unit fixtures for automated tests; camera and perception tests should use sensor-only or synthetic inputs.

## 5. Offline handoff procedure

### Full-history transfer

On an online development machine:

```bash
git status --short --branch
git bundle create ../dobot_picknplace_yolo.bundle --all
```

Copy the `.bundle` file to the offline PC and run:

```bash
git clone ../dobot_picknplace_yolo.bundle dobot_picknplace_yolo
cd dobot_picknplace_yolo
git status --short --branch
```

The resulting checkout contains the vendored sources and history without contacting GitHub. Verify that `.gitmodules` is absent and both vendor directories contain their package manifests.

### Source-only transfer

When Git history is not needed, make an archive from the committed tree:

```bash
git archive --format=tar.gz --output=../dobot_picknplace_yolo.tar.gz HEAD
```

Extract it on the offline PC. Install or stage the compatible ROS 2 and retained system dependencies separately; source completeness alone cannot supply operating-system packages. Gazebo and MoveIt are not part of the current profile.

## 6. Vendor refresh workflow

Vendor refreshes happen only on an online maintenance machine:

1. Clone the official upstream repository into a temporary directory at the intended branch.
2. Record the old and new commit IDs and inspect `git diff --no-index`.
3. Copy only the reviewed snapshot into the matching `src/` directory; do not copy its `.git` directory. For Dobot, reapply the hardware-only CR10 profile; remove all Gazebo/simulation, MoveIt, `servo_action`, `ServoJ`, and `ServoP` sources and integration assets; and do not reintroduce other model assets.
4. Confirm licenses and notices are still present.
5. Build or run the relevant non-hardware checks.
6. Update the baseline table and add a dated diary entry before committing.
7. Recreate an offline bundle and validate it on an isolated machine.

Never use a floating “latest” version in an issue, script, or deployment note. Always name the upstream commit used.

## 7. Diary entries

### 2026-09-01 — workspace bootstrap

- Initialized the `main` branch and connected `origin` to `esgange/dobot_picknplace_yolo`.
- Added the official Dobot V4 and Orbbec ROS 2 repositories.
- Initially tracked them as submodules to pin exact commits.
- Converted both checked-out trees into vendored ordinary files when the offline-PC requirement was clarified.
- Added `AGENTS.md`, this diary, offline transfer instructions, and vendor-refresh rules.
- Parent bootstrap commit: `ffbf0f4c20ba1d690af380a7e1652f4485737d57`.

### 2026-09-01 — rule persistence

- New rule: every new or changed project rule must be added to this diary in the same change.
- Reason: keep the project contract durable across agent and contributor handoffs.

### 2026-09-01 — CR10-only Dobot profile

- Change: removed all non-CR10 Dobot MoveIt packages, robot URDF/Xacro files, and robot mesh directories; retained `cr10_moveit`, CR10 descriptions/assets, and shared driver/runtime packages.
- Reason: target hardware is Dobot CR10, and the offline project should not carry unused robot configurations.
- Upstream base commit: `def21d05149576b9aed261f38e62ea236a5d8ec5` on `Dobot-Arm/DOBOT_6Axis_ROS2_V4:main`.
- Defaults changed to the fixed `cr10` profile in bringup, RViz, Gazebo, MoveIt, and action-client entry points.
- Validation performed: checked remaining model asset directories, package manifests, launch defaults, and documentation references; non-CR10 references left in `V4新增指令` are generic protocol documentation, not installed robot configurations.
- Follow-up: build the CR10-only workspace on a ROS 2 Humble host before hardware operation.

### 2026-09-01 — Project-wide runtime configuration

- Change: added the tracked root `.env.example` and taught `dobot_bringup_ros2.launch.py` to load the root `.env` automatically at launch.
- Reason: keep machine-specific robot/network settings in one ignored file that travels with the offline workspace without storing them in Git.
- Configuration contract: the launch requires the root `.env`, valid `DOBOT_ROBOT_LAN1_IP`/`DOBOT_ROBOT_LAN2_IP` values, and an explicit bounded `DOBOT_CONNECTION_TIMEOUT_MS`; malformed lines, `export` syntax, unsupported keys, duplicate keys, missing values, invalid IP addresses, duplicate interface addresses, and invalid timeout values hard-fail. LAN1 is attempted before the explicitly configured LAN2 diagnostic failover; after both fail, one bounded outage event is recorded and the explicit sequence is retried.
- Vendor patch: `src/DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/launch/dobot_bringup_ros2.launch.py` strictly loads the root configuration and initializes package logs, `src/DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/src/cr_robot_ros2.cpp` requires both interface parameters and a datalog directory, and the commander implements the explicit two-address sequence. No third-party dotenv dependency is required.
- Validation performed: Python syntax compilation, strict `.env` parser checks, and launch-source inspection; no hardware launch was performed.
- Follow-up: add camera and perception settings to `.env.example` only when those integrations are introduced.

### 2026-09-01 — Absolute configuration policy

- Rule: all future project configuration and launch behavior is canonical and explicit. Do not add compatibility aliases, fallback values, or alternate configuration workflows.
- Reason: prevent unexplained behavior when moving the offline workspace between machines or changing agents.
- Enforcement: strict configuration files and launch code must hard-fail on missing, malformed, unsupported, or ambiguous values.

### 2026-09-01 — Package organization and documentation

- Change: added `src/README.md` and a package-local README beside all 13 ROS package manifests; documented the Dobot and Orbbec package groups.
- Reason: make package ownership, entry points, and offline source layout clear when the workspace is transferred or an agent changes.
- Architecture: packages remain inside their official vendor snapshot directories under `src/`; no package was moved or renamed.
- Vendor documentation patch: the new package READMEs are project-added documentation inside the vendored trees; upstream source, license, and attribution files remain unchanged.
- Validation performed: enumerated every `package.xml` and verified each package directory contains `README.md`; no hardware launch was performed.

### 2026-09-01 — Explicit LAN failover and bounded package datalogs

- Change: replaced the single Dobot IP with required `DOBOT_ROBOT_LAN1_IP=192.168.20.204` and `DOBOT_ROBOT_LAN2_IP=192.168.200.1` settings. Bringup attempts LAN1 first, then the explicitly requested LAN2 diagnostic address.
- Reason: support the robot's primary LAN and manual diagnostic LAN without hiding which interface was selected.
- Result behavior: each successful connection writes a definitive `robot_connection_result` with `status=connected` and the selected interface/IP. Both-address failure writes one `status=failed` result per continuous outage with both attempted addresses. After failure, the driver retries the same explicit sequence; a recovery is recorded when either configured address succeeds.
- Datalog architecture: package-level logging writes `logs/<package>/events.jsonl`; bringup startup creates an isolated file for every package, and each file overwrites before record 1,001. `scripts/compile_logs.py` merges timestamped package records on demand; no logger-only ROS package is used.
- Validation performed: strict two-address config parsing, package log initialization, C++ build, and bounded logger tests; no hardware connection was attempted.
- Follow-up: use the standalone compiler when a universal cross-package record is needed; keep source events package-local.

### 2026-09-01 — English-only README documentation

- Change: removed the vendor Chinese README files `src/DOBOT_6Axis_ROS2_V4/README_ZH.md`, `src/DOBOT_6Axis_ROS2_V4/V4新增指令/README.md`, and `src/OrbbecSDK_ROS2/README_CN.MD`; retained the corresponding English README files and removed their Chinese-language selector links.
- Reason: keep README documentation English-only for predictable offline handoff and agent/contributor use.
- Scope: this is an explicit user-approved exception to the normal “preserve upstream README files” rule. Non-README vendor protocol/reference documents and all LICENSE/NOTICE/attribution files remain intact.
- Validation performed: scanned all remaining README files for CJK characters and stale links to removed translations; no hardware launch was performed.
- Offline transfer validation: generated artifacts from prior build/test runs are removed before the next source archive.

### 2026-09-01 — Root `.env` is the only bringup configuration

- Change: removed `src/DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/config/param.json` and moved its active single-robot settings to the ignored root `.env` and tracked `.env.example`: `DOBOT_ROBOT_TYPE`, `DOBOT_ROBOT_NUMBER`, `DOBOT_TRAJECTORY_DURATION`, and `DOBOT_ROBOT_NODE_NAME`. LAN1/LAN2 addresses remain in the same file. The obsolete multi-robot `current_robot` selector and duplicate node entry were not migrated.
- Reason: eliminate the vendor multi-robot configuration path and keep one explicit, inspectable configuration workflow for offline transfers.
- Enforcement: bringup accepts only the eight canonical keys, requires all eight, validates local-only ROS operation, IPv4 addresses, and the bounded TCP timeout, enforces the CR10-only/single-robot profile, validates trajectory duration and node name, and has no defaults or JSON fallback.
- Validation performed: source reference audit, strict configuration parser checks, launch syntax compilation, and package inventory; no hardware launch was performed.
- Offline transfer validation: the source archive contains no package-local `param.json` configuration.

### 2026-09-01 — `motion_debug` keeps bringup operator-controlled

- Change: established `motion_debug` as a GUI-only project package. Its launch file starts only the GUI; it never includes or starts `dobot_bringup_v4` and never opens a robot TCP connection implicitly.
- Rule: `motion_debug` checks for the canonical Dobot bringup service and prompts the operator with the explicit bringup command when that service is unavailable. The GUI remains open and waits for the service rather than selecting a fallback driver or launch path.
- Reason: keep hardware startup explicit and prevent a diagnostic GUI from causing an unexpected real-robot connection while still making the missing runtime dependency clear.
- Architecture: `dobot_msgs_v4` remains the message/service interface dependency; `dobot_bringup_v4` remains a separately launched runtime process.
- Validation performed: launch-source inspection and static Python/XML checks; no bringup launch or hardware connection was performed.
- Offline transfer validation: the package and its documentation remain in the repository source tree; no network or submodule is required.

### 2026-09-01 — ROS 2 middleware is local-only

- Change: added required `ROS_LOCALHOST_ONLY=1` to the canonical root `.env` contract and added `scripts/source_ros_workspace.bash` as the strict shell environment loader.
- Rule: every terminal used for this workspace must source the canonical loader. The loader rejects missing, duplicate, unsupported, or malformed configuration, clears inherited ROS overlays, sources exactly ROS 2 Humble and this workspace, and exports the `.env` local-only value without a compatibility fallback.
- Scope: local-only applies to ROS 2 discovery and DDS communication. It does not disable the Dobot driver's explicit TCP connection to the two configured robot controller addresses.
- Shell integration: the current development PC sources the tracked `scripts/source_ros_workspace.bash` from its user `.bashrc`; an offline PC must explicitly source the loader from its own checkout.
- Vendor patch: the Dobot bringup launch now requires and validates `ROS_LOCALHOST_ONLY=1` before starting its node so direct bringup launches cannot silently use network-wide ROS discovery.
- Validation performed: shell syntax and strict environment checks, Python syntax checks, clean-shell package discovery, and non-hardware launch parsing; no robot or camera was launched.
- Offline transfer validation: the loader and `.env.example` are tracked and require no network fetch or third-party dotenv package.

### 2026-09-01 — Explicit Dobot connection-result datalog

- Change: standardized the definitive connection outcome as a `robot_connection_result` event in `logs/dobot_bringup_v4/events.jsonl`. Each successful connection records `status=connected`, the selected `interface`, and its `ip`; both-address failure records one `status=failed` event per continuous outage with both configured addresses.
- Reason: make it unambiguous whether bringup connected and whether LAN1 or LAN2 was selected, while retaining the individual attempt and attempt-failure events for diagnosis.
- Vendor patch: updated `src/DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/src/command.cpp`; this project behavior is not part of the upstream Dobot snapshot.
- Validation performed: clean ROS 2 Humble rebuild of `dobot_msgs_v4` and `dobot_bringup_v4`, built-binary log-contract inspection, and non-hardware launch parsing; no robot command or hardware launch was performed.
- Offline transfer validation: the implementation and documentation are tracked source and require no network service.

### 2026-09-01 — Dobot LAN2 diagnostic address correction

- Change: set the canonical Dobot LAN2 diagnostic address to `192.168.200.1` in the active root `.env`, tracked `.env.example`, and Dobot project documentation. LAN1 remains `192.168.20.204` and is always attempted first.
- Reason: the controller's correct manual diagnostic IPv4 address is `192.168.200.1`; the previous value was invalid for this robot setup.
- Vendor documentation patch: updated the network table in `src/DOBOT_6Axis_ROS2_V4/README.md` to match the canonical project configuration; upstream executable behavior is unchanged by this documentation edit.
- Validation performed: strict `.env` loading in an isolated shell, confirmation that only the current workspace overlay is active, and non-hardware bringup launch parsing showing LAN2 `192.168.200.1`; no robot connection or hardware command was performed.
- Offline transfer validation: `.env.example` and the documented address are tracked; the machine-local `.env` remains intentionally ignored.

### 2026-09-01 — MoveIt and servo-action excluded from this run

- Rule: the current CR10 profile has no MoveIt planning stack and no `servo_action` package. Do not reintroduce MoveIt packages, configuration, plugins, launch files, dependencies, or servo-action control without an explicit user scope change recorded in this diary.
- Change: removed `cr10_moveit`, `dobot_kinematics_plugin`, `dobot_moveit`, and `servo_action`; removed the MoveIt-only FK utility, images, Gazebo launch/configuration assets, and the CR10 Xacro's trajectory-control plugin block. `dobot_gazebo` is retained strictly for controller-free model/world visualization.
- Reason: this run does not use MoveIt or servo trajectory motion, and unused planning/control packages created unnecessary build dependencies and unexplained partial-build failures.
- Retained scope: CR10 description/meshes, Dobot bringup/messages/demos/RViz, controller-free Gazebo visualization, Orbbec camera packages, and `motion_debug`. The workspace package count is 10.
- Vendor patches: pruned the four upstream Dobot packages and related assets; updated `cra_description/urdf/cr10_robot.xacro`, `dobot_gazebo`, `dobot_rviz` comments, and the Dobot English README to remove the excluded integration path. Removed obsolete `tests_require` metadata from `dobot_demo`; its test dependencies remain in `package.xml`.
- Build cleanup: removed the corresponding ignored `build/`, `install/`, and package datalog directories. The removed datalogs contained only automatic package-initialization events. Removed obsolete `tests_require` metadata from the project `motion_debug` package as well.
- Validation performed: confirmed an exact 10-package inventory, no excluded dependency in retained package manifests/build files, valid package XML and CR10 Xacro, successful plain `colcon build` of all 10 packages, retired packages absent from the clean runtime overlay, and non-hardware launch parsing for bringup, RViz, `motion_debug`, and Orbbec. Gazebo launch parsing hard-fails on this development PC because the separately retained `gazebo_ros` system dependency is not installed; source build succeeds and no alternate launch path is provided.
- Offline transfer validation: removed packages and their generated local artifacts will not be part of a source archive; no replacement dependency or fallback workflow is introduced.

### 2026-09-01 — Deterministic Dobot TCP failover timeout

- Rule: every Dobot feedback/dashboard TCP connection attempt uses the required root `.env` key `DOBOT_CONNECTION_TIMEOUT_MS`, validated as a canonical integer from 100 through 60,000 ms. The current absolute value is `3000`; the driver never falls back to the operating system's unbounded/default connect timing.
- Change: replaced blocking TCP connect behavior with nonblocking connect plus a bounded poll, prints each LAN/interface/IP attempt immediately, and includes `timeout_ms` in its package datalog attempt record. LAN1 is still strictly first and LAN2 is still strictly second.
- Reason: an unavailable LAN1 route remained in `SYN-SENT` through Wi-Fi for the operating system's long TCP timeout, hiding the already reachable LAN2 controller. Both LAN2 controller ports, `30004` and `29999`, were independently confirmed reachable before implementation.
- Vendor patches: updated the Dobot launch/config parser, bringup node parameters, commander, and TCP socket implementation. Updated the canonical shell loader and `motion_debug` parser to accept and validate the same eighth key without aliases or defaults.
- Validation performed: shell/Python syntax checks, successful strict loading of the eight-key active `.env`, clean rebuild of `dobot_msgs_v4`, `dobot_bringup_v4`, and `motion_debug`, built-binary timeout-message inspection, and non-hardware launch parsing showing `3000` ms. Independent TCP checks confirmed LAN2 ports `30004` and `29999`; the user-started old process eventually established both LAN2 sockets and recorded `status=connected interface=LAN2 ip=192.168.200.1` after its 134-second operating-system LAN1 timeout. The new bounded runtime behavior requires that old process to be stopped and relaunched and was not hardware-launched by the agent.
- Offline transfer validation: timeout logic and configuration templates are tracked source and require no external library or network fetch.

### 2026-09-01 — Ordered `motion_debug` startup initialization

- Rule: before enabling its operator controls or modifying controller settings, `motion_debug` runs one exact startup sequence: `StopMoveJog`, `DisableRobot`, `EnableRobot`, confirm controller mode `5`, SpeedFactor `50%`, Tool `0`, Tool `1` TCP `{0,0,0,0,0,0}`, and CP `100%`.
- Explicit exceptions: `StopMoveJog` and `DisableRobot` are best-effort preconditioning calls. If either call is unavailable, fails, throws, or times out, the package records a `WARNING` and continues. If Disable reports success but mode `4` is not confirmed within five seconds, that also records a `WARNING` and continues to Enable. These are the only non-fatal exceptions to the project's strict behavior rule and do not prove robot or connection health.
- Strict boundary: `EnableRobot`, confirmation of mode `5`, and all four setting operations hard-fail on service absence, negative response, exception, or five-second timeout. A strict failure sends no later startup step and requires an explicit `motion_debug` restart; there is no automatic retry.
- Safety/UI behavior: the sequence starts only from controller mode `4`, `5`, or `11`; other non-transient modes reject initialization. All GUI command controls stay disabled while initialization is active. Initialization itself sends no joint or Cartesian motion target.
- Logging: each phase uses the package's UTC `logs/motion_debug/events.jsonl`; tolerated precondition failures have level `WARNING`, strict failures have level `ERROR`, and completion records the applied absolute values.
- Reason: establish the requested stop/disable/enable controller reset before changing commissioning settings while keeping its two explicitly tolerated preconditions visible and deterministic.
- Validation performed: Python syntax compilation, source-order inspection, simulated callback-path tests, and a non-hardware `motion_debug` package build; no ROS launch, robot connection, or hardware command was performed.
- Offline transfer validation: implementation and documentation are tracked inside `src/motion_debug`; no network service, new dependency, compatibility alias, or alternate workflow was introduced.

### 2026-09-01 — Canonical debug-script directory

- Rule: `motion_debug` saves and loads script JSON files only from root `config/debug_script`. The absolute filename contract is `config/debug_script/<script-name>.json`.
- Change: replaced the former `config/motion_calibrate` directory name in the GUI and package documentation. The old local directory was inspected and was empty, so no user script required migration.
- Strict boundary: do not read, recreate, or fall back to `config/motion_calibrate` or any package-local script directory.
- Reason: use a directory name that directly identifies these files as `motion_debug` scripts.
- Validation performed: reference audit, Python syntax compilation, installed-source comparison, and a non-hardware `motion_debug` package build; no ROS launch, robot connection, or hardware command was performed.
- Offline transfer validation: the canonical directory is created locally by the GUI and all saved JSON files remain ordinary workspace files available for source/archive transfer when tracked.

### 2026-09-01 — `gripper_control` requires live bringup

- Rule: the canonical `dobot_bringup_v4` process must already be separately launched, connected, and publishing before `gripper_control` starts.
- Startup contract: `gripper_control` must require the existing `/dobot_bringup_ros2/srv/DO` service, `/dobot_msgs_v4/msg/RobotStatus`, and `/dobot_bringup_ros2/msg/FeedInfo`. It must not start bringup, prompt the operator to start it, search alternate interface names, or remain in an indefinite waiting mode.
- Failure contract: if any required canonical interface or connected-state feedback is absent during the bounded startup check, write an explicit failure to `logs/gripper_control/events.jsonl` and terminate startup without enabling the GUI controls.
- Reason: the operator controls Dobot bringup separately, and opening the gripper controller is an assertion that bringup is already healthy.
- Implementation status: enforced by `src/gripper_control`; implementation and non-hardware validation are recorded in the follow-up package entry below.

### 2026-09-01 — `gripper_control` canonical Dobot IO integration

- Change: added the project-level `gripper_control` package as an IO-only gripper/suction diagnostic GUI. It uses the existing `/dobot_bringup_ros2/srv/DO` service, `/dobot_msgs_v4/msg/RobotStatus`, and `/dobot_bringup_ros2/msg/FeedInfo` topic; no custom bridge, topic, service, or launch override remains.
- Startup: the process waits at most five seconds for the DO service, connected RobotStatus, and valid FeedInfo. Missing or stale startup requirements are written to `logs/gripper_control/events.jsonl` and terminate the process before GUI creation. It never starts or prompts for bringup and never waits indefinitely.
- IO behavior: `digital_outputs` drives the fixed DO1, DO2, DO13, and DO14 display and `digital_input_bits` drives DI1 suction status. Each action is serialized, each DO response is checked, and each output transition must be confirmed by FeedInfo. Grip is DO14 OFF → DO1 OFF → DO2 ON → DO13 ON. Release is DO2 OFF → DO13 OFF → DO1 ON → 250 ms delay → DO1 OFF → DO14 ON → 100 ms delay → DO14 OFF. All requests use the existing `DO(index,status,time=0)` service; pulses use explicit GUI delays followed by OFF calls.
- Shutdown: closing the GUI requests DO1, DO2, DO13, and DO14 OFF regardless of cached state, attempts every channel, records failures, and has a five-second hard shutdown bound.
- Metadata: added `std_msgs` dependency, removed obsolete `tests_require`, installed the package BSD-3-Clause license, and removed the broken `ros_domain.py`/launch-argument path.
- Reason: keep the gripper controller compatible with the retained Dobot bringup interfaces and the project's operator-controlled bringup rule, with no alternate behavior hidden behind fallbacks.
- Validation performed: AST/XML checks, canonical interface audit against Dobot bringup source and `DO.srv`, simulated startup/service/feedback/action/shutdown paths, launch-description parsing, and package/full-workspace non-hardware builds; no ROS launch, robot connection, or IO command was performed.
- Offline transfer validation: package code, license, README, and fixed interface contract are tracked source; runtime logs remain ignored package-local files and no network dependency was introduced.

### 2026-09-01 — Absolute `gripper_control` IO channel map

- Rule: `gripper_control` controls exactly `DO1`, `DO2`, `DO13`, and `DO14`. The fixed roles are DO1 suction exhaust, DO2 gripper close, DO13 finger close, and DO14 gripper open. It monitors generic DI1 and DI2 checks; their semantic names are intentionally deferred. No channel override, alternate mapping, or compatibility fallback is permitted.
- Change: corrected the output roles and translated the Grip/Release sequences to the operator-confirmed channel map. UI rows, request validation, FeedInfo bit confirmation, Grip/Release sequences, manual timers, and close-time OFF requests all use one canonical channel tuple.
- Feedback mapping: Dobot `digital_outputs` uses channel N at bit N-1, so DO13 and DO14 are read from bits 12 and 13 respectively. DI1 and DI2 are read from bits 0 and 1 of `digital_input_bits`.
- Safety: no hardware command was sent while applying or validating this mapping.
- Offline transfer validation: the channel contract is ordinary tracked source and documentation with no runtime configuration or network dependency.

### 2026-09-01 — Generic DI1 and DI2 checks

- Rule: `gripper_control` displays `DI1` and `DI2` as generic HIGH/LOW digital-input checks. Do not assign semantic names to either input until the user explicitly defines them.
- Change: replaced the single named DI1 suction-status indicator with two channel-number-only indicators backed by the canonical FeedInfo `digital_input_bits` field. DI1 uses bit 0 and DI2 uses bit 1.
- Safety: this change only reads feedback and does not add any new output command or hardware action.
- Offline transfer validation: the generic input-check behavior is tracked package source and documentation with no configuration or network dependency.

### 2026-09-02 — Bounded Orbbec camera connection attempts

- Rule: `orbbec_camera_launcher` gets at most three total camera launch attempts during one launcher run: one initial attempt and two retries. The budget includes startup and any later supervised recovery, so the package can never restart cameras indefinitely.
- Attempt contract: one attempt covers the complete configured camera set. Every configured Gemini 335 must be detected by its exact serial number, start successfully, and publish its required color and depth streams within the explicit startup timeout. A missing or unhealthy camera makes the whole attempt fail; partial-camera operation is forbidden.
- Retry contract: after a failed attempt, terminate only camera processes owned by this launcher, record the exact failure and attempt number in `logs/orbbec_camera_launcher/events.jsonl`, then wait exactly three seconds before the next attempt. Root `.env` must contain the strictly validated absolute values `ORBBEC_MAX_ATTEMPTS=3` and `ORBBEC_RETRY_DELAY_SEC=3`; there is no default, alias, exponential backoff, manual budget reset, or alternate override.
- Final failure: after the third failed attempt, write the terminal failure event, stop all package-owned camera processes, and terminate the launcher. Connection checks and supervision remain mandatory and cannot be bypassed.
- Configuration workflow: the non-headless GUI is the only editor of camera configuration. It atomically replaces the complete canonical `ORBBEC_*` key set in root `.env` while preserving Dobot values and comments. The internal headless launcher is read-only, accepts no launch arguments, and hard-fails until every active camera name and exact serial is complete and unique. Opening the GUI never starts cameras; Launch Cameras is an explicit operator action.
- Local/process boundary: the GUI launch, internal headless launch, and supervisor require inherited `ROS_LOCALHOST_ONLY=1`. Every vendor Gemini 330-series process receives `enumerate_net_device=false`. The GUI owns only its headless supervisor process group, the supervisor owns only the vendor launch process groups it creates, and existing publishers on required camera topics reject an attempt instead of being mistaken for owned camera output.
- Change: replaced the package-local YAML, legacy root aliases, optional environment sourcing, multi-terminal partial startup, global process-table cleanup, selectable watchdog bypass, manual restart service, and unlimited per-camera exponential restart with one strict root `.env` parser, GUI-only atomic editor, read-only internal launcher, complete-set supervisor, bounded package datalog, and five simulated contract tests. Added the package license, installed README, canonical console entry points, root/package documentation, and the twelfth package inventory entry; removed generated caches and obsolete `tests_require`/PyYAML/`std_srvs` metadata.
- Project configuration: `.env.example` and the ignored active `.env` now contain every required Orbbec mapping, stream, local USB enumeration, timeout, and retry key. One or two active camera slots are supported explicitly. Empty active serials are permitted only so the GUI can perform first-time configuration; headless startup hard-fails while any active serial is empty.
- Vendor patch: extended the intentionally patched Dobot bringup `.env` parser to require and validate the canonical Orbbec keys so the single project configuration remains strict. Extended `motion_debug`'s supported root-key set for the same schema; neither package uses the camera values at runtime.
- Validation performed: Bash syntax and isolated canonical-loader checks; Python AST and package XML parsing; strict root config validation; successful clean plain `colcon build` of all 12 packages; GUI and Dobot launch-description parsing; expected pre-hardware headless rejection for empty serials; five passing non-hardware tests covering first-time versus launch-ready configuration, atomic GUI writes, 1,000-event rollover, exact third-attempt terminal failure, owned-process launch, stream health, and lifetime retry consumption. No camera node, robot node, or hardware command was launched.
- Offline transfer validation: all implementation, tests, configuration templates, documentation, and the BSD-3-Clause package license are ordinary tracked source. The active `.env`, package logs, and generated build/test output remain ignored, and no new downloaded dependency, submodule, or online service was introduced.
- Safety: no camera node, robot node, or hardware command was launched while recording this rule.

### 2026-09-02 — Detected Orbbec serial click-to-copy

- Rule: selecting a detected Orbbec serial number in the non-headless launcher's device list immediately copies that exact serial to the system clipboard. Selection does not assign the serial to a camera slot or modify `.env`; slot assignment and Save to `.env` remain separate explicit operator actions.
- Change: bound the detected-device list selection event to one clipboard operation, visible copied-SN status, and a package `serial_copied` event. Clipboard failure is visible and logged as `serial_copy_failed`.
- Safety: this is a GUI-only clipboard action; it does not launch a camera, modify camera configuration, or command robot hardware.

### 2026-09-02 — Five-second Orbbec stream startup timeout

- Rule: `ORBBEC_STARTUP_TIMEOUT_SEC` is exactly `5`. During each numbered camera launch attempt, every configured camera must publish both its color and depth image streams within five seconds of the complete-set process launch.
- Failure behavior: if any required stream is still absent after five seconds, the whole attempt fails, every package-owned camera process is stopped, and the fixed three-attempt/three-second-rest policy continues with its remaining lifetime budget.
- Change: updated both tracked `.env.example` and the ignored active root `.env`; no package-local timeout, launch override, or fallback value was introduced.
- Validation performed: strict project configuration parsing and canonical shell-loader validation; no camera node, robot node, or hardware command was launched.

### 2026-09-02 — RViz is an actual-robot and ROS TF viewer only

- Rule: `dobot_rviz` is read-only. It never starts Dobot bringup, publishes or relays joint states, creates manual joint controls, substitutes zero/synthetic positions, or offers a non-hardware model mode. It uses one launch command with no arguments.
- Canonical data path: the fixed CR10 `robot_state_publisher` subscribes directly to `/joint_states`; it converts the actual bringup joint feedback and installed CR10 URDF into `/tf` and `/tf_static`. RViz displays that RobotModel and every TF published on the local ROS 2 graph with `base_link` as its fixed frame.
- Strict source contract: the root-namespace node named by root `.env` key `DOBOT_ROBOT_NODE_NAME` must be the sole `/joint_states` publisher. Messages must contain exactly `joint1` through `joint6` in canonical order, six finite positions, and a non-zero timestamp. A missing valid stream after five seconds, a wrong or additional publisher, malformed feedback, publisher loss, or feedback older than one second terminates the viewer; there is no fallback pose.
- Datalog contract: viewer startup, stream health, shutdown, and strict failures are written with UTC timestamps to `logs/dobot_rviz/events.jsonl`, which overwrites before event 1,001 under the existing package-log rule.
- Vendor patch: replaced the Dobot vendor package's `joint_state_relay.py`, `/rsp_joint_states` remapping, `/joint_states_robot` branch, manual `joint_state_publisher_gui`, and `gui`/`live_hardware`/model/config launch overrides with direct canonical feedback, a read-only strict monitor, and the fixed installed CR10 visualization assets. Updated package dependencies, metadata, documentation, and non-hardware contract tests.
- Reason: ensure RViz visualizes only the physical CR10 feedback and any real TF generated by local ROS nodes, without hiding missing hardware data behind a simulated or zero joint state.
- Validation performed: Python AST checks, five isolated monitor/logger contract tests, all five ROS package tests/lints, a successful `dobot_rviz` build, installed launch parsing showing no arguments, and a retired-interface reference audit passed. The stale generated relay symlink left by the incremental install was removed. No robot, camera, or RViz process was launched while implementing the change.
- Offline transfer validation: all viewer code, CR10 assets, tests, and documentation are ordinary vendored source files; no new network service, downloaded dependency, or alternate configuration file was introduced.

### 2026-09-02 — Hidden-by-default expanding `motion_debug` Scripts panel

- Rule: all `motion_debug` script-management and playback controls remain inside the main application window but are hidden when the GUI starts. The `Scripts` button is flush right in the full-width `Robot Status` header; it expands the main window to the right and reveals the panel. `Hide Scripts` retracts the same window. Do not create a separate script window or restore an always-visible script panel.
- Panel behavior: expansion records the current main-window width, adds the panel's requested width, and keeps the current height. Retraction removes the panel from the layout and restores the recorded width. Both actions preserve the currently loaded script, editor selections, playback controls, tolerance, and datalog state.
- Change: moved the existing script widgets into one initially unmanaged in-window frame, added the main-window toggle, and reduced the default width now that scripts no longer reserve space at startup. This replaces the intermediate popup design before it became the final workflow. No script file format, storage path, robot service, motion command, or startup initialization behavior changed.
- Validation performed: Python AST/import checks, a successful canonical plain `colcon build --packages-select motion_debug`, launch-description parsing with no arguments, installed distribution/entry-point resolution from `/home/erds`, and an isolated installed-package fake-node Tkinter lifecycle check. The GUI check confirmed one top-level window, exact collapsed/expanded/restored widths of `1000`/`1433`/`1000` pixels, exact flush-right button alignment, and preserved script UI state after retraction. An intermediate `--symlink-install` build was rejected after its generated launcher failed outside the repository with missing `motion-debug` metadata; the package was immediately restored with the project's plain-build workflow.
- Safety: this is a GUI layout/lifecycle change only; no robot, camera, ROS node, or hardware command was launched.
- Offline transfer validation: the behavior uses only the existing Python Tkinter runtime and tracked package source; no dependency, configuration key, network service, or alternate workflow was added.

### 2026-09-02 — Manual-prefix, ChArUco-only camera calibration

- Superseding rule: `camera_calibration` calibrates exactly one manually selected fixed camera per run and uses ChArUco only. This replaces the earlier same-day proposal to read selectable prefixes from `.env`; the package must never read, infer, persist, or fall back to `ORBBEC_CAMERA_*_NAME`. The prefix field is empty at every process start and requires an explicit Apply Settings action.
- Derived camera contract: the applied prefix exclusively derives `/<prefix>/color/image_raw`, `/<prefix>/color/camera_info`, `<prefix>_color_optical_frame`, and `<prefix>_link`. Exact image and CameraInfo frame IDs are mandatory. Changing any setting disables capture until Apply Settings validates the complete configuration, recreates subscriptions, and clears samples and the prior solution.
- ChArUco contract: the GUI exposes OpenCV's predefined 4x4, 5x5, 6x6, and 7x7 ArUco dictionaries plus `DICT_ARUCO_ORIGINAL`, checker squares X/Y, checker-square size in millimetres, ArUco-marker size in millimetres, minimum detected ChArUco corners, and minimum calibration samples. Marker size must be smaller than checker size; board geometry and corner gates hard-fail when invalid. Four-independent-marker boards and alternate target types are removed.
- Calibration contract: the camera must be fixed relative to `base_link`, the ChArUco board must be rigidly attached to the robot tool, and the package reads already-running camera data plus fresh actual `base_link -> Link6` TF. It never launches the Orbbec driver, Dobot bringup, robot-state publisher, RViz, or any motion command. A wrist-mounted camera requires a separate future tool-to-camera workflow and is not accepted as a fallback mode.
- Sampling and solve: ChArUco target TF must be no older than 0.5 seconds and stable within 2 mm/1 degree over 0.5 seconds. Robot TF must be non-zero and no older than one second. Samples must differ from every prior robot pose by at least 20 mm or 5 degrees. The OpenCV eye-to-hand solve produces `base_link <- <prefix>_link`, reports translation/rotation RMS consistency, and rebroadcasts the preview while the node remains alive.
- Output/logging: atomic output is isolated as `calibration/<prefix>_to_base.yaml` and records the explicit `target_from_source` convention, full ChArUco geometry, frames, translation metres, XYZW quaternion, UTC time, sample count, and RMS quality. Runtime events use UTC `logs/camera_calibration/events.jsonl`, overwriting before event 1,001.
- Change: replaced the imported 3,468-line dual-mode C++/Qt package with a self-contained Python ChArUco detector, strict GUI, tested eye-to-hand solver core, one no-argument local-only launch, package license, canonical README, and explicit ROS dependencies. Removed the missing `aruco_perception` dependency, four-marker depth board fitter, eye-on-hand mode, launch/topic/mode overrides, `ros_domain.py` lookup, subprocess calibrator, MoveIt-era YAML fields, package-local sample calibration, and alternate GUI launch.
- Validation performed: Python AST and package XML checks; retired interface/reference audit; four pure tests covering manual-prefix/board validation, a synthetic eye-to-hand transform with effectively zero solve error, isolated YAML schema/path, and exact 1,001-event rollover; successful canonical package build; all five reported package tests passed; installed GUI/core imports and installed launcher resolution from `/home/erds`; no-argument launch parsing; an offscreen GUI contract check confirming an empty prefix, 17 dictionary choices, and disabled sampling before Apply Settings; and a successful root-level plain `colcon build` of all 13 workspace packages.
- Package inventory: `camera_calibration` is the fourth project-level package and the thirteenth workspace package. Root and source package indexes now include it, and the source index uses the canonical plain `colcon build` workflow.
- Safety/offline status: no camera, robot, RViz, calibration GUI process, or hardware command was launched. All source, tests, launch, README, and BSD-3-Clause license files are ordinary tracked workspace files with no new download, submodule, or online runtime dependency.

### 2026-09-02 — Explicit camera-to-hand and camera-on-hand calibration modes

- Superseding rule: `camera_calibration` has exactly two explicitly selected modes and starts with no mode selected. Camera-to-hand is the fixed-camera workflow and solves `base_link <- <prefix>_link`; the camera stays fixed relative to `base_link` and the ChArUco board is rigidly attached to `Link6`. Camera-on-hand is the wrist-camera workflow and solves `Link6 <- <prefix>_link`; the camera is rigidly attached to `Link6` and the ChArUco board stays fixed relative to `base_link`. This supersedes the earlier fixed-camera-only limitation.
- Frame contract: the retained CR10 Xacro defines the final robot link as exact frame `Link6`. Both `base_link` and `Link6` are fixed by mode and are not editable, inferred, or replaced by alternate-frame fallbacks. Both modes still require fresh actual `base_link -> Link6` TF and never launch or command robot hardware.
- Solver contract: camera-to-hand retains the eye-to-hand equation and checks consistency of the board pose relative to `Link6`. Camera-on-hand uses a separate eye-in-hand equation and checks consistency of the stationary board pose relative to `base_link`. Both solve the camera optical frame first, then use the required live Orbbec internal TF to express the result at `<prefix>_link`.
- Output rule: every Save action creates a new atomic YAML in `calibration/`. Exact filename families are `camera_to_hand_calibration_<UTC_TIMESTAMP>.yaml` and `camera_on_hand_calibration_<UTC_TIMESTAMP>.yaml`, using filesystem-safe `YYYYMMDDTHHMMSS_microsecondsZ`; the camera prefix remains inside YAML metadata and prior calibration files are never overwritten. YAML records the mode, camera mounting, board mounting, `target_from_source` convention, and exact reference/source frames.
- GUI lifecycle: Qt is initialized on the main thread before the ROS node, and the node is spun by an explicitly owned single-threaded executor that is stopped before node/context destruction. The process owns SIGINT/SIGTERM handling so Ctrl-C exits Qt before executor/node/ROS teardown. This makes process shutdown ownership deterministic; it does not add a retry or runtime fallback.
- Validation performed: Python compilation and `ament_flake8`; five passing pure tests including independent synthetic solutions for both modes, both reference-frame/filename contracts, collision no-overwrite behavior, and the 1,000-event log bound; six passing reported package test results; successful plain root `colcon build` of all 13 packages; installed offscreen GUI checks confirming no default mode, empty prefix, disabled capture, and exact mode/reference mappings; no-argument launch inspection; and a real-display empty-prefix launch followed by SIGINT that exited cleanly with status zero. The launch produced no camera subscription because settings were never applied, and no robot or camera hardware command was sent.
- Offline transfer validation: both solvers, tests, GUI, filenames, documentation, and lifecycle behavior are ordinary tracked source with no network service, new dependency, submodule, or package-local configuration.

### 2026-09-03 — Unified copyable camera-launcher GUI log

- Superseding rule: the non-headless `orbbec_camera_launcher` GUI has one chronological, read-only displayed log containing device-scan progress/results, detected serials, raw scan output, configuration actions, and supervisor lifecycle messages. It must not split these records across separate list/log windows or erase prior displayed events when a new device scan runs.
- Clipboard contract: normal text selection plus `Ctrl+C` copies only selected log text, and the explicit `Copy Log` button copies the complete displayed log. Clipboard success/failure is recorded in the package event log. Selecting a detected camera serial no longer copies it and only selects it for the existing `Use selected` assignment action; this explicitly supersedes the 2026-09-02 detected-serial click-to-copy rule.
- Change: replaced the four-row detected-serial list with a compact read-only serial selector above the single expanded log, changed scan output from replacement to chronological append behavior, and labeled displayed scan/configuration/supervisor messages by source.
- Validation performed: Python compilation and `ament_flake8` passed; seven simulated package tests passed, including exact full-log and selected-text clipboard contents; the installed GUI inspection confirmed one text log, zero listboxes, appended scan/raw output, an unbound serial selector that leaves the clipboard unchanged, and both clipboard actions; and a plain root `colcon build` completed all 13 packages. The tests exercised watchdog state with fake scans/processes only; no camera or robot process was launched.
- Offline transfer validation: this uses existing Tkinter widgets and the package logger without a new dependency, configuration key, network service, or alternate launch workflow.
- Safety: this is a GUI display/clipboard change; no camera or robot hardware command is introduced.

### 2026-09-03 — Explicit single-camera terminal diagnostics

- Superseding rule: each camera-row action in the non-headless `orbbec_camera_launcher` GUI launches exactly that active, saved camera directly through `orbbec_camera gemini_330_series.launch.py` in a separate `/usr/bin/gnome-terminal`. The former `Use selected` serial-assignment actions are removed; detected-serial selection is informational and never assigns `.env` or changes the clipboard.
- Direct-launch contract: the GUI first atomically saves and strictly validates the complete root `.env`, passes the selected camera's exact name and serial, uses `device_num=1`, passes the same complete vendor stream/feature argument set as supervised launches, forces `enumerate_net_device=false`, and inherits `ROS_LOCALHOST_ONLY=1`. It has no watchdog or retry and is stopped explicitly with Ctrl-C in its visible terminal. Missing `/usr/bin/gnome-terminal` hard-fails; no alternate terminal workflow exists.
- Supervised-launch contract: the lower action is `Launch Both (Watchdog)` for a two-camera configuration and `Launch Camera 1 (Watchdog)` for a one-camera configuration. It retains the read-only headless supervisor, exact three total complete-set attempts, exact three-second rests, mandatory exact serials, and mandatory color/depth health. The earlier prohibition on partial startup now applies specifically to this supervised workflow; the explicit row action is the sole permitted unsupervised single-camera diagnostic mode.
- Concurrency and ownership: only one camera launch mode may run at a time. While a direct camera terminal is alive, scanning, saving, the other row launch, and the supervised launch are disabled. The operator terminal is intentionally not terminated when the GUI closes; the GUI stop action continues to target only its owned headless supervisor process group.
- Configuration/argument boundary: project launcher files continue to expose no launch overrides. Camera name, serial, preset, required color/depth profile, registration/alignment, frame sync, temporal filter, point cloud, and USB-only enumeration become official vendor launch arguments. Camera count determines supervised `device_num`, while scan/startup/health/check/shutdown timing and the fixed attempt/rest values remain GUI/supervisor controls rather than vendor camera arguments.
- Device profile note: Orbbec's Gemini 335 specification advertises RGB up to 1920x1080 at 30 FPS and depth up to 1280x800 at 30 FPS. The project passes the explicit configured pair without silent clamping; actual simultaneous-mode support remains a strict driver/device/firmware check.
- Validation performed: Python compilation and `ament_flake8` passed; nine simulated package tests passed, including exact single-camera vendor/terminal arguments, terminal lifecycle, inactive-slot rejection, canonical configuration, bounded logging, and the unchanged three-attempt watchdog; installed GUI inspection confirmed both row launch labels, the dynamic complete-set watchdog label, and the watchdog-only stop control; and a plain root `colcon build` completed all 13 workspace packages. The failed initial test invocation only replaced rather than extended `PYTHONPATH`; it was corrected immediately and made no source or runtime change. No camera or robot process was launched.
- Offline transfer validation: this uses the vendored Orbbec launch and the Ubuntu 22.04 GNOME terminal already present on the target workstation; it adds no downloaded dependency, submodule, network service, configuration alias, or fallback workflow.
- Safety: implementation and tests do not launch a camera or robot; the new camera action remains an explicit operator click.

### 2026-09-03 — Five-millimetre ChArUco stability tolerance

- Superseding rule: the `camera_calibration` target stability gate tolerates at most 5 mm translation and 1 degree rotation across the fixed 0.5-second stability window. This replaces the earlier 2 mm translation limit; the rotation limit and stability duration are unchanged.
- Gate behavior: exactly 5 mm/1 degree remains accepted because movement blocks capture only when it exceeds either limit. Target pose age, ChArUco-corner count, actual robot TF freshness, and inter-sample robot-pose separation rules are unchanged.
- Reason: tolerate the observed millimetre-scale ChArUco pose noise while retaining a strict angular stability bound before sample capture.
- Validation performed: Python compilation and `ament_flake8` passed; the new gate test accepted exactly 5.00 mm, rejected 5.10 mm, and rejected 1.10 degrees; all six source tests and all seven reported package tests passed; and `camera_calibration` rebuilt successfully. No camera, robot, GUI, or motion command was launched.

### 2026-09-03 — One-second ChArUco stability window

- Superseding rule: `camera_calibration` must observe the ChArUco target across exactly one second before its stability gate may become ready. The accepted motion remains at most 5 mm translation and 1 degree rotation.
- Change: increased `TARGET_STABILITY_WINDOW_SEC` from 0.5 to 1.0 seconds. Target pose freshness remains a separate 0.5-second requirement, so current detections must continue arriving while the longer history window fills.
- Gate behavior: a half-second history remains blocked with the existing hold-still message; a complete one-second history is evaluated against the unchanged 5 mm/1 degree limits.
- Validation performed: the non-hardware gate test rejects a half-second history, accepts a complete one-second history at exactly 5.00 mm, rejects 5.10 mm, and rejects 1.10 degrees; Python compilation and `ament_flake8` passed; all six source tests and all seven reported package tests passed; and `camera_calibration` rebuilt successfully. No camera, robot, GUI, or motion command was launched.

### 2026-09-03 — Persistent last-session state for project GUIs

- Superseding camera-calibration rule: the calibration mode and prefix are empty only when `logs/camera_calibration/last_session.json` does not yet exist. Every successful explicit Apply atomically replaces that file with schema version 1, a canonical UTC timestamp, the validated mode and prefix, dictionary, board dimensions, checker/marker sizes, minimum corners, and minimum samples. A later process restores these values as an unapplied prefill and tells the operator to select Apply Settings.
- Safety boundary: startup never auto-applies restored settings, creates camera subscriptions from them, starts another package, commands hardware, or restores captured samples or a computed transform. The operator must explicitly Apply after every restart, and normal Apply continues to clear stale sample/solution state.
- Strict state contract: the package-owned state file is ignored runtime data beside `events.jsonl`, not project-wide configuration and not an alternate `.env` workflow. Missing means the explicit first-run blank form. If a present file has malformed JSON, unknown/missing keys, a noncanonical schema/timestamp/type, or invalid field values, startup hard-fails and records `ui_state_load_failed`; no default, migration, alias, or compatibility fallback is allowed.
- Project GUI rule: every new or reworked GUI with reusable operator setup fields must use one authoritative ignored persistence workflow. Project-wide values remain exclusively in root `.env`, and named artifacts such as motion-debug scripts remain authoritative for their own content. Other reusable package-owned form state uses atomic `logs/<package>/last_session.json`. Restoration is prefill-only; transient status and one-shot command targets are not persisted.
- Existing UI audit: `orbbec_camera_launcher` already restores its reusable setup from canonical root `.env`; `motion_debug` persists named reusable motion programs in `config/debug_script/*.json` while live robot commands remain intentionally transient; `gripper_control` has no reusable setup form. No duplicate last-session store is added to those packages.
- Validation performed: Python compilation and `ament_flake8` passed; eight source tests and all nine reported package tests passed, covering missing first-run state, strict round-trip, atomic replacement, malformed-key rejection, exact GUI field restoration, and confirmation that restoration does not configure the node or enable Capture; `camera_calibration` rebuilt successfully; and an installed-package GUI inspection restored the expected fields while remaining unapplied. The installed inspection emitted desktop inotify/Wayland environment warnings but completed successfully. No camera, robot, subscription, or motion command was launched.

### 2026-09-03 — Automatic three-sample camera calibration preview

- Superseding solve rule: `camera_calibration` has one fixed minimum of exactly three accepted samples. Samples one and two only populate the calibration set; sample three automatically calculates the first solution, and every later accepted sample automatically recalculates from the complete set. The manual Compute action and editable minimum-sample field are removed. Undo recalculates whenever at least three samples remain; dropping below three or resetting removes the current solution. Save YAML is enabled only while a valid current solution exists.
- TF boundary: ChArUco detection and its board axes remain video-only. The package never publishes `charuco_board` as ROS TF because no calibrated reference chain exists before the solve. After a successful solve it broadcasts only the calculated camera mounting transform: `base_link -> <prefix>_link` for camera-to-hand or `Link6 -> <prefix>_link` for camera-on-hand. The preview continues at 0.5-second intervals while the node and solution remain alive so the separately launched RViz viewer can display it.
- Video overlay: after a successful solve, every camera frame includes a clearly labelled calculated-camera block containing the exact reference/source frame pair, XYZ translation in metres, RPY rotation in degrees, total sample count, and translation/rotation RMS. ChArUco corner and board-axis drawing continues independently as detection feedback.
- Strict last-session state: the state schema is now exactly version 2 and records `minimum_samples: 3`. Version 1 or any other minimum hard-fails without migration, fallback, or inferred values. This workstation's ignored last-session record was explicitly updated from schema 1/minimum 5 to schema 2/minimum 3 as part of the requested change.
- Validation performed: Python compilation and `ament_flake8` passed; 11 source tests and all 12 reported package test results passed, covering exact three-sample solves in both modes, automatic calculation at sample three and recalculation at sample four, fixed schema-2 state, video-overlay contents, computed-camera TF frames, and removal of the ChArUco TF publisher. The package and all 13 workspace packages completed a canonical plain `colcon build`. An installed offscreen GUI inspection confirmed the fixed three-sample label, absence of a Compute action, Save disabled without a solution and enabled with one, calculated-pose text, and successful strict loading of this workstation's schema-2 session. The inspection emitted only the existing Wayland/inotify desktop warnings. No camera, robot, RViz, visible GUI, or motion command was launched.
- Offline transfer validation: all behavior, tests, and documentation are ordinary tracked source and require no network service, new dependency, submodule, configuration alias, or alternate workflow. Runtime samples and solutions remain intentionally unrestored.

### 2026-09-03 — Two-degree ChArUco stability tolerance

- Superseding rule: the `camera_calibration` target stability gate accepts at most 5 mm translation and 2 degrees rotation across the fixed one-second window. This replaces the prior 1-degree rotation limit; exactly 2 degrees remains accepted because capture is blocked only when either limit is exceeded.
- Unchanged boundaries: target pose freshness, ChArUco-corner count, actual robot TF freshness, inter-sample robot-pose separation, automatic three-sample solving, video overlay, calculated-camera TF publication, and YAML behavior are unchanged.
- Validation performed: Python compilation and `ament_flake8` passed; all 11 source tests and all 12 reported package test results passed. The stability test accepts exactly 2.00 degrees, rejects 2.10 degrees, retains the exact 5.00 mm acceptance boundary, and rejects 5.10 mm. A canonical plain `colcon build --packages-select camera_calibration` completed successfully. No camera, robot, RViz, GUI, or motion command was launched.
- Offline transfer validation: the tolerance is ordinary tracked source and documentation with no new dependency, configuration path, network service, or fallback.

### 2026-09-03 — Enlarged calculated-TF video overlay

- Change: increased the `camera_calibration` calculated-TF overlay from its former 0.42–0.62 one-pixel font to a resolution-aware 0.8–1.2 scale with a two-pixel stroke. At the configured 848-pixel color width the scale is approximately 1.0, while the existing five-line content, high-contrast panel, colors, and top-left placement remain unchanged.
- Reason: make the calculated camera frame, XYZ/RPY, sample count, and RMS readable in the live calibration view.
- Validation performed: Python compilation and `ament_flake8` passed; all 11 source tests and all 12 reported package test results passed. The overlay test confirms exact 0.8/approximately 1.0/1.2 scaling at 640/848/1920-pixel widths, a two-pixel stroke, unchanged pose text, and rendered output. A canonical plain `colcon build --packages-select camera_calibration` completed successfully. No camera, robot, RViz, GUI, or motion command was launched.
- Offline transfer validation: this is ordinary tracked UI source and documentation with no new dependency, configuration path, network service, or fallback.

### 2026-09-03 — Calibration consistency diagnostics and holdout validation

- Superseding quality rule: `camera_calibration` keeps the exact one-second, 5 mm/2 degree stability gate, stores its newest accepted ChArUco pose without averaging, and uses the latest fresh robot TF. Calibration and validation samples receive stable, non-reused `C#` and `V#` IDs within one applied session and must each satisfy the existing 20 mm-or-5-degree robot-pose separation against both sample sets.
- Diagnostics: each successful solve reports calibration fit RMS and maxima, change from the immediately previous solution, maximum pairwise robot-pose coverage, and per-sample values in neutral physical units. From four calibration samples onward, leave-one-out recomputes the camera TF with each sample omitted. An omitted-sample solve failure keeps the full-set overlay and RViz preview active, identifies the failing ID, records an error, and disables YAML saving; leave-one-out is explicitly unavailable and non-blocking at exactly three samples.
- Validation boundary: holdout samples are captured only after a valid solve, never enter the calibration solver, and are rescored whenever the calibration set changes. Zero validation samples is explicitly `not_performed`, one or two is `fewer_than_3_samples`, and three or more is `complete`; validation is informative and never required for saving.
- Operator control: the quality summary and read-only sample table appear below the video. Maximum-metric rows are labelled without subjective quality grades. No point is deleted automatically; the operator must confirm one selected-row removal. Undo affects only the newest calibration point, reset clears both sets and their ID counters, and dropping below three calibration samples clears the solution while retaining holdout captures for later rescoring.
- Output contract: calibration artifacts now use exactly schema version 2 and always record calibration fit, previous-solution change, leave-one-out, pose coverage, explicit validation state, and per-sample diagnostics. Schema-1 output, compatibility aliases, and alternate writers are not retained. Last-session persistence remains form-prefill-only and never stores samples, diagnostics, or solutions.
- Validation performed: Python compilation and `ament_flake8` passed; all 18 direct source tests and all 19 reported package test results passed, covering both calibration modes, fit maxima, previous-solution change, robot-pose coverage, leave-one-out results and save blocking, validation isolation, cross-set pose diversity, manual removal, computed-TF preview state, and schema-2 YAML with and without validation. A strict YAML parser also accepted the generated no-validation artifact and its explicit empty validation diagnostics. `camera_calibration` rebuilt successfully and a canonical plain root `colcon build` completed all 13 workspace packages. The offscreen Qt unit test created no visible window. No camera, robot, RViz, hardware-facing GUI, or motion command was launched.
- Offline transfer validation: all implementation, tests, and documentation are ordinary tracked source; the change adds no dependency, submodule, configuration path, network service, or fallback workflow.

### 2026-09-03 — Loadable schema-3 calibration sessions

- Superseding operator rule: the optional holdout-validation capture set and its `V#` IDs are removed. The former validation action is now exactly **Load Calibration**. Calibration uses only stable, non-reused `C#` IDs, and its fit, solution-change, pose-coverage, and leave-one-out diagnostics remain neutral consistency measurements.
- Load contract: every calibration artifact uses exactly schema version 3 and stores each accepted sample's raw `base_link <- Link6` and camera-optical-from-ChArUco transforms. **Load Calibration** is an explicit action that accepts only schema 3, validates every canonical section and rigid transform, confirms before replacing a non-empty session, applies the artifact's exact mode/camera/board settings, restores its original IDs, advances the next ID beyond the largest restored ID, and recomputes the result from the restored observations using the required live camera-link-to-optical TF. Schema 1 and 2 files, conversion, aliases, inferred fields, and compatibility fallbacks are rejected.
- Persistence boundary: startup `last_session.json` restoration remains unapplied form prefill only and never restores samples or a solution. A deliberately loaded artifact is the sole sample-restoration path and saves its validated form fields back to the same authoritative UI-state file. The stored final transform is informational output and is never substituted for recalculation.
- Save feedback: **Save YAML** shows an explicit success dialog with the exact created path, or an explicit failure dialog. Timestamped artifacts remain create-only and are never overwritten.
- Dependency: YAML parsing and emission use the declared Ubuntu/ROS system dependency `python3-yaml`; no downloaded runtime component or online service is introduced.
- Validation performed: Python compilation and `ament_flake8` passed; all 19 direct source tests and all 20 reported package test results passed. The tests cover strict schema-3 raw-observation round-trip, schema-2 and malformed-transform rejection, stable ID restoration and continuation, exact **Load Calibration** UI naming, post-save confirmation, both calibration modes, leave-one-out gating, manual sample removal, and calculated-TF preview behavior. `camera_calibration` rebuilt successfully and a canonical plain root `colcon build` completed all 13 workspace packages. No camera, robot, RViz, visible GUI, or motion command was launched.
- Offline transfer validation: the calibration data and loader are ordinary YAML and tracked package source. The only added runtime dependency is the declared Ubuntu/ROS `python3-yaml` system package; no network service, submodule, alternate configuration, migration utility, or fallback workflow was added.

### 2026-09-03 — Serial OpenCV ArUco crash containment

- Failure evidence: three `camera_calibration` launches on 2026-09-03 ended with `SIGSEGV`/exit `-11`. Kernel records at 15:46:48, 15:47:38, and 16:01:16 identify the same instruction offset inside `libopencv_aruco.so.4.5.4`; symbol resolution places it in `cv::aruco::detectMarkers` while candidate vectors are joined. The package log proves startup, UI-state loading, Apply, camera subscription, live ChArUco detection, and sample capture succeeded before the faults. A matching ArUco-library segfault was also recorded on 2026-09-02, before schema-3 loading was introduced. The Qt Wayland warning, YAML logic, hand-eye solve, and robot TF were therefore excluded as causes.
- Source finding: Ubuntu's exact OpenCV `4.5.4` build reports TBB as its parallel framework and defaulted to 16 OpenCV threads. The corresponding upstream ArUco source executes both adaptive-threshold scales and candidate identification through `parallel_for_`; the repeated native fault occurred within that detector path and cannot be caught as a Python exception.
- Superseding runtime rule: `camera_calibration` now requires OpenCV exactly `4.5.4` and, before creating subscriptions, configures exactly one OpenCV worker thread and disables OpenCL. It uses only the stable `Dictionary_get` and `DetectorParameters_create` API pair, shares one dictionary instance between detection and the ChArUco board, and copies every detector input into an owned contiguous `uint8` grayscale array. The wrapper validates runtime state, input, IDs, and corner arrays on every call. Any version or runtime drift hard-fails explicitly; the 16-thread TBB path, parallel detection, alternate APIs, automatic retries, and detector fallbacks are forbidden.
- Validation performed: Python compilation and `ament_flake8` passed; all 21 direct source tests and all 22 reported package test results passed. A separate 20,000-frame native ArUco stress run used the exact 3x3 `DICT_4X4_50`, 28 mm/21 mm board profile with shifted and blurred 640x480 frames; all 20,000 frames returned all four markers under `opencv_version=4.5.4`, `opencv_thread_count=1`, and `opencv_opencl_enabled=false` without a fault. `camera_calibration` rebuilt successfully and a canonical plain root `colcon build` completed all 13 workspace packages. No camera, robot, RViz, visible GUI, or motion command was launched by the agent.
- Offline transfer validation: the fix uses the already-declared Ubuntu `python3-opencv` dependency at its strictly checked version and adds no downloaded library, network service, submodule, compatibility API, alternate detector, retry, or fallback workflow.

### 2026-09-03 — Executor-thread OpenCL initialization

- Failure evidence: the camera image and CameraInfo topics were live, but the first image callback stopped with `RuntimeError: OpenCV OpenCL was enabled after initialization`. A local two-thread probe on the required OpenCV `4.5.4` runtime proved that `cv2.ocl.setUseOpenCL(False)` in the Qt/main thread leaves a newly created ROS executor thread at its independent default of `useOpenCL() == True`.
- Superseding runtime rule: retain the exact OpenCV `4.5.4`, one-worker-thread, OpenCL-disabled contract. Configure and verify it first in the main thread before camera subscriptions are created, and independently in the single ROS executor thread before that thread calls `executor.spin()` and can process any queued camera callback. Executor-thread initialization has an exact five-second completion bound and hard-fails startup on timeout or rejection. The existing per-frame runtime-drift guard remains mandatory.
- Reason: OpenCV 4.5.4 stores OpenCL enablement per thread, so main-thread initialization alone cannot establish the required runtime in the thread that executes ChArUco detection.
- Validation performed: a direct OpenCV 4.5.4 two-thread probe reproduced `main=False` and `new_worker=True` before worker initialization, then confirmed `new_worker=False` after explicit worker initialization. Python compilation passed; all 22 direct tests and all 22 reported package tests passed, including a regression that verifies the executor thread reaches one OpenCV worker and OpenCL disabled before spin. `camera_calibration` rebuilt successfully, and a canonical plain root `colcon build` completed all 13 workspace packages. No camera, robot, RViz, visible GUI, or hardware command was launched by the agent.
- Offline transfer validation: the correction uses existing Python threading and OpenCV APIs only; it adds no dependency, network service, configuration path, retry, detector variant, or fallback workflow.

### 2026-09-06 — Hardware-only CR10 profile without simulation or servo streaming

- Superseding rule: the Dobot vendor profile targets only the physical CR10. Robot simulation is permanently out of scope: do not restore `dobot_gazebo`, Gazebo worlds or launch/configuration files, Gazebo URDF/Xacro tags, simulator dependencies, or any alternate simulated-robot workflow. Servo streaming is also permanently out of scope: do not restore `servo_action` or the Dobot `ServoJ`/`ServoP` message, parser, service-registration, or handler paths.
- Change: removed the complete remaining `dobot_gazebo` and `dobot_demo` packages, the CR Gazebo world, and the seven Gazebo-only link blocks from `cra_description/urdf/cr10_robot.xacro`; removed `ServoJ.srv` and `ServoP.srv` from `dobot_msgs_v4` and its interface generator; and removed both command parsers, includes, declarations, ROS service registrations, and handlers from `dobot_bringup_v4`. Updated current package inventories and hardware-only documentation. The workspace now contains exactly 11 packages: four retained Dobot packages, three retained Orbbec packages, and four project packages.
- Retained motion boundary: point-to-point and operator-jog interfaces such as `MovJ`, `MovL`, `MoveJog`, and `StopMoveJog` remain because they are used by the physical-robot workflow and are not the removed high-rate `ServoJ`/`ServoP` streaming API. All real motion remains subject to explicit operator authorization and hardware safety checks.
- Vendor patch: this deliberately prunes simulation assets and two upstream service interfaces from the Dobot snapshot based at `def21d05149576b9aed261f38e62ea236a5d8ec5`; the upstream MIT license and attribution remain intact.
- Validation performed: the CR10 Xacro passed `xmllint`; an active-source audit found no Gazebo, `gazebo_ros`, `servo_action`, `ServoJ`, or `ServoP` implementation reference; all 11 retained packages have package-local READMEs; and a clean isolated build from ROS 2 Humble completed all 11 packages. The isolated install exposes exactly the four retained Dobot packages and no removed servo interface. A broad isolated `colcon test` was also attempted but is not a green project gate: the Dobot vendor lint configuration reports its existing repository-wide CRLF, CMake/flake8 style, and bundled `nlohmann`/uncrustify failures outside this cleanup's deleted paths. No robot or camera process was launched or commanded by the agent. The active workspace overlay was not replaced because the operator's real bringup, motion-debug, and RViz processes were running from it; a clean active-overlay rebuild is required after those processes are stopped.
- Offline transfer validation: removal reduces the vendored source and system dependency surface. A transferred repository requires no Gazebo or simulation installation and contains no servo-streaming interface fallback.

### 2026-09-06 — Isolated single-owner OpenCV calibration worker

- Failure evidence: the calibration process started at 10:21:45, processed live camera data for approximately 13 minutes, captured C9 and C10, and died at 10:34:58 with exit `-11`. The kernel identified task 32644 faulting inside `libopencv_aruco.so.4.5.4d`; the fault address resolves to `cv::aruco::detectMarkers`. The last package events show detection changing from four ChArUco corners to two and then zero immediately before the native fault. The earlier Qt/IBus inotify warning was emitted at startup after VS Code exhausted the user's watch quota, but it was not the crash instruction or a disk-capacity failure.
- Superseding runtime rule: keep system OpenCV exactly `4.5.4`, but never run its calibration operations in the ROS/Qt parent process. `camera_calibration` creates exactly one child process using multiprocessing `spawn`; that process is the sole runtime owner of dictionary/board/parameter construction, marker detection, ChArUco interpolation, pose estimation, overlays, Rodrigues conversion, and hand-eye/diagnostic solves. Every request is serialized. The former in-process main-thread and executor-thread OpenCV initialization workflow is removed.
- Image contract: the ROS parent accepts only exact tightly packed `rgb8`, little-endian color messages with `step == width * 3` and an exact `height * step` byte count. It makes one owned contiguous RGB copy, the child makes the required owned contiguous grayscale detector input, and the Qt GUI renders the returned owned RGB overlay directly without an OpenCV conversion. Encoding, layout, or buffer drift is a fatal input-contract error; no conversion alias or encoding fallback is permitted.
- Failure contract: child startup and every operation have an exact five-second response bound. A native child exit, timeout, closed pipe, protocol mismatch, detector/runtime drift, or malformed result clears the current solution, stops TF rebroadcasting, records the operation, child PID/exit code/signal, last frame metadata, and `retry_attempted=false` in `logs/camera_calibration/events.jsonl`, then terminates the GUI. There is one worker per application lifetime, zero automatic restarts, and no alternate OpenCV version, detector API, implementation, or fallback.
- Dependency boundary: direct `cv_bridge` image conversion is removed from `camera_calibration`; raw ROS `rgb8` validation and copying use NumPy already required by the package. The child process uses only Python standard multiprocessing plus the already-required exact system OpenCV and NumPy installations, so no online runtime or downloaded component is introduced.
- Validation performed: Python compilation and `ament_flake8` passed; all 29 direct package tests and all 30 reported package results passed. Tests cover spawned runtime initialization, graceful shutdown, a forced child death with no replacement PID, fatal solution/TF clearing and `SIGSEGV` event fields, exact RGB ownership, 30 alternating full/partial/empty ChArUco frames, a complete hand-eye/diagnostic solve in the child, and the existing calibration contracts. An isolated ROS 2 Humble root build completed all 11 currently discoverable packages, and its installed package exposes the worker module, canonical executable, and no launch arguments. The already-running user robot, camera, RViz, motion-debug, and older calibration processes were not stopped, relaunched, or commanded.
- Offline transfer validation: the worker module, strict request protocol, tests, and documentation are ordinary tracked package source and use no submodule, online service, compatibility path, or external supervisor.

### 2026-09-06 — Schema-4 calibration samples retain robot joint positions

- Superseding artifact rule: every newly captured calibration sample records the exact canonical `/joint_states` positions for ordered `joint1` through `joint6` in radians alongside its existing `base_link <- Link6` and camera-optical-from-ChArUco transforms. Calibration artifacts now use exactly schema version 4. `Load Calibration` accepts only schema 4 and rejects schema 1, 2, and 3 without conversion, inferred values, aliases, or fallback.
- Capture contract: sample capture requires a `/joint_states` message no older than one second, with exact ordered canonical names, exactly six finite positions, and a non-zero ROS timestamp. Missing, stale, reordered, incomplete, non-finite, or zero-stamped feedback rejects the capture and records a package event; it is never silently omitted from a sample.
- Motion boundary: saved joint positions exist only to support a future separately designed automatic-calibration motion workflow. `camera_calibration` remains read-only with respect to the robot and never publishes, replays, or commands a saved joint pose. Any future replay requires explicit user scope, safety behavior, and a new diary rule before implementation.
- Output/load behavior: schema-4 YAML declares `/joint_states`, the exact joint order, and radians as the absolute unit, and stores a keyed `joint_positions_rad` mapping in every `captured_samples` entry. Loading preserves these values with the sample IDs while the current solver continues using the stored Cartesian transforms.
- Validation performed: Python compilation and `ament_flake8` passed; the 29 direct tests and 30 reported package results include canonical joint-message acceptance/rejection, captured joint-value retention, strict schema-4 round-trip, required per-sample joint data, and explicit schema-3 rejection. The isolated ROS 2 Humble root build completed all 11 currently discoverable packages. The existing live `/joint_states` topic was read once without commanding hardware and confirmed a non-zero timestamp, ordered `joint1` through `joint6`, and six finite positions; the existing `/bin_camera/color/image_raw` encoding was likewise read once and confirmed exact `rgb8` for the new image contract.
- Offline transfer validation: schema validation, joint data, tests, and documentation are ordinary tracked source/YAML and add no dependency, network service, submodule, or alternate data path.

### 2026-09-06 — Package-private OpenCV 4.10 ChArUco runtime and four-sample Tsai solve

- Superseding runtime rule: `camera_calibration` no longer uses Ubuntu's system OpenCV 4.5.4. The exact `opencv_python-4.10.0.84-cp37-abi3-manylinux_2_17_x86_64.manylinux2014_x86_64.whl` artifact is vendored at `third_party/wheels/` with SHA-256 `9ace140fc6d647fbe1c692bcb2abce768973491222c067c131d80957c595b71f` and its upstream license material preserved. `colcon build` must verify this digest and extract the wheel into the `camera_calibration` install prefix without internet access or a global Python installation. Other ROS packages retain their system OpenCV environment.
- Process and API boundary: the ROS/Qt parent never imports `cv2`. Exactly one spawned worker loads OpenCV exactly `4.10.0` from the package-private runtime and owns all detection, drawing, pose, and hand-eye operations for its complete lifetime. It uses exactly `getPredefinedDictionary`, `DetectorParameters`, `CharucoBoard`, `setLegacyPattern(True)`, `CharucoDetector.detectBoard`, `CharucoBoard.matchImagePoints`, and `solvePnP(..., SOLVEPNP_ITERATIVE)`, with one OpenCV thread and OpenCL disabled. The previous 4.5.4 legacy ArUco calls and runtime are not compatibility paths.
- Failure contract: a missing or altered wheel, module-path escape, version/API mismatch, thread or OpenCL drift, malformed detector result, timeout, protocol failure, or worker exit is terminal. The solution and calibrated TF are cleared, the package records the worker PID, exit code/signal, and frame metadata, and the GUI terminates with code 1. The worker is never retried, recycled, or replaced, and system OpenCV is never selected as a fallback. The package event logger is named separately from `rclpy.Node`'s internal logger so fatal failures reach both ROS logging and `logs/camera_calibration/events.jsonl` without a secondary logger exception.
- Superseding solve rule: retain the one fixed OpenCV `CALIB_HAND_EYE_TSAI` method, but require exactly four accepted calibration samples before the first automatic solve because OpenCV 4.10 rejects the three-pose Tsai problem as insufficiently informative. Sample four and every later accepted sample automatically recompute from the complete set. Leave-one-out is unavailable and non-blocking at exactly four samples and becomes mandatory from five samples onward, ensuring every omitted subset still contains the required four poses. No hand-eye-method chain or fallback is allowed.
- State and artifact boundary: last-session form state is now exactly schema version 3 and records `minimum_samples: 4`; older state hard-fails without migration or defaults. This workstation's ignored state was explicitly advanced from schema 2/minimum 3 to schema 3/minimum 4 with the implementation. Calibration artifact YAML remains strict schema version 4 as requested; loading a schema-4 artifact with fewer than four raw samples fails the current solver contract rather than selecting a compatibility method.
- Frame-retention boundary: raw RGB frames, grayscale detector copies, and rendered overlays are transient memory only. The GUI retains only the latest overlay, and neither an accepted sample, `last_session.json`, package events, nor schema-4 YAML stores an image or video path. Calibration duration therefore does not create an accumulating frame archive; persistent calibration data is limited to numerical observations, joint positions, settings, diagnostics, and explicit YAML saves.
- Validation performed: Python compilation and `ament_flake8` passed; all 38 direct package tests and all 39 reported test results passed. They cover both calibration modes with the fixed Tsai method, four-sample automatic solving, five-sample leave-one-out, exact runtime/module/checksum/API/thread/OpenCL/legacy-board checks, legacy-layout board detection and pose, client-only parent imports, malformed runtime states/results, invalid-pose terminal classification, response timeout, and terminal forced `SIGSEGV` behavior. Missing-wheel and altered-wheel configure probes both hard-failed before extraction. A final one-worker stress run processed all 10,000 mixed 1920x1080 full, partial, blank, blurred, shifted, and edge-clipped frames in 268.1 seconds without worker exit, retry, malformed response, runtime drift, or continuing RSS growth; the observed worker range was 113,328–119,408 KiB and the loaded module was the extracted OpenCV 4.10.0 runtime. A clean root build with inherited workspace paths removed completed all 11 packages, and `camera_calibration` alone was then rebuilt successfully into the active install while the calibration GUI was stopped. `git diff --check` passed. No camera, Dobot bringup, RViz, visible GUI, or hardware command was launched by these checks; existing operator-started hardware-facing processes were left running and untouched.
- Offline transfer boundary: the wheel, checksum manifest, attribution, extractor, worker code, and tests are tracked repository content. The target remains Ubuntu 22.04 x86-64 with Python 3.10 and ROS 2 Humble; no online dependency resolver or alternate platform/runtime path is provided.

### 2026-09-06 — Registered-depth-constrained ChArUco calibration

- Superseding input rule: `camera_calibration` now requires `/<prefix>/depth/image_raw` together with RGB and CameraInfo. The depth image must be the Orbbec color-registered output: exact tightly packed little-endian `16UC1` millimetres, exact RGB dimensions, exact `<prefix>_color_optical_frame`, and no more than 50 ms from the RGB timestamp. Camera-launcher depth registration, color alignment, and frame synchronization must already be enabled; calibration never changes camera configuration and has no RGB-only, unaligned-depth, unit-conversion, topic-alias, or synchronization fallback.
- Fusion rule: modern RGB ChArUco PnP remains authoritative for board origin X/Y and its in-plane X direction. The worker projects the complete board boundary, erodes it by five pixels, rejects zero/out-of-range depth and local discontinuities above 20 mm, samples every two pixels, rejects points more than 50 mm from the median, and deterministically refits a plane three times with a 5 mm inlier threshold. The depth normal refines board tilt while the plane intersection at the RGB origin's camera-X/Y coordinates refines translation only along camera Z.
- Capture gate: a fused pose is accepted only with at least 100 plane inliers, depth-plane RMS at most 3 mm, RGB/depth origin-Z disagreement at most 15 mm, and RGB/depth normal disagreement at most 5 degrees. Missing depth or a measurement-quality disagreement blocks capture visibly; malformed encoding/layout/frame/resolution is a terminal input-contract failure. The existing one-second 5 mm/2 degree stability gate then evaluates the fused pose and continues to store only its newest accepted value.
- Visualization/output: the GUI has side-by-side annotated RGB and colorized registered-depth views. Both show the projected board region, exact depth metrics, fused axes, and calculated camera solution; the depth view additionally marks plane inliers. Artifacts now use strict schema version 5 and record the original RGB pose, accepted fused pose, depth-plane diagnostics, exact fusion constants, robot transform, and joint positions for every `C#` sample. Schema 4 and every older version are rejected without conversion. Raw RGB/depth frames and overlays remain transient and are never persisted.
- Runtime boundary: the same exact package-private OpenCV 4.10.0 worker remains the sole OpenCV owner. Its API/runtime check now also covers the fixed registered-depth plane operations, and runtime drift during either detection or solve is terminal rather than a recoverable solve error. No Orbbec vendor source was changed.
- Validation performed: Python compilation and flake8 passed; all 41 direct source tests and all 42 reported package test results passed. Tests cover exact RGB/depth ROS image contracts, schema-5 round-trip and schema-4 rejection, the split GUI views and depth columns, both calibration modes, terminal runtime drift during solve, and synthetic registered-depth planes with known tilt, missing pixels, and explicit RGB/depth disagreement. A clean isolated ROS 2 Humble root build completed all 11 packages. The existing one-worker stress test was upgraded to send registered depth and processed all 10,000 mixed 1920x1080 RGB/depth pairs in 924.3 seconds with one unchanged PID, no exit/retry/timeout/protocol failure, and bounded RSS of 139,496–157,316 KiB; full, blurred, and shifted boards were ready while partial, blank, and edge-clipped boards were blocked as designed. `git diff --check` passed. No camera, Dobot bringup, RViz, visible GUI, or hardware command was launched by these checks; existing operator-started processes remained running and untouched.
- Offline transfer validation: the implementation uses the already-vendored OpenCV wheel, NumPy, ROS sensor messages, and existing Orbbec registered-depth output; it adds no package download, online service, submodule, camera launch argument, or alternate runtime path.

### 2026-09-06 — Bounded board-ROI depth processing

- Superseding depth-range rule: registered-depth plane fitting accepts only the inclusive 200 through 1,000 mm interval. Every zero, sub-200 mm, and above-1,000 mm value is rejected before neighborhood, median, or plane calculations; no configurable range or fallback is provided.
- Processing rule: after RGB ChArUco PnP projects the full physical board boundary, the worker clips one bounding box to the image and performs depth conversion, visualization, board masking, discontinuity rejection, sampling, and plane fitting only inside that ROI. The worker no longer creates another owned full-frame depth copy after the process boundary. Pixels outside the ROI stay black in the depth view and never enter depth-plane calculations.
- Artifact rule: the depth algorithm identifier is exactly `CharucoDetector.detectBoard+registered_depth_board_roi_v2`; calibration YAML advances to strict schema version 6 and records `processing_scope: projected_board_bounding_box_only`, `minimum_depth_m: 0.2`, and `maximum_depth_m: 1.0`. `Load Calibration` rejects schema 5 and every older artifact without conversion or fallback.
- Validation performed: Python compilation and flake8 passed; all 42 direct calibration tests and all 43 reported package results passed. The new tests prove the inclusive 200/1,000 mm boundaries, reject 199/1,001 mm in visualization, verify that the colorizer receives an image region smaller than the source frame, confirm pixels outside the projected ROI remain black, exercise RGB/depth pose fusion at an in-range distance, and verify strict schema-6 output plus schema-5 rejection. A canonical root `colcon build` completed all 11 packages. `git diff --check` passed. No camera, Dobot bringup, RViz, GUI, or hardware command was launched by these checks.
- Offline transfer validation: the range and ROI implementation uses only the already-vendored OpenCV/NumPy runtime and introduces no dependency, network access, alternate input, or compatibility path.

### 2026-09-06 — Full-range raw depth visualization with ROI-only calibration math

- Superseding visualization rule: the depth panel displays the complete registered-depth frame, colorized only across the inclusive 200 through 1,000 mm interval. Missing, sub-200 mm, and above-1,000 mm display pixels are black. This replaces the immediately preceding cropped-display rule.
- Calibration boundary: only visualization is full-frame. Depth conversion and filtering for plane fitting remain restricted to the clipped RGB-projected ChArUco board bounding box and eroded board mask; visible background depth never enters calibration math.
- Artifact boundary: calibration data and its strict schema version 6 are unchanged because the numerical depth algorithm, range, processing scope, and saved observations are unchanged. No raw depth image is persisted.
- Validation performed: Python compilation and flake8 passed; all 42 direct calibration tests and all 43 reported package results passed. The range-boundary test still proves 200/1,000 mm inclusion and 199/1,001 mm rejection, while the live-detection regression now proves the colorizer receives the complete raw frame and a valid background depth pixel outside the projected board is visible. The numerical fusion tests continue to exercise ROI-only plane fitting. `camera_calibration` rebuilt successfully into the active install. `git diff --check` passed. No camera, Dobot bringup, RViz, GUI, or hardware command was launched by these checks.
- Offline transfer validation: this display-only change uses the existing package-private OpenCV/NumPy worker and introduces no dependency, network access, alternate input, or compatibility path.

### 2026-09-06 — Invalid camera messages are skipped and duplicate camera launches were stopped

- Superseding frame-failure rule: one color, registered-depth, or CameraInfo message—or one RGB/depth pair—with an invalid frame ID, encoding, layout, timestamp, intrinsics, or dimensions is a logged `WARNING` frame rejection. It clears current target/capture readiness and removes invalid cached depth or CameraInfo where applicable, but it does not clear an existing solved calibration, stop its TF, restart the OpenCV worker, resize/convert data, or terminate the GUI. Only a later independently valid frame may restore readiness. This is an explicit exception to the earlier terminal ROS image-contract rule.
- Terminal boundary: package configuration/state failures and every OpenCV worker runtime, checksum, API, detector-output, process, protocol, or timeout failure remain terminal with no retry or fallback. An invalid ROS message is never sent to the worker, and repeated invalid frames keep capture blocked.
- Runtime cleanup: diagnosis found two operator-terminal Orbbec launches for `bin_camera` and serial `CP9JA530007E`, started at 10:27 and 15:03, with duplicate color, depth, and CameraInfo publishers. At the user's explicit request, the exact camera launch process groups `37524` and `309567`, including component containers `37543` and `309582`, were sent SIGINT and confirmed exited. The camera-launcher GUI was not terminated and no robot process or command was touched.
- Failure evidence: the calibration session started its verified OpenCV 4.10.0 worker at 11:32:45 UTC and processed valid 1920×1080 registered depth until one native 848×480 frame appeared against 1920×1080 color at 11:35:24 UTC. The old implementation deliberately exited with code 1 on that mismatch. The IBus inotify warning was unrelated. Live inspection after the failure found two publishers and 270 consecutive normal 1920×1080 registered-depth frames before cleanup.
- Validation performed: Python compilation and flake8 passed; all 44 direct calibration tests and all 45 reported package results passed. New regressions verify that one mismatched 848×480 depth frame paired with 1920×1080-style color dimensions is skipped before worker detection, invalid cached depth and target readiness are cleared, no fatal state is set, and an existing solution/diagnostics remain intact. The worker-failure regression still proves native exit is terminal. `camera_calibration` rebuilt successfully into the active install, `git diff --check` passed, and a final process audit found no remaining Orbbec camera launch or component container. No camera was relaunched and no robot process or command was touched.
- Offline transfer validation: the skip boundary uses existing ROS callbacks and package logging only; it introduces no dependency, network service, image conversion, resize path, retry loop, or compatibility fallback.

### 2026-09-06 — Independent calibration TF callback execution

- Failure evidence: RViz remained smooth and an independent live tf2 probe received every `base_link -> Link1 -> ... -> Link6` transform with a common timestamp only 1–3 ms old, while `camera_calibration` repeatedly rejected its own composed `base_link <- Link6` result at approximately 10 seconds old. The calibration node used one `SingleThreadedExecutor`; each 1920×1080 RGB callback synchronously waited for the isolated detector worker, while tf2's default dynamic subscription retained 100 messages. At the measured 10 Hz robot TF rate, that queue represented the observed 10-second callback backlog. RViz was unaffected because it uses its own process and executor.
- Superseding execution rule: `camera_calibration` uses one `MultiThreadedExecutor` with exactly two threads. Camera, CameraInfo, joint-state, and timer callbacks stay serialized in the node's default mutually exclusive callback group. `TransformListener` retains its separate reentrant callback group, allowing the second executor thread to service `/tf` independently while image processing is active. Do not return this package to one-thread ROS execution or move OpenCV work into the ROS/Qt parent.
- TF and calibration boundary: the canonical external `robot_state_publisher` remains the only source of `base_link -> Link6`; the calibration node does not calculate CR10 forward kinematics and RViz is not a TF source. The one-second robot-TF freshness requirement, camera-internal TF lookup, sample contents, hand-eye equations, solver method, diagnostics, artifacts, and calibrated-TF broadcast are unchanged. No alternate TF source, stale-transform acceptance, retry, or fallback was added.
- Validation performed: Python compilation and `ament_flake8` passed; all 45 direct `camera_calibration` tests passed, and the installed package suite reported 46 results with zero errors, failures, or skips. The new executor regression blocks a simulated sensor callback, publishes a unique TF while that callback is still blocked, and proves the TF buffer receives it before the callback is released using the exact runtime executor constructor. `camera_calibration` rebuilt successfully, and a clean root `colcon build` completed all 11 packages. `git diff --check` passed. No camera, calibration GUI, RViz, Dobot bringup, or hardware command was launched by these validation steps; operator-started processes were left untouched.
- Offline transfer validation: this change uses only the existing ROS 2 Humble executor and tf2 callback-group APIs already required by the package. It adds no dependency, network service, submodule, generated artifact, compatibility path, or vendor modification.

### 2026-09-06 — 500 ms RGB/registered-depth synchronization limit

- Superseding synchronization rule: the maximum absolute timestamp difference between an RGB frame and its registered-depth frame is exactly 500 ms, replacing the earlier 50 ms limit. A pair at exactly 500 ms is accepted; a pair above 500 ms is logged as a `WARNING`, clears current capture readiness, and is skipped until a later independently valid pair arrives.
- Scope: the single authoritative limit drives parent-side pairing, worker protocol validation, GUI status and overlays, and strict schema-6 artifact writing/loading. There is no configurable tolerance, timestamp conversion, retry, stale-frame reuse, or synchronization fallback.
- Calibration boundary: dimensions, encodings, frame IDs, camera intrinsics, depth range and plane-quality gates, the one-second target-stability gate, and the robot-TF freshness requirement are unchanged.
- Validation performed: Python compilation passed; all 46 direct calibration tests passed and the installed package suite reported 47 results with zero errors, failures, or skips. The new regression proves that exactly 500 ms is accepted, 500.001 ms is rejected, and schema-6 YAML records `color_sync_max_ms: 500.0`. `camera_calibration` rebuilt successfully and a root `colcon build` completed all 11 packages. `git diff --check` passed. No camera, calibration GUI, RViz, Dobot bringup, or hardware command was launched.
- Offline transfer validation: this is a constant and documentation change using the existing ROS/OpenCV pipeline; it adds no dependency, network access, submodule, or alternate runtime path.

### 2026-09-06 — Two-second synchronization limit and frozen stale-depth preview

- Superseding synchronization rule: the maximum absolute RGB/registered-depth timestamp difference is exactly 2,000 ms, replacing the 500 ms limit. A pair at exactly 2,000 ms is accepted; a pair above it is logged as a `WARNING`, clears target/capture readiness, and is not sent to the worker.
- Visualization rule: after the worker has produced at least one valid synchronized depth visualization, a rejected or missing depth update preserves that last rendered depth image instead of replacing the pane with black waiting text. A top-left `DEPTH STALE` badge reports the increasing age of the frozen preview and refreshes every 100 ms. The badge also appears if no valid rendered pair arrives for the existing 0.5-second live-pose age limit.
- Safety boundary: the frozen visualization is display-only. Its pixels are never resent to the worker or reused for ChArUco detection, depth-plane fusion, stability history, readiness, or capture. A later independently valid synchronized pair replaces it and removes the badge; no retry, timestamp conversion, stale-data fallback, or alternate pairing path is added.
- Artifact boundary: strict schema-6 YAML now records `color_sync_max_ms: 2000.0`; its other settings, samples, diagnostics, and load behavior are unchanged.
- Validation performed: Python compilation passed; all 48 direct calibration
  tests passed and the installed package suite reported 49 results with zero
  errors, failures, or skips. The regressions prove the exact 2,000 ms boundary,
  schema-6 output, preservation of the last valid depth visualization, increasing
  stale age after a rejected update, and automatic staleness when valid rendering
  stops. An offscreen installed-package Qt check rendered the stale badge
  successfully, a root `colcon build` completed all 11 packages, and
  `git diff --check` passed. No camera, visible calibration GUI, RViz, Dobot
  bringup, or hardware command was launched.
- Offline transfer validation: this uses only existing in-memory NumPy previews and Qt painting; it adds no dependency, stored frame, network access, submodule, or alternate runtime.

### 2026-09-09 — RGB-only MoveIt ROS 2 calibration logic, five samples, schema 7

- Superseding pipeline rule: reproduce the relevant standalone calibration logic from official `moveit/moveit_calibration` ROS 2 commit `3f9d48ebe843caf1de060bfafe78160585c7c26f`. This does not restore any MoveIt package, plugin, dependency, robot motion or vendor source. The package name, launch command, GUI, two explicit mounting modes and manual prefix remain unchanged. Attribution and BSD notices are installed with the package in `NOTICE.md`.
- Superseding detection rule: remove calibration depth subscriptions, synchronization, plane fitting, fusion, diagnostics and depth view completely. Camera-launcher configuration remains independent and unchanged. Use RGB grayscale -> modern `CharucoDetector.detectBoard` -> `CharucoBoard.matchImagePoints` -> iterative `solvePnP`, with color CameraInfo intrinsics/distortion throughout, explicit measured dictionary/board geometry, `setLegacyPattern(True)`, `CORNER_REFINE_NONE`, two adjacent markers and marker recovery off. Require four non-collinear ChArUco corners and one valid complete RGB board pose. Draw axes directly from that pose, video-only. Missing/partial/collinear detection or ordinary no-pose blocks the frame; malformed native output remains terminal.
- Superseding capture rule: no stability history, one-second wait, 5 mm/2 degree gate, editable corner minimum, averaging, or translation-only diversity exemption. Store the newest valid RGB board pose (at most 0.5 seconds old), robot TF and canonical six-joint observation (each at most 1 second old). Both robot orientation and camera-relative board orientation must differ by at least 5 degrees from every prior sample, including non-adjacent samples. Rejections identify the conflicting C# and failed angle. Inclusive angular boundaries account only for floating-point acos roundoff, not a configurable tolerance. Show hold-stationary and multi-axis-rotation guidance.
- Superseding solve rule: minimum five accepted samples; sample five and subsequent captures/removals automatically recompute using only OpenCV `CALIB_HAND_EYE_TSAI` (upstream `Tsai1989`). Camera-on-hand passes `base_link <- Link6` directly; camera-to-hand passes its inverse; both pass RGB `optical <- board` observations. Compose the optical solution with inverse(`camera_link <- optical`) from required live internal camera TF. Below five samples clears solution and stops calibrated-TF publication. The expanded single RGB view retains solved XYZ/RPY and FIT RMS; only the solved camera-link mounting TF is published.
- Diagnostics rule: retain neutral FIT RMS/maxima/IDs, previous-solution changes, pose coverage and manual removal. Add separately labelled adjacent-pair AX=XB RMS, matching upstream forward/inverse translation-distance averaging and relative-rotation RMS, with explicit mm/degrees. Upstream returns rotation first despite the GUI's labels; named fields prevent copying that unit-label error. This is not image-pixel reprojection error or a physical accuracy guarantee. Leave-one-out is unavailable and non-blocking at five; from six any failed omission retains the full preview, names/logs the omission and disables YAML saving. No holdout set or automatic outlier removal is added.
- Superseding artifact rule: write/read only strict schema 7. Store one RGB board pose, robot transform, six joints, stable IDs/order, settings, diagnostics and pinned pipeline provenance. Remove all fused/depth fields. Preserve existing files; reject schemas 1–6 explicitly without conversion. Existing filenames/directory, save confirmation and explicit Load Calibration remain. Load validates five samples and both all-prior angle checks, confirms replacement, preserves IDs/order/joints and recomputes through live internal camera TF. Joint positions remain data only, never motion commands.
- Superseding prefill rule: strict schema 4, no `minimum_corners` field, `minimum_samples: 5`. During this implementation the workstation's validated ignored schema-3 state is explicitly rewritten preserving mode, prefix, dictionary and board measurements; runtime accepts only schema 4 and always requires Apply. No runtime migration or sample restoration through last_session.json exists.
- Preserved boundaries: one lifetime spawned private OpenCV 4.10.0 worker, verified wheel/checksum, one OpenCV thread, OpenCL off, no cv2 in ROS/Qt parent, independent TF callback execution, terminal native/runtime/protocol failure, bounded UTC events, and transient-only RGB images. No camera, bringup, RViz or hardware command is launched for validation.
- Validation performed: all 54 synthetic regression tests passed, including both-mode parity with the pinned upstream Tsai input order and adjacent AX=XB equations, nonidentity internal camera-TF composition, distorted four-corner geometry/axes/metric units, partial/blank/collinear/no-pose frames, RGB-only callbacks with no depth publisher, immediate capture without stability history, all-prior robot/board angular checks, fifth-sample solving, sixth-sample leave-one-out, removal below five stopping TF/saving, schema-7 round trips/rejections, ID/joint preservation, schema-4 unapplied prefill, independent TF execution and terminal worker failure/logging. Package tests reported 55 results (54 pytest cases plus their CTest wrapper), zero failures/errors/skips. Python compilation, ament_flake8 and git diff --check passed. The package build and a root colcon build with inherited ROS workspace paths cleared completed successfully; root built all 11 packages. Installed-module checks confirmed schema 7, UI schema 4, five-sample minimum, absence of depth callback, no cv2 imported by the ROS/Qt parent and installed attribution. The prior validated workstation prefill retained camera-to-hand/bin_camera/DICT_5X5_50/5x7/30 mm squares/22 mm markers and remains unapplied; all six existing calibration YAML files were left unchanged.
- RGB stress verification: one spawned worker processed 10,000 mixed 1920x1080 full, partial, blank, blurred, shifted and edge-clipped RGB frames in 270.1 seconds with no worker replacement/exit/timeout/malformed result/runtime drift. Full/blurred/shifted frames were ready, partial/edge-clipped frames had insufficient corners, and blank frames had no detection. Worker RSS sampled every 500 frames ranged from 106,448 to 111,900 KiB and plateaued at 111,900 KiB through the final 7,000 frames. Only synthetic inputs and offscreen Qt tests were used; no hardware-facing node or hardware command was launched.
- Offline boundary: all logic uses the existing vendored runtime and installed ROS dependencies. No new download/runtime service, global Python installation, submodule, or vendor patch is introduced. No new offline-transfer milestone is claimed.

### 2026-09-09 — Sequential two-camera startup and complete-set recovery

- Superseding startup rule: every supervised attempt preserves canonical root `.env` slot order. It scans and validates the complete configured serial set, launches Camera 1 alone, and requires Camera 1 color and depth within an independent exact five-second deadline measured from Camera 1 process creation. Camera 2 is not launched until Camera 1 is ready; Camera 2 then receives its own exact five-second deadline measured from Camera 2 process creation. One-camera configurations use the same single-step state machine.
- Supervision rule: while Camera 2 starts, Camera 1's process and both stream timestamps remain under the configured health watchdog. Once every camera is ready, all owned processes and all color/depth streams remain mandatory. A failure of either or both cameras invalidates the complete set, stops every process group owned by that attempt, and never permits partial operation or an individual-camera retry.
- Recovery rule: the three-attempt lifetime budget and exact three-second rest remain absolute. The rest begins only after owned-set shutdown completes. Each remaining attempt rescans the complete serial set and repeats Camera 1 then Camera 2; there is no manual budget reset, concurrent startup, exponential delay, alternate order, or unlimited restart.
- DDS ownership rule: the supervisor records exact publisher endpoint GIDs that appear after its owned camera processes start. During later attempts in that same supervisor run, only those exact retired-owned GIDs may be excluded from collision checks while DDS discovery expires. Any baseline or newly observed endpoint not proven current-owned or exact retired-owned remains a strict unowned-publisher collision.
- Control and reporting: image callbacks only record stream timestamps; the single control timer owns every scan/start/ready/healthy/failure transition. Package events identify each ordered launch, per-camera readiness, next-camera transition, startup timeout, runtime failure, retired-owned filtering, complete-set health, and numbered full-set recovery. The GUI evaluates the final `supervisor_stopped` event as well as the wrapper return code, so three-attempt exhaustion is never reported as a normal stop merely because `ros2 launch` returns zero.
- Validation performed: Python compilation and `ament_flake8` passed; all 20 simulated launcher tests passed directly and through `colcon test`. Regressions cover strict Camera 1 then Camera 2 creation, absent/color-only/depth-only/late Camera 1 preventing Camera 2 launch, independent startup deadlines, Camera 1 health during Camera 2 startup, process exits and individual/both color/depth failures during startup and healthy operation, all-owned shutdown and exact three-second complete-set recovery, Camera 1-first retry order, one-camera operation, exact retired-GID filtering, unknown-publisher rejection, the fixed three-attempt budget, bounded/cursor-based event logging, and exact-process GUI failure reporting when the wrapper exits zero. The package rebuilt successfully, a plain root `colcon build` completed all 11 packages, and a clean isolated root build with inherited workspace paths removed also completed all 11 packages. `git diff --check` and untracked-source whitespace checks passed. The generated 881 MB isolated build directory was deleted after verification. No physical camera or robot process was launched or commanded.

### 2026-09-09 — Fixed-camera ChArUco platform teaching

- New workflow: `item_perception_yolo` now installs the local-only GUI node
  `platform_teach`. The old four-marker `bin_teach_yolo` executable/launch was
  removed, and the imported YOLO teach/detect prototype sources are not
  installed until their later repository-alignment work is complete. There is
  no legacy launch alias or alternate platform-origin workflow.
- Input rule: the operator must explicitly select a strict schema-7
  `camera_to_hand` artifact directly from root `calibration/`. The selected
  artifact supplies the one authoritative camera prefix, RGB/CameraInfo topics,
  camera frames, dictionary, board dimensions, measured square/marker sizes,
  and legacy pattern. Camera-on-hand artifacts, older schemas, files from other
  directories, automatic newest-file selection, `.env` camera inference, and
  configuration overrides are rejected.
- Capture rule: place the saved ChArUco board origin at the bin-mount corner
  selected as the origin of the complete robot pick area. One explicit capture
  uses the newest valid RGB board pose at most 0.5 seconds old and the live
  camera-internal transform, then composes `base_link <- camera_link <-
  color_optical_frame <- board`. The result defines `platform_reference`.
  Depth, robot TF/joints, hand-eye solving, sample averaging, camera launch,
  bringup, RViz launch, and robot motion are outside this node.
- Visualization rule: the GUI shows transient live RGB marker, ChArUco-corner,
  and complete-board-axis overlays using the exact private OpenCV 4.10.0 worker
  owned by `camera_calibration`. After capture it overlays the calculated
  `base_link <- platform_reference` XYZ/RPY and dynamically broadcasts only
  that TF for RViz inspection. Confirmed Retake clears the capture and stops
  further broadcasts. Frames and overlays are never written to disk.
- Artifact rule: confirmed saving creates strict schema 1 in the same root
  directory as camera calibrations, named
  `platform_calibration_<UTC_TIMESTAMP>_<DOBOT_ROBOT_LAN1_IP>.yaml`. LAN1 is
  the robot identity; LAN2 remains diagnostic only. The artifact stores the
  selected camera-calibration filename/SHA-256/schema/provenance, inherited
  camera/ChArUco settings, source frame sequence/timestamp/corner count, all
  three transforms in the capture chain, and final `base_link <-
  platform_reference`. It stores no image. Saving rejects a changed source
  calibration and an existing output path.
- State/log rule: successful Apply atomically writes only the calibration
  filename to strict schema-1 `logs/item_perception_yolo/last_session.json`.
  Restart restores that filename as unapplied prefill and never subscribes,
  captures, broadcasts, or starts anything automatically. Platform events use
  `logs/item_perception_yolo/events.jsonl` with UTC timestamps and the existing
  1,000-record overwrite boundary.
- Shared calibration API: the camera-calibration schema-7 loader now returns
  the transform it already strictly validates. This prevents platform teaching
  from reparsing the file through a weaker second reader and does not change
  camera-calibration math, schema, filenames, or UI behavior.
- Validation performed: Python compilation and `ament_flake8` passed. Seven
  new platform tests cover the full three-transform composition, strict
  camera-to-hand selection, canonical root LAN1 identity, prefill-only UI
  state, schema-1 round trip/no-frame storage, changed-source rejection, and
  bounded package logs. All 54 camera-calibration regressions also passed after
  exposing its already-validated transform. `colcon test` reported eight
  platform results including the CTest wrapper with zero failures/errors/skips.
  The installed package exposes only `platform_teach` and
  `platform_teach.launch.py`; launch inspection reports no arguments. A clean
  root `colcon build` completed all 12 packages, installed-source checks loaded
  the current schema-7 `bin_camera` calibration while confirming the Qt parent
  did not import `cv2`, and `git diff --check` passed. No camera, platform GUI,
  robot, RViz, or hardware command was launched.
- Offline boundary: the new node reuses the existing vendored OpenCV wheel and
  ROS 2 dependencies. It adds no network fetch, global Python installation,
  submodule, model download, compatibility fallback, or generated image data.

### 2026-09-10 — Four-marker bin ROI teaching

- New workflow: `item_perception_yolo` now installs the local-only `bin_teach`
  GUI and `bin_teach.launch.py`; the exact name has no `yolo` suffix and no
  legacy launch alias. `platform_teach` remains the only workflow that defines
  `platform_reference`, and `bin_teach` cannot Apply without an explicitly
  selected strict schema-1 platform calibration from root `calibration/`.
- Input/provenance rule: applying Bin Teach validates the platform artifact,
  its LAN1 robot identity against root `.env`, and the exact referenced
  schema-7 camera-to-hand filename/SHA-256. It inherits that camera's prefix,
  RGB/CameraInfo topics, link/optical frames, and calibrated
  `base_link <- camera_link`. It also requires one explicitly selected OpenCV
  5x5 dictionary and one positive measured common marker size in millimetres.
  No automatic calibration selection, alternate configuration, depth, robot
  TF/joints, camera launch, RViz launch, or robot motion is introduced.
- Detection/geometry rule: one lifetime package-private OpenCV 4.10 worker uses
  `ArucoDetector.detectMarkers`, `CORNER_REFINE_NONE`, and fixed
  `SOLVEPNP_IPPE_SQUARE`. The visible marker set must be exactly IDs
  `{0,1,2,3}`; a missing or additional ID blocks the frame. Marker IDs define
  membership only, never spatial ordering. The node uses live
  `camera_link <- color_optical_frame`, composes
  `platform_reference <- optical`, and transforms all four physical corners of
  every marker. For each marker, the unique corner farthest from the centroid
  of the four marker centres is the outside corner. The four resulting metric
  XY points are sorted clockwise starting at the lexicographically smallest
  XY point. A tied outside corner or duplicate, degenerate, non-clockwise, or
  non-convex quadrilateral blocks capture without an alternate choice.
- Visualization/capture rule: the RGB GUI draws detected marker boundaries and
  IDs, selected outside corners, ordered point labels, and the ROI polygon.
  One explicit capture requires a result no older than 0.5 seconds. Retake is
  confirmed and clears only the captured ROI. Images, overlays, individual
  marker origins/poses, depth, and robot state remain transient or absent and
  are never written.
- Artifact rule: confirmed Save creates only strict schema 1 at
  `calibration/bin_teach_<UTC_TIMESTAMP>_<DOBOT_ROBOT_LAN1_IP>.yaml`. The
  geometric payload is exactly four XY points in metres relative to
  `platform_reference`; point provenance records the source marker ID and
  marker-corner index. The artifact also records the exact platform/camera
  hashes, selected ArUco settings, detector/runtime identity, camera contract,
  and capture timestamp/sequence. Source-file change and existing output paths
  hard-fail.
- GUI-state rule: `platform_teach` and `bin_teach` now share the one canonical
  package-owned `logs/item_perception_yolo/last_session.json` at strict schema
  2. Each Apply atomically updates its own section while preserving the other;
  restoration is unapplied prefill only. The workstation's prior strict
  platform schema-1 state was explicitly rewritten to schema 2 with an empty
  bin section; there is no runtime schema migration or fallback reader.
- Validation performed: Python compilation and targeted ament flake8 passed;
  all 15 item-perception teaching tests and all 55 camera-calibration/OpenCV
  worker tests passed. The new regressions cover ID-order-independent outside
  corner selection, full transform composition, schema-2 shared UI state,
  strict schema-1 four-point YAML round trip/no-frame storage, changed-source
  save rejection, 5x5/size validation, and one spawned exact-four-marker
  OpenCV detection. A clean-environment root `colcon build` completed all 12
  packages. The installed package exposes exactly `platform_teach` and
  `bin_teach`; `bin_teach.launch.py --show-args` reports no arguments. An
  offscreen installed-GUI check confirmed blank first-run ArUco fields,
  disabled capture/save before Apply, and no `cv2` import in the ROS/Qt parent.
  No camera, teaching GUI, robot, RViz, or hardware command was launched.
- Offline boundary: the node reuses the existing vendored OpenCV wheel and ROS
  dependencies. It adds no network fetch, global installation, submodule,
  model artifact, compatibility path, or vendor patch.

### 2026-09-10 — Mode-flexible platform and bin teaching

- Superseding camera-selection rule: `platform_teach` now accepts either one
  strict schema-7 `camera_to_hand` artifact or one strict schema-7
  `camera_on_hand` artifact explicitly selected from root `calibration/`. It
  never chooses a file or mode automatically. Fixed-camera mode uses calibrated
  `base_link <- camera_link` directly. On-hand mode requires a live, non-zero
  `base_link <- Link6` TF no older than exactly one second and resolves
  `base_link <- Link6 <- camera_link`. Both modes then use the required live
  camera-internal TF and newest fresh RGB ChArUco board pose to calculate
  `base_link <- platform_reference`. The robot and target must remain stationary
  for capture. There is no depth, joint subscription, motion command, alternate
  transform source, stale-TF acceptance, retry, or fallback.
- Superseding platform-artifact rule: new platform outputs are strict schema 2.
  They record the selected camera mode and reference frame, source SHA-256,
  calibrated mounting transform, resolved `base_link <- camera_link`, camera
  internal transform, RGB board pose, and final platform transform. On-hand
  output additionally records the exact `base_link <- Link6` transform and ROS
  timestamp used plus the absolute one-second freshness contract; fixed-camera
  output records robot TF as explicitly unused. Schema 1 remains untouched but
  is rejected without migration or a compatibility reader.
- Superseding bin-teach rule: `bin_teach` now requires strict platform schema 2
  and reloads the exact SHA-256-bound schema-7 camera artifact in the same mode.
  Fixed mode retains the static calibrated camera chain. On-hand mode requires
  a fresh live `base_link <- Link6` TF while producing the live ROI and again at
  capture freshness evaluation. The saved four metric XY points remain in
  `platform_reference`, and ArUco detection/outside-corner/order rules remain
  unchanged. New bin outputs are strict schema 2 and record the selected mode,
  complete resolved optical transform chain, and required on-hand robot
  TF/timestamp. Schema 1 is preserved but rejected without conversion.
- GUI-state rule: the one shared package prefill is now strict schema 3 because
  platform camera filenames may belong to either explicit calibration family.
  The schema-2 workstation state was explicitly rewritten in place while
  preserving its still-valid platform camera filename. Its bin section was
  explicitly cleared because the prior selected platform artifact is schema 1
  and must not be presented as valid input to the schema-2-only workflow.
  Restore remains unapplied prefill only; no process, subscription, TF, or
  capture starts automatically.
- Preserved boundaries: both nodes remain local-only two-thread ROS/Qt viewers,
  use the exact private OpenCV 4.10 worker, retain transient-only RGB views and
  bounded package events, and never launch cameras, Dobot bringup, robot-state
  publisher, RViz, or robot motion. Platform preview still broadcasts only the
  captured `base_link -> platform_reference`; bin teaching creates no TF.
- Validation performed: 20 synthetic teaching tests pass for both transform
  modes, explicit mounting resolution, mandatory on-hand robot TF, schema-2
  platform/bin round trips, rejection of schema-1 artifacts and schema-2 UI
  state, source hashes, four-point ROI geometry, and shared schema-3 prefill.
  `ament_flake8` passed all seven changed source/test files. A clean-environment
  package build installed both teaching executables and `colcon test` reported
  all 20 tests with zero failures/errors/skips. Both real schema-7 workstation
  camera artifacts loaded with their exact modes/reference frames. A clean
  root `colcon build` completed all 12 packages. No camera, teaching GUI, Dobot
  bringup, robot-state publisher, RViz, robot motion, or hardware command was
  launched.

### 2026-09-10 — Fixed two-camera Orbbec configuration

- Superseding configuration rule: the canonical Orbbec set now always contains
  exactly Camera 1 and Camera 2. `ORBBEC_CAMERA_COUNT` was removed from the
  ignored root `.env`, tracked `.env.example`, strict shell loader, Python
  configuration contract, and launcher GUI. The retired key is unsupported and
  therefore hard-fails if restored; there is no alias or compatibility reader.
- Runtime rule: both canonical camera name/serial pairs are always validated and
  returned in slot order. The headless launch and watchdog require exactly two
  cameras and use internal `device_num=2`. The per-row diagnostic action still
  launches exactly one selected camera with `device_num=1`; it does not alter
  the fixed configured set.
- GUI rule: the editable camera-count selector and inactive-slot behavior were
  removed. Both camera rows are always visible and editable, while the supervised
  action is always labelled `Launch Both (Watchdog)`.
- Shared-reader/vendor-patch record: the canonical shell loader,
  `motion_debug`, and the intentionally patched vendored Dobot bringup strict
  key sets no longer require or accept the retired field. The bringup patch is
  necessary because it validates the entire canonical project `.env`; it does
  not add camera launch or camera ownership to Dobot bringup.
- Validation performed: shell syntax validation and Python compilation passed;
  all 21 Orbbec launcher/supervisor tests passed; targeted `ament_flake8` passed
  all five launcher source/test files; the strict installed parser accepted the
  current two-slot `.env`; and both the package build and clean-environment root
  `colcon build` completed all 12 packages. No camera, robot, RViz, or hardware
  command was launched.

### 2026-09-10 — Bin-corner RViz TF preview

- Superseding visualization rule: the earlier statement that `bin_teach`
  creates no TF is superseded. After one successful bin capture, the node
  dynamically broadcasts the complete preview chain
  `base_link -> platform_reference -> bin_corner_1..4` at exactly 10 Hz. It
  still never launches RViz; the read-only `dobot_rviz` viewer is started
  separately by the operator.
- Geometry rule: `base_link -> platform_reference` comes from the explicitly
  selected strict platform-calibration artifact. `bin_corner_1` through
  `bin_corner_4` correspond exactly to saved clockwise P1 through P4. Because
  the strict bin artifact stores a two-dimensional ROI, each child uses saved
  `(x, y)`, exact `z=0`, and identity rotation in `platform_reference`; noisy
  marker-pose Z must not be substituted.
- Lifecycle rule: the preview begins only after capture and continues after
  YAML saving while the node remains alive. Retake, re-Apply, terminal worker
  failure, or node exit clears/stops it. No TF preview values are added to the
  schema-2 artifact, and no camera, robot, RViz, or motion process is started.
- Validation performed: Python compilation and targeted `ament_flake8` passed;
  all 21 item-perception teaching tests passed (22 reported package test
  results), including an exact synthetic check of platform translation,
  platform rotation, stable P1–P4 frame naming, saved XY coordinates, zero Z,
  identity corner rotations, and shared timestamps. The package rebuilt and a
  clean-environment root `colcon build` completed all 12 packages. No camera,
  teaching GUI, robot, RViz, or hardware command was launched.

### 2026-09-10 — Portable bin ROI and station platform references

- Scope clarified by the user: Platform Teach and Bin Teach create teaching
  files; their RViz displays are operator checks only. Future item detection
  will load the files independently, and its detection/picking behavior is not
  finalized. No depth filtering, pick-height limits, motion, or item-detection
  implementation is part of this change.
- Superseding geometry contract: keep four metric XY points in
  `platform_reference`. The platform board and bin-corner markers are
  approximately coplanar within each station; their absolute base-relative
  height may change at another station. Portable reuse requires the same
  physical board origin, board-axis directions, bin dimensions, and bin offset.
  A common point alone is not a common coordinate frame. The explicit
  `charuco_pick_corner_xy_v1` convention records origin/axes, metres, and the
  platform-plane Z=0 interpretation. No inferred floor/rim heights, automatic
  physical-alignment detection, scaling, rotation override, or fallback is added.
- Superseding artifact contract: platform and bin writers/readers now require
  strict schema 3. Platform files add the shared reference convention. Bin
  files separate portable `roi`/`reference` from `teaching_provenance`, which
  contains original robot/camera/platform identity, hashes, detector settings,
  capture evidence, and the original platform transform needed to validate the
  internal source transform chain. Original source hashes remain traceability
  evidence and do not authenticate absent source files. Fresh capture/save
  still strictly checks the currently applied platform/camera files and robot.
- Portability rule: loading a bin template reads the copied bin file alone;
  it does not require its original `.env`, platform file or camera file. An
  explicitly selected destination platform supplies `base_link <-
  platform_reference`; original station transforms, camera prefixes and
  mounting mode never place the copied ROI. Source/destination mounting modes
  may differ. Both the destination platform and its camera remain strictly
  checked against destination robot identity and saved hashes when applied.
- Teaching GUI: `Load Bin ROI` is explicit, requires an applied destination
  platform, and confirms matching physical placement before replacing the
  current capture/preview. It previews unchanged points in RViz at platform
  Z=0, without requiring marker detections or live robot TF for that stored
  geometry. The RGB view continues showing live markers and is labelled
  separately; live detections are never presented as the loaded ROI. Loading
  creates no capture or output file and keeps saving disabled; confirmed Retake
  permits fresh teaching. Re-Apply, terminal failure and exit stop publication.
  Close Platform Teach after saving before Bin Teach broadcasts that platform,
  avoiding duplicate teaching-preview authorities.
- Schema/workstation boundary: camera artifacts remain strict schema 7 and
  shared unapplied form-prefill stays schema 3 with unchanged fields. Loaded
  templates/captures are never restored through prefill. Existing platform/bin
  schema-1/2 files are left untouched and explicitly rejected; re-teach to
  create schema 3. Filenames/output directory remain unchanged. The robot IP
  in a bin filename identifies the teaching station, not a deployment limit.
- Verification: all 27 synthetic teaching tests pass (28 reported package test
  results including the wrapper). Regressions cover both source camera modes,
  copying only a bin file into another station without the original files or
  `.env`, destination height/rotation changes, opposite source/destination
  mounting modes, unchanged XY coordinates, reference mismatch, schema-1/2
  rejection, malformed provenance chains, explicit load without a capture,
  save blocking, and preview stopping on Retake, re-Apply or fatal failure.
  Targeted lint and Python compilation pass. A clean-environment root build
  completed all 12 packages. An installed offscreen Qt check verified first-run
  and loaded-preview controls and no `cv2` import in the parent. `git diff
  --check` passed. No camera, robot, RViz, visible GUI, or hardware command was
  launched; existing calibration artifacts were not modified.

### 2026-09-10 — Bin teaching data separated from calibration

- Superseding storage rule: bin ROI artifacts belong only in root
  `offline_teach/bin_teach/`, with unchanged filename pattern
  `bin_teach_<UTC_TIMESTAMP>_<SOURCE_ROBOT_LAN1_IP>.yaml`. Camera and platform
  calibration files remain in `calibration/`. This overrides earlier diary
  entries that placed bin files alongside calibrations.
- The bin writer, strict reader, GUI output label and Load Bin ROI dialog all
  use that one directory. Bin reads/writes targeting `calibration/` are rejected;
  no compatibility lookup, automatic search, or alternate output override is
  introduced. Applying a station's platform still reads the platform and its
  camera artifact from `calibration/`.
- Schema 3, geometry, provenance, teaching previews, and source-robot filename
  meaning are unchanged. Item detection/picking behavior remains undecided and
  its code was not changed. A README keeps the teaching directory in the source
  layout and documents transfer of bin templates into the same destination
  directory.
- Workstation check: no existing bin YAML files were present to relocate;
  existing camera/platform files were left untouched.
- Verification: all 27 teaching tests passed, including save/load from the new
  directory, explicit rejection of the old calibration location, and portable
  loading at a destination with no calibration folder or original source
  files. Package tests reported 28 results including the wrapper with no
  errors/failures/skips. Targeted lint, Python compilation, and `git diff
  --check` passed; a clean-environment root build completed all 12 packages.
  No camera, robot, RViz, or hardware command was launched.

### 2026-09-10 — Green loaded-bin border on live RGB

- Superseding preview rule: Load Bin ROI now shows the saved geometry on live
  RGB as a green unfilled border with a green `Loaded Bin Teach | <filename>`
  label at top left. This replaces the earlier RViz-only loaded-geometry
  visualization; the existing RViz preview remains unchanged. Normal fresh
  marker teaching retains its yellow detection border.
- Loaded mode skips marker detection entirely. The saved XY points remain on
  the currently applied destination platform's Z=0 plane, transformed into the
  current camera optical frame with the destination camera calibration and
  camera-internal TF. Camera-on-hand additionally uses fresh robot TF. Original
  source-station transforms never place the copied template.
- The existing shared private OpenCV worker gains a serialized `project_points`
  operation using `projectPoints` and current color CameraInfo K/D. Thirty-two
  samples per edge represent the lens-distorted border. Only explicit
  `plumb_bob`/`rational_polynomial` distortion models are accepted for this
  projection. The ROS/Qt parent still does not import cv2; there is no new
  process, runtime, fallback, detector, or calibration solve behavior.
- Projection requires RGB no older than 0.5 seconds, valid matching CameraInfo
  and internal TF, plus robot TF no older than one second in on-hand mode.
  Invalid/missing inputs, stale streams/robot TF, or behind-camera geometry
  hide the border and explain why without discarding the loaded template.
  Malformed projection output and worker failures remain terminal. Atomic
  image/geometry snapshots and configuration generations prevent old detections
  or borders surviving Load, Retake or re-Apply.
- No artifact schema, storage location, provenance, capture, or save behavior
  changes. Loading remains preview-only, saves no new file, and does not persist
  geometry in UI prefill. No vendor or item-detection code is changed.
- Verification: all 42 teaching tests pass, covering green border/label pixels,
  unfilled ROI, projection without visible markers, both mounting modes with
  nontrivial destination/internal/robot transforms, missing and stale inputs,
  behind-camera geometry, unsupported distortion model, Retake during an
  in-flight projection, and malformed worker responses. A real isolated-worker
  test matches independently calculated lens-distorted pixels. Camera
  calibration regression tests report 56 results with no failures/errors/skips.
  Targeted lint, Python compilation and `git diff --check` pass; the root build
  completed all 12 packages. An installed import check confirms no parent cv2
  import and the private projection API is present. Verification uses synthetic
  inputs and offscreen Qt only; no cameras, robot, RViz, or hardware commands
  were launched. Existing teaching/calibration files were not modified.

### 2026-09-13 — Item teaching and sole robot-controller ownership decision

- User-directed architecture rule: create a new `robot_controller` package/node
  as the sole application-level issuer of robot motion, stop, enable/disable,
  settings, and gripper/I/O commands. It uses canonical Dobot bringup as the
  hardware transport/feedback layer. Teach/perception and the existing
  motion/gripper GUIs must request actions through this controller. Their
  established initialization/safety semantics must be preserved by the
  migration; no direct-service/TCP bypass is allowed when the controller is
  unavailable. Physical emergency stops remain independent.
- Implementation boundary: this entry records the decision only. No controller
  was created and no current runtime call path was changed. `motion_debug` and
  `gripper_control` currently still have direct Dobot clients, so sole command
  ownership is not yet enforced. Do not describe this policy as implemented
  until all application command paths are migrated and tested.
- Item-teach direction: remove the separate `item_detect_yolo_debug` prototype
  during the agreed implementation; rework teaching into a pre-trained `.pt`
  model/profile editor plus manually triggered detection and explicit pick
  requests. No training/annotation workflow. Pair the item YAML and copied
  model under `offline_teach/item_teach/`. These runtime/deletion changes are
  pending feature alignment and were not performed in this discussion.
- Confirmed terminology: `standoff_height` compensates the gripper length in
  the final Link6 pick target, not a hover clearance. Rename `final_zheight`
  to `zheight_offset` for the shared first approach/final retract position
  above the candidate. Do not double-apply a configured controller TCP offset;
  the exact reference/direction for each height must be finalized. Existing
  `prepick_height`, intermediate retract height and settling timing also need
  final definitions rather than importing fixed numbers from the prototype.
- Gripper direction: use the existing package only to inspect service/feedback
  and wiring, not to reuse its Grip/Release patterns. The user describes
  `use_grip=true` as keeping the gripper open throughout the routine and
  `grip_onpick` as closing upon suction detection. Precedence when both are
  true, the false/false case, exact close outputs, suction activation/purge,
  sensor channel/polarity, and confirmation timing remain unresolved. The
  current project map is DO1 exhaust, DO2 gripper close, DO13 finger close,
  DO14 gripper open; DI1/DI2 are still generic in the current implementation.
- Explicit retry direction: the user permits a bounded next-candidate recovery
  after unsuccessful suction (example: top three candidates), but only after
  reaching the current candidate's final retract position. This is a planned
  exception for failed acquisition, not permission to ignore robot faults,
  missing/stale feedback, failed stop/retract, or configuration errors. Exact
  acquisition timeouts, candidate freshness/ranking, retry budget and exhausted
  outcome remain to be finalized; no infinite or implicit retry is approved.
- Source review: the imported `item_pick` has useful MovLIO, monitored suction
  descent and ranked-candidate structures, but uses DO1/2/3/4 with different
  meanings, old CATARM paths/schema, optional compatibility behavior and a
  `TrayInterceptStart` interface absent from this workspace. Its source limits
  same-frame candidates to three while its README says five. Treat it as
  reference, not an aligned runnable implementation, and do not copy its fixed
  home, purge, tray, height or queue behavior without a project decision.
- Verification: read-only inspection of the relevant imported pick/gripper
  code, manifests, current Dobot interfaces and documentation; documentation
  diff checked. No code/artifact deletion or modification, build, node launch,
  or hardware command was performed in this discussion.

### 2026-09-13 — Corrected gripper wiring and bounded detection requests

- Superseding physical I/O contract supplied by the user: DO1 exhaust; DO2
  finger close; DO13 suction; DO14 finger open; DI1 suction detection; DI12
  finger fully open. This replaces the earlier DO13 finger-close meaning and
  generic DI1/DI2 monitoring contract. Runtime remapping has not been performed
  in this discussion; existing `gripper_control` Grip/Release patterns must not
  be used for the new controller. Open/close/suction/exhaust sequencing will be
  designed explicitly, not copied from either old package.
- Superseding flag semantics: `use_grip=true` enables finger control, not
  permanently open fingers. False disables finger-control behavior and makes
  `grip_onpick` ineffective. With finger control enabled, `grip_onpick=true`
  closes on confirmed suction. The close timing for `grip_onpick=false` is
  still unspecified; do not silently import the prototype's 90%-retract close.
- Sensor clarification pending: the user requires inability to confirm full
  opening to block operation as a finger fault. A fully-open limit signal is
  normally inactive when intentionally closed, so low DI12 alone is not a
  diagnosis of damage. The phase in which DI12 must confirm, electrical
  polarity, opening deadline and suction confirmation/debounce timings need
  explicit agreement before any runtime actuation is implemented.
- Controller/detector ownership: `robot_controller` requests item poses from
  `item_detect`; the detector loads the selected item/model pair and the
  current station camera/platform/bin files, draws overlays and returns valid
  targets in `platform_reference`. Targets are unadjusted item positions;
  Link6 gripper standoff belongs to controller execution, not detection.
  Runtime geometry must not depend on teach GUIs or their RViz preview staying
  alive. This explicitly advances item-detection design beyond the previous
  teaching-only scope; platform/bin artifact geometry itself remains unchanged.
- Confirmed count semantics: save `retry_limit` per item. A value of three
  requests up to three valid distinct ranked poses and permits three total
  candidate attempts including the first, not an initial attempt plus three
  more. Only a missed suction pickup may advance to another candidate, after
  the preceding final retract has actually completed. Missing feedback, robot
  faults, failed stop or failed retract remain separate terminal conditions.
- Save a separate `yolo_max_det` detection cap per item. Example: cap a frame
  at 20 detections, validate/rank those detections using the current calibrated
  bin context, and return up to the three requested candidates. Neither cap
  guarantees enough visible/valid objects; no duplicated/fabricated poses or
  hidden reacquisition loop is allowed. Fewer-result handling, exact ranking,
  expiry/revalidation and batch-exhaustion behavior still need final agreement.
- Vertical-only scope: the user wants gripper-length `standoff_height` and no
  per-item tool-orientation search or TCP-management workflow. Pick/place
  approach and withdrawal are vertical; the fixed command attitude, vertical
  reference axis and exact height equations must be specified before motion.
  There must be no double application of an existing controller tool offset.
- Item artifact direction: all supported operator-adjustable pick and YOLO
  inference settings, including retry/detection limits and detection mode,
  belong in the final item YAML beside its hash-bound `.pt` under
  `offline_teach/item_teach/`. Proposed controls include input resolution,
  confidence, target classes, applicable NMS settings, segmentation controls,
  and candidate ranking. Supported tasks and exact pinned runtime are still
  proposals, not implemented modes. A task selector must never pretend to
  turn box-only weights into a segmentation or OBB model. Hardware-wide
  configuration continues to belong to root `.env`.
- Verification: inspected the imported teach/detector prediction paths,
  current gripper constants/feedback bit mapping and official Ultralytics
  prediction/segmentation/OBB documentation. The imported detector has box
  and optional-mask handling, not a verified new multi-task contract. Changed
  documentation only and checked `git diff --check`; no runtime code, saved
  artifacts, packages or hardware state were changed and no build/launch ran.

### 2026-09-13 — Initial item profile editor and non-actuating controller

- Added canonical `item_perception_yolo/item_teach` and new project package
  `robot_controller`. This is an explicitly non-executing initial stage:
  file/profile editing, fresh home recording and controller-side validation.
  No training, model deserialization, inference, camera debug view, detector
  request, physical motion or I/O execution is implemented by this change.
  There is no pinned/vendored Ultralytics runtime in this workspace; do not
  pretend that copying weights verifies model-task compatibility. Declared
  detect/segment/obb task labels are metadata, not task conversion or inference.
- User rule: the source `.pt` may be selected anywhere on the PC. All final
  output is a new strict schema-1 YAML and same-stem `.pt` copy in canonical
  `offline_teach/item_teach/`, named `item_teach_<NAME>_<UTC_TIMESTAMP>`.
  Copy/hash without executing weights; verify unchanged source/copy, publish
  YAML last, never overwrite existing files, and confirm both saved paths.
  YAML stores only the local model filename, SHA-256 and `file_sha256_only`
  verification. Preserve source weights. Transfer both final files together.
- Group the artifact into item, model, units, home, motion, timing, gripper,
  retry, yolo and the exact validation-only controller contract. Distances are
  millimetres, timing seconds, home joints radians. Canonical grouped detection
  cap is `yolo.max_detections`, superseding the provisional flat `yolo_max_det`
  spelling; no alias. `retry.retry_limit` remains total attempts/request count,
  including the first, and cannot exceed the per-frame detection cap. Store
  explicit confidence/IoU/input-size/class IDs and operator-declared task.
- New home requirement: record finite joint1 through joint6 from the sole
  canonical root-namespace Dobot `/joint_states` publisher, requiring source
  timestamp and local receipt age at most one second. Preserve source stamp,
  recording UTC, IP, publisher and topic; display degrees but save radians.
  No zero/default or automatic home movement. Intended full-routine home use
  is start and end, but actual execution remains pending.
- Superseding user clarification: identical robots may reuse the same saved
  home joint positions. Source robot IP and node are provenance only, not a
  load/save/controller-validation restriction. Removed the temporary identity
  binding before completion. This is not a collision-free travel guarantee;
  future physical moves still require explicit execution and safety checks.
- Shared package UI state advances to strict schema 4 with an item section
  containing only the selected named profile filename. Its fields come from
  that authoritative YAML/model pair as unapplied prefill; no auto-submission
  or joint replay. Platform/bin sections preserve each other and item state.
  Explicitly rewrote this workstation's validated schema-3 prefill once,
  preserving its selected camera/platform, dictionary, marker measurement and
  timestamp, adding an empty item selection. Runtime rejects older schemas
  without conversion. Camera schema 7 and platform/bin artifact schema 3 and
  geometry are unchanged.
- Controller interface: explicit `item_teach_file` parameter through canonical
  atomic parameter service, a Trigger revalidation service and non-armed JSON
  status. No Dobot service/TCP clients. Missing controller, invalid pair or
  five-second GUI request timeout is explicit; no bypass/retry. Controller
  status reflects last validation, not continuous model hashing. Both packages
  log UTC events under their own 1,000-event package limits.
- Removed retired training-oriented item_teach_yolo script/launcher/config and
  separate item_detect_yolo_debug script/launcher, after preserving copies in
  a temporary directory outside the repo. Kept the detector prototype as
  uninstalled reference. Added `COLCON_IGNORE` to imported `item_pick`, whose
  unaligned execution path remains reference-only. No vendor changes.
- Command ownership migration remains pending for existing motion/gripper
  clients. Do not claim this validation scaffold enforces sole ownership or
  completes the pick pipeline. Finalize height equations/frame/attitude,
  sensor polarity/confirmations/timeouts, home/travel/place safety, candidate
  ranking/freshness, pin offline inference and implement those stages next.
- Verification so far: 73 synthetic core/GUI/controller/platform/bin tests
  passed, including unrestricted model selection, independent paired copies,
  tamper/duplicate/schema/copy-failure rejection, portable home, stale feedback,
  unapplied UI prefill, service rejection and bounded events. Both package
  builds and root `colcon build` passed (13 packages); repeated the root build
  in a clean ROS-Humble-only shell to avoid self-overlay warnings. Colcon
  package tests passed (68 perception tests and 5 controller tests). Python
  compilation, flake8 and `git diff --check` passed; installed imports verified
  that neither OpenCV nor model runtimes enter these new nodes. Both installed
  launch descriptions/executables were checked without starting nodes. No
  hardware was launched or commanded; no offline deployment milestone claimed.

### 2026-09-13 — Item measurement plane and candidate geometry decision

- User decision for the item-teach/detection rework: measure item length and
  width by projection onto the taught `platform_reference` XY plane at Z=0,
  even when an item is physically above or below that plane. Do not move the
  measurement plane to an item's measured depth. These are explicitly
  reference-plane projected dimensions, not height-corrected physical sizes.
- The four saved bin corners are XY coordinates with implicit Z=0. They
  therefore define that same platform plane, not an independently measured
  plane with a new tilt or height. Keep platform teach authoritative for the
  plane and bin teach authoritative for its ROI; do not change either artifact
  schema or infer unsaved corner heights.
- Mask geometry uses a minimum-area enclosing rectangle; OBB geometry uses
  the model's oriented rectangle. When the loaded model actually supplies both
  outputs, provide an explicit choice; never fabricate an unavailable output.
  The longer measured side is item X (`height`) and the shorter side item Y
  (`width`), in millimetres, with one plus/minus millimetre `tolerance` for both.
- Confidence and dimensions/tolerance are eligibility filters. User priority
  is proximity to the center, not confidence or greatest depth height. The
  implementation interpretation is the taught bin ROI's center. Keep YOLO's
  overlap/IoU setting distinct from dimensional tolerance.
- Depth sampling remains a separate pick-position calculation around the exact
  pick point. The user-named `pickdepth_radius` is explicitly the circle's
  **diameter**, default 30 mm, despite its name. It must not change the fixed
  length/width measurement plane.
- Status: this entry records the agreed geometry contract; the live preview
  rework is in progress and the armed calibrated-pose service is not yet
  implemented. Do not describe a disabled Armed button or stored settings as
  an operational pose service. No hardware was launched or commanded for this
  decision. Geometry implementation and synthetic verification remain pending.

### 2026-09-13 — Item pick-depth sampling and overlay decision

- The pick pixel is the selected item's rectangle center: the intersection of
  its long-axis and short-axis midlines. Center the registered-depth sampling
  circle on that exact pixel; do not move the pick pixel to an accepted depth
  sample or to the centroid of the retained samples. `pickdepth_radius` retains
  the user-defined diameter meaning and 30 mm default.
- Show the sampling circle on the depth image and plot the sampled pixel
  locations. Explicit user colors: accepted depth samples **black**, rejected
  samples **red**. Include a textual legend, accepted/rejected counts and the
  resulting filtered depth so the colors are not the only feedback. Invalid
  samples can be marked at their pixel positions, but must never become 3D
  points or influence the depth estimate.
- Reject missing, non-finite, non-positive and contract-invalid depth before
  computing statistics. Apply the requested three-sigma MAD outlier filter to
  the remaining depths: m = median(depth), MAD = median(abs(depth - m)),
  robust_sigma = 1.4826 * MAD; accept abs(depth - m) <= 3 * robust_sigma.
  Use the median of accepted depths as the center-ray depth estimate. This is
  robust-sigma scaling, not a claim that all noise is normally distributed.
  Statistical reference: SciPy's official median_abs_deviation documentation,
  https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.median_abs_deviation.html.
- Coordinate interpretation for implementation: rectangle center supplies
  image (u,v), not already-metric platform XY. Back-project that exact center
  using color intrinsics and filtered registered depth, then transform the
  complete 3D point into `platform_reference`. Its transformed Z is the item
  height; never append camera-optical depth directly to platform-plane XY.
  This does not change the separate fixed-plane length/width measurement rule.
- Status: recorded requirements only in this follow-up; depth subscription,
  sampling/filtering/overlays and the armed pose service remain pending.
  No application code or hardware state changed. `git diff --check` passed.

### 2026-09-13 — Item class checkboxes and read-only pose pipeline implementation

- User approved the complete request-driven pose workflow and asked for one
  checkbox per actual model class. Implemented model inspection in a private
  worker, explicit trusted-model loading, checked IDs saved in the item profile,
  mask/OBB selection, raw/YOLO toggle, split RGB/depth diagnostics, explicit
  station/bin application and Armed toggle. New models start with no checked
  classes; a saved selection is unapplied/unverified until Load Model. No
  training, annotation or hardware execution is added.
- Supersedes initial-stage rules 30–31 for perception only. Added canonical
  headless `item_detect` sharing the GUI's detector backend, and the small
  `item_perception_interfaces` message/service package (root now 14 packages).
  `/item_detect/get_item_poses` exists only while explicitly armed with matching
  verified profile/model and valid station/RGB/registered-depth/CameraInfo/TF.
  Unknown duplicate service owners reject arming. Headless paths, model trust
  and arming are explicit launch parameters, never auto-selected from UI state.
- Each request uses a new RGB/depth observation after arrival, serialized native
  inference and one timestamp-consistent transform snapshot. Concurrent calls
  return BUSY. Source/profile mutation, disarm during processing, missing/stale
  inputs and expired results cannot return old poses. Camera-on-hand composes
  live base_link<-Link6 at the RGB timestamp; fixed-camera uses its saved mount.
  Three executor threads retain independent sensor/TF execution. No TF is
  borrowed from a teaching preview node.
- Timing refinement: once a valid request pair is selected, retain that same
  pair while awaiting its timestamped TF, rather than continually replacing it
  with newer images. It must still satisfy input freshness before inference.
  Arming checks latest TF availability only; pose generation never uses latest
  TF as a substitute for observation-time TF. Explicit operator shutdown cancels
  a pending native operation without mislabelling it as a native crash.
- Fixed-plane dimensions: project the mask-derived pixel rectangle or model
  OBB to platform Z=0, then use its metric minimum-area enclosing rectangle
  because perspective may produce a quadrilateral. Long X is height; short Y
  is width. Reject class/confidence/size failures and footprints outside the
  bin. A mask whose rectangle center is outside its polygon is rejected, not
  recentered. The exact pixel rectangle center remains the pick ray.
- Sampling-circle interpretation is explicit: diameter in millimetres on the
  reference plane, centered at the pick-ray/plane intersection, projected with
  lens distortion to depth pixels (not necessarily a screen-space circle).
  Process only its bounded pixel window. Mask/OBB-exterior, null, non-positive,
  non-finite and out-of-range depths are rejected. Apply 3*1.4826*MAD; MAD=0
  accepts only median-equal values, with no epsilon/fallback filter. Require
  retained count/fraction; median retained optical depth back-projects the
  original center, then transform complete XYZ to platform_reference. Length
  and width remain independent of item elevation. Image-clipped circles reject
  rather than silently shrinking. Accepted depth pixels black, rejected red;
  show legend, counts, filtered depth, platform Z and age. No frames are saved.
- Rank by distance to bin polygon area centroid in platform XY; descending
  confidence and source index break exact distance ties. Return up to requested
  count with explicit shortage/no-items status, batch-local IDs, metre poses,
  timestamps, depth statistics and source/transform evidence. Heading follows
  long X with a deterministic sign and platform-normal Z; no surface tilt/TCP
  orientation is claimed. The unchanged controller's motion state remains
  non-armed; its new read-only Trigger request delegates count=retry_limit and
  validates/logs the returned batch, never executing it. Existing direct robot
  client migration and full pick/I/O behavior remain pending.
- Item YAML advances to strict schema 3 (unreleased intermediate schema 2 also
  rejected). Added geometry_source, geometry and quality groups. Explicit
  initial operator values: input age 0.5s, synchronization 0.1s, robot TF age 1s,
  request deadline 10s, result age 2s, minimum 30 accepted depths and 50% of
  circle pixels, depth range 200–1000mm. These are editable saved fields, not
  missing-key runtime defaults. Home requirement/portable provenance and
  canonical paired-copy paths remain unchanged; saving does not execute model
  weights and file integrity remains labelled file_sha256_only. Preserve old
  artifacts, with no conversion reader. Camera/platform/bin schemas unchanged.
- Shared unapplied UI state advances to schema 6 with item profile filename,
  explicit preview prefix and station/bin filenames. Explicitly validated and
  rewrote this workstation's schema-5 prefill once, preserving all existing
  platform/bin selections and timestamps. Runtime has no older-schema reader.
  Never persist images, candidates, automatic model loading or arming in state.
- Vendored exact Ultralytics 8.4.150, NumPy 1.26.4, cloudpickle 3.1.1, Polars
  1.35.2 (+ matching runtime), THOP 2.1.6 and nvidia-ml-py 12.560.30 wheels,
  reusing OpenCV 4.10.0.84. SHA-256 lock and package NOTICE preserve upstream
  attribution/licenses (including AGPL dependencies). Build verifies/extracts
  offline into package-private prefix, no global install. Worker alone imports
  native modules; fixed CPU, four Torch threads, one OpenCV thread, OpenCL off,
  network/autoinstall blocked. Exact separately provisioned Torch/torchvision
  and system dependencies are checked against the lock; their full dependency
  closure is not vendored, so no offline deployment milestone is claimed.
- Verification complete: 96 tests passed (90 perception, 6 controller), both
  directly and through package colcon tests. Synthetic geometry covers mask
  and OBB selection, fixed-plane sizes at different depths, center-ray XYZ,
  center-first ranking, ROI/class/size rejection, MAD-zero/outlier/null behavior
  and exact black/red depth pixels. A real isolated-domain ROS service test
  covers fresh synthetic streams, timestamped TF, delayed TF with an unchanged
  selected pair, typed shortage responses, disarm and re-arm. Controller tests
  verify read-only requests and frame/freshness rejection.
- Native tests load tiny synthetic untrained detection/segmentation checkpoints
  (no training or production model), inspect actual class metadata, run repeated
  RGB inference and the packed RGB+registered-depth end-to-end worker path.
  Forced SIGSEGV, timeout, corrupt weights and closed-worker tests confirm
  terminal behavior without a replacement PID. Offline extraction tests cover
  missing wheels, checksum tampering and path traversal; all eight real wheel
  SHA-256 checks passed. Runtime Python compilation/flake8 and git diff --check
  passed. No operator-provided production model or physical device was tested.
- Clean ROS-Humble-only root build with forced CMake reconfiguration passed all
  14 packages. Installed interface imports and schema versions (item 3/UI 6)
  were verified; importing the installed GUI/detector does not import cv2,
  Torch or Ultralytics into the parent. No physical cameras, RViz, bringup or
  robot commands were launched. Separately provisioned dependency closure and
  offline transfer validation remain outside this milestone; no deployment
  completeness claim. No commit/push or unrelated vendor change was made.

### 2026-09-13 — All detections first, clickable plane measurements, then production filters

- User clarification: first establish that the model sees items, click a visible
  item to learn its dimensions, then enter dimensions/tolerance and test filters.
  Added explicit All detections / measure (initial stage) and Filtered detections
  / pick checks. This supersedes applying production fields to every preview,
  not the strict production service contract. All mode deliberately uses all
  verified classes and the visible fixed profile confidence .25, IoU .70,
  image_size 640, cap 100. These inference bounds still apply; it is not every
  raw network proposal and is not a missing-production-setting fallback.
- Blank/invalid production fields do not enter the All-mode parser. Fresh RGB
  alone is sufficient to see detections; no depth, platform/bin, home or saved
  profile is required. Changed GUI labels distinguish production controls and
  0–1 confidence/IoU units. Filtered/Save/Armed use field-specific missing or
  invalid-number errors instead of unlabelled float('')/int('') failures.
- Optional measurements use applied hash-validated station/camera calibration,
  matching color CameraInfo and camera-internal TF at that RGB timestamp.
  On-hand additionally needs RGB-time base_link<-Link6 no older than one second.
  No depth, ROI, class or taught-size filter participates in this measurement.
  Extracted one shared metric rectangle calculation for both teaching and
  production: project mask's minimum-area pixel rectangle or the selected OBB
  to platform Z=0, then measure its metric enclosing rectangle. X/height is
  the long side, Y/width the short side, displayed in mm, independent of depth.
  These are reference-plane dimensions, not height-corrected physical sizes.
  Load Model selects the sole verified mask/OBB output; both outputs require
  operator choice. Box-only models remain preview-only without substitute
  metric geometry. Missing projection inputs retain detections with an explicit
  unavailable reason, not cached/guessed measurements.
- Clicking uses the exact displayed image's scaling and centered letterbox.
  Margin clicks do nothing; overlapping hits choose smallest enclosing pixel
  rectangle area, then confidence and frame-local source index. Freeze that
  frame/result, outline the selected rectangle in cyan, and show measured long
  X/height and short Y/width at top-left with frozen-frame age. Resume Live
  releases it. No tracking/index reassignment, automatic field overwrite,
  tolerance inference, image archive or restored selection. One frozen view
  is bounded transient memory, not a capture/production target.
- Editing production fields leaves All mode/measurement available, but disarms
  and invalidates saved-profile eligibility. Edits stop Filtered mode until
  explicitly enabled again. Arming from either view separately validates exact
  saved production settings. Every service request still acquires a fresh
  RGB/depth/TF observation and applies production filters; All detections and
  frozen measurements cannot supply cached targets. Headless behavior, paired
  item artifact schema 3, UI schema 6 and no-command ownership remain unchanged.
- Checked actual workstation state read-only: last_session stores camera prefix,
  station/bin filenames and no selected item profile. Unsaved text fields are
  not autosaved; saved named YAML remains authoritative. Did not change/migrate
  that state or overwrite an artifact. Observed 129 RGB and 132 depth messages
  over five seconds at 848x480 (rgb8/16UC1); the bin camera was streaming. The
  logged toggle failure was empty numeric form parsing, not a dead camera.
- Verification: 103 synthetic tests passed across perception/controller,
  including real private-worker preview transport with detections and no depth
  or production fields; shared mask/OBB plane dimensions; missing measurement
  inputs; click scaling/letterbox/freeze/Resume; blank and invalid production
  boxes; and strict arming. Existing service, MAD/depth, artifact, controller and
  terminal-worker tests passed. Python lint and git diff --check passed.
  Repeated through colcon: 97 perception and 6 controller tests passed. A clean
  ROS-Humble-only root colcon build passed all 14 packages. Installed GUI/backend
  imports confirm the new stages/profile and unchanged schema 3; neither imports
  cv2/Torch/Ultralytics into the parent. Compilation checks passed for package
  Python modules. No physical hardware was launched or commanded; the camera
  check only subscribed to existing streams. No commit or offline-deployment
  milestone is claimed.

### 2026-09-13 — Pick axes instead of item boxes; recovered ROI; no image-size textbox

- User correction: item rendering is for selecting pick poses, not bounding-box
  inspection. Removed YOLO box/OBB outlines, their per-box class/confidence
  labels, mask enclosing-rectangle outlines and the selected cyan rectangle.
  The rectangle remains internal for click hit-testing, dimensions and the
  immutable center-ray pick pixel. Mask shading remains in All mode.
- Draw full centered red X along the pixel rectangle's longer direction and
  green Y along its shorter direction, exactly between opposite edge midpoints.
  Their intersection is the unchanged pick pixel, drawn as a white dot with a
  black rim. Selection highlights only that dot with a cyan ring and still
  freezes the exact displayed result and shows dimensions at top-left.
  All mode displays geometric pick pixels without claiming validated 3D poses;
  Filtered mode draws pick axes only for candidates that pass production gates.
  Depth sampling, platform-plane dimensions, candidate orientation/ranking and
  response validation are unchanged. This supersedes rule 33's rectangle outline.
- Added a green unfilled `Loaded Bin ROI` border in All/Filtered and YOLO-OFF
  views after explicit Apply Station + Bin ROI. It uses the applied destination
  platform/camera/bin files and exact hashes, saved XY at Z=0, RGB-time internal
  TF, intrinsics and lens distortion, and RGB-time robot TF for an on-hand camera.
  Use 32 samples per edge. Clip only to the image boundary; out-of-range or
  behind-camera projection is unavailable, never a guessed/clamped geometry.
- A pure `overlay_roi` request in the same lifetime private worker implements
  YOLO-OFF inspection. It loads no weights, calls no inference and requires no
  depth. Native runtime/transport failures remain terminal without replacement.
  Missing/changed/stale inputs hide the border with a reason; no old projected
  pixels are reused on a new frame. Live views older than 0.5 seconds revert to
  raw RGB, while an explicitly frozen inspection remains labelled historical.
  Changing station selection clears the old overlay/selection immediately.
  Applying a station does not auto-load a model, arm or issue hardware commands.
- Removed only the operator's `image_size` textbox, as clarified by screenshot.
  New profiles use internal 640. Existing strict schema-3 profiles preserve
  their exact validated `yolo.image_size` when loaded/saved/headless; there is no
  missing-key default, silent override, migration or schema change. Selecting a
  new source model starts the new-profile 640 value. Confidence/IoU remain 0–1,
  not percentages. All other production fields/filters remain explicit.
- Updated package/root/quickstart/artifact documentation and AGENTS. Shared UI
  schema 6, named profile authority, no sample/image persistence and bounded
  package logs are unchanged. No vendor changes, camera launches/stops, robot
  commands, global installs or artifact rewrites were performed.
- Verification: 107 synthetic perception/controller tests passed. Added checks
  for rotated-rectangle midpoint axes, long/short direction, white center dot,
  no box-rendering calls, retained mask fill, projected/unfilled green ROI,
  behind-camera/missing/stale ROI hiding, YOLO-OFF real private-worker projection
  without model/depth, absent image-size widget and unchanged loaded input size.
  Existing class/filter/geometry/MAD/pose service/save-load/fatal-worker tests pass.
  Clean ROS-Humble-only root colcon build passed all 14 packages; flake8 and
  git diff --check passed. Colcon tests passed (101 perception + 6 controller).
  Installed imports confirm the ROI-only operation and new overlay helpers;
  compilation checks passed and the ROS/Qt parent still imports no cv2, Torch
  or Ultralytics. Source invalidation clears any prior overlay immediately and
  disarms a live service; unarmed inspection does not repeatedly invalidate its
  own generation while reporting an unavailable projection. Rechecked detector,
  GUI and synthetic service tests after that invalidation refinement. No commit
  or offline deployment milestone is claimed.

### 2026-09-13 — Keep YOLO annotations visible with the bin border

- User correction supersedes the preceding box-removal rule: remove only the
  extra axis-aligned YOLO box. Keep mask shading and one mask-derived minimum-area
  rectangle; for OBB use its native oriented rectangle. Both retain centered
  long-X/short-Y axes and exact-center dot. The bin border belongs on that same
  image. Filtered geometry/axes remain limited to valid pick candidates; All is
  a geometric teaching preview, not a validated pick result.
- The reported ON screenshot had 23 detections but raw RGB and no inference-time
  suffix. The GUI selected the annotated result then replaced it with newer raw
  RGB whenever the result's source stamp was over 0.5 seconds old. Thus CPU
  inference exceeding that interval hid every completed overlay while leaving
  its detection count visible. This was display logic, not evidence of a dead
  camera or a model returning no detections.
- Completed All/Filtered results now remain displayed with their original RGB,
  masks/geometry/axes and ROI until the next result. Their actual source-frame
  age and inference duration stay visible. Over 0.5 seconds the title explicitly
  says RESULT SNAPSHOT, the timer says STALE, and ROI status says historical,
  not a live projection. Never mix old annotated pixels with a newer raw image.
  Raw RGB without a result now says waiting for an annotated result rather than
  claiming the previous count. Keep blocked-preview errors visible alongside a
  retained result. YOLO-OFF live ROI still expires after 0.5 seconds. This display
  exception supersedes the preceding stale-result-to-raw behavior, not input or
  production freshness: pose requests still acquire independently fresh data
  and obey all existing deadline, result-age, source and disarm-generation checks.
- Shared mask shading is retained in Filtered as well as All output. Only one
  selected geometry outline is drawn; no additional YOLO axis-aligned box,
  second selected rectangle or invented OBB mask is introduced. Applying a
  station explicitly reports YOLO OFF and tells the operator to enable it for
  combined overlays; it does not execute a model or arm automatically.
- Updated AGENTS, root/package READMEs and quickstart. No artifact schema,
  inference thresholds/runtime, transform/depth mathematics, camera settings,
  image persistence or hardware behavior changes. No vendor patches or global
  installs. Verification uses synthetic geometry, mocked GUI inputs and the
  existing private worker only.
- Verification completed: 110 perception/controller tests pass, including slow
  All/Filtered results retaining their annotated source image and historical ROI,
  visible blocked-preview errors, replacement by a newer result, no stale count
  on raw RGB, and unchanged YOLO-OFF expiry. Synthetic mask/OBB tests verify a
  single selected outline, mask fill, centered axes/dot and simultaneous green
  bin border in All/Filtered; OBB receives no fabricated mask or extra box.
  Existing service tests still reject expired results and disarmed requests and
  acquire new observations rather than returning cached preview poses.
  Clean Humble-only root build passed all 14 packages; colcon package tests passed
  104 perception + 6 controller tests. Compilation, changed-file flake8 and
  git diff --check passed. Installed imports confirm the new snapshot/geometry
  code and no cv2/Torch/Ultralytics import in the ROS/Qt parent. No physical camera
  or robot process was launched, stopped or commanded. The operator must close
  and relaunch the existing GUI to load the rebuilt code. No commit or offline
  transfer milestone is claimed.

### 2026-09-13 — Automatic Item Teach station/bin preview and ROI shrink audit

- Removed Item Teach's Apply Station + Bin ROI button per user request. Selecting
  both named platform/bin files validates them once, resolves the bound camera
  and automatically subscribes for the read-only preview. Restored valid station/
  bin paths do this on startup too, as a narrow superseding exception to unapplied
  form-prefill rules for this binding only. No camera process is launched and no
  model is loaded/executed, YOLO enabled or service armed automatically.
- Valid RGB/CameraInfo/required TF arriving later automatically produces the
  green border via the existing ROI-only private-worker operation. File loading
  is not repeated per timer tick. Missing/invalid files show a nonmodal reason,
  preserve no old active binding/overlay, and do not silently adopt another file.
  Explicitly reselecting the same corrected path revalidates it. Changes disarm,
  stop YOLO and clear old/frozen geometry before validation. Source-hash, stream,
  transform and production-service freshness checks remain unchanged. Strict UI
  schema 6 stores validated paths/prefix while other sections and named profiles
  remain authoritative. Platform/bin teaching and headless startup are unchanged.
- Separately audited the user's newly saved bin ROI showing a full yellow border
  during capture but a small green border on load in both Bin Teach and Item
  Teach. The new schema-3 file records 30 mm markers and approximately 310 by
  350 mm saved XY geometry. Capture's yellow border uses detected pixel corners;
  stored XY instead comes from marker-size-dependent PnP corners transformed into
  platform coordinates. Loading necessarily projects saved XY at platform Z=0,
  dropping the original marker Z under the existing 2D artifact contract.
- One bounded read-only RGB/CameraInfo/depth observation and the existing private
  ArUco worker reproduced the mismatch: 848x480, fx about 461.2 pixels; marker
  sides about 32–34 pixels. At configured 30 mm, PnP gave 410–430 mm camera depth
  and platform Z about -417 to -457 mm; registered depth near the marker centers
  instead read 795–811 mm. A representative detected outside corner (206,54)
  projected to approximately (313,131) after substituting platform Z=0. Thus this
  is a metric-depth/plane inconsistency, not an Item Teach-only display scaling
  issue. Requested the physical black-square side length, excluding white border,
  to verify the leading marker-size mismatch hypothesis. No exact replacement
  size is assumed; depth is diagnostic evidence only, not a new teaching input.
  No calibration/teach files or bin/platform geometry behavior were modified.
  Images/depth stayed transient in memory; no hardware processes were launched,
  stopped or commanded. A short-lived subscriber/one-shot audit worker exited
  normally; neither replaced an application's lifetime worker.
- Verification completed: 116 synthetic perception/controller tests pass.
  New GUI cases cover removal of Apply, waiting for both paths, automatic
  restored binding, no model/arming activation, delayed stream arrival, one-time
  validation rather than timer reload, invalid/missing/hash-mismatched artifacts,
  explicit same-path reselection and clearing the old overlay on changes.
  Clean Humble-only root build passed all 14 packages; changed-file flake8,
  compilation and git diff --check passed. Installed-module inspection confirms
  the button is absent and automatic preview loading is present, with no native
  ML/OpenCV import in the GUI parent. Existing application processes were not
  restarted; relaunch Item Teach to load the rebuilt GUI. No offline-transfer
  milestone or commit is claimed.

### 2026-09-13 — One platform-plane geometry for Bin Teach capture and load

- User approved fixing planar ROI teaching/loading without flattening the
  platform relative to the robot base. This supersedes rule 23's marker-PnP
  XYZ-to-XY conversion only; the taught platform transform retains all rotation
  and translation, including tilt. Platform teaching itself is unchanged.
- Bin Teach now undistorts all 16 detected marker image corners in the existing
  private OpenCV worker, transforms those rays into platform_reference and
  intersects platform Z=0. Outside-corner selection and clockwise P1–P4 ordering
  use those plane intersections. IPPE_SQUARE marker poses/axes and measured
  marker size remain, but PnP XYZ is no longer an ROI input. No depth, averaging,
  alternate geometry, calibration flattening or native runtime fallback added.
- Non-finite, parallel and non-forward intersections block that frame. The new
  checked image_rays protocol keeps cv2 exclusively in the lifetime child;
  runtime verification also requires undistortPoints. Native/protocol/worker
  failures still terminate through the existing fatal path without replacement.
- Yellow fresh teaching and green loaded borders use the same saved-plane
  geometry and existing distorted 32-samples-per-edge projection. Readiness
  events include the method and per-corner pixel round-trip differences; capture
  events name the method. These are consistency diagnostics, not independent
  metric accuracy. Physical corners must lie on the taught plane and camera/
  platform calibration must be correct for portable metric geometry.
- Preserve schema-3 platform/bin files, schema-7 camera files and schema-6 UI
  state. No file rewriting, compatibility reader or automatic geometry repair.
  Existing bin files still load their original XY; explicitly re-capture/save
  to obtain the corrected geometry. Teach/RViz/save/load contracts and canonical
  directories remain. No item-detector behavior or vendor source changed.
- Updated root/package/camera READMEs, bin teaching directory instructions,
  AGENTS and visible Bin Teach guidance. Verification uses synthetic geometry,
  mocked ROS inputs and the existing private runtime only; no hardware actions.
- Verification completed: 192 package tests pass (131 perception, 55 calibration,
  6 controller). New cases cover both camera modes, tilted source/destination
  frames, plumb-bob/rational distortion, schema-3 save/load and sub-0.0001-pixel
  corner round trips, matching fresh/loaded borders despite deliberately wrong
  PnP depths, invalid rays/planes, missing undistort API, malformed native output,
  terminal worker failure and Retake discarding in-flight geometry. The initial
  combined ad-hoc pytest command lacked the calibration test's private-runtime
  environment; the final package-specific colcon test run uses its configured
  environment and passes all three suites.
- Clean Humble-only root colcon build passed all 14 packages. Changed-file
  flake8, Python compilation and git diff --check passed. Installed imports
  confirm planar-ray teaching with no cv2/Torch/Ultralytics in the ROS/Qt parent.
  Calibration/teach artifacts and last-session state were not rewritten. No
  hardware was launched, stopped or commanded; relaunch Bin Teach to use the
  rebuilt implementation. No commit or offline-transfer milestone is claimed.

### 2026-09-14 — Item Teach shares portable Bin Teach edge geometry

- User requested the same loaded-bin edge logic in Item Teach after confirming
  the newly re-taught planar ROI. Audit found Item Teach already loads strict
  portable bin XY onto the destination's calibrated platform plane and shows
  it automatically, but independently duplicated the edge sampling/projection
  geometry. No missing platform flattening or source-station placement is needed.
- Extracted one native-import-free planar border helper shared by Bin Teach's
  fresh/loaded views and Item Teach's renderer. It constructs exactly 32 samples
  per saved XY edge at platform Z=0 and converts them with the full current
  platform-from-optical transform. NumPy is supplied by each caller, so neither
  ROS nor a native runtime is imported by the helper. Lens-distorted projection
  remains inside each existing private lifetime worker with no extra process,
  restart, fallback, global installation or vendor patch.
- Preserve the current automatic platform/bin selection and camera subscription
  workflow, green unfilled Loaded Bin ROI alongside masks/rectangles/axes, and
  YOLO-OFF inspection without a model, depth or markers. Destination tilt/height
  remain unchanged; source station transform/IP/camera are teaching provenance,
  never destination placement. Invalid/stale/hash-changed inputs still hide the
  border, and completed inference snapshots remain age-labelled historical views,
  never fresh service targets. The shared headless renderer follows the same
  geometry without changing candidate, depth, ranking or service policy.
- No schema change (camera 7, platform/bin/item 3, UI 6), file/state rewrite,
  automatic newest-file selection, re-teaching, resizing or plane flattening.
  Select the corrected new bin YAML explicitly. Across stations reproduce the
  same physical origin/axis directions, bin size/offset and coplanarity.
- Added destination-loading tests for both camera modes using a copied schema-3
  template after deleting only synthetic source-station files. Assert original
  XY, full tilted destination transform, timestamped internal/robot TF and no
  model/depth requirements; reject wrong camera, missing CameraInfo/TF, stale
  on-hand TF and changed sources. Private-runtime geometry tests compare both
  GUI border projections with zero/rational distortion at two tilted placements,
  verify metric round trips and keep item annotations intact. No physical
  hardware was launched, stopped or commanded.
- Verification completed: 200 package tests pass (139 perception, 55 camera
  calibration, 6 controller), including private-worker inference/ROI-only paths
  and existing stale-source, snapshot, service and artifact regressions. Clean
  Humble-only root colcon build passed all 14 packages. Python compilation,
  changed-file flake8 and git diff --check passed. Installed import inspection
  confirms both GUIs use the same border helper and their ROS/Qt parent imports
  no cv2/Torch/Ultralytics. Existing operator artifacts/UI state were untouched.
  Relaunch Item Teach and select the destination platform plus the intended bin
  file; no new Apply step or model activation is required for the border.
  No commit or offline-deployment milestone is claimed.

### 2026-09-14 — Model loading takes priority over automatic Item Teach previews

- Confirmed the reported loop in package events: repeated `Model load busy`
  errors occurred while YOLO-OFF bin projections occupied the shared GUI job
  slot. Each completed preview immediately scheduled another frame, including
  while the modal trust dialog ran its nested Qt event loop. This is GUI job
  starvation, not evidence of an invalid checkpoint or dead camera.
- Reserve the model-load slot before showing the trust question, preventing
  new automatic preview jobs. After confirmation, finish the current operation
  and dispatch exactly one queued model inspection in the same lifetime worker.
  Keep the GUI responsive and show queued/loading progress. Duplicate clicks
  cannot queue additional loads. YOLO/Armed controls remain disabled during the
  reservation and OFF after an accepted load; successful loading then permits
  explicit YOLO activation and resumes the automatic bin preview.
- Declining trust resumes previews without loading. A changed selection cancels
  a queued load; changed selection during loading discards its result. Closing
  cancels queued work. A terminal native failure still exits without dispatching
  the queued load, restarting/replacing the worker or retrying an operation.
- No artifact/UI schema, file, native runtime, inference setting, geometric
  calculation or headless/service-policy change. The separately audited late
  Armed-response race is not part of this model-load scheduling correction.
- Verification: 147 perception tests and 6 controller tests passed, including
  seven GUI scheduling/cancellation cases and one real private-worker ROI ->
  inspect -> inference test retaining the same PID. Both queued ROI and YOLO
  jobs, modal-dialog timer reentrancy, duplicate clicks, cancellation, changed
  selections, shutdown and terminal failure are covered. Changed-file flake8,
  Python compilation and git diff --check passed. Root colcon build from a
  Humble-only environment passed all 14 packages; installed GUI source matches
  the fix and imports successfully with compiled message/service support and
  no cv2/Torch/Ultralytics in the parent. Tests use mocked streams plus a trusted
  synthetic model; no operator model, hardware node or robot command is executed.
  Relaunch Item Teach to use the rebuilt code. Existing artifacts and UI state
  remain untouched; no commit or offline-transfer milestone is claimed.

### 2026-09-14 — One teaching view, live settings and clicked RGB/depth pose TF

- User supersedes the fixed All-mode inference profile: the GUI preview now
  use the visible confidence/IoU/max_detections. First-run fields explicitly
  show 0.25/0.70/100; these are not fallbacks for missing/invalid values. New
  profiles retain internal image_size 640; loading retains the artifact's size.
  The user changed the interim two-toggle design before it was built: remove
  both Detect All/Filtered modes, the dropdown and Resume Live button. There is
  one detection view with explicit YOLO ON/OFF and Armed. A mid-implementation
  audit caught the incomplete GUI/backend signature and missing click/TF wiring;
  that intermediate state was not a completed or deployed feature.
- Display all model classes including size failures. Measure the same pixel
  rectangle on platform Z=0: green within length/width tolerance, red outside,
  gray when size fields/plane measurement are unavailable. Green checks size
  only, not full pose validity. Preserve mask shading, one rectangle, axes/dot
  and simultaneous green loaded-bin border. Registered depth is shown with RGB;
  missing/mismatched depth leaves RGB visible but cannot supply a pose.
- Clicking freezes the exact displayed detection/RGB/depth/TF observation and
  calculates only that item in the same lifetime worker, without another YOLO
  prediction or newer sensor substitutions. Apply existing checked-class, size,
  ROI footprint/center, range and MAD depth checks. Require freshness/sync when
  acquiring the pair and result_max_age_sec at click and completion. Show
  dimensions and platform-relative XYZ/yaw, with accepted depth black/rejected
  red inside the circle. Rejected/expired/busy clicks show a reason and no TF;
  there is no automatic retry. Home/saved artifact are not required for manual
  inspection, but filled valid pick geometry/quality/classes are required for poses.
- A valid click publishes frozen teaching-only base_link -> item_teach_selected_item
  at 10 Hz using the platform's full transform. This avoids a second
  platform_reference publisher and does not flatten the platform or launch RViz.
  The frame is an age-labelled snapshot, not live tracking or a robot target.
  Click the image again to resume/clear it; settings, station/model/camera,
  arming, YOLO OFF, source/native failure and exit also stop publication. TF
  consumers can retain past transforms briefly after publication stops.
- Numerical YOLO fields, class, geometry, dimensions and quality edits immediately
  disarm, invalidate saved-profile eligibility, clear old/frozen detections and
  pause an enabled preview. After 300 ms of no further edits, validate and apply
  the latest values without a toggle cycle or model reload. Invalid fields stay
  paused with a nonmodal explanation until edited correctly; never use the old
  values, clamp inputs or retry a failed native operation. Already queued and
  in-flight old-setting previews are discarded by a GUI revision check plus the
  backend's existing epoch check. Incomplete dimensions explicitly mean unchecked
  size, never old/guessed dimensions. Other form edits clear selection/TF and
  disarm without stopping detection. Headless still uses explicit saved data.
- No automatic saving/arming, new production-service behavior, hardware action, schema
  change, model/runtime replacement, vendor patch or operator-artifact rewrite.
  Source/settings validation, model-load priority, no image archives and the
  existing neutral platform-plane geometry are preserved.
- Verification complete: 170 perception and 6 controller tests pass. Tests cover
  the single-view controls, live settings/debounce and invalid fields, unchanged
  model-load priority, exact displayed selection, rejected/expired/cancelled
  clicks, malformed native identity, no TF on rejection, platform tilt/pose
  composition and TF invalidation. Private-worker tests verify simultaneous RGB
  and depth preview without automatic candidate generation, known selected XYZ,
  black/red depth pixels, ordinary depth rejection in the same child, green/red
  size colors, and existing artifact/service/bin/platform regressions.
  Python compilation, changed-runtime-file flake8 and git diff --check pass.
  Package build and clean Humble-only root colcon build pass (all 14 packages).
  Installed GUI/backend/geometry/native modules match source, import checks
  exclude cv2/Torch/Ultralytics from the parent, and the installed GUI has none
  of the removed controls. Tests use synthetic inputs and mocked feedback;
  no operator model or camera/robot/RViz hardware launch was executed. Existing
  calibration/item artifacts and UI state remain untouched. No commit or new
  offline-transfer milestone is claimed.

### 2026-09-14 — Selection ring follows the depth-sampling diameter

- Replace Item Teach's fixed 9-pixel cyan selection ring with the projected
  physical sampling circle. Preserve `pickdepth_radius` as diameter in mm;
  default 30 mm is a 15 mm radius, not a renamed/doubled interpretation.
- Extract one native 96-point circle construction shared by depth sampling and
  selection visualization, centered at the exact rectangle-center ray's
  platform-Z=0 intersection. Apply the same camera transform, lens distortion
  and perspective, so a tilted view can show an ellipse. The parent receives
  validated pixels and draws the outline with its RGB image's scaling/letterboxing;
  no cv2 import, guessed intrinsics or rectangle-size approximation.
- Include the outline in the frame-local preview independently of size/depth
  pose eligibility, so it can be inspected with blank size fields or a rejected
  pose. Missing/invalid projection hides the ring with an explicit reason, not
  a fixed-pixel fallback. Diameter edits keep existing clear-selection/live
  update semantics: click an item again to inspect the updated size.
- No artifact/UI schema changes, operator-file rewrites, TF/service-policy
  changes, altered sampling acceptance/MAD or camera/robot/RViz launch.
- Verification complete: 175 perception and 6 controller tests pass. New checks
  cover 30/60 mm diameter scaling, mask/OBB centers, exact agreement with the
  depth-sampling projection, tilted/distorted ray-plane round trips, cyan
  outline rendering without the old 9-pixel ring, unavailable projection,
  invalid diameter edits and malformed projection metadata. Python compilation,
  changed-runtime-file flake8 and git diff --check pass. Root colcon build
  passes all 14 packages; installed modules match source and the GUI parent
  imports no cv2/Torch/Ultralytics. No hardware launches, operator-model loading,
  artifact/state rewrites, commit or offline-transfer milestone.

### 2026-09-14 — Visual-first Item Teach and registered-depth pixel models

- Operator requested one vertical settings column and emphasized overlays as
  the main teaching feedback. Reorganized the sidebar into camera/station,
  item/model, live YOLO, dimensions/depth, quality, then home/motion/gripper/retry.
  Keep all fields/actions; Load/Save remain visible in the header. Most of the
  window is allocated to labelled, resizable RGB/depth panes. The optional
  bounded activity log starts collapsed, with latest feedback still visible.
  No auto-enabling inference/arming, unsaved-field persistence or schema change.
- Read-only inspection of the already-running bin_camera CameraInfo topics
  found both at 848x480 with exactly matching K (fx 461.21518, fy 461.19028),
  but nonzero RGB distortion and all-zero registered-depth distortion. This
  matches the Orbbec SDK's documented Gemini 330-series D2C behavior:
  https://github.com/orbbec/OrbbecSDK_v2/blob/main/docs/tutorial/orbbec_camera_distortion.md
  The application incorrectly required whole CameraInfo dictionaries to match.
  It was rejecting streamed depth, not observing a dead camera.
- Retain strict optical-frame, dimensions and K equality, plus all existing
  input/freshness/sync checks. Preserve both validated camera models in each
  selected observation. In the same private worker, project the physical
  platform-plane circle separately into RGB and depth; intersect native depth
  rays with the plane for circle membership and project those rays into RGB
  for mask membership. Each original depth pixel is counted once, without
  interpolation/resizing. Depth overlays stay in native pixel coordinates.
  The unchanged RGB center, RGB distortion and accepted median depth still
  define the pick point. No guessed/copy-over distortion or missing-model
  fallback. A genuinely different K or frame remains invalid.
- The shared clicked-pose/headless path uses the same corrected geometry;
  filters, MAD, sample-count limits, TF/service rules and image storage remain
  unchanged. Camera calibration, vendor code, camera settings, existing teach
  files and UI state were not modified. No hardware was launched or commanded.
- Verification: 177 perception tests, 55 calibration tests, 6 controller tests,
  15 RViz-monitor tests and 21 mocked camera-launcher tests pass. Motion/gripper
  packages have no discovered tests. Changed Python compilation/flake8 and both
  working/index git diff checks pass; clean Humble-only root colcon build passes
  all 14 packages. Offscreen layout inspection confirms one column, larger
  image panes, persistent Save and optional log; no physical GUI was launched.
  Native tests exercise deliberately different RGB/depth distortion, correct
  native sample counts/colors, unchanged depth bytes and original-center XYZ.
  A read-only subscriber accepted a fresh pair from the existing bin camera
  with actual CameraInfo under default age/sync limits. Installed source
  matches the changes; the parent imports no cv2/Torch/Ultralytics. Relaunch
  Item Teach to load the rebuilt code. No image/archive or operator-file writes.

### 2026-09-14 — Requested source-control checkpoint

- User explicitly requested committing/pushing the previously local changes.
  The prior tracked checkpoint predates the calibration/teaching applications,
  camera supervisor and approved physical-only vendor cleanup. Stage these
  together with current Item Teach fixes, documentation/tests and checksum-bound
  private runtime wheels as one coherent source checkpoint; preserve vendor
  notices and the documented reference-only item_pick/COLCON_IGNORE boundary.
- Exclude ignored configuration/logs/build/install output, station calibration
  YAML, bin/item teaching YAML and operator .pt weights. Include teaching-folder
  READMEs only; source control is not a backup of local station/model artifacts.
  Remove the old workstation-specific absolute path from this diary before
  publication. No shared-history rewrite, force push or automatic Git policy
  installation. No complete offline-deployment milestone is claimed.

### 2026-09-14 — Explicit Item Teach load also loads its paired model

- User requested automatic loading of the paired .pt when explicitly loading
  an item teach file. Combine the existing replacement and model-trust questions
  into one confirmation, then queue the exact SHA-256-bound same-stem model in
  the existing lifetime worker. No second Load Model click. The direct standalone
  model workflow retains its separate trust confirmation.
- Reuse reserved next-slot scheduling to avoid bin-preview starvation; disable
  overlapping model/profile loads. Preserve saved settings, home, class checks,
  geometry and inference size. YOLO and Armed stay OFF; no movement, camera
  launch, native-worker restart, fallback model or automatic retry.
- Validate the pair before queueing, recheck its YAML/model hash before native
  load and at completion, and pass the expected model hash into inspection.
  Require saved task, class IDs and geometry support to match actual metadata.
  Missing/changed/incompatible pairs fail visibly and cannot enable poses.
  Ordinary form edits still invalidate saved-profile eligibility.
- Startup restoration continues as unapplied prefill only and never executes
  the .pt. Artifact/UI schemas and paths are unchanged; operator files are not
  rewritten. This supersedes only the extra manual-model-load step after an
  explicit profile Load, not the startup/model-execution safety contract.
- Verification: 189 perception and 6 controller tests pass, including combined
  confirmation/automatic paired loading, preserved profile values/classes,
  queued/busy/duplicate-load handling, cancellation, missing/tampered pairs,
  expected-hash preflight and metadata incompatibility. Startup prefill remains
  weight-free. Python compilation, changed-runtime flake8 and git diff --check
  pass; clean Humble-only root colcon build passes all 14 packages. Installed
  modules match source and parent imports exclude cv2/Torch/Ultralytics.
  Tests use synthetic files/models and mocked feedback only; no operator model
  or camera/robot/RViz was launched or commanded. Existing artifacts/UI state
  remain unchanged. Source-only follow-up commit/push follows the user's request.

### 2026-09-14 — Horizontal paired views and simplified teaching windows

- User requested horizontal RGB/depth, matching overlays, removal of duplicated
  above-video help, and the same simplification for Platform/Bin Teach. On
  clarification the user explicitly chose to FREEZE the clicked RGB/depth pair,
  not keep views live. Preserve exact-snapshot pose calculation and second-click
  resume; do not substitute current depth or run another prediction for a click.
- Item Teach now uses equal horizontal panes. The same private worker maps RGB
  mask shading, size border, long-X/short-Y axes and exact dot into the registered
  depth model. Sample rectangle/axis edges to retain distortion geometry; project
  bin XY with the depth model and full platform transform. Valid color/depth
  CameraInfo alone enables pixel overlays when no station is applied, without
  inventing metric geometry. No depth resampling or changed candidate math.
- Frozen depth retains all displayed item geometry while calculating only the
  clicked pose. The cyan diameter-based circle maps into each view; accepted
  pixels remain black/rejected red. Both views show selected dimensions/pose
  and source age. Removed redundant above-video paragraph; current inference
  settings are now also on-image. Existing size/class/ROI/freshness/MAD gates,
  frame-local identity, TF publication and service separation remain strict.
- Platform/Bin Teach share a compact scrollable setup and larger RGB area,
  persistent Save/capture actions, concise physical guidance and expandable
  calibration/status/output details. Blocked capture remains visible in the
  image/waiting view and compact status. Apply/capture/load/retake/save and TF
  behavior are unchanged; no artifacts/UI-state rewrite, vendor edits, new
  persistence, worker replacement, camera/robot/RViz launch or motion.
- Verification: 191 perception and 6 controller tests pass. Offscreen layout
  tests/inspection confirm horizontal item panes, removed help paragraph,
  large platform/bin video, expandable details and persistent gated actions.
  Synthetic native tests check differing-distortion border colors, projected
  centers/circles and retained non-selected outlines during a selected-pose
  operation; GUI tests confirm neither frozen image is replaced by newer data.
  An existing mock candidate was extended with actual production depth-count
  fields to test the new overlay counters. Final package tests, compilation,
  seven changed-module flake8 checks and git diff --check pass. Clean Humble-only
  root colcon build passes all 14 packages; installed GUIs/shared layout import
  without cv2/Torch/Ultralytics in the parent. No operator models, physical
  cameras, robot, or RViz were launched or commanded. Artifacts/state remain
  unchanged. Source-only commit/push follows the user's request.

### 2026-09-14 — Standing tested-change commit and push workflow

- User explicitly confirmed committing and pushing after each completed, tested
  change from now on. Record the authorization in rule 8 and the development
  workflow so it survives contributor/agent handoffs; no repeated prompt or
  automatic Git hook is needed. This supersedes the earlier checkpoint's lack
  of a standing Git policy, not its source-only artifact exclusions.
- Check relevant tests, status, whitespace and the staged diff before committing
  scoped source/tests/docs, then push normally and verify the remote commit.
  Never bundle unrelated edits or local station/model artifacts, force-push,
  rewrite shared history, or claim a blocked validation/push succeeded.
- Documentation-only change: review the three-document diff and run
  `git diff --check` plus staged-diff checks. No runtime changes, hardware
  launches, artifact rewrites or new offline-transfer milestone.

### 2026-09-14 — Item Teach feedback moves into the top black pane bands

- User requested moving the text overlay off the image into the available black
  area above it. Each RGB/depth pane now has a wrapping plain-text black status
  band immediately below its heading. Result/frozen status, live ages, inference
  settings and selected dimensions/pose/counts retain their existing content but
  no longer cover camera pixels or shrink with the source image. Unavailable
  views clear their obsolete bands. This supersedes rule 43's on-image text
  placement only; geometry overlays remain on their respective images.
- Keep image-only click mapping, letterbox rejection, frozen paired observations,
  cyan circles, masks/axes/borders and black/red depth samples unchanged. No
  inference/pose math, schemas, settings persistence, source artifacts, native
  workers or hardware behavior change. No cameras, robot, models or RViz launched.
- Verification: 193 perception tests and 6 controller tests pass. Offscreen
  checks at two window sizes verify top-band placement, wrapping, unchanged
  displayed pixels, centered-image click mapping, ignored header/margin clicks,
  selected pose/count feedback and clearing missing-view status. Inspected both
  live-result and frozen-selection layouts with synthetic images. Compilation,
  changed-runtime flake8 and git diff --check pass; package build and clean
  Humble-only root build pass all 14 packages. Installed GUI matches source and
  imports no cv2/Torch/Ultralytics in the parent. Operator artifacts/configuration
  remain unchanged; no hardware launched. Scoped source commit/push follows
  the standing user authorization.

### 2026-09-14 — Pose candidate count and recoverable Item Teach drafts

- User renamed `retry_limit` to `pose_candidates`: the maximum ranked pose count
  requested from the detector for the controller to use for retries. Keep the
  existing `retry` YAML group; this field no longer claims to implement robot
  retry execution. The detector bounds `max_candidates` against it and the
  read-only controller requests exactly it. Keep 1–1000 and no greater than
  `yolo.max_detections`; actual returned count may be lower or zero.
- Advance strict production item schema to 4. GUI Save, headless detection and
  controller use only `retry.pose_candidates`, with no production old-name alias.
  Platform/bin schema 3, camera schema 7 and shared UI schema 6 are unchanged.
- User initially approved one-time updated copies, then superseded that plan:
  let old/corrupt files load for editing, leaving unclear/empty variables blank.
  Implement a GUI-only recovery draft, not a runtime compatibility reader or
  automatic bulk conversion. Keep individually validated known-format fields;
  recover an unambiguous old count only in schemas 1–3. Clear invalid values,
  conflicting pairs and values with unknown units; never retain prior form
  values or invent defaults. Unknown flags use a partial checkbox and must be
  resolved. An invalid home record is cleared as a whole. Malformed/duplicate
  YAML or unknown formats produce an empty draft with reasons.
- Verify the exact same-stem .pt SHA-256 independently before offering/queueing
  recovered model loading, then recheck YAML/model at queue execution/completion.
  Keep explicit trust, worker isolation and no automatic YOLO/arming. Missing
  or tampered pairs clear the model, not silently establish a new hash. Actual
  verified model metadata governs available task/classes/geometry for drafts.
  Named-file startup recovery is prefill only, without model execution.
- Recovery is visibly labelled, logs what was cleared and cannot set saved-profile
  eligibility. Operator corrections followed by explicit Save produce a new
  validated schema-4 YAML/.pt pair without overwriting originals; only then may
  the operator arm or request controller validation. No source artifacts or
  persisted UI fields are automatically rewritten. Strict shared UI-state and
  production loaders remain unchanged in failure policy. This is the narrow
  user-authorized exception to earlier GUI no-recovery rules, not relaxed
  calibration/bin/platform readers or robot safety.
- Verification: 220 perception tests and 6 controller tests pass. Coverage includes
  exact request counts/caps, strict schema-4 round trips and production old-name
  rejection, old/corrupt startup prefill without execution, validated-field
  recovery, unit/conflict clearing, unknown booleans, ambiguous/malformed YAML,
  model-hash failures and queued changes, and explicit recovered Save into a
  new pair without altering originals. Both existing workstation item files
  were inspected read-only: their count recovers as 3 and paired hashes verify;
  no values or artifacts were rewritten and no operator weights were executed.
  Package builds and clean Humble-only root colcon build pass all 14 packages;
  compilation, five changed-module flake8 and git diff --check pass. Installed
  GUI/recovery/controller use schema 4 and import no cv2/Torch/Ultralytics in
  their parent process. No cameras, RViz, robot or motion were launched. Scoped
  source-only commit/push follows the standing user authorization.

### 2026-09-14 — Audit and implement the missing Simulate Trigger action

- Audit confirmed the preceding source change implemented schema-4
  retry.pose_candidates, strict production/controller counts and GUI-only
  invalid-file recovery, but omitted the requested Simulate Trigger control.
  It was not an install-only problem: no button or simulation handler existed.
- Add the main row YOLO Detect, Simulate Trigger, Armed. Simulation is local,
  one-shot and non-actuating, requires the exact saved/loaded valid profile and
  verified model/current settings plus YOLO ON and station/bin. Armed may stay
  OFF; no temporary service, controller request, batch TF or robot command.
- Extract the real request implementation into a shared batch path. Both paths
  acquire a new synchronized pair after arrival, use its timestamped TF, apply
  identical class/size/ROI/center/depth/MAD filters and center-first ranking,
  and build the same typed response with count limits, evidence and explicit
  shortage/no-items/errors. Keep one request lock and one native operation lock,
  exact worker/runtime, terminal native failure, no replacement/retry/cache.
- Queue one GUI simulation behind existing work, with model-load priority,
  disabled repeat trigger and progress. Include queue time in the saved request
  deadline; bind the queued profile digest and cancel by GUI generation without
  killing native work. Revalidate source/profile and result age at completion.
  Settings/arming/source/model/YOLO changes discard late results; a real service
  remains independent of the frozen teaching display and reports BUSY on overlap.
- Audit found that production drawing annotated every valid candidate before
  response truncation. Add explicit overlay cap to native detection: after
  ranking, render the returned subset from the untouched RGB/depth pair. Keep
  only that subset's masks, one green outline, axes/dots, physical cyan sampling
  rings, black accepted and red rejected depth pixels, P1..Pn labels and bin ROI.
  Keep the complete valid count/rejection diagnostics in the response; excluded
  or excess items leave no overlays. Pose mathematics are unchanged.
- Successful simulation freezes the exact annotated pair including a zero-item
  result, with explicit status and source ages. Top black bands show up to three
  expanded pose details to preserve image space; every returned pose is labelled
  on-image and listed in bounded Activity. Click RGB again to resume/cancel;
  margins/status bands do not act as image clicks. Source/profile changes clear
  even an aged historical batch. No image archive or saved candidate cache.
- Camera/platform/bin geometry, item schema 4, shared UI schema 6, saved operator
  files, calibration and hardware configuration are unchanged. Robot retry
  execution and sole-command-ownership migration remain pending.
- Verification: 254 perception and 6 controller tests pass (colcon also counts
  one CTest wrapper per package). Added real/simulated response equivalence,
  fresh synthetic ROS streams/TF with Armed OFF, exact caps/priorities, BUSY,
  queued-profile mutation/deadline, cancellation, empty/shortage, stale/source/
  native failures and no replacement/old-result display. GUI tests verify the
  actual three-control row, next-slot scheduling, frozen RGB/depth, image-only
  resume, source-change clearing, and bounded top-band details. Private native
  tests verify mask/OBB subset-only overlays on both images, unchanged ranked
  poses and null/outlier exclusion before MAD, plus the real worker's cap
  protocol in its same lifetime process. Existing recovery/schema/count/model,
  selected-pose, platform/bin and controller regressions remain green.
- Package builds and a clean Humble-only root build pass all 14 packages.
  Python compilation, four changed-runtime-module flake8 and git diff --check
  pass. Installed offscreen GUI inspection confirms the visible three-button
  row and frozen paired feedback without importing cv2/Torch/Ultralytics in
  the parent. No operator model, camera, robot or RViz was launched/commanded;
  existing operator artifacts/configuration were not edited. Scoped source
  commit/push follows rule 8; no new offline-deployment milestone is claimed.

### 2026-09-14 — Warn when Item Teach selects a different platform for a bin ROI

- User requested a warning when the selected platform differs from the platform
  recorded in a bin teach. Existing schema-3 bin YAML already saves the source
  platform filename/SHA-256 and source transform under teaching_provenance.
  Expose that validated hash in the loaded artifact; no schema or writer change.
- Item Teach compares the full source/destination platform hashes after automatic
  selected/restored station validation. Show a persistent amber, plain-text,
  nonmodal notice directly below the file selectors with both filenames and full
  hashes in its tooltip. Record one bounded WARNING event per mismatched binding,
  not per frame/timer, and keep the notice separate from transient video feedback.
  Matching selection, incomplete/invalid selection or camera unbinding clears it.
- Preserve intentional portability: the warning does not reject a destination,
  alter XY, use the original transform, require absent source artifacts, change
  strict destination validation, or enable model execution/arming. Identical file
  content at another path is not a mismatch; a changed hash with the same name
  is. Hash identity is not physical alignment. Explicitly tell the operator to
  check the common origin, X/Y directions, bin size and placement before reuse.
- Scope is Item Teach feedback and the native-import-free bin reader/helper;
  platform/bin teaching geometry, headless service behavior, schema versions,
  operator artifacts/configuration and existing Bin Teach load confirmation stay
  unchanged. Tests use synthetic artifacts and offscreen Qt only; no cameras,
  robot, RViz or operator models are launched/executed. No offline milestone.
- Verification: 263 perception and 6 controller tests pass, including same-name
  changed hashes, same-content copies, portable geometry without source files,
  restored selections, non-blocking ROI-only rendering, one warning per binding,
  and clearing on matching/incomplete/invalid selections or camera unbinding.
  Clean Humble-only root build passes all 14 packages; changed-module compilation,
  flake8 and git diff --check pass. Installed offscreen inspection verifies the
  amber notice and preserves the image area; long timestamped filenames wrap
  within the sidebar, with exact text/hashes in the tooltip and bounded event.
  Parent imports exclude cv2/Torch/Ultralytics. Read-only inspection confirmed
  this workstation's selected bin/platform currently have matching SHA-256.
  Existing artifacts and saved UI state were not rewritten. Source-only commit
  and verified push follow the standing user authorization.

### 2026-09-14 — Simulate Trigger publishes every returned pose for RViz inspection

- User requested the same RViz feedback as clicking an item, for all simulated
  pose candidates. Supersede rule 45's no-batch-TF restriction only: the shared
  fresh-request acquisition, filters, ranking, caps and production service remain
  unchanged. No controller request, automatic arming, RViz/hardware launch or motion.
- Extend the existing 10 Hz teaching broadcaster to hold either the clicked
  frame or one atomic candidate batch. Use base_link -> item_teach_candidate_1
  through item_teach_candidate_N, matching P1..Pn and all returned poses, not the
  first three expanded overlay details. Compose the exact platform-relative
  response poses with the destination's full platform transform using the same
  helper as a click. No duplicate platform_reference, flattened tilt, standoff
  compensation, robot TCP attitude or subsequent live-TF/vision tracking.
- Before installation, check source/profile identity, epoch, fresh matching
  snapshot stamp, successful platform-frame response, unique candidate IDs,
  ordered priorities, and finite normalized pose geometry. Reject the whole
  preview, not a partial set. Store only transforms and identity evidence in
  broadcaster state; keep the existing single frozen RGB/depth pair in the GUI.
- Shared clearing handles clicked/batch replacement, resume, changes, YOLO OFF,
  failed or empty batches, and exit. The timer also checks native/fatal/epoch and
  source/profile identity independently of Qt progress; it refreshes only TF
  broadcast timestamps while holding frozen geometry. No stale service targets.
  TF/RViz consumers can retain stopped frames until their buffer/display timeout;
  no invalid zero transforms or unrelated-TF clearing is used to hide them.
- Both top bands identify candidate frame names. A bounded package event maps
  batch ID/source timestamp/candidate IDs to frames; image annotations and
  original observation ages remain. Item/platform/bin/camera/UI schemas, saved
  artifacts, vendor code, camera settings and operator model files are unchanged.
- Verification: 285 perception and 6 controller tests pass. New tests cover
  all returned priorities including beyond the three text entries, complete
  platform tilt/height and metre/quaternion composition, exact shared clicked
  geometry, atomic clicked/batch replacement, smaller/empty batches, stale or
  malformed responses, and source/profile/epoch/native invalidation without Qt
  progress. A real ROS TF subscriber receives five then two synthetic frames
  and confirms publication stops on clearing; add tf2_msgs as a test dependency.
  The initial test-only domain 233 exceeded the DDS port range; corrected to
  isolated local domain 231 and reran the full suite successfully.
- Clean Humble-only root build passes all 14 packages; Python compilation,
  changed-runtime/new-test flake8 and git diff --check pass. Installed offscreen
  inspection confirms frame names in both frozen top bands and no cv2/Torch/
  Ultralytics imports in the parent. Existing artifacts/UI state/model weights
  remain untouched; no cameras, robot or RViz launched or commanded. Scoped
  commit and verified push follow rule 8; no offline milestone is claimed.

### 2026-09-14 — Loaded item profiles are saved; Save updates the same named item

- User requested no redundant Save after loading, and overwriting the loaded
  item teach unless its item name changes. Strictly valid explicit loads and
  startup named-file restoration now retain saved eligibility. Startup remains
  weight-free; explicit trusted model loading, YOLO ON, manual Armed and all
  production station/model/freshness checks are unchanged. No automatic trigger,
  controller request, hardware action or teaching-image persistence.
- Separate the loaded document's save target from its edited/saved eligibility.
  Edits still disarm and invalidate pose eligibility, but do not lose the target.
  Save Item Teach confirms the exact overwrite filename; same known name updates
  its YAML/.pt, renamed/new/unknown-original-name items create a timestamped pair.
  A successful new save becomes the next target. Keep the actively selected model
  path/fingerprint unchanged rather than implicitly switching/reloading weights;
  the saved YAML references its verified local copy for future loading. Saving
  is disabled during a queued/in-progress model load.
- Preserve created_at_utc for valid same-item updates and schema 4. Before
  overwrite, compare observed YAML/model hashes, reject symlink targets and stage
  verified replacement data. Keep exactly one hidden .<stem>.previous.zip per
  updated document, containing its previous YAML and model when present. It is
  manual recovery data, not a compatibility reader or alternate artifact path.
  Unchanged model bytes are never replaced. Replacement weights precede atomic
  YAML publication; a YAML write error restores the original weights. The pair
  is not claimed to be a multi-file atomic transaction: interrupted mixed files
  fail strict model-hash validation and the previous pair remains in the backup.
- Supersede the earlier no-overwrite/new-pair-only restriction for explicit
  saves, including corrected GUI recovery drafts with a known unchanged name.
  Recovery drafts remain unarmed and cannot generate production poses until all
  missing/unclear fields are corrected and saved as strict schema 4. A load alone
  never mutates any file; headless/controller readers remain strict. UI schema 6,
  other teach artifacts, model inference and pose mathematics are unchanged.
- Verification is synthetic/offscreen only. Source changes do not rewrite this
  workstation's operator profiles, UI state, model weights or calibration files;
  no cameras, robot or RViz launched/commanded. No offline milestone is claimed.
- Verification: 304 perception and 6 controller tests pass, including 151 focused
  editor/core cases. Cover same-name and renamed saves, repeated saves with one
  backup, unchanged model inode, replacement weights, missing-model recovery,
  unknown original names, external changes/removals/symlinks, backup/write error
  rollback, cancel/busy-load behavior, recovery save gates, and loaded/startup
  profile simulation/arming without a redundant Save. Root build passes all 14
  packages; Python compilation, changed-runtime and changed-line test flake8,
  and git diff --check pass. Installed modules expose the save-target API and
  import no cv2/Torch/Ultralytics in the parent. Scoped commit/push follow rule 8.

### 2026-09-14 — Remove Item Teach controller validation; review execution baseline

- User requested removal of Validate Saved Profile in Controller and a current
  controller summary before developing robot execution. Remove the button, its
  parameter-service client, request state/polling, unused imports and package
  dependency. Do not leave a hidden callable request path behind the removed UI.
- Item Teach remains independent with unchanged saved/edited eligibility,
  profile Save/Load/recovery, YOLO, local Simulate Trigger, Armed read-only pose
  service and teaching TF. Controller configuration is explicit at its own
  item_teach_file launch argument or standard ROS parameter interface. Retain
  strict controller profile validation, revalidation service and pose trigger.
- Current controller validates a schema-4 YAML and hash-bound paired weights,
  publishes non-armed status and, on a separate trigger, requests pose_candidates
  fresh detector poses with the profile hash. It checks response identity/frame,
  timestamps, unique IDs, ordered priorities and finite normalized geometry,
  then returns/logs JSON. One request at a time, bounded events, no auto-retry.
  It has no Dobot command clients, feedback subscriptions or TF listener, and no
  initialization/home/pick/place/gripper execution or enforced sole ownership.
- Execution remains pending until fixed-vertical attitude/height equations,
  home/travel/place safety, exact I/O confirmation/deadline behavior and safe
  retry/retract transitions are decided. Do not inherit imported item_pick
  defaults or existing gripper patterns. This change starts with the requested
  summary rather than implementing unresolved hardware policy.
- Verify synthetic/offscreen tests, installed package/root build, compilation,
  changed-line lint and git diff --check. Existing station artifacts, weights,
  root .env and saved UI state remain untouched; no physical cameras/robot/RViz
  launched or commanded. No new offline deployment milestone.
- Verification: 305 perception and 6 controller tests pass in a serialized
  full-suite run. Added an offscreen/AST regression proving no controller button,
  client, request state or polling remains; retained loaded saved eligibility,
  recovery/model-failure gates, local simulation and Armed coverage. Root build
  passes all 14 packages; installed modules retain the independent controller
  pose trigger and import no native vision/ML modules in the parent. Compilation,
  changed-runtime/changed-test-line flake8 and git diff --check pass. An initial
  build running alongside tests replaced cv2.abi3.so during one native test's
  import; stop overlapping install writes and rerun the complete suite, which
  passes without source/runtime fallback or hardware execution. Scoped source
  commit and verified push follow the standing authorization.

### 2026-09-14 — Automatic latest station calibration for Item Teach and Detect

- User requested removal of manual platform/camera selection in Item Teach and
  Item Detect. Add shared deterministic station selection in root calibration/:
  latest platform for canonical root .env robot LAN1 identity, derive its camera
  prefix, then latest camera calibration for that prefix across both modes.
  Use canonical filename UTC timestamps, not mtimes, preserving ordering after
  copies/transfers. Other robot platforms and identifiable other-camera files
  do not replace this station/camera. Missing/tied/noncanonical/symlinked or
  malformed/unidentifiable candidates and invalid selected artifacts fail
  without quietly trying an older compatible pair.
- Preserve strict schema-3 platform/schema-7 camera hash, mode, settings and
  mounting-chain validation. The newest same-prefix camera must be the exact
  camera referenced by the newest platform; otherwise require platform reteach.
  Never combine a fresh camera mount with an old measured platform transform.
  Recheck selected paths/hashes before installing the binding; no adoption of
  files changed between discovery and application. Keep pose math, full platform
  tilt/height, portable bin evidence/warnings and service freshness unchanged.
- Item Teach displays read-only latest platform/camera paths with Reload Latest
  Calibration, retains explicit portable bin selection and auto preview binding.
  Ignore restored platform filename as selection authority; restore bin and
  discover station on startup. Reload/bin changes stop YOLO/disarm and clear old
  views/teaching TF before validation. Nonmodal invalid status, no timer rescan,
  no live station switch during requests, auto model execution or auto arming.
- Headless Item Detect removes platform_teach_file and selects the same station
  at startup. Existing explicit item/bin paths, trusted_model/armed flags and
  read-only request-driven inference remain; flat runtime_teach catalog loading,
  default deployment activation and runtime/debug image-saving modes are
  separate pending work, not claims of this change. Platform/Bin Teach retain
  explicit calibration selection. No schema changes/migration or artifact writes.
- Add rule 50, superseding only Item Teach/Detect's earlier no-selection/search
  and explicit-platform-path restrictions. Update root/package README/quickstart.
  Verification uses real strict synthetic YAML writers/readers and offscreen
  mocked ROS/processes only; no cameras, robot, RViz or operator weights launched.
  Read-only workstation inspection selects bin_camera's current platform/camera
  pair rather than its newer robot_camera calibration. Operator artifacts,
  weights, root .env and saved UI state are not rewritten by this implementation.
- Verification: 329 perception and 6 controller tests pass (335 total), including
  23 new strict synthetic selector/headless cases and an offscreen reload case.
  Cover both modes, filename ordering vs mtimes, other robot/prefix exclusion,
  newer-camera reteach requirements, invalid newest artifacts with no fallback,
  ambiguous timestamps, noncanonical/symlinked/unidentifiable candidates,
  filename/content timestamps, changed selection hashes and headless launch/main.
  GUI coverage retains restored-bin auto preview, mismatch warnings and strict
  save/arming gates, verifies old platform prefill is ignored, reload clears old
  geometry/TF without auto activation or periodic rescan. Root build passes all
  14 packages; package build and packaged colcon tests also pass. Compilation,
  changed-runtime/new-test and changed-
  test-line flake8, and git diff --check pass. Installed inspection selects the
  correct workstation pair and imports no cv2/Torch/Ultralytics in the parent.
  No offline milestone is claimed; scoped commit and verified push follow rule 8.

### 2026-09-14 — Explicit controller Home/pick with GUI/headless and TF-only debug

- User confirmed controller execution scope after the scaffold summary: one
  shared GUI/headless implementation, explicit Home/Pick controls, default safe
  TF-only debug and explicit real mode. No automatic Home/pick, bringup/camera/
  detector/RViz launch, placement, model inference or operator artifact rewrite.
  Rule 51 supersedes historical non-actuating controller restrictions only;
  Item Teach/Detect remain read-only. Full legacy-client migration and enforced
  sole-command ownership are still pending, not claims of this stage.
- Real startup follows Motion Debug's order but user-selected SpeedFactor 100%:
  StopMoveJog, DisableRobot, EnableRobot/enabled confirmation, speed, Tool 0,
  Tool 1 TCP zero, CP 100%. Only StopMoveJog/DisableRobot are best effort; other
  failures terminate startup without retries. Motion Debug remains 50% unchanged.
  Reject duplicate controller/canonical command-service providers and known
  running legacy motion/gripper applications. Hardware verification is forbidden
  in this implementation; all service/stream injection is synthetic.
- Canonical static CR10 forward kinematics derives Cartesian Home from the six
  taught radians, independent of RViz. Real GetPose(user=0,tool=0) and nearest
  IK must agree with canonical model within 2 mm/0.5 degrees and joint limits;
  no fallback or alternate model. Every Home reaches its equivalent base Z first
  at actual XY/attitude using GetPose plus RelMovLUser, then MovLIO joint mode
  reaches exact recorded Home joints. Relative Home Z is an explicit user
  exception to the prior all-MovLIO request; all pick/transit/retract stays MovLIO.
- Confirmed base-Z/Home-attitude pick equations: itemZ+standoff is Link6 pick;
  add zheight_offset for initial/final, prepick_height for pre-pick,
  retract_height for intermediate retract. Reject zheight_offset below either
  pre-pick/retract and Home below any candidate clearance. Convert full tilted
  platform XYZ to base first; transit XY at Home Z before vertical descent.
  Checks are not collision planning/commissioning; do not repair teach settings.
- User resolved suction deadline with final approach plus saved pick_settling
  (example 0.2 seconds), not a new timer field. DO1 stays off; DO13 on at final
  approach, active-high DI1 observed throughout that descent. Early DI1 interrupts
  with Stop, including delayed move acknowledgement, and waits for accepted Stop
  plus fresh queue-idle stationary feedback before retract. Do not continue to
  nominal pick after sealing. No DI at completed pick waits taught settling then
  becomes a miss. Both retract stages use at least actual stopped Z so early
  contact cannot cause another downward move. Unexpected DI before suction,
  stale/fault feedback, Stop/I/O/motion/retract failures or lost vacuum fail closed.
- Final explicit finish selection is hold at final retract, suction ON (supersedes
  the earlier free-text mention of returning Home). Explicit Home may carry the
  held item with suction monitoring. use_grip=false never touches DO2/DO14;
  true opens with DO2 off/DO14 on, stays open unless grip_onpick=true closes only
  on confirmed DI1. User deferred full-open sensor checks (DI12 in canonical
  wiring), superseding rule 14's controller full-open requirement this stage only;
  do not diagnose damage or import old Grip/Release/purge patterns.
- Request one ranked fresh batch from independently armed detector with exact
  item/model/bin/camera/platform hashes. pose_candidates is maximum distinct
  attempts, only missed suction retryable after confirmed final retract and
  return Home; RGB/depth result age checked before every attempt and before final
  approach after transit. Expiry
  requires another explicit request, never automatic reacquisition or stale
  reuse. Fresh actual joint/status/FeedInfo sole canonical publishers and an
  advancing controller_timer are required: repeated cached packets do not count.
  Service acceptance and Stop request alone are never completion. Cancellation
  preserves vacuum; late accepted movement receives safety Stop containment,
  not a movement retry. Physical emergency stop remains independent.
- GUI loads canonical offline item/bin artifacts explicitly and preserves strict
  schema-1 filename-only unapplied prefill in its own bounded-log directory.
  Item Teach alone supports Home. Headless reads exactly one schema-4 item YAML,
  verified pair .pt and portable schema-3 bin from flat root runtime_teach/;
  reject partitions, symlinks, unknown/ambiguous inputs and overrides. Shared
  strict readers have explicit deployment-only paths, not compatibility readers;
  writers/teach dialogs remain offline_teach. Reuse latest bound station selector
  in calibration/ for controller, retaining destination full tilt/height and bin
  provenance warning. Multi-item/tray catalogs and Item Detect's pending flat
  runtime/default activation/debug-image workflow remain separate work.
- Debug creates no Dobot command clients and broadcasts only uniquely named
  base-relative robot_controller_debug_* Home/all-candidate stage targets at
  10 Hz after explicit actions. Fresh actual joints supply current debug Home,
  not synthetic joints or GetPose. Cancel/exit/source edits stop publication;
  timer checks file signatures, never repeatedly hashes large model weights.
  Full hashes validated at load/action/request. Preserve artifact schemas,
  private native workers, bounded package events and station/operator files.
  Require exactly one canonical GUI/headless pose-service provider and verify
  returned confidence, not only class/frame/hash/timestamps. Preflight
  checks detector availability/provider before preliminary Pick Home travel;
  acquire the fresh batch only after Home. Stationary confirmation
  compares cumulative drift against its stationary anchor, not just tiny
  per-frame differences. Validation summaries use PROFILE_VALIDATED separately
  from execution_state; no historical NOT_ARMED label while executing.
  Validate GUI prefill before constructing real command clients/initialization;
  malformed state fails first. Preserve schema-4 historical controller_contract
  as artifact validation metadata, never execution permission: real launch plus
  explicit actions and current safety checks are the authority, not file loading.
  Own SIGINT/SIGTERM shutdown notification without rclpy auto-context teardown;
  headless/GUI cancel and request Stop while DDS is alive, then close executor/
  context and restore previous handlers. GUI also closes on external ROS shutdown.
- Real ROS-node construction regression discovered the scaffold's parameter
  callback accidentally shadowed Node._set_parameters, preventing declaration;
  rename to _on_parameters. Synthetic/offscreen controller tests cover startup,
  Home/pose units, full platform conversion, early suction Stop/late acknowledgement,
  finger rules, final retract/holding, missed retries, faults/expiry, runtime
  copies, strict prefill, source invalidation and actual debug node construction.
  Full regression, lint/compilation, package/root build, packaged tests and
  git diff --check are required before the standing scoped commit/push. No new
  offline milestone or physical commissioning is claimed.
- Verification: 53 controller and 329 perception tests pass (382 pytest cases;
  packaged CTest summaries include 54/330 results with both wrappers), zero
  errors/failures/skips. Package build and final root build pass all 14 packages;
  compilation, changed-runtime/new-test flake8 and git diff --check pass. Installed
  launch argument inspection confirms GUI/headless and safe debug defaults;
  read-only installed runtime catalog inspection validates this workstation's
  water pair/latest bin_camera station, imports no cv2/Torch/Ultralytics and
  detects zheight_offset below retract_height. Leave those operator files intact:
  they require explicit Item Teach correction before Pick; Home clearance must
  also satisfy every actual candidate. No real robot/camera/RViz or operator
  weight deserialization. Scoped source commit and verified push follow rule 8.

### 2026-09-14 — Editable per-motion speed and acceleration in Item Teach

- User extended the implemented controller routine: final approach and both
  retract stages start at 6% speed, all remaining motion at 100%, with editable
  acceleration initially 100% throughout. Add rule 52 superseding only the
  earlier item schema-4/preserved-schema restriction for these new mandatory
  motion fields; existing pick/height/freshness/Stop safety contracts remain.
- Item Teach presents separate speed and acceleration groups in the routine
  portion of its one-column visual-first form. Each saves explicit integer
  travel_percent, approach_percent, retract_percent in 1–100. New form values
  are speed 100/6/6 and acceleration 100/100/100; loaded values are preserved,
  edits disarm/invalidate saved eligibility without interrupting read-only
  inference, automatic saving or robot commands.
- Item artifacts advance to strict schema 5 with speed/acceleration percent
  units. Production readers reject schemas 1–4 with no compatibility reader,
  aliases or inferred rates. GUI-only recovery still retains independently
  valid known fields; missing/invalid/unknown-unit motion rates stay blank until
  explicit correction and Save. Shared UI schema 6, filename-only controller
  schema 1, camera schema 7 and platform/bin schema 3 remain unchanged.
- Controller targets carry speed and acceleration through Home/pick generation
  and early-contact retract adjustment. Travel includes both Home stages,
  XY transit, initial positioning and descent to pre-pick. Approach is only
  pre-pick-to-pick descent; retract includes intermediate and final retract,
  whether successful or missed. Canonical MovLIO and the approved Home-height
  RelMovLUser carry v=/a= in param_value; global SpeedFactor stays 100%, with no
  new global setting calls. The vendored Dobot V4.6.5 TCP interface reference
  defines v/a as integer 1–100 percentages; speed= would mean mm/s and is not
  substituted. No vendor source patch or acceleration override is introduced.
- Explicit workstation-only update, authorized by the user as an Item Teach
  edit: preserve the water document names, model weights/hashes and all existing
  settings, updating only schema, percent units and new rate groups in the
  offline teach YAML and its runtime_teach copy. Retain each original YAML/.pt
  pair in a hidden previous-version ZIP. These artifacts/weights remain outside
  source commits. Do not fix the unrelated zheight_offset=50/retract_height=100
  mismatch: Pick remains blocked until the operator corrects and saves heights.
- Verify rate validation/boundaries/mandatory keys/units, strict older-schema
  rejection, schema-5 round trips/overwrites and untouched paired weights,
  GUI controls/prefill/recovery/disarming, per-stage v/a command dispatch,
  editable Home rates and early Stop/retract preservation using only synthetic
  services/streams/artifacts and offscreen GUI. Slow movement does not expand
  motion or candidate-age deadlines; expired later batch poses still abort.
  Package/root builds, package tests, compilation/lint and git diff --check are
  required before the standing scoped source commit and verified push. No
  physical robot/camera/RViz, operator weight execution or offline milestone.

- Verification: all 351 perception and 65 controller tests pass (416 pytest
  cases, 418 packaged results including CTest wrappers), zero failures/errors/
  skips. Package-up-to-controller and clean-environment root builds pass all
  6/14 packages. Python compilation, all changed runtime/controller-test files
  and all changed Python lines pass flake8; 51 pre-existing unchanged-line test
  diagnostics are excluded from the scoped lint gate, not silently repaired.
  Installed offline/deployment readers and the runtime latest-station catalog
  validate both updated water profiles. Original-to-updated comparison proves
  all unrelated settings unchanged; both previous-version ZIPs and their model
  hashes verify, and live model inodes/sizes remain unchanged. The existing
  height mismatch still blocks Pick. git diff --check and scoped staged review
  precede the standing source commit/push; no hardware/weights were executed.

### 2026-09-15 — Conditional preliminary Home-height move

- User confirmed that a current Link6 pose already equal to or above taught
  Home Z may move directly to the exact taught Home joints. Add rule 53,
  superseding only rule 51's unconditional preliminary Home-Z segment.
- Home planning uses the validated current base_link-relative Link6 Z and
  canonical FK-derived Home Z. Strictly below Home keeps the existing vertical
  RelMovLUser rise at current XY/attitude, then joint-mode MovLIO Home. Equal or
  above produces only the joint-mode MovLIO Home target; it never first descends
  to the Home plane. The comparison has no hidden tolerance or alternate path.
- Debug and real actions share the exact branch. TF-only preview publishes
  robot_controller_debug_home_height only below Home Z; the direct branch shows
  only robot_controller_debug_home. Existing profile rates, FK/GetPose checks,
  fresh feedback, held-item suction monitoring, cancellation, Stop/fault logic,
  explicit actions and no inferred collision planning remain unchanged.
- Verify pure target construction below/equal/above Home, direct MovLIO-only
  dispatch, retained below-Home RelMovLUser then MovLIO order/rates, controller
  Home and debug candidate-frame counts with synthetic services/feedback only.
  Run controller and affected full regressions, compilation/scoped lint, package
  and root builds, git diff checks, staged review and standing source commit/push.
  No robot, camera, model, RViz or operator artifact is launched or modified.
- Verification: all 69 controller tests pass directly and through the packaged
  CTest wrapper, zero failures/errors/skips. New regressions cover strict-below,
  exact-equality and above-Home branching; direct MovLIO-only dispatch; retained
  below-Home RelMovLUser then MovLIO rates/order; controller Home and complete
  debug-pick frame indexing. Python compilation, focused flake8 and git diff
  checks pass. Package-up-to build completes six dependencies/packages and a
  clean-environment root build completes all 14 packages. Only synthetic
  transport/feedback and offscreen/debug data were used; no hardware/RViz/model
  or local operator artifact was launched, commanded or changed.

### 2026-09-15 — Service-driven controller Live gate, Home-return pick and Stop recovery

- User replaced launch-time debug/real selection with a runtime actuation gate,
  then explicitly made headless the production-live mode. Add rule 54: the GUI
  starts Live OFF with TF-only actions and no Dobot command clients. Its Home,
  Pick, Stop and red Live controls call the same public ROS services exposed
  headlessly. `/robot_controller/set_live` ON constructs the command transport
  and completes one-time initialization before READY; GUI OFF is idle/not-holding
  only, removes command clients without disabling the robot, and a later ON
  initializes again. Headless loads the deployment set, automatically starts
  permanently Live, initializes, and rejects Live OFF, but never invokes Home or
  Pick itself. Remove the `debug` launch argument.
- Refactor execution into reusable `home()`, `pick()` and `stop()` functions.
  Pick validates detector/station/profile before motion, completes Home, requests
  one fresh ranked batch and attempts only those candidates. A miss completes
  retract and Home before the next candidate. Success completes retract and Home
  while monitoring/holding suction; exhaustion returns Home and reports failure.
  Preserve rule 53's conditional preliminary Home height, item rates, fixed
  vertical geometry, finger rules, freshness, FK/IK and sole-provider checks.
- Stop cancels the interrupted routine and sends canonical Stop. Only after Stop
  acknowledgement, fresh stationary feedback and action termination may recovery
  run. DI1 OFF performs no return motion. DI1 ON requires the remembered most
  recent pre-pick target, returns there with suction continuously required, then
  disables suction, enables exhaust and opens enabled fingers. Missing recovery
  context or any suction/motion/I/O/feedback fault retains the item and fails
  closed; physical emergency stop remains independent. Item Teach Armed ON is
  also red as a visual service-state warning without changing arming validation.
  A second explicit Stop during confirmation/return sends Stop again and cancels
  recovery without another return routine; shutdown also prevents recovery from
  clearing cancellation and resuming motion/release. Serialize Stop acceptance.
- Add a separate troubleshooting-image gate in both modes, unrelated to Live.
  `/robot_controller/set_debug_images` defaults OFF and its Boolean is sampled
  once per candidate request; `/robot_controller/status` publishes that state and
  the latest save outcome. Extend GetItemPoses with `save_debug_images`. When true,
  Item Detect atomically saves exactly that request's already-rendered RGB and
  registered-depth overlay pair as PNG under ignored root `debug/pick_img/` and
  returns absolute paths. Never subscribe to another camera stream or continuously
  archive frames. Persistence failure is reported in diagnostics/status but does
  not alter candidates, ranking or robot motion.
  Revalidate generation/source hashes, result age and request deadline after image
  persistence; optional saving never permits a stale/disarmed response.
- Verification: all 355 perception and 82 controller tests pass directly and
  through their packaged CTest wrappers. Regressions cover service names and GUI
  service clients, red Live/Armed styling, GUI OFF/no-transport startup, headless
  automatic Live initialization/permanent gate, GUI ON/OFF/re-enable gating,
  independent debug-image service/status, exact same-batch PNG pair persistence,
  non-blocking persistence errors, Home-return success/miss/exhaustion, remembered
  pre-pick and Stop confirmation/DI/no-release branches. The changed runtime and
  controller tests compile and pass focused flake8; `git diff --check`, package-up-to
  and all-14-package root builds pass. All validation used synthetic ROS services,
  feedback/images and offscreen Qt; no robot/camera/RViz/model was launched or
  commanded and no operator artifact was changed.

### 2026-09-15 — Ordered response-aware startup and precise readiness failures

- Investigated the operator's latest controller failure before changing code.
  Both latest Live initializations logged successful StopMoveJog, DisableRobot,
  EnableRobot, SpeedFactor, Tool, SetTool and CP responses, followed by READY
  and the generic feedback-watchdog failure. No startup service failure was
  recorded. A passive five-second subscription found connected/enabled mode 5,
  zero error/collision flags and user/tool 0, but isPauseCmdFlag=1. This current
  paused-queue flag matches the blocked-readiness condition; historical logs
  did not record the individual flag values at the original failure instant.
- Add rule 55 after the user's explicit instruction to wait for every response.
  Preserve Motion Debug's startup ordering and the controller's 100% SpeedFactor.
  Boundedly wait for all strict service discovery before preconditioning.
  Missing/rejected StopMoveJog and DisableRobot remain warnings, as does missing
  Disabled confirmation. Serialize normal calls and retain unresolved futures:
  an unanswered response timeout stops startup without a later command, even
  for an otherwise optional step. Late responses never auto-advance/retry;
  independent safety Stop can still interrupt ambiguous motion acceptance.
  Motion Debug's separate best-effort timeout contract is not changed.
- Startup progress names the current service; failed calls log their exact name.
  After the final settings response, check full readiness before advertising
  READY. Failed feedback says startup calls completed and lists every exact
  enabled/mode/error/collision/pause/user/tool blocker with its value. Preserve
  all safety guards: no automatic Continue, pause bypass, alarm clearing or
  new robot settings. The vendored Dobot V4.6.5 manual describes Continue as
  resuming a paused motion queue/program, not a harmless readiness check.
- Verification: 101 controller tests pass directly, including delayed sequential
  responses, unanswered optional calls/no late advancement, missing/rejected
  optional steps, strict named failures/no later settings, bounded strict-service
  discovery, no false READY on paused feedback and exact/multiple blocker errors.
  All 101 cases also pass the packaged CTest wrapper. Compilation/scoped lint,
  git diff checks, six-package dependency build and clean-environment all-14-
  package root build pass before the standing source commit/push. Verification uses synthetic
  transport/feedback/offscreen Qt only; the investigation used passive feedback
  subscriptions. No robot/camera/RViz was launched, no hardware command was sent,
  no operator weights were executed and no station artifact was changed.

### 2026-09-15 — Explicit Enable Robot and coherent final startup feedback

- User requested Enable Robot in the controller after the final startup check
  reported RobotStatus.is_enable=False. The latest log already records a
  successful EnableRobot response and enabled confirmation before settings.
  A passive current-feedback check reports both RobotStatus enabled and mode 5/
  EnableStatus 1, but isPauseCmdFlag remains 1. The original disabled snapshot
  may be transient asynchronous feedback; the source publishes RobotStatus
  separately and derives is_enable from mode 5. No vendor patch is required.
- Add rule 56: GUI Enable Robot calls `/robot_controller/enable_robot` Trigger,
  also available headlessly. With Live ON and all startup settings complete,
  an explicit action sends only EnableRobot once, awaits its response and fresh
  coherent idle/enabled/fault/pause/user/tool feedback. No teach file is needed.
  Replies acknowledge acceptance; ENABLING, READY or exact FAILED details are
  published on status, including startup_settings_applied. No automatic Home,
  Pick, DisableRobot replay, setting changes or Continue is added.
- Reject incomplete/fatal startup, active actions/Stop recovery, held items,
  ambiguous motion, DI1 ON, queued/running/fault/collision feedback, stale inputs,
  pending Stop and competing command owners. Serialize acceptance with Stop;
  preserve command-response ordering, cancellation and independent safety Stop.
  Explicit enable failure cancels and reports failure, never retries or releases.
- Final startup readiness now waits boundedly up to five seconds for coherent
  asynchronous feedback after all settings responses instead of failing one
  transient disabled snapshot. No command is resent; persistent disabled/pause/
  fault/coordinate or stale-feedback failure remains blocked and precisely named.
- Verification: 124 synthetic/offscreen controller tests cover the new service/
  button and Live-OFF gating, enable-only dispatch, delayed final status recovery,
  persistent disabled/paused/running blocks, unsafe-state rejection, incomplete
  startup and worker failure/cancellation. All 124 cases pass directly and via
  the packaged CTest wrapper, zero errors/failures/skips. Compilation/scoped lint,
  six-package dependency and clean-environment all-14-package root builds and
  git diff checks pass before scoped commit/push.
  Investigation used passive subscriptions only; no robot/camera/RViz launch,
  hardware command, operator model execution or station-artifact mutation.

### 2026-09-15 — Queued vertical pick, additive clearance and motion-timed gripper I/O

- User approved the final alignment and confirmed that end-of-prepick-retract
  finger closing happens only on DI1-confirmed success. Add superseding rule 57.
  Remove zheight_offset from the form/artifact/controller; production item
  schema becomes 6, with only standoff_height, prepick_height and retract_height.
  P=item base-Z+standoff; pre=P+prepick; clearance=pre+retract. Home attitude/base
  Z and Home Z>=clearance remain required. Do not reinterpret the user's pre-pick
  waypoint as item Z+prepick below the compensated pick point.
- Pick completes the exact shared Home function before acquiring one fresh
  ranked batch. Queue item XY at Home Z, clearance, pre-pick at travel rates,
  then P at approach rates. On return, queue pre-pick at retract rates,
  clearance and shared conditional Home at travel rates. Initial speeds remain
  travel/approach/retract 100/6/6, acceleration 100/100/100; remaining clearance
  and Home return now use travel rates. Debug previews include conditional
  per-candidate Home targets, not actuator clients or executed motion.
- Preflight every endpoint with canonical FK/nearest previous IK while idle.
  Dispatch motion calls one at a time, awaiting every response without waiting
  at intermediate arrivals. Confirm full owned-queue idle and fresh stationary
  terminal pose/exact Home joints before settling/completion/candidate advance.
  The vendor MovLIO response exposes only res, not ResultID; never guess IDs or
  treat acknowledgements as arrival. Per-command cp=0 prevents blending past
  vertical corners or speed/event boundaries; startup/global CP remains 100%.
  Home's conditional relative-Z exception uses its validated preceding endpoint
  as the queued delta origin, preserving XY/attitude and avoiding GetPose while
  moving. Existing 5-second service/30-second motion bounds are not expanded;
  the motion bound applies to a whole batch.
- Canonical local Dobot V4.6.5 TCP/IP manual (vendored PDF, MovLIO pp. 88–89)
  defines parallel output tuples, percent 50/100 and distance-mode zero as
  motion-start. Existing parseTool.cpp passes mdis through unchanged; no vendor
  patch. With use_grip, clearance carries DO2 OFF/DO14 ON at 50%; final descent
  carries `{1,0,13,1}` for suction at start, keeping exhaust OFF. Fresh output
  feedback must confirm motion events before batch success. DI1 after actual
  suction start interrupts the forward queue, requires Stop acknowledgement
  and fresh stationary feedback, and prevents further descent submission.
  A motion acknowledgement arriving after acquisition Stop gets a new safety
  Stop; confirm that latest Stop before retract, never retry the movement.
  grip_onpick=true closes after confirmed acquisition/Stop before retract;
  false places DO14 OFF/DO2 ON 100% events on retract-to-prepick ONLY on success.
  Misses never close fingers. Disabled finger control never writes DO2/DO14.
- Return geometry starts from actual stopped XY/attitude with upward-clamped Z;
  contact above nominal pre-pick also updates remembered Stop-recovery height.
  Success returns Home holding; misses finish retract/Home before vacuum OFF
  and the next still-fresh received candidate. DI1 during a classified missed
  return is a fault requiring Stop, not another candidate or release. Apply
  the same DI1-clear guard before initial/missed vacuum OFF and its response/
  output confirmation, including a late pickup immediately after Home.
  Preserve second-Stop/shutdown cancellation, independent safety Stop, ownership,
  source/candidate expiry, no automatic batch reacquisition and physical-estop
  independence. Collision/path safety still requires hardware commissioning.
- Inspection found a related existing guard defect: vendor command.cpp
  isEnable() returns robot_mode==5 and is false during modes 7/8, despite enabled
  motion. No vendor patch: require canonical FeedInfo EnableStatus exactly 1
  during enabled actions. False RobotStatus while mode 7/8 and EnableStatus=1
  means not idle, not disabled. Idle readiness still requires RobotStatus
  enabled; pause/error/collision/user/tool/freshness checks remain unchanged.
- Production rejects schemas 1–5 without conversion. GUI-only recovery omits
  zheight_offset and blanks old retract_height because its reference changed;
  require explicit correction/Save, retaining independent valid settings. Shared
  UI schema 6 and camera/platform/bin schemas/geometry stay unchanged.
- Authorized workstation-only edit updates the current water offline/runtime
  YAMLs to schema 6 and removes zheight_offset, preserving all other values,
  filenames and paired weight hashes/inodes. Each original YAML/.pt pair has a
  separate hidden before_schema6 ZIP, retaining previous backups too. Operator
  artifacts/weights/backups are excluded from source commits. Loading alone
  never converts profiles or executes weights/robot commands.
- Verification: 356 perception and 165 controller tests pass (521 cases), with
  synthetic queued/delayed/failed/unanswered responses, no intermediate arrival
  waits, endpoint IK/FK preflight, phase rates/I/O, DI1 early/late/failed Stop,
  late acknowledgement containment, actual-stop geometry, conditional shared
  Home, success-only finger closing and GUI/schema/recovery regressions. Package
  tests have zero errors/failures/skips. Compilation, runtime/controller-test
  flake8 and changed perception-test lines pass; 14 pre-existing unchanged test
  diagnostics are excluded, not silently repaired. The six-package dependency
  build and clean-environment all-14-package root build pass. Installed schema-6
  readers verify both updated profiles and the complete runtime/station set;
  backup comparison proves only schema/removal changed and weight hashes/inodes
  are untouched. git diff checks and staged review precede the standing scoped
  commit/verified push. No real robot/camera/RViz launch, operator weight
  execution, vendor patch, or new offline milestone.

### 2026-09-15 — Explicit controller global-speed slider and headless setting

- User requested a controller-level 1–100 speed slider, separate from the item
  profile's individually assigned MovLIO/RelMovLUser speed/acceleration. Add
  superseding rule 58: every Live initialization still explicitly sets global
  SpeedFactor 100%, then an operator may adjust it while Live/idle. No change
  to per-phase v/a, taught offsets, queue/I/O/pick behavior or artifact schemas.
- GUI mouse changes apply after release; keyboard changes debounce 300 ms.
  The GUI calls shared `/robot_controller/set_global_speed` using the existing
  dobot_msgs_v4 SpeedFactor type, not vendor bringup directly. Public service
  type metadata is safe in TF-only mode; real command clients remain exclusively
  in the Live-gated transport. Headless exposes the same bounded setting
  response: res=0 only after a successful robot response; otherwise -1 with
  precise reason in bounded events/status, never asynchronous acceptance.
- Disable/reject changes during Live OFF, incomplete/fatal startup, active
  action/recovery or pending commands. Acquire action ownership atomically;
  preserve response serialization, sole canonical provider, fresh enabled,
  fault-free/unpaused/user/tool-0 idle feedback and no unknown queued motion.
  Idle HOLDING permits speed changes only while DI1 remains ON. Safety Stop
  remains independent; cancellation/failure never resumes or retries commands.
  Report SPEED_SETTING while waiting; success restores the previous idle state.
  Accepted setting failure fails closed. Unknown/late acceptance never silently
  restores READY or the previous confirmed percentage.
- Status includes global_speed_percent and global_speed_message. The percentage
  denotes an acknowledged command, not measured motion speed; null means unknown
  or Live OFF. This slider is a transient robot-command target, not operator
  setup or Item Teach content: no persisted speed, startup auto-application,
  new .env key, configuration workflow or schema migration. Root/controller
  documentation explains reset/idle/service behavior. Do not extend existing
  five-second command/30-second motion/result-age limits for slow global speed.
- Verify slider range/debounce/service routing and Live/busy/pending gates;
  actual delayed/rejected/unanswered responses and unknown/late containment;
  enabled-idle/ownership/suction/cancellation checks; unchanged taught v/a and
  100% reinitialization. Run synthetic/offscreen controller regressions,
  compilation/lint, packaged tests, dependency/root builds and git diff checks
  before the standing scoped commit/verified push. No robot/camera/RViz/model
  is launched or commanded; operator artifacts/weights remain untouched.
- Verification completed: 208 synthetic/offscreen controller tests pass directly
  and in the package CTest wrapper (zero errors/failures/skips). These include
  isolated ROS wire-service invocation, Live/busy/ownership/freshness/DI1 gates,
  1/100 boundary values, delayed/rejected/unanswered Stop/cancel behavior,
  no late-ack READY restoration, preserved per-target v/a and 100% startup reset.
  Changed Python compiles and passes flake8; git diff --check, six-package
  dependency build and clean-environment 14-package root build pass. No real
  hardware/weights/station artifact was exercised or modified.

### 2026-09-15 — Live enable and Stop/Clear Home recovery

- User corrected the controller interaction: Live ON, not a separate GUI button,
  must enable the robot. Stop/Clear must cancel an owned queue, confirm fresh
  stationary/empty-queue feedback, conditionally ClearError, re-enable, and with
  DI1 OFF use the existing Home function from a new GetPose/FK-checked actual
  Link6 pose. A discarded queue is never resumed with Continue and a cached EE
  target is not a recovery origin. The previously confirmed DI1 ON last-prepick
  return/release rule remains; without a loaded Home, idle re-enable alone is
  reported. This is superseding rule 59 in AGENTS.md.
- Motion Debug's controller-mode-10 manual recovery sends Stop before EnableRobot;
  its ClearError pathway checks for a still-active error and prompts that the
  emergency stop may be pressed. The controller adapts that sequence with strict
  response serialization, five-second acknowledgements/readiness, sole canonical
  command ownership, fresh advancing feedback, a stationary/queue-empty Stop
  check and no silent motion on failure. Live uses at most one post-settings
  recovery for persistent readiness failure. Explicit Stop/Clear may repeat an
  incomplete startup only as a new operator request, never after an ambiguous
  unanswered command. A failed/ambiguous global SpeedFactor setting makes its
  factor unknown; Stop/Clear then re-runs full initialization to re-establish a
  confirmed 100% factor before Home, while enable-only cannot bypass it; that
  optional service also rejects fresh pause/user/tool offset feedback.
  Ordinary failure stays visible in GUI/headless FAILED;
  unexpected fatal runtime failures still terminate.
- The GUI Enable Robot button/client is removed; the existing optional headless
  enable-only service remains. In Live FAILED, Home/Pick buttons remain clickable
  with loaded files to produce an exact safety refusal/prompt, not actual motion.
  The GUI prompts once per failed recovery to check the emergency stop without
  diagnosing it from a generic fault. No camera, robot, RViz or model is launched
  or commanded for verification. Synthetic pause/error/held-item, Stop/Home and
  offscreen GUI regressions plus scoped build/lint/commit checks are required.
- Operator follow-up: actual Stop, ClearError, EnableRobot and Home behavior must
  be verified under physical safety supervision before production reliance;
  software stationary feedback is not collision planning or a physical E-stop.
- Verification: 223 synthetic/offscreen controller tests pass directly and in
  package CTest (zero failures/errors/skips). The changed Python compiles and
  passes flake8; a clean-environment root `colcon build` finishes all 14 ROS
  packages, and `git diff --check` is clean. No physical robot, camera, RViz or
  operator model is launched or commanded in verification.

### 2026-09-15 — Dobot ROS pose/IK result payload alignment

- A real Home attempt received GetPose with `res=0`, then failed at the
  controller's "Malformed canonical GetPose reply" check before any motion
  command. The vendored bridge's TCP parser copies only the substring from
  `{` through `}` into ROS `robot_return`, while the raw TCP error prefix is
  delivered separately as `res`. The previous controller parser and synthetic
  fixtures incorrectly expected the complete TCP reply and command echo.
- Rule 60 supersedes that parser assumption for both GetPose and InverseKin:
  check successful `res` first, accept exactly a brace-delimited six-finite-value
  ROS payload, and keep malformed or full raw TCP strings blocked. Preserve the
  GetPose/FK and IK/FK comparisons and all response ordering/safety gates.
  No vendor patch, fallback parser, motion-service substitution or hardware
  command is part of this correction.
- The operator reconfirmed MovLIO joint mode for the six taught Home joints
  (radians in the artifact, converted to degrees on the service request) and
  MovLIO Cartesian pose mode for item waypoints. Conditional Home-Z clearance
  remains RelMovLUser. Synthetic fixtures now reflect the ROS field rather
  than raw TCP transport, and malformed field tests cover both services.
- Verification: synthetic/offscreen controller tests, compilation/lint,
  package/root builds and scoped git review/commit/push are required. Physical
  robot validation is still pending; no launch or command to hardware is
  authorized for software verification.
- Verification completed: 238 synthetic/offscreen controller tests pass both
  directly and via the package CTest wrapper (zero errors/failures/skips).
  Changed Python compiles and passes flake8; the clean-environment root build
  finishes all 14 packages, and git diff --check is clean. No physical robot,
  camera, RViz or operator model was launched or commanded. The source commit
  and remote push are recorded by the final handoff, not treated as proof of
  physical commissioning.

### 2026-09-15 — Direct MovLIO joint/pose inputs without controller InverseKin

- The operator pointed out that vendor MovLIO itself accepts either joint or
  Cartesian pose input and directed removal of the redundant controller
  InverseKin prerequisite. The pinned wrapper's `mode` switch emits `joint=`
  when true and `pose=` when false; the vendored Dobot V4.6.5 manual (MovLIO,
  p. 88) also names both target forms. No InverseKin response is needed to
  construct either MovLIO request, and joint-form MovLIO remains a linear move.
- Rule 61 supersedes the historical IK/FK waypoint preflight requirements.
  Remove the InverseKin client and both batch/single-target calls. Preserve
  GetPose versus fresh current-joint FK, model-limit/FK checks for exact taught
  Home joints, finite rigid target geometry, upward relative-Home constraints,
  user/tool-0 binding, response ordering, actual queued arrival and output
  feedback. Cartesian reachability/joint branch is now not known before the
  motion service; a rejected/ambiguous/uncompleted move faults and stops later
  commands, not an ordinary suction retry. No vendor patch or alternate IK.
- The manual also says MovLIO takes at least one parallel DO tuple, but the
  wrapper drops `mdis=[]`. After being offered plain MovL for no-event segments,
  the operator explicitly chose to retain empty-I/O MovLIO for Home and other
  no-event waypoints. Do not invent a no-op output event or silently switch
  motion service. That selected no-event request must be tested with physical
  firmware under supervised commissioning; synthetic success cannot establish
  firmware acceptance.
- Verification: controller synthetic/offscreen tests, compilation/lint,
  package/root builds, source-only staged review and standing commit/push;
  no real robot, camera, RViz or operator model launch or command.
- Verification completed: 230 synthetic/offscreen controller tests pass both
  directly and through package CTest with zero errors/failures/skips. Tests
  prove the Live transport creates exactly 13 Dobot clients with no InverseKin,
  Home remains joint-mode MovLIO with empty `mdis`, Cartesian waypoints remain
  pose-mode MovLIO, and invalid rigid targets/taught-joint FK mismatches block
  before movement. Changed Python compiles and passes flake8; the clean root
  build finishes all 14 packages and git diff checks are clean. No physical
  robot/camera/RViz/model was launched or commanded; empty-`mdis` firmware
  acceptance remains the explicit supervised commissioning follow-up.

### 2026-09-15 — Cached Home reference and measured endpoint tolerances

- A real Home request was blocked before motion because the controller compared
  canonical CR10 FK from the live joint stream with GetPose(user=0,tool=0), and
  those Cartesian representations did not agree within the former 2 mm/0.5
  degree gate. The operator clarified that taught Home joints are authoritative
  for the final Home target while GetPose is authoritative for the current
  Cartesian pose used to select the conditional vertical-clearance branch.
- Rule 62 supersedes the live GetPose/current-joint-FK agreement requirements in
  rules 59–61. On a successful Item Teach load, calculate and cache canonical FK
  from the six taught Home joints. Use that cached transform only as Home/pick
  planning geometry; do not recalculate it during actions or compare it with
  GetPose. Keep the exact cached joint tuple beside it so a changed profile is
  rejected until explicitly reloaded.
- A joint-mode MovLIO Home is complete only after fresh sole-publisher
  `/joint_states` puts every joint within plus/minus one degree of the taught
  value and fresh enabled/fault-free/queue-idle/stationary feedback remains
  coherent. Its Cartesian FeedInfo/GetPose position is not an additional Home
  arrival gate. Cartesian MovLIO/RelMovLUser terminal targets instead use 5 mm
  translation and one degree rotation against actual FeedInfo
  `tool_vector_actual`, plus the same feedback gates. Successful service
  acknowledgement alone never means arrival.
- Exact taught-joint model/FK validation, target rigidity, relative upward-only
  Home geometry, response serialization, timeouts, suction/Stop handling,
  user/tool zero, debug TF-only operation, and empty-I/O MovLIO remain unchanged.
  Verification is synthetic/offscreen only; no physical robot, camera, RViz or
  operator model may be launched or commanded for this change.
- Verification completed: all 232 controller synthetic/offscreen pytest cases
  pass through the package CTest wrapper (zero errors/failures/skips), including
  mismatched live GetPose/FK acceptance, cached profile Home, and both sides of
  the joint/Cartesian tolerance boundaries. Controller Python compiles and all
  package Python/tests pass ament_flake8. The package build and root build finish
  successfully, with all 14 packages built, and `git diff --check` is clean. No
  physical robot, camera, RViz or operator model was launched or commanded.

### 2026-09-15 — Protocol-correct MovL versus MovLIO dispatch

- A supervised Home attempt reached the real controller after the cached-Home
  correction and sent
  `MovLIO(joint={-33.489,10.555,-133.958,33.543,90.058,11.305},user=0,tool=0,v=100,a=100,cp=0)`.
  The robot rejected it with `-20000` before motion, then `robot_controller`
  issued Stop. The official local Dobot TCP/IP V4.6.5 reference defines
  `-20000` as parameter-count error and MovLIO pages 88–89 require at least one
  `{Mode,Distance,Index,Status}` group. The unchanged vendor parser correctly
  omits the absent `mdis=[]`, proving the previously selected empty-I/O MovLIO
  request is not accepted by this firmware.
- Rule 63 supersedes the empty-I/O MovLIO decision. Add the canonical vendor
  MovL ROS client to `robot_controller`. Targets with no motion-timed output now
  dispatch as MovL with their unchanged joint/pose mode, coordinates and
  user/tool/v/a/cp values. Targets containing genuine saved I/O events remain
  MovLIO and pass those tuples unchanged. Home therefore uses joint-mode MovL;
  ordinary Cartesian transit/retract uses pose-mode MovL; finger/suction event
  waypoints remain MovLIO. No fake DO tuple, host-timed replacement event or
  vendor bringup modification is permitted.
- MovL joins MovLIO and RelMovLUser in late-ack Stop containment, cancellation,
  response serialization, command ownership and failure reporting. Existing
  planning geometry, queued order, target/rate checks, endpoint confirmation,
  output feedback, suction/Stop recovery and artifact schemas remain unchanged.
  Synthetic queue transport now asserts that every MovLIO request has a
  non-empty I/O tuple and every MovL request has no `mdis` field, preventing the
  firmware-incompatible request from passing mocked tests again.
- Verification requires the controller package test wrapper, compilation/lint,
  package/root builds, source-only staged review and `git diff --check`. No
  physical robot, camera, RViz or operator model may be launched or commanded.
- Verification completed: all 234 controller synthetic/offscreen pytest cases
  pass through the package CTest wrapper with zero errors/failures/skips.
  Coverage includes the exact mixed forward/return service order, joint-mode
  Home through MovL, non-empty event-bearing MovLIO, both services' rates/modes,
  response sequencing and late-ack Stop containment. Controller Python compiles
  and all package Python/tests pass ament_flake8. The package build and root
  build succeed, with all 14 packages built, and `git diff --check` is clean.
  No physical robot, camera, RViz or operator model was launched or commanded.

### 2026-09-15 — Robot Controller v2 deterministic action architecture

- The earlier controller accumulated incompatible Live/debug/recovery semantics,
  Trigger commands, JSON status, hardware and Qt in one process, and readiness
  decisions spread across polling paths. The operator approved a clean v2
  architecture focused on deterministic Home and Pick rather than continuing
  incremental repair of that control surface.
- Rule 64 supersedes the combined GUI/controller lifecycle portions of rules
  51–63. A new `robot_controller_interfaces` package defines typed Home and Pick
  actions, lifecycle/configuration/speed/preview services, and reliable
  transient-local status. Every action carries the active configuration digest;
  Pick's candidate count remains solely Item Teach `pose_candidates` and its
  debug-image flag is a one-shot goal field.
- Runtime responsibility is split three ways. `robot_controller` is headless and
  is the only process that creates canonical Dobot command clients.
  `robot_controller_preview` requests detector candidates and broadcasts only
  planned TF targets, and `robot_controller_gui` is only an API client. The
  ordinary launch starts all three; `headless:=true` starts only the authority
  and strictly loads `runtime_teach/`. Neither mode calls Startup automatically,
  enables the robot, changes outputs, or moves on launch.
- The controller now has one exclusive operation generation and explicit
  `UNCONFIGURED`, `INACTIVE`, `STARTING`, `READY`, `HOMING`, `PICKING`,
  `HOLDING`, `STOPPING`, `RECOVERY_REQUIRED`, `RECOVERING`, `HELD_UNKNOWN`, and
  `FAULT` states. Normal Dobot requests remain serialized and acknowledged one
  at a time. An independent Stop client pre-empts a pending action; action
  cancellation and `/stop` both discard motion, confirm stationary/empty queue,
  preserve gripper outputs, and require explicit Recover. Late motion replies
  receive another Stop. No Continue or InverseKin is created or called.
- Startup is an explicit service and performs ownership/fresh-feedback checks,
  best-effort StopMoveJog, strict Stop, cold-DI1 protection, Disable,
  conditional ClearError, Enable, one bounded persistent-pause correction,
  SpeedFactor/User/Tool/TCP/CP setup, unheld output reset, and stable READY. It
  never calls Home. Recover performs Stop/error clear/enable/settings/readiness
  without Home and retains a confirmed global factor. Cold DI1 preserves I/O in
  HELD_UNKNOWN; after the operator resolves the physical condition, an explicit
  Stop observing DI1 clear permits Recover.
- Home reads canonical GetPose for the conditional current-XY rise and ends at
  exact taught joints with joint-mode MovL. Pick executes Home, requests one
  fresh hash/calibration/bin/model-matched pose batch, applies the existing
  platform-to-base transform and schema-6 geometry/rates/I/O timing, and returns
  Home after every attempt. Trusted holding state is established immediately on
  DI1 acquisition so a later retract/Home fault preserves correct recovery
  context. Only coherent DI1-not-acquired is a retryable miss; stale feedback,
  command rejection, output mismatch, cancellation, or motion ambiguity Stops
  and does not advance candidates. Rule 63's protocol-correct MovL versus
  non-empty MovLIO dispatch is unchanged.
- Final audit corrections bind the preview response to its generated
  `tf_frames` field, require advancing FeedInfo samples for every stable-time
  window, monitor trusted held-item DI/output integrity throughout Stop and
  Recover, preserve the most recently acknowledged SpeedFactor even if a later
  integrity check faults, and pre-empt unexpected idle queue motion through the
  independent Stop path before entering recovery.
- Software verification is intentionally synthetic/offscreen. No robot, camera,
  detector model, RViz, or physical motion was launched or commanded. Physical
  commissioning remains a separate, explicitly authorized safety task.
- Verification completed with 53 focused controller-v2 synthetic tests passing
  through package CTest, changed Python compiling, and ament_flake8 reporting no
  issues across controller runtime/tests/launch/scripts. Typed interface
  generation and the package build pass; the root build finishes all 15 packages.
  An offscreen three-process launch remained UNCONFIGURED, exposed only the new
  typed commands/actions, and exited without sending Startup or a hardware call.
  The full root test sweep also passed the 356 Item Perception cases and all
  controller cases; its only failing package is the unchanged vendored
  `dobot_bringup_v4` lint suite (pre-existing upstream copyright, cpplint,
  flake8, lint_cmake and uncrustify findings). Per repository rule, vendored
  sources were not silently reformatted or patched to hide those failures.

### 2026-09-16 — Late persistent-pause recovery check

- A passive live-robot inspection after two operator-triggered Recover calls
  found that every service response succeeded: Stop, EnableRobot, SpeedFactor,
  User, Tool, SetTool, CP and DO1/2/13/14. Canonical feedback remained fresh at
  approximately 100 Hz and reported connected/enabled idle mode 5, an empty
  queue, zero running/error/collision flags, user/tool 0 and DI/output bits 0.
  The sole READY blocker was `isPauseCmdFlag=1`; no robot service had failed.
- The existing one-correction policy checked for persistent pause immediately
  after Enable, while this controller asserted the flag later during remaining
  setup. Startup and Recover now check immediately after Enable and, only when
  the correction remains unused, once again after settings/output reset. At
  most one strict Stop→Enable correction is still issued; Continue remains
  forbidden and there is no retry loop or pause bypass.
- Final READY timeout reporting now evaluates the latest fresh snapshot and
  names the exact mode/enable/pause/error/collision/user/tool/queue blocker. A
  valid snapshot that merely fails the 200 ms stability window is reported
  separately. The inspection was read-only; no service or motion command was
  sent by the verifier.
- Verification passes all 55 focused controller tests through package CTest,
  including the late-pause and exact-blocker cases. The complete 15-package
  root build and the generated controller-interface tests also pass.

### 2026-09-16 — Explicit Pause/Continue with unconditional direct Stop

- The operator selected all three canonical Dobot queue controls. Rule 65 adds
  typed `/robot_controller/pause` and `/robot_controller/continue` services to
  the existing unconditional `/robot_controller/stop`; GUI and headless clients
  never call bringup directly. External nodes may invoke controller Stop from
  any state and never need to Pause first.
- `PAUSED` now preserves the active Home/Pick generation, previous lifecycle
  state, phase/waypoint, queued robot path, gripper outputs and trusted holding
  context. Pause and Continue use independent controller-owned canonical clients,
  await each service response, and confirm pause-flag/stationary or cleared-pause
  feedback respectively. Host-side phase, waypoint and I/O dispatch waits while
  paused, and intentional pause time is excluded from feedback and motion
  deadlines. Paused feedback and held I/O remain supervised.
- Stop still pre-empts without taking the Pause/Continue lock. It invalidates the
  active action and requires acknowledged stationary/empty-queue feedback, but a
  still-latched pause flag no longer makes a confirmed Stop fail. Stop clears
  suspended context and requires recovery; untrusted active DI1 is preserved as
  `HELD_UNKNOWN`. Ambiguous Pause or Continue is contained with the same direct
  Stop path.
- The GUI Start control becomes Continue only for a confirmed/pending Pause.
  Its amber Pause control immediately becomes red Stop after the first click. A
  rapid second click queues Stop locally and sends it only after the Pause reply,
  preserving the established wait-for-response rule. In non-pausable states the
  red button calls Stop directly.
- No physical robot, camera, detector, RViz or maintenance application was
  launched or commanded during implementation and software verification.
- Verification passes all 64 focused controller tests through package CTest,
  including paused-state restoration, direct Stop, retained pause flags,
  suspended deadlines and serialized rapid GUI clicks. Python compilation and
  ament_flake8 pass, as do the generated interface tests and complete
  15-package root build. An isolated-domain offscreen launch exposed all seven
  intended controller services and remained `UNCONFIGURED`; SIGINT then stopped
  the controller, preview and GUI cleanly. The GUI now owns SIGINT/SIGTERM long
  enough to stop its Qt timer before destroying the ROS context, preventing the
  prior shutdown-only invalid-context traceback.

### 2026-09-16 — Enable pause latch is not a READY gate

- A controller/vendor-log correlation identified the transition precisely. The
  earlier executable confirmed Stop only after 300 ms with
  `isPauseCmdFlag=0`; it then sent `EnableRobot()` with no intervening command,
  after which READY observed `isPauseCmdFlag=1`. The same Stop→Enable pattern
  occurred twice. Thus EnableRobot, not Stop or Pause, asserted the observed bit.
- With the operator's explicit authorization, a live no-motion check first
  confirmed enabled mode 5, `isRunQueuedCmd=0`, `RunningStatus=0`, zero
  error/collision, DI/output bits zero and `isPauseCmdFlag=1`. One raw
  `Continue()` returned `res=-1`, left the bit at one and produced no motion.
  This proves the idle latch is not a resumable paused queue.
- Rule 66 supersedes all global uses of the bit. Startup/Recover no longer run
  the ineffective Stop→Enable pause-correction sequence. READY, command
  admission, motion feedback and idle supervision ignore the latch while still
  requiring fresh connected/enabled mode 5, EnableStatus 1, RobotStatus enabled,
  empty/not-running queue, no fault/collision, user/tool zero and stationary
  coherence. Mode 10 remains blocked outside explicit PAUSED state.
- Explicit Pause/Continue semantics remain contextual and strict. Only a
  successful controller-owned Pause with retained generation/state may enter
  PAUSED; that transaction still confirms the asserted bit, and Continue still
  requires its own successful response and cleared-bit samples. A latch without
  controller context never enables Continue or changes the GUI lifecycle.
- Verification passes all 66 focused controller tests directly and through the
  package CTest wrapper; controller plus generated-interface results report 67
  tests with zero errors/failures/skips. Python compilation and ament_flake8
  pass, the package build succeeds, and the complete workspace builds all 15
  packages. An isolated-domain offscreen launch exposed the expected lifecycle
  services in `UNCONFIGURED` and all three processes stopped cleanly. No movement
  target was sent during the live audit or implementation verification.

### 2026-09-16 — Global-speed slider sends the selected position

- The live event log proved the typed speed service and robot both succeeded,
  but every post-Startup GUI request still contained `ratio=100`. The defect was
  local to Qt: the slider used non-tracking mode and read `value()` from
  `sliderReleased`, which could still contain the previous committed value.
- Rule 67 makes the GUI track and send `sliderPosition()` on release, shows the
  selected/requesting/confirmed value, suppresses unchanged requests, and keeps
  the 10 Hz status refresh from snapping the handle back during editing or the
  service/status round trip. Keyboard and groove edits use one 350 ms debounce.
  Controller-side validation, stationary READY/HOLDING gating, canonical service
  serialization and status authority are unchanged.
- Verification passes all 68 focused controller tests directly and through the
  package CTest wrapper; controller plus generated-interface results report 69
  tests with zero errors/failures/skips. Regressions prove the request uses the
  live position rather than the old committed value, unchanged values make no
  request, and keyboard edits start the debounce. Python compilation and
  ament_flake8 pass, both packages build, and the full workspace builds all 15
  packages. An isolated-domain offscreen controller/preview/GUI launch starts
  and stops all processes cleanly. No hardware command was sent during this
  implementation check.

### 2026-09-16 — Runtime enable semantics and complete Dobot call audit

- Two supervised Pick attempts reached the robot and received successful raw
  V4 acknowledgements for GetPose, RelMovLUser and joint-mode MovL. The accepted
  Home queue IDs were 8 and 9. Roughly 0.67 seconds later the controller stopped
  each path with `Robot readiness blocked: RobotStatus.is_enable=False`; no
  motion service had failed. The controller event log contained only service
  names/fields, while the bringup transport log was needed to prove `res=0` and
  recover the returned queue IDs.
- Inspection of the unchanged vendored bridge proved that
  `CRCommanderRos2::isEnable()` returns exactly
  `real_time_data_->robot_mode == 5`. RobotStatus publishes that Boolean on the
  joint/status loop while FeedInfo arrives independently. It is therefore an
  asynchronous idle-mode alias, not a second enable latch. During the
  idle-to-motion transition the monitor combined a newer false RobotStatus
  sample with a different FeedInfo mode sample and incorrectly converted a
  valid accepted Home into a robot fault.
- Rule 68 keeps fresh connected RobotStatus mandatory and retains strict
  mode-derived status convergence for explicit enable/disable, idle READY and
  final stationary arrival. Active/transitioning motion readiness now uses
  FeedInfo `EnableStatus=1`, allowed robot mode, fault/collision and user/tool
  fields without treating `RobotStatus.is_enable=False` as an independent
  failure. This narrowly removes the cross-topic race without weakening final
  Home-joint/Cartesian, stationary, queue, fault or held-item confirmation.
- Every actual canonical Dobot request from the production controller now emits
  a correlated console and bounded-event `SEND` record plus a terminal accepted,
  rejected, timeout/cancellation/response-error or late-response record. Records
  include a process-local request ID, exact endpoint and fields, ROS `res`, any
  `robot_return`, outcome and elapsed milliseconds. The independent Stop and
  explicit Pause/Continue paths use the same audit contract. Service success
  remains acceptance only; all established feedback confirmation follows it.
- Production command ownership remains unchanged: `robot_controller` is the
  only runtime Dobot command client. `motion_debug` may still command Dobot as a
  mutually exclusive maintenance application, and its presence blocks
  production Startup rather than creating a bypass.
- Verification is synthetic/source-only and must include controller feedback,
  transport, lifecycle and architecture tests, Python compilation/lint, package
  and root builds, staged review and `git diff --check`. No hardware service or
  action is invoked by these checks.
- Verification completed with all 70 focused controller tests passing directly
  and through the package CTest wrapper (71 reported tests, zero errors,
  failures or skips), controller Python compilation and all 21 Python/test files
  passing ament_flake8. Both controller/interface packages and the complete
  15-package workspace build successfully; `git diff --check` is clean. The
  workspace-wide historical test-result aggregation still reports the unchanged
  vendored `dobot_bringup_v4` lint failures and a stale removed `dobot_demo`
  result path; neither package was edited. No robot, camera or detector service
  was launched or commanded during verification.

### 2026-09-16 — Copyable controller command log in the GUI

- The operator requested that the unused lower instruction/blank area of the
  Robot Controller GUI become a live text log, specifically so every controller
  Dobot service call can be inspected and copied without opening a terminal or
  the package JSONL file.
- Rule 69 adds reliable transient-local `/robot_controller/operator_log` with a
  retained depth of 1,000 in both GUI and headless deployments. The authority
  publishes timestamped state transitions, operation phases, and the exact
  paired service `SEND`/terminal lines introduced by rule 68. This is a
  human-readable observability topic only; typed status/actions/services and the
  bounded JSONL event log remain authoritative.
- The GUI replaces the lower static explanatory label and stretch with a dark,
  read-only, no-wrap `QPlainTextEdit` capped at 1,000 lines. It drains ROS
  callbacks through a locked bounded queue on the Qt refresh timer, preserves
  message order, follows the tail only when already at the bottom, supports
  selection/Ctrl+C, and provides one `Copy Log` button for the complete visible
  buffer. The GUI still creates no Dobot client.
- Verification is source-only/offscreen: controller tests cover complete-buffer
  clipboard copying and ordered queue draining; compilation, ament_flake8,
  package/root builds, staged review and `git diff --check` remain required. No
  hardware service or action may be invoked by verification.
- Verification completed with all 73 focused tests passing directly and through
  the package CTest wrapper (74 reported tests, zero errors, failures or skips),
  Python compilation and all 21 controller/test files passing ament_flake8.
  The controller package and full 15-package workspace build successfully. An
  isolated ROS domain 229 offscreen launch started all three processes without a
  hardware call; a reliable transient-local subscriber received the retained
  startup line from `/robot_controller/operator_log`, and Ctrl+C stopped all
  processes cleanly. `git diff --check` is clean.

### 2026-09-16 — Skip redundant initial Home within completion tolerance

- The operator asked for the existing arrival tolerance to be stated and reused
  before Home, so Hardware Home and Pick do not send a redundant Home path when
  the robot is already sufficiently close to taught Home.
- The established completion limits are unchanged: Home is within one degree on
  each of six actual joints; Cartesian targets are within 5 mm Euclidean
  translation and one degree orientation. Both require fresh enabled,
  fault-free, queue-empty/stationary feedback coherent for 300 ms. Home remains a
  joint-only completion contract and does not add an FK/GetPose equality test.
- Rule 70 applies the exact joint Home gate before only the shared initial Home
  step used by both native actions. A passing gate logs `motion skipped` and
  bypasses GetPose plus every Home motion service. An outside-tolerance sample
  follows the unchanged conditional GetPose/vertical-rise/joint-MovL plan.
  Trusted holding still validates DI1 and all expected outputs throughout.
- Pick-attempt return Home is deliberately not optimized away: that Home is
  appended while retract targets are still merely planned, so current feedback
  cannot prove the future queue endpoint. Those queued returns continue to end
  at exact taught Home before success or candidate advance.
- Verification is synthetic/source-only and must cover the inside/outside
  one-degree boundary, queue-idle requirement, 300 ms coherence, shared
  Home/Pick skip path and unchanged queued returns, plus compilation, lint,
  package/root builds, staged review and `git diff --check`. No hardware service
  or action may be invoked.
- Verification completed with all 77 focused controller tests passing directly
  and through the package CTest wrapper (78 reported tests, zero errors,
  failures or skips). Tests accept the exact one-degree boundary, reject 1.01
  degrees, require an empty queue and the 300 ms stable gate, prove the shared
  initial Home sends no plan/move, and prove queued pick returns bypass the skip
  check. Python compilation and all 21 controller/test files pass ament_flake8;
  the controller package and complete 15-package workspace build successfully,
  and `git diff --check` is clean. No hardware was launched or commanded.

### 2026-09-16 — Accept the armed Item Teach candidate-service authority

- A live Pick reached DETECT after correctly skipping an already-complete Home,
  but the controller rejected the sole `/item_detect/get_item_poses` server
  because the advertising node was `/item_teach`. It then issued and confirmed
  the required safety Stop; no pick waypoint was dispatched. The ROS graph
  showed exactly one provider, and Item Teach was explicitly Armed. Simulate
  Trigger had independently succeeded because it uses the same internal fresh
  batch implementation without calling through the controller client.
- This exposed an internal ownership inconsistency: Item Teach intentionally
  advertises the canonical production service while Armed, and TF Preview
  already accepted the sole root `/item_teach` or `/item_detect` provider, but
  hardware Pick accepted only `/item_detect`.
- Rule 71 centralizes the service name and allowed node identities. Both hardware
  Pick and TF Preview now accept exactly one root provider named `item_teach` or
  `item_detect`; missing, namespaced, unknown, or simultaneous GUI/headless
  providers remain rejected. Profile/model/station/bin hashes, fresh request
  capture, response validation and all robot safety gates are unchanged.
- Verification is source/synthetic only: cover both allowed identities and every
  rejected ownership shape, run controller tests, compilation/lint, package and
  root builds, staged review, and `git diff --check`. Do not send a detector
  request or robot command during verification.
- Verification completed with all 83 focused controller tests passing directly
  and through the package CTest wrapper (84 reported tests, zero errors,
  failures or skips). The tests accept each sole canonical provider and reject
  missing, unknown, non-root and simultaneous providers. Python compilation and
  all 20 controller/test files pass ament_flake8; the controller package and all
  15 workspace packages build successfully. Live graph inspection was read-only,
  and no candidate request or robot command was issued during verification.

### 2026-09-16 — Stabilize current-pose acquisition after DO and Stop

- The first live Pick using the corrected armed-Item-Teach provider received and
  validated three fresh candidates. Candidate 1 confirmed DI1 clear, then sent
  and received successful `DO1=0` and `DO13=0` acknowledgements. Six milliseconds
  after the latter acknowledgement, `move_batch` sampled feedback once before
  GetPose and rejected it as not stationary READY. No GetPose or motion command
  was dispatched; the controller sent and confirmed Stop and correctly entered
  `RECOVERY_REQUIRED`.
- Service acknowledgement did not guarantee that the independently published
  RobotStatus/FeedInfo transition had already returned to idle. Treating that
  one transient sample as terminal was therefore an internal race, not a missing
  operator prerequisite.
- Rule 72 makes current-pose acquisition wait at most two seconds for 300 ms of
  advancing coherent idle feedback before dispatching GetPose. It uses the exact
  existing idle fields and trusted holding checks and remains cancellation/Pause
  aware. If the wait fails, no GetPose is sent: the error lists every current
  blocker, or distinguishes valid fields that failed only the stability window.
- Verification is source/synthetic only: test the stable wait parameters,
  delayed GetPose ordering, detailed blocker timeout, instability-only timeout,
  held-item validation and unchanged motion behavior; then run controller tests,
  compilation/lint, package/root builds, staged review and `git diff --check`.
  Do not send a detector request or robot command during verification.
- Verification completed with all 86 focused controller tests passing directly
  and through the package CTest wrapper (87 reported tests, zero errors,
  failures or skips). New tests prove the two-second/300 ms gate, that GetPose is
  dispatched only after it passes, and that blocked/unstable timeouts dispatch
  no GetPose and report the appropriate diagnostics. Python compilation and all
  20 controller/test files pass ament_flake8; the controller package and all 15
  workspace packages build successfully. No hardware or detector request was
  issued during verification.

### 2026-09-16 — Remove candidate result-age expiry

- The operator confirmed that accepted detection poses should remain valid until
  a later acquisition replaces them, rather than expiring while the robot runs
  Home, approach, settle, retract or candidate-retry motion. The editable
  `quality.result_max_age_sec` setting was therefore unnecessary and caused a
  valid batch to fail after motion consumed more than its two-second default.
- Rule 73 advances strict production Item Teach artifacts to schema 7 and removes
  `result_max_age_sec` from the YAML shape, Item Teach form/recovery, clicked and
  simulated previews, Item Detect response construction, and Robot Controller
  response/execution validation. Existing schema-6 artifacts remain loadable only
  as GUI recovery drafts and can be explicitly saved as schema 7; runtime readers
  do not gain a compatibility alias or implicit conversion.
- Acquisition safety is unchanged: RGB/depth/TF freshness and RGB/depth timestamp
  synchronization are checked while selecting the fresh request observation;
  `request_timeout_sec` still bounds queueing, inference, optional debug capture
  and response creation. Positive, synchronized, non-future result timestamps
  remain mandatory. Profile/model/camera/platform/bin hashes, generation/disarm,
  cancellation/Stop and source-file validation still invalidate results.
- Once one response passes those checks, Robot Controller latches that exact batch
  for its owning Pick action with no wall-clock expiry and no automatic
  reacquisition. Configuration/source validation continues before each candidate,
  so this change removes only elapsed-time rejection and does not permit stale
  configuration, altered artifacts or cached batches from another request.
- Shared Item Perception UI schema 6, camera schema 7, platform/bin schema 3,
  model bytes, item geometry/ranking, controller actions, motion and all robot
  feedback/safety gates are unchanged. The user-authorized workstation water
  teach/runtime YAML copies are updated separately to schema 7 with the removed
  quality key; their paired model is not modified and those operator artifacts
  remain outside the source commit.
- Verification is synthetic/source-only and must cover strict schema shape, UI
  field absence, accepted old clicked/simulated/service results, future/invalid
  timestamp rejection, request timeout, source invalidation, controller batch
  acceptance and unchanged synchronization checks. Run perception/controller
  tests, compilation/lint, package/root builds, staged review and
  `git diff --check`; do not call a detector service or robot command.
- Verification completed with all 442 focused perception/controller tests
  passing. Package-native CTest reports 356 Item Perception and 88 Robot
  Controller results with zero errors, failures or skips; the package builds and
  complete 15-package root build pass. Python compilation, Robot Controller
  ament_flake8, strict schema-7 offline/runtime profile loading, paired model hash
  validation and `git diff --check` pass. The broad Item Perception ament_flake8
  invocation still reports its existing 84-error package style backlog on
  pre-existing lines; no new functional failure was introduced. An accidental
  generated symlink-install cache was removed and the private runtime rebuilt as
  ordinary installed files; isolated OpenCV 4.10 import and the complete native
  worker tests then pass. No camera, detector service or robot command was launched.

### 2026-09-16 — Reload Robot Controller teach configuration without restart

- The non-headless controller previously accepted `/robot_controller/configure`
  only in `UNCONFIGURED` or `INACTIVE`, while its GUI disabled the configuration
  button after Startup reached `READY`. An operator therefore had to restart the
  controller process merely to reload an updated Item/Bin Teach pair.
- Rule 74 permits explicit configure/reload from idle unheld `READY` as well as
  the existing pre-Startup states. The normal single-operation lock still rejects
  concurrent Startup, Recover, Home, Pick, speed, or configuration work, and all
  active, paused, holding, Stop/recovery, unknown-held, and fault states remain
  ineligible.
- The complete replacement is loaded and source-validated before installation.
  Failure preserves the existing immutable configuration, configuration ID,
  Startup state, expected outputs, and lifecycle state. Success sends no Dobot
  command, installs the new snapshot/hash, clears preview TFs, invalidates prior
  Startup/global-speed/output assumptions, and transitions to `INACTIVE`; the
  operator must explicitly Start again before Home or Pick.
- The GUI dynamically labels the control `Load Teach Configuration` before the
  first load and `Reload Teach Configuration` while configured. Headless
  `runtime_teach/` configuration remains immutable until process restart.
- Verification is source/synthetic only: cover successful READY reload, rejected
  replacement preservation, every disallowed lifecycle state, held-item and
  headless rejection, GUI selection persistence/preview clearing, state-machine
  legality, compilation/lint, controller and workspace builds, staged review and
  `git diff --check`. Do not issue a detector request or robot command.
- Verification completed with all 103 focused Robot Controller tests passing
  directly and all 104 package-reported results passing with zero errors,
  failures, or skips. Python compilation and all 20 controller/test files pass
  ament_flake8; the controller package and complete 15-package workspace build
  successfully. No detector request or robot command was issued.

### 2026-09-16 — Align the gripper green axis to each item's short axis

- Investigation confirmed perception was not the source of the observed fixed
  orientation. The native geometry already defines candidate local X as the
  metric long axis and local Y as the short axis and returns a normalized
  platform-plane yaw quaternion. Candidate validation retained that quaternion,
  but controller and TF-preview planning transformed only XYZ; `pick_targets`
  copied the taught Home rotation into every item waypoint. Live audit records
  corroborated this: distinct candidate positions were all dispatched with the
  same `[-179.874, 0.085, -134.794]` degree RPY.
- Rule 75 consumes the existing heading without treating it as a full TCP pose.
  Compose the candidate through the destination platform transform, project its
  base-relative short axis onto the plane perpendicular to taught Home tool Z,
  then spin the Home attitude only about that local Z until Link6 green/Y is
  parallel to the short-axis line. Preserve tool Z exactly. The undirected
  rectangle permits a modulo-180-degree equivalent; choose the nearest solution,
  limiting the change from Home to 90 degrees.
- One aligned attitude applies to candidate transit, clearance, pre-pick, pick,
  retract and final clearance. Early-suction stopped-pose recovery continues to
  preserve the actual stopped attitude while rising, and the exact joint Home
  target restores taught orientation. Platform tilt continues to affect XYZ and
  item-heading transformation but never tilts the TCP or changes base-Z height
  equations.
- Hardware and TF Preview use the same candidate-pose composition and target
  planner. Reject nonfinite, non-normalized, non-yaw candidate quaternions,
  non-rigid transforms and degenerate short-axis projections before motion.
  Record candidate quaternion, transformed short/commanded green axes, applied
  tool-axis rotation and target RPY in the bounded controller event log.
- No ROS interface, Item Teach schema, model, station artifact or operator teach
  file changes are required. Existing source hashes, request ownership,
  cancellation, Stop, vertical geometry, I/O, retry and completion policies are
  unchanged.
- Verification is source/synthetic only: test rotated items, tilted platforms,
  nonvertical taught tool Z, modulo-180 shortest motion, every waypoint, malformed
  headings, shared hardware/preview construction, compilation/lint, controller
  package tests, package/root builds, staged review and `git diff --check`. Do not
  request detector poses or issue robot commands. Physical verification begins
  separately with TF Preview and then a supervised low-speed clearance trial.
- Verification completed with all 110 focused Robot Controller tests passing
  directly and all 111 package-reported results passing with zero errors,
  failures, or skips. Python compilation and all 20 controller/test files pass
  ament_flake8; the controller package and complete 15-package workspace build
  successfully. An additional source-only station-math probe covered headings
  from -90 through 179 degrees and confirmed exact tool-Z preservation, perfect
  short-axis/green-axis line alignment and a commanded turn no greater than 90
  degrees. No detector request or robot command was issued.

### 2026-09-16 — Make forward and return pick queues explicit

- The requested Home-to-pick then pick-to-Home batching is already the core
  behavior of `move_batch`: it dispatches all targets in order without waiting
  for intermediate Cartesian arrival, then monitors only the final target. The
  first candidate queue is transit at Home Z, clearance, pre-pick and pick. After
  terminal pick/stopped-pose confirmation and suction settling, the second queue
  is stopped-pose retract, clearance, conditional Home-Z rise and exact joint
  Home. Return construction does not begin before the terminal pick decision.
- Rule 76 makes those boundaries explicit and observable as
  `candidate_N_home_to_pick` and `candidate_N_pick_to_home`. Record batch start,
  each admitted target, complete queue admission, early-suction interruption and
  terminal completion with the batch name. GUI/action progress identifies the
  batch while its targets are being admitted.
- “Do not care about responses” is interpreted as no intermediate *motion
  completion* wait, not fire-and-forget ROS requests. The vendored bringup exposes
  MovL and MovLIO as distinct ROS services and synchronously writes each command
  over the dashboard socket. Preserve serialized request/response ordering:
  every response is mandatory evidence that its target entered the queue before
  the following target is sent. It is never treated as physical arrival. Ignoring
  those acknowledgements could reorder cross-service requests or queue later
  motion after an earlier rejection, violating deterministic Stop containment.
- Existing early-DI1 Stop, cancellation, Pause/Continue, late acknowledgement,
  timed outputs, settle, missed-pick retry, output integrity and terminal feedback
  policies are unchanged. There is no interface/profile/artifact schema change
  and no vendored source edit.
- Verification is synthetic/source-only: prove exact forward/return batch names
  for success and multiple missed candidates, no per-waypoint arrival API, return
  queue forwarding through the shared Home function, batch observability, focused
  controller tests, compilation/lint, controller/root builds, staged review and
  `git diff --check`. Do not issue detector requests or robot commands.
- Verification completed with all 112 focused Robot Controller tests passing and
  all 113 package-reported results passing with zero errors, failures or skips.
  Python compilation and all 20 controller/test files pass ament_flake8; the
  controller package and complete 15-package workspace build successfully.
  Source review confirms each normal command still waits for queue-admission
  acknowledgement and that only the final target enters the arrival loop. No
  detector request or robot command was issued.

### 2026-09-16 — Let every motion inherit global CP 100

- The latest hardware log proves Startup/Recover successfully sent `CP(100)`,
  while every subsequent MovL, MovLIO and RelMovLUser request explicitly supplied
  `cp=0`. The controller did not wait for intermediate positions: for candidate
  one it admitted all four forward commands before one terminal-pick completion,
  then admitted all four return commands before one exact-Home completion. The
  visible stop at each waypoint therefore came from the local zero-blending
  override, not a per-waypoint arrival loop.
- The vendored Dobot V4.6.5 manual defines per-command `cp` as the transition
  blend from the current instruction into the following instruction; when neither
  `cp` nor `r` is supplied, the configured global CP value applies. It also warns
  that smoothing can bypass the exact intermediate point and can execute timed
  output during the transition.
- Rule 77 removes `cp` and `r` from every controller MovL, MovLIO and RelMovLUser
  `param_value`. The strict Startup/Recover `CP(100)` response remains mandatory
  and is now the sole blending setting. Retain user/tool zero, per-target speed
  and acceleration, response serialization, named forward/return batches,
  terminal pick settling/stopped-pose confirmation, and exact taught-joint Home
  confirmation.
- This means transit, clearance, pre-pick, retract and conditional Home-height
  coordinates are blended planning control points, not guaranteed physical stop
  points. No schema, interface, teach artifact, target geometry or vendored source
  changes. Physical path-clearance commissioning remains an explicit separate
  task because CP 100 may round the horizontal/vertical and return-to-Home
  transitions.
- Verification is source/synthetic only: assert every generated motion parameter
  list contains exactly user/tool/v/a and no cp/r, preserve all-command admission
  before the single tail check, run compilation/lint, the full controller tests,
  package/root builds, staged review and `git diff --check`. Do not issue detector
  requests or robot commands.
- Verification completed with all 112 focused Robot Controller tests passing and
  all 113 package-reported results passing with zero errors, failures or skips.
  Python compilation and all 20 controller/test files pass ament_flake8; the
  controller package and complete 15-package workspace build successfully. The
  transport test proves each generated request contains exactly user/tool/v/a,
  both commands are admitted before the sole terminal check, and architecture
  tests reject any controller `cp=` or `r=` parameter. No detector request or
  robot command was issued.

### 2026-09-16 — Teach offset rotation and skip Home between missed candidates

- Item Teach now saves strict schema 8. Rule 78 adds one required top-level
  `pick_rotation` value with explicit degree units and an inclusive 0–90 range.
  It is an unsigned offset from the detected short-axis line. Production readers
  reject schemas 1–7; GUI recovery preserves those files and intentionally leaves
  the new field blank so the operator must review it before a schema-8 Save.
- The shared controller/preview orientation planner evaluates the +offset and
  -offset lines together with each line's modulo-180 equivalent while preserving
  taught Home tool Z. Candidate one minimizes absolute rotation from Home; later
  candidates minimize from the preceding candidate's selected attitude. Logs
  record the configured offset, chosen clockwise/counter-clockwise side, motion
  from the current reference, source/target axes and target RPY.
- A missed non-final candidate no longer returns exact Home. Its second queue is
  `candidate_N_pick_to_retry`: actual stopped-pose vertical retract followed by
  that candidate's final clearance. Only after terminal feedback and DI1-clear
  confirmation is suction turned off and the next forward queue started. Success
  and final exhaustion retain `candidate_N_pick_to_home` and exact taught Home.
  The same accepted candidate batch remains latched, and non-suction faults never
  advance.
- No ROS interface, detector response, station/bin/camera schema, vendored source,
  I/O timing, CP, response serialization, Stop/cancellation, or terminal-feedback
  contract changes. Verification is source/synthetic only and must not request
  detector poses or issue robot commands.
- Verification completed without hardware access: all 482 direct Item Perception
  and Robot Controller tests passed. Installed package testing reported 365 Item
  Perception and 119 Robot Controller results with zero failures, errors or skips,
  including the private native inference runtime. Python compilation and focused
  ament_flake8 checks for the changed core/controller/planner tests passed. Both
  changed packages and the complete 15-package workspace built successfully.
  Source review and synthetic tests cover schema-8 round trip/recovery, inclusive
  rotation limits, both offset directions, retry-reference selection, first-miss
  clearance-only routing, second-candidate success and final exact-Home return.
  No detector request or robot command was issued.

### 2026-09-16 — Calculate every candidate orientation independently from Home

- Operator correction: direct motion between missed candidates must not make
  their angular plans cumulative. Rule 79 supersedes rule 78's previous-candidate
  reference. Before any attempt, hardware Pick and TF Preview calculate every
  candidate's absolute attitude independently from the exact taught Home
  orientation, evaluating both signed `pick_rotation` offsets and every
  modulo-180 line equivalent.
- A failed non-final candidate still retracts only to its final clearance and
  proceeds directly to the next item. The robot may physically rotate from one
  candidate attitude to the next during that travel, but the next target itself
  is the precomputed minimum-turn solution from Home; no prior target angle is
  added to it. Logs now name the signed `rotation_from_home_deg` explicitly.
- Schema 8, Item Teach UI, pose batches, direct retry queues, success/final Home,
  tool-Z preservation, I/O, CP, response ordering, Stop/cancellation and feedback
  rules are unchanged. Verification remains source/synthetic only; do not request
  detector poses or issue robot commands.
- Verification completed with all 118 direct Robot Controller tests and all 119
  package-reported results passing with zero failures, errors or skips. Python
  compilation and focused ament_flake8 checks passed, the controller package and
  complete 15-package workspace built successfully, and architecture tests reject
  any previous-candidate orientation reference in controller, preview or planner.
  No detector request or robot command was issued.

### 2026-09-16 — Item-specific inward bin-wall pick clearance

- Rule 80 advances production Item Teach from schema 8 to strict schema 9 and
  adds `bin_clearance.p1_p2`, `p2_p3`, `p3_p4` and `p4_p1`. Each nullable value
  is an inward millimetre offset from the matching directed edge of the loaded
  portable Bin Teach polygon. Four blanks retain the prior behavior and draw no
  extra border; older profiles remain untouched and open only as GUI recovery
  drafts with the four new fields blank.
- The shared pure geometry shifts each edge toward the convex ROI centroid and
  intersects adjacent shifted lines in `platform_reference`. Negative values and
  collapsed, inverted, non-convex or outside inner polygons are rejected. A
  configured valid polygon is projected with the calibrated camera model as a
  light-blue border on both RGB and native registered-depth views.
- The existing green ROI remains authoritative for the complete item footprint
  and final depth-derived pick point. The new blue polygon is deliberately only
  a wall-clearance test on the exact depth-derived pick XY; its boundary is
  accepted and an item's detection/mask may cross it. Filtering occurs inside
  the shared clicked-pose, Simulate Trigger, Armed Item Teach and headless Item
  Detect candidate generator before ranking. Robot Controller receives the
  filtered candidates and has no duplicate border calculation.
- Invalid current-bin inset geometry blocks applying detection settings, Save,
  simulation and arming. Camera/platform/bin/shared UI schemas, station
  portability, model pairing, depth/MAD, pose orientation, controller motion,
  queues and I/O are unchanged. Verification is source/synthetic only; no
  detector service request or robot command is authorized for this change.
- Validation completed without hardware access: Python compilation and 230
  focused schema/GUI/detector tests passed; the complete source suite passed all
  367 tests, including private OpenCV inference/geometry/overlay execution.
  All 118 Robot Controller tests also passed. `colcon test` reported 368 Item
  Perception results with zero errors, failures or skips. The changed package
  and full 15-package workspace built successfully, and final diff checks
  passed. No detector request or robot command was issued.

### 2026-09-16 — Green ROI overlap eligibility and exact pick-point containment

- Rule 81 supersedes the complete-footprint portion of rules 32 and 80. A
  calibrated detection is eligible when its selected platform-Z=0 polygon
  overlaps or touches the green Bin Teach ROI. Only a fully disjoint detection
  is ignored; partial crossings, edge contact, containment in either direction,
  and edge crossings with no contained vertex are accepted.
- The shared calibrated live RGB/depth preview omits fully disjoint detections.
  Clicked selection, Simulate Trigger, Armed Item Teach and headless Item Detect
  apply the identical geometry rule in the candidate generator. If calibrated
  projection itself is unavailable, the live detection remains visible with its
  explicit measurement reason, but pose generation remains blocked.
- Candidate validity remains stricter at the exact pose: the final
  depth-derived pick XY must independently be inside/on the green ROI and,
  whenever configured, inside/on the light-blue wall-clearance polygon. A mask
  or OBB may cross either border; crossing light blue does not admit an outside
  pick point. Schema 9, artifact formats, model pairing, size/depth filters,
  ranking and Robot Controller behavior are unchanged.
- Verification completed without hardware access. Regressions cover partial
  overlap, exact touch, complete disjointness, containment in either direction,
  the no-contained-vertex edge-crossing case, candidate pick-point containment,
  calibrated live-preview omission, and filtered mask rendering. All 367 direct
  Item Perception tests passed; package testing reported 368 results with zero
  errors, failures or skips. Python compilation, focused source `ament_flake8`,
  `git diff --check`, the changed package build and the complete 15-package
  workspace build passed. No camera/detector node, pose-service request, Dobot
  bringup or robot command was launched.

### 2026-09-16 — Conservative projected-pixel pick clearance

- A live Simulate Trigger audit exposed item-height parallax at the bin edge.
  One returned point was `(-415.676, 320.037) mm` in `platform_reference`,
  mathematically 11.076 mm inside the configured blue inset, while its exact RGB
  center pixel `(310, 358)` appeared outside the blue polygon projected at
  platform Z=0. Its vertical plane projection would be near `(341.7, 353.7)`,
  explaining the discrepancy, but that substituted pixel is not the pick ray.
- Rule 82 adds a conservative visual gate without weakening physical geometry.
  A candidate must still pass depth-derived metric XY containment. Its unchanged
  RGB rectangle-center pixel must additionally be inside/on the projected blue
  polygon, or projected green polygon when no inset is configured. The worker
  rejects visible parallax failures before depth sampling and reports a precise
  projected-border reason; it never moves the center or changes returned XYZ.
- Clicked selection, Simulate Trigger, Armed Item Teach and headless Item Detect
  share the candidate generator, so the new gate applies uniformly before
  ranking. Detection-footprint overlap and live all-detection preview remain
  governed by rule 81 and may cross the light-blue point-only boundary.
- Verification reproduced both parallax directions synthetically: metric-safe
  but visually outside is rejected, visually inside but metric-outside remains
  rejected, and exact projected-boundary contact is accepted. All 367 direct
  Item Perception tests passed; package testing reported 368 results with zero
  errors, failures or skips. Python compilation, focused source `ament_flake8`,
  the changed package build, `git diff --check`, and the complete 15-package
  workspace build passed. The running Item Teach process and its historical
  read-only event log were inspected, but no camera/detector node was launched,
  no pose-service request was made, and no robot command was issued.

### 2026-09-16 — Link6 robot-camera-origin clearance and safe mirror

- Rule 83 extends the strict calibrated pick contract without changing Item
  Teach schema 9, Bin Teach schema 3, camera schema 7, or the raw
  platform-relative candidate message. In addition to the platform-bound bin
  camera, Item Teach/Detect and controller select the newest independently
  validated `robot_camera` schema-7 `camera_on_hand` calibration, strictly
  `Link6 <- robot_camera_link`. The robot-camera artifact supplies only a saved
  rigid transform: no robot-camera RGB/depth/CameraInfo/TF subscription or new
  live camera prerequisite. Missing, ambiguous, invalid, superseded or changed
  calibration blocks pose generation/planning without an older-file fallback.
- The shared pure `pick_planning` module holds canonical CR10 Home FK, the
  original shortest taught-Home-relative item short-axis/`pick_rotation`
  attitude, and the new clearance selector. Compose Link6 at the planned
  item's base XYZ plus `standoff_height`, then compose `Link6 <- robot_camera_link`
  and convert the camera origin to `platform_reference`. Its XY must be inside
  or exactly on the green Bin Teach ROI. If the normal shortest attitude is
  outside, retest precisely 180° around the unchanged taught Home tool-Z;
  the fallback reverses Link6 red/X and green/Y without changing pick XY/Z,
  item-axis line, offset angle, or vertical geometry. Only this safety fallback
  may exceed the normal ≤90° Home-relative turn.
- If both camera origins are outside green, discard the detection before
  center-first ranking and `pose_candidates` capping. The next safe detection
  moves into the batch and is tried on a missed-suction retry; no unsafe pose is
  returned. Clicked Item Teach, Simulate Trigger, Armed Item Teach and headless
  Item Detect use the same worker candidate generator. Controller TF Preview
  and hardware independently recompute the same selection using the selected
  calibration and saved Home; disagreement blocks planning before any new
  candidate motion and is a non-suction failure, not a retry. Robot-camera
  SHA-256 is included in controller configuration and detector evidence.
- Draw selected magenta `CAM`/`CAM 180` camera-origin footprints projected onto
  platform Z=0 on both bin RGB and registered-depth views; uncapped rejected
  previews show both outside camera origins in red with a precise reason, and
  candidate-only simulation preserves the previous returned-candidates-only
  overlay while reporting excluded camera candidates in text/diagnostics.
  Existing green detection overlap, exact green/blue pick-point tests,
  projected-pixel parallax check, size/depth/MAD filters, model-pair integrity
  and queue/Stop/I/O behavior remain intact. Green safety considers only the
  calibrated camera-link origin, not its housing; physical green-ROI safety
  margin is the operator's responsibility. No physical commissioning occurred.
- Software verification: 495 direct Item Perception and Controller tests pass,
  including unsafe-first/safe-next ranking, exact tool-Z mirroring, shared
  waypoint attitude, strict latest calibration binding, and independent
  controller evidence checks. Scoped `colcon test` summaries pass (375 Item
  Perception tests and 122 Controller tests, zero errors/failures/skips).
  The full 15-package `colcon build`, Python compilation, focused Python lint,
  unchanged `ItemCandidate` interface inspection, and `git diff --check` pass.
  No camera stream or physical robot was commanded or commissioned.

### 2026-09-16 — Two-second controller service replies

- Rule 84 supersedes only the previous five-second Dobot service-response
  deadline. Every serialized normal request, independent Stop acknowledgement,
  and Pause/Continue request now has two seconds for a ROS reply with `res=0`.
  A timeout remains ambiguous: send no later normal command, preserve the
  existing Stop/recovery containment, and log the exact timed-out service.
- Service discovery remains five seconds. The distinct five-second DO/output
  feedback window is retained rather than silently shortening physical output
  confirmation. Feedback age, mode transitions, motion watchdog/cap, and final
  target arrival checks are unchanged; queued pick commands still await each
  service reply but do not wait for intermediate physical arrival.
- Verification is software-only; 122 direct Robot Controller tests and 123
  package-reported tests pass with zero errors/failures/skips, including a
  synthetic MovL reply timeout that blocks the next dispatch at two seconds.
  Python compilation, focused lint, `git diff --check`, and the complete
  15-package `colcon build` pass. No robot, camera, or motion service was
  commanded.

### 2026-09-16 — Dispatch complete motion groups before reply verification

- Runtime evidence showed global `CP(100)` was accepted and no motion request
  overrode it, but sequential service-response waits plus synchronous progress
  work took roughly one second between adjacent queue entries. Short segments
  could therefore empty the Dobot queue and decelerate even though CP remained
  configured.
- Rule 85 supersedes rules 64, 76, 77 and 84 only for motion-group admission.
  Build the complete named `MovL`/`MovLIO`/`RelMovLUser` request group, dispatch
  every request in target order without waiting between entries, then validate
  all group replies under the two-second response deadline. Only after every
  reply is `res=0` does the controller accept the group and continue its existing
  terminal-only physical feedback verification. Non-motion calls remain strictly
  response-serialized.
- A partial dispatch, response exception, nonzero/empty response, cancellation,
  or group timeout immediately requests independent Stop. Unfinished responses
  are marked ambiguous, and a later motion acknowledgement invokes Stop again.
  Per-request console/operator/event auditing remains mandatory even though
  replies are verified as a group.
- DI1 remains monitored during group-response validation and physical descent.
  Early acquisition performs final Stop containment after all dispatched replies
  resolve, then retracts under the established held-item contract. If terminal
  pick feedback completes with DI1 clear, the attempt is now a miss immediately;
  the controller no longer waits `timing.pick_settling`. Schema-9 profiles retain
  that field to avoid invalidating current paired artifacts, but runtime execution
  does not consume it. Terminal 300 ms stationary confirmation is retained as
  motion verification, not suction settling.
- Verification is source/synthetic only: exercise complete-before-wait dispatch,
  all-response acceptance, rejection/timeout/cancellation Stop containment,
  late response handling, terminal-only arrival, no final suction timer, CP
  parameter preservation, compilation/lint, package tests, package/root builds,
  staged review and `git diff --check`. Do not launch bringup, request detector
  poses, or command physical hardware.
- Verification completed with 125 direct Robot Controller tests passing and 126
  package-reported results with zero errors, failures or skips. The synthetic
  transport tests prove that every request is dispatched before reply waiting,
  all replies are then validated, rejection requests Stop only after complete
  dispatch, and a group timeout plus each late reply invokes Stop containment.
  Python compilation and all 20 controller/test files pass `ament_flake8`; the
  controller package and complete 15-package workspace build successfully. The
  first full build encountered a transient generated egg-info race while two
  unrelated Python packages rebuilt concurrently; an unchanged rerun completed
  all packages. `git diff --check` passes. No bringup, detector request, camera
  process or physical robot command was launched.

### 2026-09-16 — Pace grouped motion dispatch and track timed outputs

- Rule 86 refines rule 85: send adjacent `MovL`, `MovLIO`, and `RelMovLUser`
  requests in a named group at least 50 ms apart using monotonic time, while
  still dispatching the complete group before waiting for replies. Independent
  Stop remains immediate and bypasses this pacing; non-motion commands retain
  strict response serialization.
- Runtime service evidence showed the prior complete-group implementation sent
  adjacent requests only one to three milliseconds apart. The 50 ms lower bound
  is now explicit, cancellation-aware and continues feedback/suction monitoring
  during each short inter-dispatch interval.
- The same evidence exposed a false held-item fault. A successful pick with
  `use_grip=true` and `grip_onpick=false` correctly commanded DO14 OFF and DO2 ON
  at 100% of retract, but held-item monitoring still compared feedback with the
  pre-retract finger state. The controller now records pending timed outputs,
  accepts only the previously confirmed or exact commanded state during the
  transition, commits the commanded state when observed, and reconciles that
  legal observed state if Stop interrupts the group. DI1/DO13 loss, uncommanded
  channels and wrong final output feedback remain failures.
- Verification was source/synthetic only. All 129 direct Robot Controller tests
  and 130 package-reported tests pass, including exact 0/50/100 ms dispatch
  times, activation of each output allowance only after its owning MovLIO send,
  the planned DO2/DO14 return transition, rejection of an uncommanded DO1 change,
  and the existing Stop/late-response cases. Python compilation and all
  20 controller/test files pass `ament_flake8`; the package build and complete
  15-package workspace build pass. No bringup, detector request, camera process
  or physical robot command was launched.

### 2026-09-17 — Continuous CP-100 missed-pick retry queue

- Rule 87 supersedes rule 85's immediate terminal-suction miss and rule 78's
  intermediate clearance return. After the first Home-to-pick group reaches its
  final pose with DI1 clear, observe DI1 for the taught `timing.pick_settling`
  interval before declaring a miss. DI1 acquired in descent still invokes
  immediate independent Stop containment; terminal idle acquisition needs no
  extra motion Stop.
- On a missed non-final candidate, admit one group with stopped-pose vertical
  retract directly to the old pre-pick at `v=100`, the next candidate pre-pick,
  and its final descent. At 20% of retract, turn DO13 OFF and DO1 exhaust ON;
  when `use_grip`, also set DO2 OFF/DO14 ON. At 0% of the next pre-pick transfer,
  turn exhaust OFF and reissue enabled finger-open outputs. At 0% of the next
  final descent, turn suction ON. No old clearance, Home-Z, Home, or per-waypoint
  arrival wait is inserted. The next attitude remains independently derived
  from taught Home. If the missed-pick DO13-OFF feedback is not observed, or
  DI1 activates before that reset, fault and Stop rather than crediting the
  next item. After final exhaustion, finish the release/retract, clear exhaust,
  and run shared Home. Success still retracts/returns Home holding suction.
- Every motion request continues to omit `cp` and `r`; Startup/Recover's strict
  global `CP(100)` applies. Group requests retain ≥50 ms spacing and all ROS
  response verification. If DI1/Stop/cancellation occurs during dispatch,
  later targets in that group are not sent. Independent Stop is not paced.
  Candidate source hashes are checked before each group, not repeatedly inside
  every 100 Hz feedback callback; repeated file hashing had delayed motion
  admission and could empty the CP-blended queue.
  CP blending makes intermediate points approximate, so direct low-height
  inter-item travel requires separate physical collision-clearance commissioning.
- The same controller audit fixes the ROS logger's severity-by-callsite conflict:
  INFO/WARNING/ERROR service audit lines now originate at distinct static lines.
  A failed Stop or Recover response must be reported as itself, not masked by
  rclpy's "Logger severity cannot be changed between calls" exception.
- Verification is software-only: inspect timed-I/O ordering and motion
  parameter lists, test missed/success/final retry and interruption behavior,
  run controller tests, compilation/lint, package and root builds, and review
  the staged diff. Verification completed with 133 direct controller tests and
  134 package-reported tests passing, including timed-output order, no per-call
  CP/r override, observed vacuum reset, settled miss, and suppression of later
  motion dispatch after suction interruption. All 20 controller/test Python
  files pass compilation and ament_flake8. The controller package and full
  15-package workspace build pass; `git diff --check` passes. No Dobot service,
  detector service or live camera was commanded by this change.

### 2026-09-17 — Verified Home height and higher repick approach

- Rule 88 supersedes rule 85's simultaneous multi-service dispatch and rule
  87's pre-pick-height missed-pick transfer. During a live final-miss return,
  controller audit sent `RelMovLUser(+151.010 mm)` before joint-mode `MovL`
  Home, but the Dobot dashboard log actually accepted the joint command as
  queue ID 22 before the rise as ID 23. Both returned `res=0`, yet FeedInfo
  showed no measurable motion; the controller's three-second watchdog issued
  Stop and ended in `RECOVERY_REQUIRED`. Independent ROS service callbacks do
  not preserve cross-service dashboard order. Acceptance never proves arrival.
- Every normal motion group now waits for each individual `res=0` acceptance
  before dispatching the next command, with the existing ≥50 ms lower spacing
  bound. It does not wait for intermediate physical arrival in pick/retry
  groups. Any response timeout, rejection, cancellation, feedback fault, or
  DI1 interruption sends no later group command and uses independent Stop
  containment. Late acknowledgements still invoke another Stop. This may
  reduce CP smoothing on short segments, but preserves command order.
- Every Home target, from initial Home, Pick start, successful return, or final
  missed return, uses one shared physically confirmed rule. A Pick return first
  completes and verifies its above-item approach/clearance; final miss now
  reaches that same clearance before Home. If the current measured Link6 Z is
  below taught Home Z, send only the current-XY/attitude vertical
  `RelMovLUser` rise and confirm its Cartesian 5 mm/1° endpoint plus 300 ms
  stationary/empty-queue feedback before sending joint Home. When already at
  or above Home Z, the conditional rise is omitted. Exact taught Home joints
  are then sent and confirmed within ±1°. Failed height confirmation blocks
  the joint Home command. Trusted held-item suction/output monitoring remains
  active through all three return phases; Stop/cancellation still never
  releases, resumes or automatically homes.
- A missed non-final item now rises vertically to its taught approach height
  (`pick Z + prepick_height + retract_height`) at commanded `v=100`, with
  suction OFF/exhaust ON and enabled finger-open at 20% of that rise. The
  direct lateral transfer targets the next item's approach/clearance, turning
  exhaust OFF and reissuing enabled finger-open at its 0% start; descent then
  passes through next pre-pick to next final pick with suction ON at 0% of
  final approach. The group keeps only final-pick physical arrival checking.
  Candidate attitudes remain independently planned from taught Home.
- Global `CP(100)` is still established by Startup/Recover and no motion call
  overrides `cp` or `r`. Home clearance and height are intentional confirmed
  barriers, not CP-blended waypoints. No profile/interface schema, station
  artifact, vendor source, or detector change is made. Physical clearance of
  the direct high-level inter-item transfer remains a separate commissioning
  responsibility; this change does not command the robot.
- Verification is software-only: 137 direct controller tests and 138
  package-reported tests pass, including ordered cross-service admission,
  Home-height failure containment, measured-pose jitter, and higher retry
  clearance. All 20 controller/test Python files pass compilation and
  `ament_flake8`; package and full 15-package workspace builds pass, and
  `git diff --check` passes. No Dobot motion service was invoked by this work.

### 2026-09-17 — Streamed actual tool pose for controller motion origins

- Rule 89 supersedes the historical `GetPose(user=0,tool=0)` motion-origin
  requirements in rules 51, 53, 62, 64, 70 and 72 without changing their
  other Home, feedback or Stop contracts. Read-only ROS graph inspection found
  that the vendored bringup publishes `ToolVectorActual` at approximately 10 Hz
  and embeds the same actual six-axis tool vector in FeedInfo at approximately
  100 Hz. The controller already consumes FeedInfo for final Cartesian arrival,
  while repeated dashboard GetPose requests added response latency and could
  describe a later instant than the feedback sample that passed readiness.
- Each Home branch, stopped-pick measurement, and motion-batch origin now uses
  `tool_vector_actual` from the exact FeedInfo snapshot that passed the existing
  two-second maximum wait for 300 ms of coherent stationary idle state. The
  snapshot must also show an advancing `controller_timer`; duplicate ROS
  publishes are tolerated for up to 150 ms, but a frozen controller source
  cannot establish or sustain a motion origin. Keep sole canonical
  fresh/connected RobotStatus, FeedInfo and joints, mode 5,
  enabled, fault/collision/queue clear, user/tool coordinate indices zero,
  cancellation/Pause handling and trusted held-item integrity. Invalid or stale
  vectors fail closed; no cached pose, GetPose fallback or separate 10 Hz
  ToolVectorActual subscription is permitted.
- Remove the controller's GetPose service client and brace-reply parser from
  runtime prerequisites. Motion commands, exact taught-Home joint completion,
  Cartesian arrival tolerances, response ordering, suction, Stop, Item Teach
  schemas and vendor code remain unchanged. An attended, read-only comparison
  of streamed pose against explicit GetPose(0,0) should be completed during
  physical commissioning before relying on the new motion-origin source; no
  robot motion is authorized by this software change.
- Verification is software-only: 144 direct controller tests and 145
  package-reported tests pass, including pose position/attitude conversion,
  advancing-timer gating with duplicate-publish tolerance, held/idle frame
  blockers, stale/frozen/malformed feedback, and absence of the GetPose client.
  All 20 controller/test Python
  files pass compilation and `ament_flake8`; package and full 15-package
  workspace builds pass, and `git diff --check` passes. No detector request,
  Dobot service or physical robot motion was issued for this change.

### 2026-09-17 — Two Cartesian waypoints for explicit Hardware Home

- Rule 90 supersedes rule 88 only for the explicit `/robot_controller/go_home`
  action used by the Hardware Home GUI button and headless action clients. Read
  one fresh, stationary, advancing FeedInfo Link6 pose. If it is already within
  the existing 5 mm/1° Cartesian arrival tolerance of the FK-derived taught
  Home pose, send no motion. Otherwise send two separately confirmed
  Cartesian-mode `MovL` targets: current X/Y with taught Home Z and attitude,
  then the complete taught Home Cartesian pose. Each target uses taught travel
  speed/acceleration, user/tool zero, no timed I/O or per-command CP override;
  the first must be accepted and physically confirmed stationary/queue-empty
  before the second is sent. Preserve trusted held-item suction/output checks,
  native cancellation and independent Stop containment. The final confirmation
  is Cartesian, not a taught-joint equality claim.
- Pick's initial and return Home path remains the rule-88 shared conditional
  vertical `RelMovLUser` rise plus joint-mode `MovL` to exact taught joints.
  This intentionally distinguishes the explicit Hardware Home action from
  Pick's recovery/return contract without changing Item Teach schema or
  calibration/model files.
- This two-target Cartesian route is not collision-validated from an arbitrary
  starting pose: the first segment may rotate or descend at current XY, and
  the same Cartesian Home can have a different joint solution. Physical
  clearance, reachability and acceptable configuration require separate
  attended commissioning; software verification must not command the robot.
- Validation performed: 152 direct controller tests and 153 package-reported
  tests passed, including Cartesian target construction for starts below and
  above Home Z, both `MovL` requests in Cartesian mode, ordered waypoint
  completion, first-waypoint failure containment, Cartesian skip, and unchanged
  Pick return. Changed Python files passed compilation and `ament_flake8`,
  `git diff --check` passed, and the root 15-package `colcon build` passed.
  No Dobot service or physical robot motion was invoked.

### 2026-09-17 — Add old-item pre-pick to continuous repick queue

- Rule 91 supersedes rule 88 only for a missed non-final candidate's upward
  transfer. After DI1 remains clear through the taught final-pick settling,
  form one `candidate_N_pick_to_retry_M_pick` motion group in this order:
  stopped-pose vertical rise to candidate N's pre-pick, continue vertically
  at the same XY/attitude to N's clearance, cross to M's clearance using M's
  independently preplanned attitude, descend through M's pre-pick, then reach
  M's final pick. No Home, Home-Z or intermediate physical-arrival wait is
  inserted. Admit each service response in order, retain the ≥50 ms spacing,
  and check only M's terminal pick and suction. Global CP(100) may round every
  intermediate target; clearance remains a commissioning responsibility.
- Both old-item upward segments use `v=100` and the taught travel acceleration.
  At 20% of only the first rise, turn suction DO13 OFF and exhaust DO1 ON,
  and open DO2/DO14 when enabled. The second rise repeats no output event. At
  0% of M's clearance transfer, turn exhaust OFF and reissue enabled finger
  open; turn suction ON at 0% of M's final approach. Keep DI1-before-suction-
  reset and all Stop/cancellation/output checks. Final candidate exhaustion
  retains its direct single rise to clearance, exhaust clear and shared Home;
  confirmed pickup and the initial Home-to-pick queue are unchanged.
- Validation performed: 152 direct controller tests and 153 package-reported
  tests passed, including five-target retry order, one-time timed release,
  travel acceleration on both upward segments, next-item transfer I/O,
  unchanged final-miss return, and transport `MovL`/`MovLIO` selection.
  Changed Python files passed compilation and `ament_flake8`; `git diff --check`
  and the root 15-package `colcon build` passed. No Dobot service or physical
  robot motion was invoked.

### 2026-09-17 — Remove motion-group dispatch floor after accepted replies

- Rule 92 supersedes rule 86's 50 ms minimum gap between motion-service sends.
  Rule 88 already waits for each `MovL`, `MovLIO`, or `RelMovLUser` ROS response
  with `res=0` before sending the next target in a named group. Once that
  response and the existing cancellation, DI1, feedback, and output checks
  pass, dispatch the next request without a separate timer wait. Keep the
  two-second reply deadline, Stop containment, late-acknowledgement Stop,
  CP(100), and terminal-only physical-arrival checks.
- The 2026-09-17 15:17 local Pick log showed the first three motion responses
  accepted before the next requests. The final-descent Dobot TCP command was
  accepted, but the controller reported stale/unavailable feedback about one
  second later and stopped. Removing the dispatch floor does not change the
  one-second feedback-freshness requirement or identify which stream became
  stale; diagnose that separately before further physical Pick commissioning.
- Verification is software-only: 152 direct controller tests and 153
  package-reported tests passed with zero failures/errors/skips. All 20
  controller/test Python files passed compilation and `ament_flake8`; the
  controller package and complete 15-package workspace built successfully,
  and `git diff --check` passed. No physical Dobot service or camera action was
  invoked.

### 2026-09-17 — CP-blend explicit Hardware Home targets

- Rule 93 supersedes rule 90 only where it physically confirmed `home_align`
  before dispatching final Cartesian Home. Explicit `/robot_controller/go_home`
  now submits both Cartesian `MovL` targets as one named motion group, requires
  each service response with `res=0` in order and without an added delay, and
  physically confirms only the final Home target. Both requests omit per-motion
  `cp`/`r`, so Startup/Recover's global `CP(100)` blends the transition.
- `home_align` remains current X/Y with taught Home Z/attitude, but it is now an
  approximate CP control point rather than a guaranteed reached pose. The
  operator must commission the complete blended route because the controller
  has no collision model and CP may round the intermediate point. Cartesian
  skip, 5 mm/1° final tolerance, 300 ms stationary confirmation, held-item
  monitoring, response failure containment, cancellation and Stop are retained.
- Pick's shared Home contract is unchanged: above-item clearance, conditional
  current-XY rise to Home Z, and exact taught-joint Home remain separately
  confirmed safety barriers. This change does not blend those safety segments.
- Verification is software-only: 152 direct controller tests and 153
  package-reported tests passed with zero failures/errors/skips, including one
  two-target Hardware Home batch and final-target-only arrival checking. All 20
  controller/test Python files passed compilation and `ament_flake8`; the
  controller package and complete 15-package workspace built successfully, and
  `git diff --check` passed. No Dobot service, detector request, or camera action
  was invoked.

### 2026-09-17 — Mutually exclusive pick I/O states and latched misses

- Rule 94 supersedes the earlier pick-output timing in rules 57, 87 and 91.
  The controller has three explicit finger states: OPEN is DO2 OFF then DO14
  ON, CLOSE is DO14 OFF then DO2 ON, and NEUTRAL is both OFF. Vacuum SUCK is
  DO1 OFF then DO13 ON, EXHAUST is DO13 OFF then DO1 ON, and NEUTRAL is both
  OFF. Every timed or immediate transition turns the opposing output OFF before
  turning the selected output ON. Feedback with DO1+DO13 or DO2+DO14 both ON is
  a motion fault. `use_grip` gates CLOSE only; OPEN and NEUTRAL remain part of
  every attempt, and DI12 is not awaited.
- Candidate 1 queues item X/Y at Home Z with OPEN at 50%, pre-pick with no I/O,
  and final pick with SUCK at 20%; it does not visit candidate clearance on the
  initial descent. Suction is issued exactly once for that attempt. DI1 is
  eligible only after that SUCK transition. DI1 during descent is contained by
  Stop; otherwise final-pose `pick_settling` completes before success/miss is
  decided. A low DI1 at the deadline irrevocably latches the attempt as missed.
- A successful `grip_onpick=true` attempt CLOSEs after DI1 and Stop containment.
  A successful `grip_onpick=false` attempt keeps OPEN through pre-pick and
  clearance, then CLOSEs at 0% of the clearance-to-item-X/Y-at-Home-Z motion.
  `use_grip=false` never CLOSEs. Both paths return through actual-stop,
  pre-pick, clearance, item X/Y at Home Z and shared Home with SUCK maintained
  and never reissued.
- A missed retry remains one blended group: rise to the old pre-pick and enter
  EXHAUST at 80%; start the old-clearance rise by entering finger and vacuum
  NEUTRAL; cross to the next clearance and enter OPEN at 50%; descend through
  next pre-pick; enter SUCK at 20% of the next final descent. The next candidate
  is armed only after DO13 OFF and DI1 clear were observed. Any late DI1 from
  the latched old miss is ignored and cannot become a late success, fault, or
  retry blocker. Final exhaustion performs the same EXHAUST/NEUTRAL two-rise
  recovery and then shared Home; later DI1 does not reclassify the result.
- Preserve rule 92 ordered `res=0` admission, global CP(100), two-second command
  deadline, cancellation/Stop containment, terminal target checks and held-item
  supervision. This is source-only work and does not authorize physical motion.
- Verification: 157 direct controller tests and 158 package-reported tests
  pass. Coverage includes target order/timing, all six actuator states, immediate
  and motion-timed CLOSE, universal OPEN with `use_grip=false`, late-DI miss
  latching, opposite-output rejection, retry reset/arming and final-exhaustion
  Home behavior. All 20 controller/test Python files pass compilation and
  `ament_flake8`; the controller package and complete 15-package workspace
  build pass. No Dobot service, detector request, camera action or physical
  robot motion was issued.

### 2026-09-17 — Make private native runtimes safe under symlink-install

- Rule 95 closes the generated-layout failure previously repaired manually in
  the schema-7 work. Item Perception and Camera Calibration must always copy
  their extracted locked YOLO/OpenCV or OpenCV runtime into their respective
  package install prefixes as ordinary files, even when the rest of the
  workspace uses ament `--symlink-install`. No file or directory inside either
  installed private runtime may be a build-tree symlink.
- Root cause: a symlink install made each installed `cv2/__init__.py` and
  `cv2.abi3.so` resolve into its package's `build/` runtime. OpenCV computed the
  build directory as its loader directory but left the installed runtime first
  on `sys.path`; its native-module re-import therefore loaded the Python `cv2`
  package again and raised the explicit recursion error. Item Teach correctly
  treated that native startup failure as terminal. Camera/ROS streams,
  calibration artifacts and model files were not causal.
- Replace each ament-intercepted `install(DIRECTORY ...)` with an install-time
  Python copier. Each copier validates its extracted lock/OpenCV files, removes
  only the exact package-private runtime destination, copies while dereferencing
  source links, excludes bytecode caches, rejects any resulting symlink, and
  revalidates the required installed files. This introduces no fallback or
  alternate runtime.
- Regression tests simulate the broken installed symlink layout, prove it is
  fully replaced by independent regular files, reject broad destinations,
  and retain extraction checksum/path-traversal tests. Verification must include
  actual package `--symlink-install` builds, absence of symlinks in both
  installed runtimes, and startup/use of the exact installed workers. This
  validation is read-only with respect to cameras/robots and must not issue
  detector, camera or Dobot service calls.
- Verification completed 2026-09-20: both targeted `--symlink-install` builds
  and root `colcon build` (15 packages) pass; both installed runtimes contain
  no symlinks. All 378 Item Perception and 59 Camera Calibration pytest cases
  pass through their package tests, including installed OpenCV worker geometry
  round trips. The installed Item Teach worker reports ready with OpenCV 4.10.0,
  NumPy 1.26.4, Torch 2.13.0+cu130, Ultralytics 8.4.150, one OpenCV thread and
  OpenCL disabled. The changed Python files pass compilation and `ament_flake8`;
  `git diff --check` passes. No physical hardware was commanded.

### 2026-09-21 — Five-second service replies without callback starvation

- Rule 96 supersedes rules 84 and 92 only where they require a two-second
  Dobot service-response deadline. Every serialized normal/settings/I/O call,
  ordered motion-group admission, independent Stop, and Pause/Continue call now
  waits at most five seconds for its exact ROS response and still requires
  `res=0`. Service discovery and output-feedback confirmation remain five
  seconds; one-second canonical feedback freshness, mode transitions, physical
  motion watchdogs, response ordering, and Stop/late-acknowledgement containment
  are unchanged.
- The 2026-09-21 candidate-1-to-candidate-2 retry exposed a controller-local
  executor starvation rather than a slow Dobot acceptance. The controller held
  its motion-group `response_lock` while each completed future callback tried to
  acquire that same lock. With one action thread and three completed callbacks,
  all four executor workers were occupied after the third accepted request, so
  the fourth response and canonical feedback callbacks could not run. Bringup
  recorded the fourth `MovL` accepted as queue ID 14 in about 9 ms; the
  controller processed its ROS future roughly 0.93 seconds later, immediately
  after the one-second feedback-freshness failure unwound the group lock.
- Motion-group completions now use lock-free group bookkeeping while normal
  single-response completion retains its protected pending-response cleanup.
  This does not parallelize motion dispatch: each `MovL`, `MovLIO`, or
  `RelMovLUser` response must still return `res=0` before the next request is
  sent. Late responses still invoke the independent Stop path.
- Verification is software-only. A threaded five-request regression exercises
  completion callbacks from separate executor-like threads and requires each
  callback to finish before the next dispatch; it reproduces the old worker
  exhaustion contract without ROS or hardware. All 158 direct Controller tests
  and all 158 package-reported tests pass with zero failures/errors/skips. The
  changed Python files pass compilation and `ament_flake8`; the complete
  15-package workspace build and `git diff --check` pass. No robot, camera,
  detector, or Dobot service was launched.

### 2026-09-21 — Use taught settling as the final-pick confirmation

- Rule 97 supersedes the fixed 300 ms terminal gate plus a separate taught
  suction wait at the final pick. The schema-9 `timing.pick_settling` value is
  now the one interval during which the final Cartesian target, advancing
  enabled queue-idle feedback, and complete commanded final output state must
  remain coherent while eligible DI1 is observed. DI1 during descent or this
  interval keeps the existing Stop-and-confirm acquisition path; low DI1 at the
  interval deadline latches the miss.
- The exact final feedback sample supplies the actual stopped pose. The
  immediate success return, missed-pick retract, or candidate retry carries that
  confirmed pose into its next motion batch, so neither stopped-pose capture nor
  the next batch applies another 300 ms wait at the bottom. Independently
  acquired origins and every Home, clearance, Home-height, and other non-pick
  endpoint retain the fixed 300 ms gate.
- The currently selected GUI teach profile already stores
  `timing.pick_settling: 0.1`, so its new final-pick confirmation is 100 ms. No
  operator YAML/model artifact or schema was changed. Feedback freshness,
  suction reset/arming, late-DI1 isolation, output confirmation, Stop
  containment and the five-second service-response deadline remain unchanged.
- Verification was software-only. All 158 direct Controller tests and all 159
  package-reported tests pass. The regression proves that a taught 200 ms value
  controls the final terminal interval while the non-pick constant remains 300
  ms, fails if the removed sensor wait is called, validates the exact returned
  terminal pose, and checks its propagation into immediate retract/return.
  Changed Python files pass compilation and `ament_flake8`; the complete
  15-package workspace build and `git diff --check` pass. No Dobot service,
  detector request, camera action or physical robot motion was issued.

### 2026-09-21 — Remove timed settling from every non-pick gate

- Rule 98 supersedes all earlier fixed 300 ms Robot Controller confirmation
  durations. The final pick's schema-9 `timing.pick_settling` is now the only
  timed motion-settling interval. Home, Home-height, clearance, retract and all
  other non-pick endpoints complete on the first fresh enabled, fault-free,
  queue-idle sample within the existing joint or Cartesian tolerance and with
  required I/O intact. The initial Home skip uses the same one-sample gate.
- Motion-origin acquisition still requires a fresh enabled idle sample and an
  advancing FeedInfo `controller_timer`, but no dwell follows that evidence.
  Stop and Pause still require two distinct samples with an unchanged tool pose
  and their required queue state; their former 300 ms duration is removed.
  Startup/Recover's 200 ms READY lifecycle coherence is separate and unchanged.
- The active Item Teach profile remains `timing.pick_settling: 0.1`, so only
  final pick waits for a 100 ms settling window. Artifact schemas, operator
  files, target tolerances, feedback freshness, held-item/output checks and Stop
  containment are unchanged.
- Verification was software-only. All 159 direct Controller tests and all 160
  package-reported tests pass. Coverage proves zero-duration non-pick terminal
  completion, one-sample Home skip, untimed advancing motion-origin capture,
  two-distinct-sample Stop/Pause confirmation and the retained taught final-pick
  window. All 20 controller/test Python files pass compilation and
  `ament_flake8`; the complete 15-package workspace build and `git diff --check`
  pass. No Dobot service, detector request, camera action or physical robot
  motion was issued.

### 2026-09-21 — Queue final exhausted miss through Home

- Rule 99 changes only the return after the final candidate has completed its
  taught final-pick settling interval with DI1 low. The controller now submits
  the actual stopped-pose rise through old pre-pick, old clearance, conditional
  relative Home-Z rise and exact taught-joint Home as one
  `candidate_N_pick_to_home` group. The Home-Z branch is planned from the final
  clearance target. Each motion service still must return `res=0` before the
  next is sent, but clearance and Home Z are not physically confirmed; only
  exact joint Home receives terminal feedback confirmation.
- The existing failed-attempt output sequence is unchanged: EXHAUST begins at
  80% of the first rise, finger and vacuum enter NEUTRAL at 0% of the clearance
  rise, and later DI1 remains ignored for the irrevocably latched miss. Global
  CP(100), taught travel rates, target geometry, output supervision,
  cancellation and Stop containment remain active. Non-final retry groups and
  successful held-item returns are unchanged.
- Explicit Hardware Home already queues Cartesian `home_align` and final Home
  as one ordered group with only final Home physically confirmed. It now passes
  the same fresh stationary pose used for planning as the confirmed batch
  origin, eliminating the second motion-origin acquisition before dispatch.
- Verification is software-only. All 159 direct Controller tests and all 160
  package-reported tests pass with zero failures/errors/skips. Coverage verifies
  the single final-miss group, planned-clearance input to the Home-Z decision,
  preserved output events, final-Home-only confirmation, and one pose
  acquisition for explicit Hardware Home. All 20 controller/test Python files
  pass compilation and `ament_flake8`; the complete 15-package workspace build
  and `git diff --check` pass. No Dobot service, detector request, camera action
  or physical robot motion was issued.

### 2026-09-21 — Share the prefix-classified headless runtime catalog

- Rule 100 closes the deferred Item Detect deployment gap. Headless Item Detect
  and headless Robot Controller now use one shared selector for flat root
  `runtime_teach/`. Selection is by canonical filename prefix, followed by the
  existing strict readers: exactly one `item_teach_*.yaml`, its same-stem
  `item_teach_*.pt`, and exactly one `bin_teach_*.yaml`. Missing, duplicate,
  mismatched, symlinked, nested, unknown-prefix and unsupported-extension files
  fail. Hidden atomic-write entries are ignored. `tray_teach_` is reserved and
  fails explicitly until its artifact contract is designed.
- `item_detect.launch.py` now has zero launch arguments. Starting it explicitly
  selects the deployment catalog, validates/deserializes the one deployed model,
  applies current Item Teach settings and automatic station/robot-camera
  bindings, waits for fresh inputs, and advertises the read-only pose service.
  The model remains loaded while inference stays request-driven; every request
  still acquires a new RGB/depth pair after arrival and no cached pose is served.
  Runtime selection is immutable until restart, and hash/source changes retain
  the existing disarm/failure behavior.
- The current operator artifacts were copied, not moved, into `runtime_teach/`
  as ordinary files: schema-9 Item Teach YAML and paired model plus schema-3 Bin
  Teach YAML. Their source/deployment SHA-256 values match. These deployment
  files remain untracked operator artifacts and are excluded from the source
  commit.
- Verification is software-only. All 387 direct Item Perception tests and 388
  package-reported tests pass; all 160 direct Controller tests and 161
  package-reported tests pass, with zero failures/errors/skips. The seven
  changed Python/launch/test files pass compilation and `ament_flake8`; the
  complete 15-package workspace builds and `git diff --check` passes. The
  installed launch reports no arguments, and the installed selector resolves
  the copied Item YAML/model and Bin YAML by their expected prefixes. No camera,
  detector service, Dobot service or robot motion was launched.

### 2026-09-21 — Make runtime catalog count failures explicit

- Rule 101 makes every required runtime artifact count failure actionable. The
  shared catalog now reports missing Item YAML, Item model and Bin YAML inputs
  separately. A duplicate error identifies the artifact kind and lists every
  conflicting filename in deterministic order.
- Headless Item Detect already treated catalog selection failure as terminal. A
  regression test now verifies that it records the exact message as bounded
  `FATAL` `item_detector_failed`, emits the same ROS fatal log and exits before
  calibration selection or service readiness.
- Runtime deployment remains outside both consumers. It is manual now and may
  later be owned by a third-party program or remote node. That producer must
  finish one complete visible set before process start or restart; hidden
  dot-prefixed names may stage incomplete transfers. There is no automatic copy,
  directory watcher, selection retry, fallback or live replacement.
- Verification is software-only. All 393 direct Item Perception tests and 394
  package-reported tests pass; all 160 direct Controller tests and 161
  package-reported tests pass, with zero failures/errors/skips. The three changed
  Python/test files pass compilation and `ament_flake8`; the complete 15-package
  workspace builds and `git diff --check` passes. No camera, detector service,
  Dobot service or robot motion was launched.

### 2026-09-21 — Managed Pause parking and held-item put-back

- Rule 102 supersedes rules 65/66's retained vendor queue and paused-queue
  integrity policy. Controller Pause immediately requests independent Stop,
  resolves any admitted command response, then confirms another Stop with fresh
  stationary, empty-queue feedback. One operation owner performs parking and
  return; unanswered/rejected admissions prohibit all later normal commands.
  Vendor Pause/Continue clients are removed. The five-second service deadlines,
  canonical feedback/ownership/source checks and explicit recovery remain.
- Each accepted batch starts an in-memory ledger of PENDING candidates (the
  requested null/unattempted state). An accepted candidate approach marks ACTIVE;
  settling without suction marks FAILED, Pause marks an active attempt
  INTERRUPTED, confirmed suction marks HELD, paused loss marks DROPPED, and
  successful controlled held return marks RETURNED. Parking does not consume a
  pending candidate. Resume skips completed/interrupted attempts and retains the
  same batch without another detector request. Status publishes candidate IDs
  and states as parallel arrays and the GUI displays their order/states.
- Unheld Pick Pause neutralizes CLOSE/OPEN/SUCK/EXHAUST, physically confirms an
  upward-only rise at actual X/Y/attitude to at least Home Z, then crosses at
  that height and parks at the next pending pre-pick. With none left, it parks
  at safety height; Continue finishes Home with no pick. Continue establishes
  CLOSE OFF then OPEN ON before the next final approach and its normal SUCK
  event/taught settling. Idle READY Pause stays stationary. Paused standalone
  Home replans Home. No fixed non-pick dwell is introduced.
- Held Pause preserves outputs and rises vertically at actual X/Y/attitude to
  Home Z without descending if already above it. Fresh DI1 loss during parking
  or PAUSED is latched, stops any motion and runs put-back, even if DI1 bounces
  HIGH again or the item may already have fallen. Outputs stay preserved until
  the release pose. A dropped candidate remains DROPPED: completing this motion
  is not proof of physical placement. Stale/malformed feedback, unexpected
  output changes or another fault require Stop/recovery, never automatic
  release. Eligible suction acquired during the initial Stop establishes HELD;
  a latched failed attempt's late DI1 cannot become success.
- The trusted held candidate's original plan survives successful Pick at Home.
  Put-back rises to safety if needed, crosses to the original pre-pick, then
  reaches the same X/Y/attitude at nominal Link6 final-pick Z plus exactly 50 mm.
  Pre-pick below this release height or clearance at/below it is rejected before
  candidate motion and in preview. Equal waypoints omit zero-length motion, so
  neutral events always belong to a real upward retreat segment. Preview adds
  the put-back release frame. Artifact schemas/settings are unchanged.
- At confirmed release pose, command CLOSE OFF, SUCK OFF, OPEN ON, EXHAUST OFF,
  then native `DO(1,1,50)`. The vendored V4 manual's DO time argument is in
  milliseconds and automatically inverts that output on expiry; there is no
  Python sleep or runtime timing fallback. OPEN is established before EXHAUST
  and remains ON during it; separate commands do not promise simultaneous
  electrical edges. A bounded 1,000-sample output history retains pulse ON/OFF
  evidence even when it precedes the service reply. Pulse ON then OFF, neutral
  vacuum/CLOSE, OPEN ON, and DI1 LOW must be confirmed before retreat. The first
  real upward MovLIO segment commands all four outputs NEUTRAL at its start;
  pre-pick/clearance/conditional Home-Z/exact joint Home are one ordered group
  with only terminal Home physically confirmed. This is a distinct 50 ms
  release pulse; final-pick settling still comes only from the teach file.
- `/robot_controller/return_item` uses the existing typed Command service and
  reports request acceptance. Status reports PAUSING/RETURNING_ITEM and final
  READY; an interrupted native action returns CANCELED. The GUI exposes RETURN
  ITEM & STOP while paused with trusted holding, without canceling that action
  first. A paused drop uses the same return, then remains PAUSED at Home;
  Continue attempts remaining retained candidates even when the original Pick
  action had already completed. The next explicit new Pick replaces the old
  unheld ledger; process restart never restores holding/source context.
- Direct `/stop`, native cancellation, shutdown, or another Stop during parking
  or return pre-empts immediately, issues no later release/motion and requires
  recovery. An already-admitted timed exhaust pulse may still turn OFF on its
  robot timer. Continue is unavailable until parking is confirmed. The GUI keeps
  immediate Stop available while Pause/return acceptance or status is pending.
  Action completion and Continue handoffs are serialized with managed requests
  so a request at an executor boundary is neither lost nor given two owners.
- The typed status gained candidate_ids, candidate_states and can_return_item;
  rebuild the interface package and restart clients with the updated definition.
  Operator artifacts, private runtimes, vendored source and runtime deployment
  selection are untouched. No new offline-transfer milestone is claimed.
- Verification is software-only: all 197 direct controller tests pass and
  package results report 198 tests with zero errors/failures/skips. Coverage
  includes discarded-queue DO13/DO14 reconciliation, acquisition during Stop,
  paused drops and DI1 bounce, pulse feedback before its reply, unresolved
  admissions/source changes, immediate Stop during return, retained batches
  after completed Pick, and Pause/Continue/action-completion races. All 23
  controller Python/test files compile and pass ament_flake8. The full root
  symlink build succeeds for all 15 packages; installed managed-control modules
  and updated typed status fields load successfully. `git diff --check` passes.
  No real robot, camera or live detector commands were issued. Live GUI
  operation, clearance, physical pulse duration and item placement remain
  unverified.

### Future entry template

```text
### YYYY-MM-DD — short decision title

- Change:
- Reason:
- Upstream commit(s), if any:
- Validation performed:
- Offline transfer validation:
- Follow-up:
```
