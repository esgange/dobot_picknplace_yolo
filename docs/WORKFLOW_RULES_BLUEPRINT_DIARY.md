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

Future project packages should be added under `src/` with clear package boundaries. A likely application decomposition is:

```text
camera bringup → image/depth synchronization → YOLO detection
             → camera-to-robot TF/calibration → grasp target selection
             → MoveIt planning → safety-gated Dobot execution
```

Keep application code, launch files, calibration, and configuration separate from the two vendor directories. Do not fork or rename upstream package names unless there is a documented reason.

## 4. Operating rules

### Offline-first source policy

- The two vendor trees are regular tracked files, not submodules.
- A normal `git clone` of this repository must contain the vendor source; no `git submodule` command should be needed.
- Every new or changed project rule must be recorded in this diary in the same change. This diary is the durable source of truth when agents or contributors switch.
- The Dobot profile is intentionally limited to CR10: retain `cr10_moveit`, the CR10 URDF/Xacro and mesh assets, and shared runtime packages; remove other model MoveIt packages, URDF/Xacro files, and mesh directories.
- Never delete vendor license/notice/attribution files.
- Never commit a vendor update without recording its upstream URL, branch, commit ID, date, reason, and validation in this diary.
- Keep a clean separation between upstream snapshots and project patches. Prefer a new integration package; if a vendor patch is unavoidable, document the exact file and rationale.

### Development workflow

1. Read this diary and `README.md`.
2. Check `git status --short --branch` and avoid overwriting existing user changes.
3. Source the installed ROS 2 distribution and build from the repository root.
4. Make the smallest scoped change. Keep generated output in ignored `build/`, `install/`, and `log/` directories.
5. Run relevant tests or launch-file checks that do not move hardware. Run `git diff --check`.
6. Update this diary for a new package, architectural decision, vendor change, offline constraint, or failed validation.
7. Commit with a descriptive message. Review the diff and never force-push shared history.

### Hardware safety

Real Dobot motion is opt-in, not a default test. Before any hardware command, confirm the requested robot model, IP/network, remote-control mode, workspace clearance, emergency stop, speed limits, and collision assumptions. Camera and perception tests should run in simulation or sensor-only mode first.

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

Extract it on the offline PC. Install or stage the compatible ROS 2/MoveIt/Gazebo dependencies separately; source completeness alone cannot supply operating-system packages.

## 6. Vendor refresh workflow

Vendor refreshes happen only on an online maintenance machine:

1. Clone the official upstream repository into a temporary directory at the intended branch.
2. Record the old and new commit IDs and inspect `git diff --no-index`.
3. Copy only the reviewed snapshot into the matching `src/` directory; do not copy its `.git` directory. For Dobot, reapply the CR10-only profile and do not reintroduce other model assets.
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
- Defaults changed to `DOBOT_TYPE=cr10` in bringup, RViz, Gazebo, MoveIt, and action-client entry points.
- Validation performed: checked remaining model asset directories, package manifests, launch defaults, and documentation references; non-CR10 references left in `V4新增指令` are generic protocol documentation, not installed robot configurations.
- Follow-up: build the CR10-only workspace on a ROS 2 Humble host before hardware operation.

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
