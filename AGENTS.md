# Project agent instructions

Before making changes, read [`docs/WORKFLOW_RULES_BLUEPRINT_DIARY.md`](docs/WORKFLOW_RULES_BLUEPRINT_DIARY.md) and the root `README.md`. Treat the diary as the project handoff document when an agent or contributor changes.

## Non-negotiable repository rules

1. This project is offline-first. `src/DOBOT_6Axis_ROS2_V4` and `src/OrbbecSDK_ROS2` are vendored source snapshots, not Git submodules. Do not recreate `.git` markers, `.gitmodules`, or submodule entries.
2. Do not silently edit vendored upstream code. Put integration and application code in separate packages and record any intentional vendor patch in the diary with its reason and verification.
3. Preserve upstream license, notice, and attribution files. Never commit credentials, machine-specific paths, or generated `build/`, `install/`, and `log/` output.
4. Keep the repository buildable from its root with `colcon build`; test offline transfer with a Git bundle or source archive before declaring an offline milestone complete.
5. Hardware motion is safety-critical. Do not launch or command a real robot unless the user explicitly requests it and the robot/network/safety preconditions have been checked.
6. Every new or changed project rule must be written into the blueprint diary in the same change; the diary is the durable source of truth for project rules.
7. Before each commit, run `git diff --check`, inspect `git status`, and update the diary when architecture, workflow, upstream revisions, or constraints change. Do not force-push or rewrite shared history.
