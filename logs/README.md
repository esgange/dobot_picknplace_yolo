# Runtime datalogs

The project writes isolated package logs under this directory:

```text
logs/<package-name>/events.jsonl
```

Records are timestamped JSONL events. Each package file is capped at 1,000 records; record 1,001 truncates that package file before writing the newest event. Runtime logs are ignored by Git. Use `scripts/compile_logs.py` to merge package files into an explicitly selected universal output when needed; no logger-only ROS package owns this directory.

Project GUIs may also keep one ignored, atomically replaced runtime form-state
file beside their event log:

```text
logs/<package-name>/last_session.json
```

This file is a strict prefill convenience, not an event stream or an alternate
project-wide configuration source. It must never auto-apply settings or replay
hardware actions when a GUI starts.

`item_perception_yolo/last_session.json` uses strict schema 4 with independent
platform, bin and item-teach sections. Item Teach remembers only the last
validated named YAML, not a duplicate profile or external model source path.
The named YAML/model pair is authoritative and restores as unapplied prefill.
`robot_controller/events.jsonl` records explicit profile checks; the controller
does not automatically restore a previous selection.
