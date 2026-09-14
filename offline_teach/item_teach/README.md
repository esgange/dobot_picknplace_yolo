# Item teaching artifacts

Item Teach may select a source `.pt` from any folder on the PC. Saving always
creates a new, same-stem pair here:

```text
item_teach_<ITEM_NAME>_<UTC_TIMESTAMP>.yaml
item_teach_<ITEM_NAME>_<UTC_TIMESTAMP>.pt
```

Keep both files together when transferring offline. The original model is
unchanged; the YAML references only the copied filename and its SHA-256, never
the source machine's model path. No existing file is overwritten. A failed copy
or changed source prevents YAML publication; the success dialog names both files.

Strict item schema 3 groups data by purpose (schemas 1–2 are rejected, not converted):

| Section | Saved data |
| --- | --- |
| `item` | Item name |
| `model` | Paired filename, SHA-256, declared task, `file_sha256_only` verification |
| `units` | Motion distances in mm, time in seconds, home joints in radians |
| `home` | Six named joint positions, feedback timestamp, recording time, source IP/node/topic |
| `motion` | `standoff_height`, `zheight_offset`, `prepick_height`, `retract_height` |
| `timing` | `pick_settling` |
| `gripper` | `use_grip`, `grip_onpick` |
| `retry` | `retry_limit`: total candidate attempts including the first |
| `yolo` | `confidence`, `iou`, `image_size`, `max_detections`, `class_ids` |
| `geometry_source` | Explicit mask, OBB, or none for RGB-only preview |
| `geometry` | Long-side `height`, short-side `width`, ± `tolerance`, `pickdepth_radius` (all mm; the last key means diameter, default 30 mm) |
| `quality` | Input/TF age, RGB-depth synchronization, request/result deadline, depth range, minimum retained-depth count/fraction |
| `controller_contract` | Validation-only stage, motion disabled, Link6, vertical routine, start/end home and fixed I/O map |

`yolo.image_size` is retained as reproducible inference metadata, not an editable
GUI field. New profiles record 640; loading preserves the exact validated value
already in that profile. Existing files are not rewritten or silently normalized.

Source robot identity is provenance only. Identical destination robots may use
the same recorded home joints; loading never replays them. A new home must be
recorded from fresh canonical feedback, not a zero/default pose.

Save copies and verifies file integrity; it does not execute weights. Explicit
Load Model runs the private worker, discovers actual classes/task, and checks
the selected class IDs. Checkboxes save only selected IDs in `yolo.class_ids`.
Declaring a task cannot convert model outputs. Armed GUI/headless detection
requires verified mask/OBB output and current station inputs; physical picking
remains pending. These files do not authorize robot motion.

Save/Load remembers the last named artifact as unapplied form prefill. Unsaved
edits are not a second profile store. See the package
[README](../../src/item_perception_yolo/README.md) for the editor workflow.
