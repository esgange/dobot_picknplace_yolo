# Item teaching artifacts

Item Teach may select a source `.pt` from any folder on the PC. A new item name
creates a same-stem pair here; Save with the loaded item name updates that pair:

```text
item_teach_<ITEM_NAME>_<UTC_TIMESTAMP>.yaml
item_teach_<ITEM_NAME>_<UTC_TIMESTAMP>.pt
```

Keep both files together when transferring offline. The original model is
unchanged; the YAML references only the copied filename and its SHA-256, never
the source machine's model path. Explicit same-name updates keep one hidden
`.<stem>.previous.zip` containing the previous YAML and weights (when present).
The next update replaces that backup; it is manual recovery data, not a second
profile or automatic reader. Externally changed files require reloading before
overwrite. A failed copy or changed source prevents YAML publication; unchanged
paired weights are not rewritten. The success dialog names both files.

Strict item schema 9 groups data by purpose. Production rejects schemas 1–8;
only the Item Teach GUI may recover old/invalid files into an unarmed editable draft:

| Section | Saved data |
| --- | --- |
| `item` | Item name |
| `model` | Paired filename, SHA-256, declared task, `file_sha256_only` verification |
| `units` | Motion distances in mm, time in seconds, `pick_rotation` in degrees, home joints in radians, speed/acceleration in % |
| `home` | Six named joint positions, feedback timestamp, recording time, source IP/node/topic |
| `pick_rotation` | Unsigned 0–90° offset from the detected item short-axis line |
| `motion` | `standoff_height`, `prepick_height`, `retract_height` |
| `speed` | `travel_percent`, `approach_percent`, `retract_percent` (integers 1–100; initial 100/6/6) |
| `acceleration` | The same three phase keys (integers 1–100; initial 100/100/100) |
| `timing` | `pick_settling` |
| `gripper` | `use_grip`, `grip_onpick` |
| `retry` | `pose_candidates`: maximum ranked poses requested for controller retries |
| `yolo` | `confidence`, `iou`, `image_size`, `max_detections`, `class_ids` |
| `geometry_source` | Explicit mask, OBB, or none for RGB-only preview |
| `geometry` | Long-side `height`, short-side `width`, ± `tolerance`, `pickdepth_radius` (all mm; the last key means diameter, default 30 mm) |
| `bin_clearance` | Optional inward mm offsets for directed Bin Teach edges `p1_p2`, `p2_p3`, `p3_p4`, `p4_p1`; null means no inset on that edge |
| `quality` | Input/TF age, RGB-depth synchronization, request deadline, depth range, minimum retained-depth count/fraction |
| `controller_contract` | Validation-only stage, motion disabled, Link6, vertical routine, start/end home and fixed I/O map |

`yolo.image_size` is retained as reproducible inference metadata, not an editable
GUI field. New profiles record 640; loading preserves the exact validated value
already in that profile. Existing files are not rewritten or silently normalized.

`retry.pose_candidates: 3` requests up to three valid poses, not three extra
retries after an initial pick. `yolo.max_detections` remains a separate inference
cap. Explicit controller real mode executes Home/vertical picking and retries
only missed suction after confirmed final retract; default controller mode is
TF-only debug. Teaching never commands motion.

Travel rates apply to Home, XY transit, initial positioning and descent to
pre-pick; approach rates apply only to final descent. Slow retract applies to
pick-to-prepick; the remaining clearance and Home return use travel rates.
Pick Z=item Z+standoff, pre-pick Z=pick Z+prepick, clearance Z=pre-pick Z+retract,
in robot base Z. The candidate attitude preserves taught tool Z and applies the
nearest legal clockwise/counter-clockwise `pick_rotation` offset from the item
short-axis line relative to taught Home. Every candidate is solved independently;
retry orientations do not accumulate. zheight_offset is removed, not an alias.
The controller supplies vendor
per-command `v=`/`a=` while global SpeedFactor stays 100%. These are percentages,
not absolute velocity/acceleration. Slow rates do not relax acquisition
freshness or the request deadline; an accepted candidate batch has no age expiry.
Missing/invalid rates in recovery drafts are blank, never automatically filled.
Old retract_height is also blank in GUI recovery: its meaning changed from an
offset above pick to extra clearance above pre-pick. Review and fill it explicitly.
Schema-7 and older recovery drafts also leave `pick_rotation` blank rather than
assuming zero; review and enter 0–90° explicitly.
Schema-8 and older recovery drafts leave all four bin-wall fields blank. A valid
configured inset is shown light blue on RGB/depth and filters only the final
depth-derived pick point; the green ROI still requires the complete item
footprint and pick point inside it. Collapsed/inverted/non-convex inset geometry
cannot be saved or armed.

GUI recovery keeps validated fields and blanks unclear/missing ones; unknown
booleans require an explicit choice. A clear old `retry_limit` is mapped for
editing only. Home must validate as a whole; an unverified model is not loaded.
Malformed/ambiguous YAML opens an empty draft. Review the warning/activity log,
complete missing settings and Save. The same known item name updates the loaded
file with a previous-version backup; a changed/unknown original name creates a
new pair. Recovered drafts cannot simulate, arm or reach the controller until
strictly validated and saved. Loading alone never overwrites files. The shared
UI-state format remains schema 6 (named-file prefill only); no recovered values
are autosaved.

Source robot identity is provenance only. Identical destination robots may use
the same recorded home joints; loading never replays them. A new home must be
recorded from fresh canonical feedback, not a zero/default pose.

Save copies and verifies file integrity; it does not execute weights. Explicit
Load Model runs the private worker, discovers actual classes/task, and checks
the selected class IDs. Checkboxes save only selected IDs in `yolo.class_ids`.
Declaring a task cannot convert model outputs. Armed GUI/headless detection
requires verified mask/OBB output and current station inputs. Only explicit
controller real launch plus operator actions authorize robot execution; files
and teach/model loading do not authorize motion or replace collision safety.

Save/Load remembers the last named artifact. A complete validated loaded or
startup-restored profile already counts as saved; no extra Save is required for
Simulate Trigger/Armed. Startup does not execute weights, enable YOLO or arm;
those remain explicit actions with all production validation. Edits disarm and
require Save, but retain the loaded overwrite target. Unsaved edits are not a
second profile store. See the package
[README](../../src/item_perception_yolo/README.md) for the editor workflow.
