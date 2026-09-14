# item_pick

`item_pick` is the operator node for executing a pick sequence from
`item_detect` output. It consumes the selected item pose, applies a
profile-specific tool teach offset, and sends the robot through approach, pick,
retract, and final Z-up motions.

## Executable

| Executable | Purpose |
| --- | --- |
| `item_pick` | Tkinter operator GUI and service endpoint for item-pick execution. |

## Build

```bash
cd ~/CATARM/apps/edge-station-node/src/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select item_pick
source install/setup.bash
```

## Run

```bash
ros2 launch item_pick item_pick.launch.py
```

Direct run:

```bash
ros2 run item_pick item_pick
```

## Inputs

| Input | Type | Source |
| --- | --- | --- |
| `bin_seek_pose` | `geometry_msgs/msg/PoseStamped` | `item_detect` selected item pose. |
| `dobot_msgs_v4/msg/ToolVectorActual` | `dobot_msgs_v4/msg/ToolVectorActual` | DOBOT bringup TCP feedback. |
| Active items yaml | `teach_files/items/<id>.yaml` | Single yaml resolved at startup; pick: section read on demand. |

The single items yaml is resolved from `profiles_dir` (default
`~/CATARM/apps/edge-station-node/teach_files/items`). It is shared with
`item_teach_node` (which owns `teach:`) and `item_detect_node` (which owns
`detect:`); `item_pick` owns the `pick:` section. The legacy
`item_detect_selected_profile.txt`, `<item>_tool.yaml`, and
`item_pick_runtime_settings.json` files are gone.

## Services

`item_pick` exposes:

| Service | Type | Purpose |
| --- | --- | --- |
| `item_pick/track` | `std_srvs/srv/Trigger` | Arms the same pick sequence as the GUI track button. |
| `item_pick/track_status` | `std_srvs/srv/Trigger` | Returns success while track is armed and waiting for a fresh item pose. |
| `item_pick/start_sequence` | `dobot_msgs_v4/srv/TrayInterceptStart` | Arms and starts the pick sequence with explicit settings. |
| `item_pick/retry_cached_candidate` | `std_srvs/srv/Trigger` | Starts the next cached same-frame YOLO candidate when post-pick recovery or no-DI retry leaves one available. |
| `item_pick/post_pick_drop_recover_cached` | `std_srvs/srv/Trigger` | Returns directly over the cached pick pose at +50 mm, releases, and final-Z-ups before retry. |
| `item_pick/clear_candidate_cache` | `std_srvs/srv/Trigger` | Clears same-frame candidate cache after tray target acquisition or explicit reset. |

## Two-stage descent canary

The normal item-pick descent remains a single monitored `MovLIO` from 50 mm
above the target at 5% speed. `EDGE_PICK_TWO_STAGE_DESCENT_ENABLED=1` can split
that path into a 50%-speed upper leg ending 30 mm above the target and the same
5%-speed monitored final leg. The switch height and upper-leg speed are
controlled by `EDGE_PICK_FAST_DESCENT_SWITCH_Z_UP_MM` and
`EDGE_PICK_FAST_DESCENT_SPEED_PERCENT`; invalid opt-in values fall back to the
legacy path. The feature is disabled in production templates and should be
enabled only as a single-station canary after reviewing
`item_pick_two_stage_descent_plan` and `item_pick_two_stage_descent_stop_envelope`
logs.

The `start_sequence` service type is shared with `tray_intercept`, so field
names contain `tray_*`; in this package those values are interpreted as
item-pick settings.

Example:

```bash
ros2 service call /item_pick/start_sequence dobot_msgs_v4/srv/TrayInterceptStart \
"{tray_vector_wait_timeout_sec: 60.0, ee_intercept_speed_mm_s: 100.0, tray_intercept_x_offset_mm: 0.0, tray_intercept_y_offset_mm: 0.0, ee_final_pose_angle_deg: 0.0, tray_standoff_z_mm: 100.0, follow_distance_mm: 0.0, post_follow_z_up_mm: 0.0, troubleshoot_tf_only: false}"
```

Robot service clients use the DOBOT bringup service root:

```text
/dobot_bringup_ros2/srv
```

Main robot services used:

- `Stop`
- `MovJ`
- `MovJIO`
- `MovL`
- `MovLIO`
- `DO`

## Teach Files Layout

`item_pick` expects exactly one yaml in `profiles_dir` (default
`~/CATARM/apps/edge-station-node/teach_files/items`). CATARM swaps that file
atomically; restart the node to pick up a new file. The node refuses to start
when zero or more than one yaml is present so duplicate/missing state is
surfaced immediately instead of silently picking one.

Each items yaml has the shape:

```yaml
schema_version: 1
item:
  id: <id>
  display_name: <optional>
teach:                  # owned by item_teach_node
  ros__parameters:
    # vision config, ROIs, masks, bin_id reference, ...
detect:                 # owned by item_detect_node
  ros__parameters:
    # UI runtime state previously in item_detect_runtime_settings.yaml +
    # item_detect_selected_profile.txt
pick:                   # owned by item_pick.py
  ros__parameters:
    item_length:        # optional millimeters
    item_width:         # optional millimeters
    size_tolerance:     # optional percent; 5.0 means detected X/Y dimensions must be within +/-5%
    use_fingers: true
    grab_on_pick: false
    relax_fingers_on_pick: false  # optional; DO1/DO2 OFF at pick
    item_standoff_z_mm: ...
    final_z_up_mm: ...
    pre_pick_settling_time_sec: ...
    pick_settling_time_sec: ...
    tool_offset_x_mm: ...
    tool_offset_y_mm: ...
    tool_offset_z_mm: ...
    tool_offset_rx_deg: ...
    tool_offset_ry_deg: ...
    tool_offset_rz_deg: ...
    dobot_tool_offset_x_mm: ...
    dobot_tool_offset_y_mm: ...
    dobot_tool_offset_z_mm: ...
    dobot_tool_offset_rx_deg: ...
    dobot_tool_offset_ry_deg: ...
    dobot_tool_offset_rz_deg: ...
    tf_only_mode: false
    item_pose_wait_timeout_sec: ...
```

The taught-size filter activates only when `item_length`, `item_width`, and
`size_tolerance` are all populated with valid numbers. Length and width are in
millimeters; tolerance is a percentage applied independently to both axes. If
any field is empty or missing, size rejection is disabled by default.

Section-ownership table:

| Section | Writer | Reads on |
| --- | --- | --- |
| `item:` | `item_teach_node` | startup of every consumer |
| `teach:` | `item_teach_node` | `item_detect_node`, `item_pick` (sanity) |
| `detect:` | `item_detect_node` | `item_detect_node` |
| `pick:` | `item_pick.py` | `item_pick.py` |

Each writer must round-trip sibling sections verbatim; the shared
`teach_io` helpers (C++: `item_perception_eyetohand/include/.../teach_io.hpp`;
Python: `item_pick/item_pick/teach_io.py`) enforce that contract via
read-modify-tmp-rename.

The GUI can:

- read the active item teach from the single items yaml at startup;
- preview the tool offset TF in RViz;
- save updated tool teach values with `Save Tool Teach` (rewrites the
  `pick:` section atomically and round-trips `teach:` / `detect:`);
- block arming when the items yaml has no valid `pick:` section or either
  required finger-control boolean is missing.

### Bins

The bin geometry referenced by `item_detect` lives in the single yaml inside
`bin_teach_dir` (default `~/CATARM/apps/edge-station-node/teach_files/bins`).
`item_pick` only reads that yaml for camera-bin safety preference; the
`bin_id` recorded in the active items yaml is consulted as a sanity check and
warning is logged when it disagrees with the on-disk bin id.

### Calibration exception

Calibration files (`axab_calibration.yaml`, `platform_calibration_*.yaml`) and
their loaders are unchanged; they are not part of the single-file-per-dir
teach layout and live under `teach_files/calibration/` and
`teach_files/platform/`.

## Motion Sequence

On trigger, the node arms for a fresh `bin_seek_pose`. When the pose arrives, it:

1. Builds the two valid long-axis item poses: preferred and 180-degree flipped.
2. Chooses the orientation that keeps the wrist camera inside the active bin
   teach footprint. The camera offset in the `Link6` frame is read from the
   eye-in-hand calibration (`eye_in_hand_calibration_file`). With
   `require_camera_inside_bin` enabled (default), unverifiable safety data or
   poses where both orientations put the camera outside the ROI are rejected;
   the node waits for a fresh `bin_seek_pose` and retries up to
   `camera_bin_valid_pose_attempts` times.
3. Queues a `MovJIO` approach to the pose above the pick goal. At 50% of this
   approach move, DO1 is set off. DO2 is set on so the gripper opens before
   descent, or left off when `relax_fingers_on_pick: true` so both finger
   outputs are relaxed.
4. Queues a 5% `MovLIO` descent to pick depth. At 0% of this move, DO3 suction
   is turned on.
5. On DI detection, sends `Stop()`. With `use_fingers: true` and
   `grab_on_pick: true`, closes the fingers immediately, then runs the 0.10 s
   stop-settle. With `use_fingers: false`, the fingers remain actively open
   unless `relax_fingers_on_pick: true`, in which case DO1 and DO2 remain off.
6. Queues a fixed 80 mm `MovLIO` retract with suction kept on. With
   `use_fingers: true` and `grab_on_pick: false`, the fingers close at 90% of
   this retract (after about 72 mm); immediate-grab profiles retract with the
   fingers already closed.
   Relaxed suction-only profiles keep DO1 and DO2 off during retract. DO3
   suction timing is unchanged in every mode.
7. Queues the final Z-up `MovL` from that fixed retract pose.
8. Queues the fixed-home `MovJ`, then reports the pick sequence accepted by the
   controller without intermediate TCP or DI waits.

The node keeps up to 5 ranked YOLO candidates from the same frame until a tray
target is acquired. If the descent reaches full pick depth without DI, it waits
0.30 s for a late DI latch. If DI remains inactive, the final Z-up closes/purges
at 0% and relaxes/purge-off at 50%; when another cached candidate exists, the
next candidate starts directly from that final Z-up without a fixed-home wait.
If cached candidates are exhausted, the node queues fixed home and requests a
fresh seek only after `joint_states_robot` supplies a newer, non-stale J1..J6
sample matching the fixed-home target on every joint. Command-ID completion and
generic TCP stability alone do not certify home. Camera-bin pose-rejection
recovery uses the same fixed-home `MovJ` and joint proof before requesting
`item_detect/repick`; missing, stale, or off-target joint feedback fails closed.

If DI reports a post-pick drop before tray pose acquisition, the bridge calls
`item_pick/post_pick_drop_recover_cached`: the arm returns directly to the
original pick pose +50 mm, releases in the bin, final-Z-ups, and then the next
pick command tries the next cached candidate before falling back to normal seek.

Camera-bin pose preference can be disabled with `prefer_camera_inside_bin:=false`
and the hard requirement with `require_camera_inside_bin:=false`. The camera
offset comes from the eye-in-hand calibration at
`~/CATARM/apps/edge-station-node/teach_files/calibration/eye_in_hand.yaml`
(`transform.translation` is the camera origin in `Link6`); override the path
with `eye_in_hand_calibration_file:=<path>`. Runtime TF fallback is disabled
for this check; when the calibration file cannot be read, the pose is rejected
while `require_camera_inside_bin` is enabled.

## Debug TF Frames

When debug TF output is enabled, the node publishes target and tool-offset
preview frames including:

- `item_pick_tool_offset_preview`
- `item_movel_goal_tool_offset`
- `item_movel_goal_tool_axis_x_tip`
- `item_movel_goal_tool_axis_y_tip`
- `item_movel_goal_tool_axis_z_tip`
- `item_movel_goal_flange_tcp` (predicted Link6 pose at the goal TCP after
  the controller subtracts its internal Tool N TCP)

`item_movel_goal_flange_tcp` is derived from `dobot_tool_offset_*` in the
active items yaml's `pick:` section. RViz models the robot up to Link6 only,
so this frame surfaces the "fixed XYZ offset" between the URDF flange and
the controller's TCP that would otherwise be invisible.

## Notes

- Run `item_detect` first so `bin_seek_pose` is available.
- Drop exactly one items yaml into `profiles_dir` before launching; the node
  refuses to start on zero or more than one yaml.
- Use `Save Tool Teach` in the GUI to write the `pick:` section to that yaml
  before running a real pick.
- Use troubleshoot/TF-only mode to validate target frames before enabling
  robot motion.
