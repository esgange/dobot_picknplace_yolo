# item_perception_interfaces

`GetItemPoses` requests up to `max_candidates` from an explicitly SHA-256-bound
item profile. `save_debug_images` optionally writes the exact annotated RGB and
registered-depth result pair for that request into the workspace
`debug/pick_img/` directory. The armed detector advertises
`/item_detect/get_item_poses`.
Responses contain only observed item targets in `platform_reference`, with
image timestamps, batch-local IDs, diagnostics and explicit shortages/errors.
Position/dimensions use metres; item orientation is platform-normal with an
in-plane heading: local X is the short axis and local Y is the long
axis. It is not measured surface tilt or a robot TCP command. Robot Controller
uses local X only as an undirected gripper-heading line while retaining its
taught tool-Z attitude.

Acquisition ranks geometrically eligible poses first, then checks nearby depth
in rank order. Skip blocked candidates and stop when `max_candidates` pass or
the list is exhausted. `valid_count` equals the number of fully checked returned
poses; it is not an exhaustive count of all usable objects in the image.
`diagnostics_json.unchecked` lists the remaining geometric source indices whose
nearby-height check was not needed. These are neither returned poses nor failures.
The controller receives the same checked retry batch without a per-pick scan.

Schema 13 uses one float32 per-pixel median of 1/3/5 post-request depth frames
(default 3, strict majority). `depth_stamp` is the newest frame, while
`diagnostics_json.depth_frame_stamps_ns` records all contributing frames.
`clearance` and measured rejection evidence identify `platform_floor_camera_z_v1`
and include candidate/maximum floor heights, difference and point counts. Radius
uses camera XY; floor height follows camera Z at each point’s own physical XY.
`timings_ms` separates processing stages; legacy `inference_ms` remains aggregate
native processing, not YOLO alone. ROS layouts are unchanged. Restart updated
native/parent processes together and explicitly review/save/deploy schema-13 profiles.

Requests must set `pose_convention` to the generated request constant
`POSE_CONVENTION` (`item_short_x_long_y_v1`). Successful replies repeat that
value in `diagnostics_json`; Robot Controller requires an exact match before
planning. Missing/older conventions are rejected. Rebuild and restart detector
and controller clients together after this interface change. This prevents an
older long-X/short-Y consumer from silently turning the gripper by 90 degrees.
No hardware driver or command interface belongs in this package.
