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

Requests must set `pose_convention` to the generated request constant
`POSE_CONVENTION` (`item_short_x_long_y_v1`). Successful replies repeat that
value in `diagnostics_json`; Robot Controller requires an exact match before
planning. Missing/older conventions are rejected. Rebuild and restart detector
and controller clients together after this interface change. This prevents an
older long-X/short-Y consumer from silently turning the gripper by 90 degrees.
No hardware driver or command interface belongs in this package.
