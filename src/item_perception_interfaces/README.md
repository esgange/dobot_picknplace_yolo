# item_perception_interfaces

`GetItemPoses` requests up to `max_candidates` from an explicitly SHA-256-bound
item profile. `save_debug_images` optionally writes the exact annotated RGB and
registered-depth result pair for that request into the workspace
`debug/pick_img/` directory. The armed detector advertises
`/item_detect/get_item_poses`.
Responses contain only observed item targets in `platform_reference`, with
image timestamps, batch-local IDs, diagnostics and explicit shortages/errors.
Position/dimensions use metres; item orientation is platform-normal with an
in-plane long-axis heading: local X is the long axis and local Y is the short
axis. It is not measured surface tilt or a robot TCP command. Robot Controller
uses local Y only as an undirected gripper-heading line while retaining its
taught tool-Z attitude.
No hardware driver or command interface belongs in this package.
