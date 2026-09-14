# item_perception_interfaces

`GetItemPoses` requests up to `max_candidates` from an explicitly SHA-256-bound
item profile. The armed detector advertises `/item_detect/get_item_poses`.
Responses contain only observed item targets in `platform_reference`, with
image timestamps, batch-local IDs, diagnostics and explicit shortages/errors.
Position/dimensions use metres; item orientation is platform-normal with an
in-plane long-axis heading, not measured surface tilt or a robot TCP command.
No hardware driver or command interface belongs in this package.
