# Tray Perception Interfaces

`GetTrayPose` is the shared read-only `/tray_detect/get_tray_pose` service exposed
by Armed Tray Teach or headless Tray Detect, with only one provider at a time.
The caller supplies the exact saved tray YAML SHA-256. Each request acquires a
new RGB observation after request arrival, with matching calibration and exact
timestamped TF, and runs the saved model/settings once against the saved plane.
Live depth is optional and never changes that plane.

Check `success` and `found` before using `tray`. `OK` returns exactly one eligible
tray nearest image center; `NO_VALID_TRAY` is a successful observation with
`found=false`. Failed or busy requests return no usable pose. `header` contains
`base_link` and the original RGB observation timestamp; `batch_id` identifies
this request result. Diagnostics include hashes, source/plane evidence, rejected
detections and optional debug-image results.

The origin is the rectangle corner nearest the base in 3D. Positive X and Y
follow adjacent edges inward; right-handed Z follows the taught plane normal
toward its teaching camera. `length`/`width` are sorted long/short sides in metres.
`extent_x`/`extent_y` are the actual lengths along frame X/Y, also in metres;
do not assume X is the long side. Orientation is the tray frame, not robot TCP
attitude. The controller chooses placement coordinates and motion separately.

`save_debug_images=true` saves only this result's annotated RGB and, if available,
registered-depth image under `debug/tray_img/`. Missing depth or an image-save
error is diagnostic only and cannot turn a valid tray pose into a failed result.
No robot command or controller lifecycle interface belongs in this package.
