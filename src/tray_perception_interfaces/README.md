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

The origin is the rectangle corner nearest the base in 3D. Positive X follows
the short edge inward; positive Y follows the long edge inward. Right-handed
Z is X cross Y and may face either side of the reference plane. Its sign is
not forced toward the camera. `length`/`width` are long/short sides in metres,
with `extent_x = width` and `extent_y = length`. Orientation is the tray frame, not robot TCP
attitude. The controller chooses placement coordinates and motion separately.

`save_debug_images=true` saves only this result's annotated RGB and, if available,
registered-depth image under `debug/tray_img/`. Missing depth or an image-save
error is diagnostic only and cannot turn a valid tray pose into a failed result.
No robot command or controller lifecycle interface belongs in this package.


## Placement depth requests

`GetTrayPose.sample_placement_depth=true` enables a `PlacementDepthRequest` containing
positive tray-local X/Y in mm, the physical Item Teach depth sampling **diameter**,
and its exact quality settings. The provider takes fresh synchronized RGB/depth
after the trigger, performs the usual single tray inference, and samples the
requested target using original registered-depth pixels and their own intrinsics.
It uses the same median/MAD acceptance as Item Pick, excludes outside-tray samples,
and rejects invalid targets, clipped footprints or insufficient depth. The saved
plane remains unchanged. A depth failure returns ERROR without killing/disarming
the native worker/provider. Ordinary pose requests keep depth optional.

`PlacementDepthResult` contains validity, base-frame target X/Y with measured
surface Z, the depth timestamp, accepted/total counts, median and MAD sigma.
Diagnostics echo `placement_sampling` alongside bound profile/model/camera/plane
evidence. The controller checks every requested setting, source identity, result
frame, synchronization and after-trigger timestamps before motion. An invalid
response never becomes a placement target. Perception issues no robot or I/O calls.
Rebuild these interfaces and restart all tray providers and controller clients
together; `GetTrayPose`'s wire definition changed.
