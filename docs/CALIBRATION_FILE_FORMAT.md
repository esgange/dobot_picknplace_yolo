# Readable calibration files

Rule 289 uses camera calibration **schema 8** and platform calibration
**schema 5**. These are the only accepted calibration schemas. Older files,
quaternion fields, matrix pose fields and mixed representations are rejected;
there is no compatibility reader or automatic conversion. Existing files are
preserved. Use current-format calibrations before loading teaching or production
configuration with this version.

Every saved calibration pose uses the following representation (illustrative
values, not a station calibration):

```yaml
transform:
  target_frame: base_link
  source_frame: camera_fixed_link
  convention: target_from_source
  format: xyz_rxryrz_m_deg_v1
  x: 0.125
  y: -0.031
  z: 0.744
  rx: 23.0
  ry: -41.0
  rz: 137.0
```

XYZ are **metres**; RX/RY/RZ are **degrees** about fixed X/Y/Z axes.
The rotation matrix is `Rz(rz) @ Ry(ry) @ Rx(rx)`. The transform still maps
source-frame points into the target frame; frame names and mounting direction
are unchanged. Each coordinate must be a finite YAML number. The `format` tag
is required; values are never inferred to be millimetres or radians. At Euler
gimbal lock, saving chooses an equivalent attitude with RZ=0. Equivalent angle
triples describe the same physical rotation.

Camera files retain settings, pinned pipeline provenance, quality diagnostics,
sample IDs/order and all captured observations. Each sample's `base_from_tool`
and `camera_from_target` uses the same seven pose fields without a frame envelope;
their field names retain the direction. The six `joint_positions_rad` values
stay in radians. Loading checks the minimum sample count, both angular-diversity
conditions, sample/diagnostic IDs, quality fields and exact pipeline identity.
The calibration GUI still recomputes from validated samples with live internal
camera TF; it does not substitute the stored result for a solve.

Platform files retain the fixed `base_link <- platform_reference` pose and the
complete `teaching_provenance`. Every transform in that evidence, including an
on-hand capture's robot TF, uses the readable pose fields. Timestamp, source hash,
mounting, frame and reference checks remain, as do reconstruction checks for the
camera chain and final platform transform. The historical teaching camera need
not exist or equal the independently selected active camera.

The historical teaching-camera schema may be 7 or 8. It describes the original
file identified by the recorded hash, so an explicit offline conversion must
preserve it. This is provenance validation, not permission to load a schema-7
active camera. Rule 290 records the user's one-time conversion of this station's
existing files, with byte-exact backups and verified geometry/sample preservation;
runtime readers still accept only the current format.

Portable Bin Teach files retain their existing schema and historical evidence.
Their source identifiers can describe older teaching artifacts without loading
those artifacts. All **active** camera/platform selection paths use the strict
current calibration readers. This change does not rewrite teach files, calibration
files, hashes, models or `.env` selections.

Runtime matrix calculations and ROS quaternion messages remain internal transport
representations. This disk-format change does not alter detection, trajectories,
placement offsets, I/O sequences or GUI layout. Rebuild the changed packages to
use the new readers/writers; no live process is restarted by this change.
