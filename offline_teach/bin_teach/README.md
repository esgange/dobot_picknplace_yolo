# Bin teach files

This is the canonical directory for portable bin ROI teaching artifacts:

```text
bin_teach_<UTC_TIMESTAMP>_<SOURCE_ROBOT_LAN1_IP>.yaml
```

Bin Teach saves new schema-3 files here. Its **Load Bin ROI** action reads
only this directory. To reuse a bin template at another station, copy the YAML
here with its filename unchanged and explicitly apply that station's platform
calibration in Bin Teach before loading it.

New captures intersect detected corner rays with the taught platform Z=0 plane.
Teaching and loading project the same XY geometry; the platform may tilt
relative to the robot base. Physical corner markers must lie on that plane.
Existing schema-3 files remain unchanged and load their original XY. Re-capture
and save a new file to replace an ROI previously computed from marker-PnP XYZ
with Z discarded; loading cannot repair that old geometry automatically.

Camera and platform calibration files remain in root `calibration/`. Bin ROI
files are teaching data, not calibration files. No alternate load path or
automatic search of `calibration/` is supported. Teaching files are included in
offline transfers; images, runtime logs and generated build output do not belong
here. Item detection and picking behavior is not defined by this directory.
