# Item Teach and Tray Teach: inspection features

Reviewed against the implementation on 2026-09-29. Both tools are read-only;
Robot Controller owns placement/picking motion and I/O.

| Feature | Item Teach | Tray Teach |
| --- | --- | --- |
| Simulate Trigger result | Frozen returned RGB/depth batch; RGB click resumes | Same interaction; one returned tray or explicit empty result |
| Simulated RViz poses | `item_teach_candidate_1…N`, held teaching frames | `tray_teach_simulated_tray`, held teaching frame |
| Result evidence | Counts, age, XYZ, yaw, sizes, depth evidence | Counts, age, XYZ, full quaternion, size, confidence, batch and rejection reasons |
| Click-to-inspect | Frozen clicked item, depth footprint and selected-pose TF | Live size/rejection inspection; does not freeze or publish the clicked tray independently |
| Activity panel | Expandable in-window activity history | Latest status, rejection tooltips and package event log; no expandable history panel |
| Depth validation in simulation | Pick-point sampling, median, accepted/rejected pixels and cyan footprint | Tray geometry uses the saved reference plane; pose-only simulation does not exercise placement X/Y depth sampling |
| Quality controls | Editable depth range, sample minimum/fraction, freshness, sync and request timeout | Placement requests inherit these from Item Teach through the controller; no separate placement test form |
| Returned count | Ranked candidate batch capped by `pose_candidates` | Exactly one eligible tray nearest image centre |
| Live RViz | All valid item candidates and scene voxels | One selected tray and scene voxels |
| Workspace constraints | Bin ROI, per-wall pick clearance and camera-origin clearance/mirror diagnostics | Tray dimensions/class/confidence and image-edge checks; no bin-specific pick rules |
| Taught robot posture | Home | Tray Detect Pose |
| Motion/gripper profile | Heights, speeds, acceleration, settling, pick rotation, retries and gripper settings | Detect joints only; placement uses Item Teach movement settings and controller X/Y/rotation |
| Source artifacts | Item/model, bin/platform and station/robot camera calibration | Tray/model, camera calibration and its own four-corner reference plane |

The most useful remaining UI additions would be an expandable Activity panel,
a frozen clicked-tray inspection mode, and an explicit placement-depth test using
controller-equivalent X/Y and Item Teach sampling settings. They are **not added
by this change**. Multiple pick candidates, bin clearances and gripper parameters
serve Item Teach's different task and should not be copied blindly into Tray Teach.

Both simulation buttons use their service's shared fresh-input detection pipeline.
Tray pose-only simulation and controller placement have different request payloads:
the controller also requires synchronized depth and a valid sampled placement
surface. A visible frozen teaching pose never substitutes for a new service response.
