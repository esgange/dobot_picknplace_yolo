# PicknPlace inference efficiency audit — 2026-10-19

Scope: active PicknPlace learned and marker-detection nodes, their shared workers,
configuration readers and capture hot paths. CATARM and the excluded reference-only
`src/item_pick` are unchanged. This is a software audit, not a physical cycle test.

## Findings and changes

| Path | Existing behavior verified | Change |
| --- | --- | --- |
| Item Teach / headless Item Detect | One private native process; YOLO retained by path/hash; predictor/backend reused; geometry-only operations do not predict | Cache the validated profile, paired model, loaded model and station sources; stop repeated profile/model hashing and calibration/config reads on captures and frozen-display checks |
| Tray Teach / headless Tray Detect | Already caches validated profile/model/calibration sources; one retained model/predictor; intermediate/no-image captures skip overlays | Share the strong file-fingerprint helper; retain all existing pair/depth checks |
| Both YOLO workers | One serialized prediction per requested frame; runtime and model-change checks remain active | Preserve projection caches across private thread-budget commands; verify model metadata again after prediction/geometry before returning a result |
| Native clients | CPU policy and optional offline measurements load at startup | Skip serialization/hashing of workload settings when no timing entries exist |
| Camera Calibration / Platform Teach | Lifetime private OpenCV worker; ChArUco board/detector configured explicitly, reused per frame; no per-frame calibration file load | No detector change needed |
| Bin Teach | Lifetime private ArUco detector/context; saved geometry remains in memory; full source validation on explicit capture/load | No detector change needed |

The calibration configuration reader also now honors rule 293's optional
`ITEM_TORCH_THREADS` / `TRAY_TORCH_THREADS`. Previously it still required these
keys when `.env.example` listed them, despite their documented optional status.
Absent keys select automatic budgets; malformed explicit values still reject.
The private root `.env` has not been rewritten.

## Source-cache contract

Full validation remains mandatory before caching: strict schemas, samples,
provenance chains, transforms, configured robot identity, exact file locations,
paired model digest and selected model digest. Arming additionally requires the
saved profile to match the applied detection settings.

Retain only validated source/configuration data. Each bound file has a device,
inode, size, nanosecond modification time and change time fingerprint. Check
ordinary-file status and parent symlinks too. Establish fingerprints before full
validation and compare them afterward. Recheck around acquisition/inference and
before delivery, including after requested diagnostics. Edits retaining size/mtime,
same-content replacement, deletion and symlink substitution invalidate the binding.
The legacy latest-camera selector also fences directory identity/timestamps; the
normal explicitly selected camera never adopts a newer catalog file.

Changed sources are rejected without automatic replacement. Reapply the station
calibration after station/bin/root-config changes, or restart headless detection.
Explicit Item Teach Save/Load/Arm resets its profile binding for full validation;
editing an already-bound file from outside the application cannot silently change
capture settings. Saving a profile does not authorize reloading an edited active
model. The worker retains its original model identity and rejects changes.

Fresh RGB, registered depth, camera metadata, exact image-time transforms,
candidate filtering, obstacle checks and tray pair evidence remain dynamic.
Results and eligibility are never cached for a later request. Validation of
incoming native messages and runtime thread drift is retained. No additional
parallel models, executor threads, frame backlog or persistent pose cache is added.

## Measured file-validation overhead

Warm local-file measurements on this machine, 40 repetitions after three warmups,
using its existing deployed Item Teach/model and selected calibrations. Model file
size: 19,766,723 bytes. Neither model execution nor ROS nodes were started by this
measurement. Compare the previous committed validation implementation with the
new source binding using the same files and hardware.

| Operation | Previous median / p95 | Cached median / p95 |
| --- | --- | --- |
| One station-source validation | 5.389 / 5.707 ms | 0.171 / 0.186 ms |
| One item-profile validation/read | 12.172 / 12.685 ms | 0.170 / 0.179 ms |

Raw result: `logs/inference_cache_benchmark.json`. These are file-validation
costs, not prediction time, accepted-pose throughput or physical cycle latency.
Station validation occurs more than once in a capture. Normal OS scheduling and
concurrent software checks affect timings; this is not a measured CPU optimum.
Thread allocations and 640×480/30 FPS camera settings remain unchanged.

## Verification and deployment

Tests instrument the real pinned native worker with synthetic weights to count
YOLO constructions, model hashes, predictor identities and projection-cache clears.
Repeated frames, confidence changes, generation changes and a thread reduction
reuse one model/predictor; model hashing is limited to its initial integrity checks.
Changing model bytes during prediction rejects that observation before delivery.

Filesystem/service tests use temporary files and synthetic sensor/inference data:
unchanged bound requests perform no file-content reads; file replacement, retained
size/mtime edits and changes during inference, debug saving or logging reject;
initial-validation races cannot establish a binding. Strict real calibration and
profile readers, teaching/headless parity and source invalidation are exercised.

The installed broad suite passed 1,174 cases. One new catalog-test fixture used a
custom camera basename where the automatic catalog requires a timestamp name;
after correcting that fixture, all 32 binding tests and the failed-case rerun pass.
All 1,175 broad-suite cases are covered. Logs: `logs/inference_cache_final_regressions.txt`,
`logs/inference_cache_final_binding_tests.txt`, and
`logs/inference_cache_failed_case_recheck.txt`. The final focused lifecycle/capture
checks also pass 147 tests. These counts overlap and must not be added together.

Rebuild `item_perception_yolo` and `tray_perception` together, with affected
applications stopped, then relaunch the selected teaching/headless providers.
No camera restart, controller rebuild, teach upload, calibration conversion or
robot command is required for these changes. Implementation builds used isolated
outputs under `/tmp/picknplace-inference-cache`; canonical installation is unchanged.
Deployment and physical timing/accuracy checks remain operator-controlled.
