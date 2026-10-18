# Tray capture verification — 2026-10-18

Software measurements on the local AMD Ryzen 7 5800H (8 cores / 16 logical CPUs),
using the pinned CPU runtime: Torch 2.13.0+cu130, Ultralytics 8.4.150,
OpenCV 4.10.0 and NumPy 1.26.4. The CUDA-capable wheel still runs inference on CPU.
The model is the existing brown-tray segmentation artifact, SHA-256
`bb4d70d2601c434294a336ddc7a348989b63add4def40fb141bc39ecff74b2da`.
Input is full-frame 640×480; inference size 448, confidence 0.3, IoU 0.1, max 3.

Each configuration runs in a fresh process with eight warmup predictions and
20 measured pairs, cycling through the same three matching archived scenes from
2026-10-14. Each pair repeats one archived scene with synthetic advancing stamps.
Geometry uses a camera transform reconstructed from that archive's rectangle/
base-corner correspondence, solely for repeatable compute measurement. This is
not a reconstruction used in production. No robot, camera or ROS service runs.

| Torch threads | Masks | Warm prediction median / p95 | Pair compute median / p95 | CPU core use |
|---|---|---|---|---|
| 4 | Standard | 225.8 / 240.9 ms | 464.5 / 489.6 ms | 395% |
| 4 | Native resolution | 212.0 / 217.4 ms | 440.3 / 448.8 ms | 395% |
| 8 | Standard | 177.5 / 183.9 ms | 360.1 / 374.0 ms | 790% |
| 8 | Native resolution | 175.1 / 186.2 ms | 364.9 / 382.6 ms | 781% |

Pair compute includes two predictions, cleanup, saved-plane geometry, stat-based
source fences and stability comparison. It excludes live camera/TF waiting, ROS
transport, placement-depth acquisition and motion. **Do not treat prediction FPS
or this compute-only pair rate as accepted-pose throughput.** All 20 repeated-scene
pairs passed in each configuration; these are not independently captured poses.

At eight threads/native masks, median geometry/cleanup was 8.38 ms per frame,
pair validation 0.28 ms and final RGB rendering 0.98 ms. Four-thread/standard
geometry was 5.65 ms. Native masks therefore add cleanup work, while eight threads
reduce prediction latency at greater CPU use. The tested eight-thread/native pair
is about 21% faster than four-thread/standard **pair computation**, not an old
single-frame request or a physical cycle. More concurrent workloads can alter it.
No ground-truth tray measurements are present, so this does not establish improved
physical accuracy. Native/standard masks produce slightly different dimensions;
each still uses the saved tolerance, plane and existing rectangle fitter.

Reproduce each combination in a fresh process after sourcing the workspace:

```bash
source scripts/source_ros_workspace.bash
python3 scripts/benchmark_tray_capture.py \
  --runtime install/item_perception_yolo/lib/item_perception_yolo/yolo_runtime \
  --profile runtime_teach/tray_teach_brown_tray_20260929T090307_554895Z.yaml \
  --archive logs/tray_perception/evidence/2026-10-14 \
  --threads 8 --native-masks --pairs 20 \
  --output logs/tray_pair_benchmark_8_native.json
```

Use `--threads 4` for the other thread budget; omit `--native-masks` for standard
masks. Existing local inputs are required and are not part of the source commit.
Raw results: `logs/tray_pair_benchmark.txt` and the four
`logs/tray_pair_benchmark_{4,8}_{standard,native}.json` files.

The shared request tests cover mismatch recovery, empty/reset behavior, advancing
stamps without fixed spacing, second-pose/depth ownership, source edits/replacement,
interrupted requests, bounds and controller rejection of missing/tampered proof.
Actual native tests exercise full-frame masks, scaled cleanup, OBB geometry,
rendering without another prediction, independent 8/4-thread processes and runtime
drift rejection. Existing controller tests retain arrival, Pause/Stop retirement,
ten-request exhaustion, placement coordinates/attitude and next-item overlap.

Build outputs are isolated under `/tmp/picknplace-tray-pair`. Build logs are in
`logs/tray_pair_build_final.txt`; 2,896 broad regressions passed in
`logs/tray_pair_regressions_final.txt`. After bounded native retirement was added,
749 focused checks passed against the final isolated installation in
`logs/tray_pair_isolated_tests.txt`. All audited installed modules match source bytes
(`logs/tray_pair_installed_sources.txt`). `logs/tray_pair_camera_config.txt` verifies actual shared camera
launch arguments: 640×480 RGB/depth at 30 FPS, for both camera slots, without launch.
FSM export verification is in `logs/tray_pair_fsm_exports.txt` and
`logs/tray_pair_fsm_verify.txt`.

For deployment, stop the affected applications under operator control, then:

```bash
cd /home/erds/PicknPlace
source scripts/source_ros_workspace.bash
colcon build --symlink-install --cmake-force-configure \
  --packages-select item_perception_yolo tray_perception robot_controller
source scripts/source_ros_workspace.bash
```

Relaunch the tray provider and controller together; an old single-frame provider
fails the new evidence check. Relaunch cameras separately when ready to apply the
local 30 FPS settings. No teach upload, calibration conversion or trajectory change
is required. Implementation verification does not command the robot or establish
physical accuracy, stopping performance or cycle timing.
