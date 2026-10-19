# Item and tray CPU budgets

Rule 293 replaces the fixed eight-thread defaults with one hardware-aware policy
used by Item Teach/Detect and Tray Teach/Detect. Selection happens when the node
starts. Both roles must be relaunched together after a budget/configuration change;
their maxima are fixed for each worker's lifetime. No benchmark, model execution,
camera capture or hardware command occurs
as part of resource discovery. Restart the providers after changing hardware,
CPU affinity, container quotas or overrides. There is no live rebalancing between
already-running providers, or coordination with unrelated external inference jobs.

The policy reads Linux CPU affinity, physical package/core topology and every
applicable ancestor cgroup CPU quota (v1 and v2). The smallest quota bounds the
usable logical CPU count. It reserves `ceil(usable / 8)` logical CPU capacity
(at least one), then shares the remainder between the two potentially overlapping
workers, with the extra thread going to item detection. Each automatic allocation
is capped at the number of available physical cores. Each worker needs at least
one thread; systems with less than three CPUs report limited capacity. Missing
physical topology is reported, with allowed logical CPUs used as the upper bound.
This is a scheduling budget, not CPU pinning or a physical stopping guarantee.

Examples without container restrictions:

| Available hardware | Item threads | Tray threads |
| --- | ---: | ---: |
| 4 cores / 4 logical CPUs | 2 | 1 |
| 8 cores / 8 logical CPUs | 4 | 3 |
| 8 cores / 16 logical CPUs (current machine) | 7 | 7 |
| 16 cores / 32 logical CPUs | 14 | 14 |

Inter-op, OpenCV and OpenBLAS remain one thread; OpenCL remains disabled. OMP/MKL,
Torch and Ultralytics use the selected budget. The parent passes the exact count
to the private child and verifies startup; the worker checks drift before each
operation. No extra ROS executor or parallel model is introduced. Camera settings,
inference settings, image geometry, acquisition deadlines and robot behavior remain
unchanged. CPU use or faster isolated predictions alone do not establish faster
accepted poses or a faster physical cycle.

## Optional overrides

The only user configuration source is the root `.env`:

```dotenv
ITEM_TORCH_THREADS=auto
TRAY_TORCH_THREADS=auto
```

These optional keys select auto when absent. A canonical positive integer fixes
that worker's count; the other automatic worker receives the remaining budget.
Explicit counts can consume the reserved headroom, but each count and their total
must fit the usable capacity (except the unavoidable one-thread minimum per worker
on a single-CPU host). Invalid values fail visibly. An explicit override disables
measured selection for that role. Shell, camera, maintenance and Dobot launch readers
accept the keys; they do not change any driver command or camera argument.

`perception_cpu_budget` logs the topology, effective quota, both allocations,
headroom and limited-capacity flag. `perception_cpu_tuning` explains why measurements
matched or were ignored. `item_runtime_ready` records actual native thread counts
for either worker; `perception_cpu_selected` records a measured/default change.

## Offline tuning

Run this only while production/teaching inference is stopped so unrelated CPU load
does not distort the comparison. It uses explicitly supplied saved native request
bundles, never a live ROS/camera subscription. Existing image-only diagnostic PNGs
are insufficient: item replay needs its synchronized registered depth and calibrated
context, and tray replay needs its saved plane and image-time camera context.

Each JSON bundle has this envelope:

```json
{
  "version": 1,
  "role": "item",
  "requests": [
    {"header": {}, "payload": "frame_rgb_depth.bin"}
  ]
}
```

Replace `header` with the complete saved private request sent to `NativeClient.call`:
item operation `detect` with model/path/hash/task/YOLO settings, dimensions, generation,
settings, context and `depth_encoding`; tray operation `tray_preview` with its model,
dimensions, settings, plane and camera_context. Payload files contain the exact
uncompressed native protocol bytes: RGB uint8 followed by the little-endian float32
median depth image for item, RGB uint8 alone for tray. Paths are relative to the
bundle; model paths inside headers must identify the explicitly trusted local files.
The native worker still verifies their hashes. These are offline replay inputs, not
production teach files or calibration artifacts.

Provide at least one item observation and an even number of tray observations,
ordered as consecutive pairs. Use representative populated scenes. One bundle
must share the same model, inference settings, image dimensions and rendering policy.
Replay does not create new production archives or change debug retention.

```bash
source scripts/source_ros_workspace.bash
python3 scripts/tune_perception_cpu.py \
  --item-replay logs/cpu_replay/item.json \
  --tray-replay logs/cpu_replay/tray.json \
  --warmups 4 --rounds 20 --save
```

The tuner tests counts from 1, 2, 4, 8, 16, 32, 64 and each worker's automatic
maximum, bounded by the shared allocation. Each combination gets fresh processes
and warmups. Item acquisition computation overlaps two consecutive tray computations
and their existing stability comparison. It reports median/p95 request latency and
combined child CPU usage, including native IPC, prediction and item geometry/depth.
It excludes live camera/TF waits, controller guards, ROS transport, optional tray
placement-depth acquisition and physical motion. Tray comparison timestamps are
synthetic and cannot authorize placement. Reject trials whose item candidate counts
or tray pair acceptance differ from the maximum-budget baseline. Among the remaining
trials, minimize the slower p95, then summed p95, then thread count. Empty/rejected
scenes can underestimate real production cost; this is an offline compute selection.

The report is `logs/perception_cpu_benchmark.json`. With `--save`, the derived
measurements go to `logs/perception_cpu_tuning.json`. This is an optional measurement
cache, not a second user configuration file. It is atomic and bounded on read.
Missing, corrupt or stale measurements leave the hardware-derived budget in effect.
Measurements bind CPU model/topology/affinity/quota, allocation, pinned runtime lock,
installed processing code, role, model hash, YOLO/geometry settings, image dimensions,
candidate limit and rendering policy. Changing any binding invalidates selection.
Workload matching is per role. Re-tune the paired recipe when either model changes
to reassess competition; a still-matching choice for the other role is not proof
of an optimal allocation for the new pair. Reports include input hashes and timings.

Each provider reads the cache at worker startup. Before a matching native request,
it applies the measured count within that role's startup maximum. Other models or
settings use the automatic maximum; teaching previews and model inspection do not
reuse production timing measurements. Thread changes are serialized with inference,
with no model reload, second prediction or change to inter-op threads. Parent checks
the acknowledged count; the child refuses counts above its startup maximum and still
rejects thread/OpenCL drift. No automatic tuning runs during production.

After building all changed packages, relaunch providers under operator control.
No teach-file upload or calibration conversion is required.
