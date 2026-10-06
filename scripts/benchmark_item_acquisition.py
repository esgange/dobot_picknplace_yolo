#!/usr/bin/env python3
"""Synthetic CPU geometry benchmark; no camera, model, ROS node or hardware calls.

Source the workspace first. --python-root may point at a read-only export of an
older item_perception_yolo/python tree to compare the identical fixture.
"""

import argparse
import inspect
import json
from pathlib import Path
import statistics
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", required=True, type=Path)
    parser.add_argument("--python-root", type=Path)
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--runs", type=int, default=30)
    args = parser.parse_args()
    if args.runs < 5:
        parser.error("Use at least five measured runs")
    sys.path.insert(0, str(args.runtime.resolve(strict=True)))
    if args.python_root:
        sys.path.insert(0, str(args.python_root.resolve(strict=True)))
    import cv2
    import numpy as np
    from item_perception_yolo.item_geometry import generate_candidates
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)
    camera = {"width": 640, "height": 480, "distortion_model": "plumb_bob",
              "k": [1000., 0., 320., 0., 1000., 240., 0., 0., 1.], "d": [0.]*5}
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = .8
    context = {"camera": camera, "depth_camera": camera,
               "platform_from_optical": transform.tolist(),
               "roi": [[-.2, -.15], [-.2, .15], [.2, .15], [.2, -.15]],
               "pick_planning": {"home_matrix": np.eye(4).tolist(),
                                 "base_from_platform": np.eye(4).tolist(),
                                 "link6_from_robot_camera": np.eye(4).tolist(),
                                 "pick_rotation_deg": 0., "standoff_height_mm": 70.}}
    settings = {"geometry_source": "mask", "quality": dict(QUALITY_DEFAULTS),
                "geometry": {"nearby_depth_radius_mm": 150., "nearby_depth_height_mm": 60.,
                             "height": 80., "width": 32., "tolerance": .1,
                             "pickdepth_radius": 30., "depth_frame_count": 3},
                "bin_clearance": dict.fromkeys(("p1_p2", "p2_p3", "p3_p4", "p4_p1")),
                "yolo": {"class_ids": [1], "confidence": .5}}
    rgb = np.full((480, 640, 3), 80, np.uint8)
    depth = np.full((480, 640), 700, np.uint16)
    polygon = np.array([[-50, -20], [50, -20], [50, 20], [-50, 20]], np.float32)
    objects = [{"index": i, "class_id": 1, "class_name": "part", "confidence": .7,
                "center": np.array([320.+40*i, 240.]), "polygon": polygon+[320+40*i, 240],
                "rectangle": polygon+[320+40*i, 240]} for i in range(6)]
    signature = inspect.signature(generate_candidates).parameters
    options = {"candidate_limit": 3}
    if "render_images" in signature:
        options["render_images"] = args.render
    frames = [{"width": 640, "height": 480, "stamp_ns": 1+i*33_333_333,
               "depth": depth.tobytes()} for i in range(3)]
    modern = "render_images" in signature
    if modern:
        from item_perception_yolo.depth_snapshot import median_depth_snapshot
    measurements = []
    for index in range(args.runs+3):
        stages = {}
        if "timings" in signature:
            options["timings"] = stages
        acquisition_started = time.monotonic()
        snapshot = median_depth_snapshot(frames, QUALITY_DEFAULTS) if modern else None
        measured = np.frombuffer(snapshot["depth"], "<f4").reshape(480, 640) if modern else depth
        start = time.monotonic()
        result = generate_candidates(objects, rgb, measured, context, settings, cv2, np, **options)
        total = (time.monotonic()-start)*1000.
        acquisition = (time.monotonic()-acquisition_started)*1000.
        assert [c["source_index"] for c in result[2]] == [0, 1, 2]
        if index >= 3:
            measurements.append({"acquisition_cpu_ms": acquisition, "geometry_total_ms": total,
                                 "median_ms": snapshot["median_ms"] if modern else 0., **stages})

    def summary(values):
        return {"median_ms": statistics.median(values), "p95_ms": float(np.percentile(values, 95))}

    output = {"fixture": ("640x480 flat 700mm, six detections, three returned; "
                          "excludes YOLO/transport/validation"),
              "runs": args.runs, "renders": options.get("render_images", True),
              "stages": {key: summary([row[key] for row in measurements])
                         for key in measurements[0]},
              "positions_m": [c["position"] for c in result[2]]}
    if "render_images" in signature:
        from item_perception_yolo.depth_snapshot import median_depth_snapshot
        frames = [{"width": 640, "height": 480, "stamp_ns": 1+i*33_333_333,
                   "depth": depth.tobytes()} for i in range(3)]
        elapsed = []
        for index in range(args.runs+3):
            snapshot = median_depth_snapshot(frames, QUALITY_DEFAULTS)
            if index >= 3:
                elapsed.append(snapshot["median_ms"])
        output["three_frame_median_cpu"] = summary(elapsed)
        output["three_frame_span_at_30fps_ms"] = snapshot["capture_span_ms"]
        output["stream_wait_note"] = "Span excludes first-frame phase; no synthetic wait included"
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
