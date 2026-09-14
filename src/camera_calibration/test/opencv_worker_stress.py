#!/usr/bin/env python3

import os
import sys
import time
from pathlib import Path

import numpy as np


FRAME_COUNT = 10000
IMAGE_HEIGHT = 1080
IMAGE_WIDTH = 1920


def _worker_rss_kib(pid: int) -> int:
    status_path = Path(f"/proc/{pid}/status")
    for line in status_path.read_text(encoding="utf-8").splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1])
    raise RuntimeError(f"VmRSS is missing for worker PID {pid}")


def main() -> None:
    if len(sys.argv) != 2:
        raise RuntimeError(
            "Usage: opencv_worker_stress.py <extracted-opencv-runtime-directory>"
        )
    runtime_dir = Path(sys.argv[1]).resolve()
    sys.path.insert(0, str(runtime_dir))

    import cv2

    from camera_calibration_gui.calibration_core import CharucoSettings
    from camera_calibration_gui.opencv_worker import OpenCvWorkerClient
    from camera_calibration_gui import opencv_worker_runtime

    settings = CharucoSettings(
        "bin_camera",
        "DICT_4X4_50",
        3,
        3,
        28.0,
        21.0,
    )
    _dictionary, board = opencv_worker_runtime.create_legacy_charuco_board(settings)
    board_image = board.generateImage((600, 600), marginSize=40)
    board_rgb = np.repeat(board_image[:, :, None], 3, axis=2)

    full = np.full((IMAGE_HEIGHT, IMAGE_WIDTH, 3), 255, dtype=np.uint8)
    full[240:840, 660:1260] = board_rgb
    partial = full.copy()
    partial[:, 960:] = 255
    empty = np.full_like(full, 255)
    blurred = cv2.GaussianBlur(full, (5, 5), 0.8)
    shifted = np.full_like(full, 255)
    shifted[260:860, 700:1300] = board_rgb
    edge_clipped = np.full_like(full, 255)
    edge_clipped[240:840, 0:300] = board_rgb[:, 300:]
    frames = (
        ("full", full),
        ("partial", partial),
        ("empty", empty),
        ("blurred", blurred),
        ("shifted", shifted),
        ("edge_clipped", edge_clipped),
    )
    camera_matrix = np.array(
        [[1500.0, 0.0, 960.0], [0.0, 1500.0, 540.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.zeros(5, dtype=np.float64)

    worker = OpenCvWorkerClient(runtime_dir=runtime_dir)
    worker_pid = worker.pid
    if worker_pid is None:
        raise RuntimeError("OpenCV worker did not expose a PID")
    states = {}
    rss_samples = []
    started = time.monotonic()
    try:
        worker.configure(settings)
        for sequence in range(1, FRAME_COUNT + 1):
            label, image = frames[(sequence - 1) % len(frames)]
            result = worker.detect(
                frame_sequence=sequence,
                image_rgb=image,
                camera_matrix=camera_matrix,
                distortion=distortion,
                solution_overlay=None,
            )
            if result.frame_sequence != sequence:
                raise RuntimeError("Worker returned the wrong frame sequence")
            if result.overlay_rgb.shape != image.shape:
                raise RuntimeError("Worker returned the wrong overlay shape")
            if result.state not in {
                "ready",
                "insufficient_corners",
                "not_visible",
                "collinear_corners",
                "pose_unavailable",
            }:
                raise RuntimeError(f"Worker returned an invalid state: {result.state}")
            states.setdefault(label, result.state)
            if sequence % 500 == 0:
                worker.check_health()
                rss_samples.append(_worker_rss_kib(worker_pid))
                elapsed = time.monotonic() - started
                print(
                    f"frames={sequence}/{FRAME_COUNT} elapsed_sec={elapsed:.1f} "
                    f"worker_rss_kib={rss_samples[-1]}",
                    flush=True,
                )
        if states["full"] != "ready":
            raise RuntimeError(f"Full board was not ready: {states['full']}")
        if states["blurred"] != "ready":
            raise RuntimeError(f"Blurred board was not ready: {states['blurred']}")
        if states["shifted"] != "ready":
            raise RuntimeError(f"Shifted board was not ready: {states['shifted']}")
        if states["empty"] != "not_visible":
            raise RuntimeError(f"Empty frame was not not_visible: {states['empty']}")
    finally:
        worker.close()

    if worker.pid != worker_pid or worker.exit_code != 0:
        raise RuntimeError(
            "Stress test did not retain and cleanly stop its one worker: "
            f"initial_pid={worker_pid} final_pid={worker.pid} "
            f"exit_code={worker.exit_code}"
        )
    elapsed = time.monotonic() - started
    print(
        f"PASS frames={FRAME_COUNT} resolution={IMAGE_WIDTH}x{IMAGE_HEIGHT} "
        f"worker_pid={worker_pid} elapsed_sec={elapsed:.1f} "
        f"rss_min_kib={min(rss_samples)} rss_max_kib={max(rss_samples)} "
        f"states={states} opencv={cv2.__version__} module={cv2.__file__}",
        flush=True,
    )


if __name__ == "__main__":
    if os.environ.get("ROS_LOCALHOST_ONLY") not in {None, "1"}:
        raise RuntimeError("ROS_LOCALHOST_ONLY must be unset or exactly 1 for this test")
    main()
