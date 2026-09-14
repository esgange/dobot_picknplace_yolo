"""Planar teaching round trips without hardware or importing cv2 in the parent."""

from dataclasses import replace
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from camera_calibration_gui.calibration_core import CAMERA_ON_HAND, CAMERA_TO_HAND
from camera_calibration_gui.opencv_worker import OpenCvWorkerClient, OpenCvWorkerFailure
from item_perception_yolo import bin_teach_core as core
from item_perception_yolo import bin_teach_gui as gui
from test_bin_teach import _applied, _capture, _loaded_rgb_node, _marker


def pose(xyz, degrees):
    x, y, z = np.deg2rad(degrees)
    cx, cy, cz = np.cos([x, y, z])
    sx, sy, sz = np.sin([x, y, z])
    matrix = np.eye(4)
    matrix[:3, :3] = (
        np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    )
    matrix[:3, 3] = xyz
    return matrix


def plane_markers():
    markers = {index: _marker(x, y, .03) for index, (x, y) in enumerate([
        (.30, .30), (-.30, -.30), (.31, -.29), (-.32, .31),
    ])}
    for corners in markers.values():
        corners[:, 2] = 0.0
    return markers


def transform_points(matrix, points):
    return (matrix @ np.column_stack((points, np.ones(len(points)))).T).T[:, :3]


@pytest.fixture
def worker():
    root = Path(__file__).resolve().parents[3]
    runtime = root / "install/camera_calibration/lib/camera_calibration/opencv_runtime"
    client = OpenCvWorkerClient(runtime_dir=runtime)
    try:
        yield client
    finally:
        client.close()


@pytest.mark.parametrize("mode", [CAMERA_TO_HAND, CAMERA_ON_HAND])
@pytest.mark.parametrize("distortion", [
    np.zeros(5), np.array([.05, -.015, .001, -.002, .001, .003, -.001, .0001]),
])
def test_tilted_teach_save_load_roundtrip_and_station_transfer(tmp_path, worker, mode, distortion):
    applied = _applied(tmp_path, mode)
    platform = pose((.4, -.2, .15), (13, -9, 27))
    desired_view = pose((.02, -.03, .9), (174, 7, 11))
    internal = pose((.012, -.007, .003), (-90, 0, -90))
    robot = pose((.2, -.1, .6), (20, 15, 38)) if mode == CAMERA_ON_HAND else None
    base_camera = platform @ desired_view @ np.linalg.inv(internal)
    mounting = base_camera if robot is None else np.linalg.inv(robot) @ base_camera
    applied = replace(
        applied, camera=replace(applied.camera, reference_from_camera_link=mounting),
        platform=replace(applied.platform, base_from_platform=platform,
                         calibration_reference_from_camera_link=mounting),
    )
    resolved_camera = gui.resolve_base_from_camera_link(applied.camera, robot)
    view = core.compose_platform_from_optical(platform, resolved_camera, internal)
    assert np.allclose(view, desired_view)
    assert not np.allclose(platform[:2, 2], 0.0)  # Deliberately not base-flat.
    matrix = np.array([[800., 0., 960.], [0., 815., 540.], [0., 0., 1.]])
    expected = plane_markers()
    pixels, rays = {}, {}
    for marker_id, corners in expected.items():
        pixels[marker_id] = worker.project_points(
            optical_points=transform_points(np.linalg.inv(view), corners),
            camera_matrix=matrix, distortion=distortion,
        )
        rays[marker_id] = worker.image_rays(
            image_points=pixels[marker_id], camera_matrix=matrix, distortion=distortion,
        )
    actual = core.project_marker_rays_to_plane(view, rays)
    for marker_id in expected:
        assert np.allclose(actual[marker_id], expected[marker_id], rtol=0, atol=1e-7)
    points = core.select_outside_roi_corners(actual)
    capture = replace(
        _capture(applied), base_from_tool=robot, base_from_camera_link=resolved_camera,
        camera_link_from_optical=internal, platform_from_optical=view, points=points,
    )
    output = core.bin_output_path(
        applied.platform.robot_lan1_ip, root=tmp_path,
        created_at=datetime.fromisoformat(capture.captured_at_utc.replace("Z", "+00:00")),
    )
    core.write_bin_teach(
        output, applied, core.BinArucoSettings("DICT_5X5_50", 60.), capture, root=tmp_path,
    )
    loaded = core.load_bin_teach(output, root=tmp_path)
    assert loaded.points == points
    recovered = worker.project_points(
        optical_points=core.bin_border_in_optical(loaded.points, view),
        camera_matrix=matrix, distortion=distortion,
    )[::core.ROI_BORDER_SAMPLES_PER_EDGE]
    detected = np.array([
        pixels[p.source_marker_id][p.source_marker_corner_index] for p in points
    ])
    assert np.allclose(recovered, detected, rtol=0, atol=1e-4)

    # A different station carries the same local XY on its own tilted platform.
    destination = replace(applied.platform, base_from_platform=pose((.8, .1, .25), (-8, 17, -34)))
    placed = core.place_bin_roi(loaded, destination)
    local = np.array([[p.x_m, p.y_m, 0.] for p in loaded.points])
    assert np.allclose(placed, transform_points(destination.base_from_platform, local))
    destination_view = pose((.04, .02, 1.1), (177, -5, 22))
    destination_pixels = worker.project_points(
        optical_points=core.bin_border_in_optical(loaded.points, destination_view),
        camera_matrix=matrix, distortion=distortion,
    )[::core.ROI_BORDER_SAMPLES_PER_EDGE]
    destination_optical_from_base = np.linalg.inv(destination.base_from_platform @ destination_view)
    physical_pixels = worker.project_points(
        optical_points=transform_points(destination_optical_from_base, placed),
        camera_matrix=matrix, distortion=distortion,
    )
    assert np.allclose(destination_pixels, physical_pixels, atol=1e-7)


@pytest.mark.parametrize("invalid", ["parallel", "behind", "on_plane", "nan", "shape", "ids"])
def test_invalid_plane_intersections_rejected(invalid):
    transform = pose((0, 0, -1), (0, 0, 0))
    rays = {marker_id: np.tile([0., 0., 1.], (4, 1)) for marker_id in range(4)}
    if invalid == "parallel":
        transform = pose((0, 0, -1), (90, 0, 0))
    elif invalid == "behind":
        transform[2, 3] = 1
    elif invalid == "on_plane":
        transform[2, 3] = 0
    elif invalid == "nan":
        rays[0][0, 0] = np.nan
    elif invalid == "shape":
        rays[0] = np.zeros((4, 2))
    else:
        del rays[3]
    with pytest.raises(ValueError):
        core.project_marker_rays_to_plane(transform, rays)


@pytest.mark.parametrize("result", [
    None, np.zeros((4, 2)), np.full((4, 3), np.nan), np.zeros((4, 3)),
])
def test_malformed_worker_ray_result_is_terminal(result):
    client = SimpleNamespace(
        _request=MagicMock(return_value=result),
        _failure=MagicMock(return_value=OpenCvWorkerFailure(
            "malformed undistorted ray result", operation="image_rays", worker_pid=123,
            exit_code=None,
        )),
    )
    with pytest.raises(OpenCvWorkerFailure, match="malformed undistorted ray result"):
        OpenCvWorkerClient.image_rays(
            client, image_points=np.ones((4, 2)), camera_matrix=np.eye(3), distortion=np.zeros(8),
        )
    client._request.assert_called_once()


def detecting_node(tmp_path, mode):
    node, message = _loaded_rgb_node(tmp_path, mode)
    node._loaded_bin = None
    node._hard_fail_worker = MagicMock()
    observations = tuple(SimpleNamespace(
        marker_id=marker_id, image_corners_px=corners[:, :2] * 300 + [320, 240],
        optical_corners_m=np.full((4, 3), 9000.),  # Deliberately wrong PnP depths: never ROI input.
    ) for marker_id, corners in plane_markers().items())
    node._opencv_worker.detect_aruco.return_value = SimpleNamespace(
        state="ready", detected_ids=(0, 1, 2, 3), observations=observations,
        overlay_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
    )
    node._opencv_worker.image_rays.side_effect = lambda **kw: np.column_stack((
        (kw["image_points"] - [320, 240]) / 300, np.ones(len(kw["image_points"])),
    ))
    return node, message


@pytest.mark.parametrize("mode", [CAMERA_TO_HAND, CAMERA_ON_HAND])
def test_live_and_loaded_border_share_plane_geometry_and_ignore_pnp_depth(tmp_path, mode):
    node, message = detecting_node(tmp_path, mode)
    gui.BinTeachNode._on_color_image(node, message)
    expected = core.select_outside_roi_corners(plane_markers())
    assert np.allclose([(p.x_m, p.y_m) for p in node._latest_points],
                       [(p.x_m, p.y_m) for p in expected])
    yellow = node._latest_planar_border_pixels.copy()
    assert yellow.shape == (4 * core.ROI_BORDER_SAMPLES_PER_EDGE, 2)
    node._loaded_bin = SimpleNamespace(path=Path("bin_teach_test.yaml"), points=node._latest_points)
    gui.BinTeachNode._on_color_image(node, message)
    assert np.allclose(node._loaded_border_pixels, yellow)
    node._opencv_worker.detect_aruco.assert_called_once()
    node._opencv_worker.image_rays.assert_called_once()
    node._hard_fail_worker.assert_not_called()


def test_native_ray_failure_stays_terminal(tmp_path):
    node, message = detecting_node(tmp_path, CAMERA_TO_HAND)
    failure = OpenCvWorkerFailure("native failed", operation="image_rays", worker_pid=123,
                                  exit_code=-11)
    node._opencv_worker.image_rays.side_effect = failure
    gui.BinTeachNode._on_color_image(node, message)
    node._hard_fail_worker.assert_called_once_with(failure)
    assert node._latest_points is None
    node._opencv_worker.project_points.assert_not_called()


@pytest.mark.parametrize("invalid", ["parallel", "behind", "distortion"])
def test_invalid_planar_frame_clears_readiness_without_terminal_failure(tmp_path, invalid):
    node, message = detecting_node(tmp_path, CAMERA_TO_HAND)
    gui.BinTeachNode._on_color_image(node, message)
    assert node._latest_points is not None
    if invalid == "parallel":
        transform = node._tf_buffer.lookup_transform.return_value.transform
        transform.rotation.x = np.sqrt(.5)
        transform.rotation.w = np.sqrt(.5)
        # One undistorted ray is exactly parallel to the plane.
        node._opencv_worker.image_rays.side_effect = lambda **kw: np.tile([0., 0., 1.], (16, 1))
    elif invalid == "behind":
        node._applied = replace(
            node._applied,
            platform=replace(
                node._applied.platform, base_from_platform=pose((0, 0, -1), (0, 0, 0)),
            ),
        )
    else:
        node._camera_distortion_model = "equidistant"
    gui.BinTeachNode._on_color_image(node, message)
    assert node._latest_points is None
    assert node._latest_planar_border_pixels is None
    assert node._latest_ready_time is None
    node._hard_fail_worker.assert_not_called()


def test_retake_during_planar_projection_discards_inflight_geometry(tmp_path):
    node, message = detecting_node(tmp_path, CAMERA_TO_HAND)
    node._capture = _capture(node._applied)  # Retake is enabled only with an existing capture.

    def interrupted_projection(**kwargs):
        gui.BinTeachNode.retake(node)
        return np.zeros((4 * core.ROI_BORDER_SAMPLES_PER_EDGE, 2))

    node._opencv_worker.project_points.side_effect = interrupted_projection
    gui.BinTeachNode._on_color_image(node, message)
    assert node._latest_points is None
    assert node._latest_planar_border_pixels is None
    assert node._latest_ready_time is None
    assert node._latest_overlay is None
