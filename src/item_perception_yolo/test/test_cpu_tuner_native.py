"""Run an overlapping offline trial through real private item/tray workers."""
import importlib.util
import os
from pathlib import Path
import subprocess
from unittest.mock import MagicMock

from ament_index_python.packages import get_package_prefix
import numpy as np

from item_perception_yolo.item_native_client import NativeClient
from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS, file_sha256


def test_tuner_replays_item_depth_and_two_tray_observations(tmp_path):
    runtime = Path(get_package_prefix('item_perception_yolo'))/'lib/item_perception_yolo/yolo_runtime'
    config = tmp_path/'segment.yaml'
    model = config.with_suffix('.pt')
    config.write_text('nc: 2\nbackbone:\n  - [-1, 1, Conv, [16, 3, 2]]\n'
                      '  - [-1, 1, Conv, [32, 3, 2]]\n'
                      'head:\n  - [[0, 1], 1, Segment, [nc, 32, 64]]\n')
    subprocess.run(['/usr/bin/python3', '-c',
                    'import sys; sys.path.insert(0,sys.argv[1]); from ultralytics import YOLO; '
                    'YOLO(sys.argv[2],task="segment").save(sys.argv[3])',
                    str(runtime), str(config), str(model)],
                   env={**os.environ, 'YOLO_CONFIG_DIR': str(tmp_path), 'MPLCONFIGDIR': str(tmp_path),
                        'YOLO_OFFLINE': 'true', 'YOLO_AUTOINSTALL': 'false', 'OPENBLAS_NUM_THREADS': '1'},
                   check=True, capture_output=True, timeout=30)
    camera = {'width': 64, 'height': 48, 'k': [100., 0., 32., 0., 100., 24., 0., 0., 1.],
              'd': [0.]*5, 'distortion_model': 'plumb_bob'}
    transform = np.diag([1., -1., -1., 1.])
    transform[:3, 3] = [.3, .2, .8]
    context = {'camera': camera, 'depth_camera': camera, 'base_from_optical': transform.tolist()}
    client = NativeClient(MagicMock(), worker_package='tray_perception',
                          worker_executable='tray_worker', torch_threads=1)
    try:
        response, _ = client.call({'operation': 'tray_plane', 'width': 64, 'height': 48,
            'camera_context': context, 'source_stamp_ns': 100, 'depth_stamp_ns': 100,
            'pixels': [[10., 10.], [50., 10.], [50., 35.], [10., 35.]]},
            np.full((48, 64), 600, '<u2').tobytes())
    finally:
        client.close()
    yolo = {'class_ids': [0], 'confidence': .99, 'iou': .7, 'max_detections': 10, 'image_size': 64}
    common = {'generation': 1, 'width': 64, 'height': 48, 'render_images': False,
              'model': {'path': str(model), 'sha256': file_sha256(model), 'task': 'segment', 'yolo': yolo}}
    tray = {**common, 'operation': 'tray_preview', 'camera_context': context, 'plane': response['plane'],
            'settings': {'accepted_class_ids': [0], 'model_task': 'segment', 'geometry_source': 'mask',
                         'geometry': {'length_mm': 200., 'width_mm': 100., 'tolerance_mm': 2.}, 'yolo': yolo}}
    item = {**common, 'operation': 'detect', 'depth_encoding': '32FC1_mm',
            'settings': {'model_task': 'segment', 'yolo': yolo, 'geometry_source': 'mask',
                'quality': dict(QUALITY_DEFAULTS),
                'bin_clearance': dict.fromkeys(('p1_p2', 'p2_p3', 'p3_p4', 'p4_p1')),
                'geometry': {'depth_frame_count': 3, 'nearby_depth_radius_mm': 150.,
                             'nearby_depth_height_mm': 60., 'height': 70., 'width': 35.,
                             'tolerance': 5., 'pickdepth_radius': 30.}},
            'context': {'camera': camera, 'depth_camera': camera, 'platform_from_optical': transform.tolist(),
                        'roi': [[-.2, -.15], [-.2, .15], [.2, .15], [.2, -.15]],
                        'pick_planning': {'home_matrix': np.eye(4).tolist(),
                                          'base_from_platform': np.eye(4).tolist(),
                                          'link6_from_robot_camera': np.eye(4).tolist(),
                                          'pick_rotation_deg': 0., 'standoff_height_mm': 90.}}}
    rgb = bytes(64*48*3)
    item_data = rgb + np.full((48, 64), 600, '<f4').tobytes()
    path = Path(__file__).resolve().parents[3]/'scripts/tune_perception_cpu.py'
    spec = importlib.util.spec_from_file_location('tuner', path)
    tuner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tuner)
    report = tuner.trial({'item': [(item, item_data)], 'tray': [(tray, rgb), (tray, rgb)]},
                         {'item': 1, 'tray': 1}, warmups=1, rounds=5)
    assert report['outcomes'] == {'item': [0]*5, 'tray': [False]*5}
    assert all(report[f'{role}_p95_ms'] > 0 for role in ('item', 'tray'))
    assert report['cpu_core_percent'] > 0
