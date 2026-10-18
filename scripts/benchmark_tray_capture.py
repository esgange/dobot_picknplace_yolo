#!/usr/bin/env python3
"""Offline tray compute benchmark. Never starts ROS, loads active calibration or commands hardware.

Run once per thread/mask combination in a fresh process. Geometry uses a camera
transform reconstructed from an archived rectangle/corners correspondence, only
for repeatable compute measurement. A pair repeats the archived scene with two
synthetic timestamps; this is not capture throughput or physical accuracy evidence.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import statistics
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--threads', type=int, choices=(4, 8), required=True)
    parser.add_argument('--native-masks', action='store_true')
    parser.add_argument('--pairs', type=int, default=20)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    os.environ.update(OMP_NUM_THREADS=str(args.threads), MKL_NUM_THREADS=str(args.threads),
                      OPENBLAS_NUM_THREADS='1', YOLO_OFFLINE='true', YOLO_AUTOINSTALL='false',
                      YOLO_VERBOSE='false', MPLBACKEND='Agg')
    sys.path.insert(0, str(args.runtime.resolve()))
    import numpy as np
    import yaml
    from item_perception_yolo.yolo_worker_native import initialize
    from tray_perception.native import predict_trays, render_capture
    from tray_perception.source_binding import fingerprints, check
    from tray_perception.stability import sample, compare
    runtime = args.runtime.resolve()
    manifest = json.loads((runtime / 'runtime-lock.json').read_text())
    cv2, np, torch, ultralytics = initialize(runtime, manifest, torch_threads=args.threads)
    profile = yaml.safe_load(args.profile.read_text())
    model_path = args.profile.with_suffix('.pt')
    digest = hashlib.sha256(model_path.read_bytes()).hexdigest()
    assert digest == profile['model']['sha256']
    model = ultralytics.YOLO(str(model_path))
    settings = dict(profile['settings'])
    settings.pop('name')
    settings['accepted_class_ids'] = settings['yolo']['class_ids'][:]
    plane = profile['reference_plane']
    camera = plane['camera']
    source = {'paths': (args.profile, model_path),
              'fingerprints': fingerprints((args.profile, model_path))}
    scenes = []
    for metadata_path in sorted(args.archive.glob('*/detections.json')):
        metadata = json.loads(metadata_path.read_text())
        if metadata.get('model_sha256') != digest:
            continue
        detections = [d for d in metadata['detections'] if d.get('valid')]
        if not detections:
            continue
        d = detections[0]
        bgr = cv2.imread(str(metadata_path.with_name('raw.png')))
        if bgr is None:
            continue
        ok, rvec, tvec = cv2.solvePnP(np.asarray(d['corners_base_m'], np.float64),
            np.asarray(d['rectangle'], np.float64), np.asarray(camera['k']).reshape(3, 3),
            np.asarray(camera['d']), flags=cv2.SOLVEPNP_ITERATIVE)
        assert ok
        optical_from_base = np.eye(4)
        optical_from_base[:3, :3] = cv2.Rodrigues(rvec)[0]
        optical_from_base[:3, 3] = tvec.ravel()
        context = {'camera': camera, 'depth_camera': plane['depth_camera'],
                   'base_from_optical': np.linalg.inv(optical_from_base).tolist()}
        scenes.append((bgr, cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), context, str(metadata_path)))
        if len(scenes) == 6:
            break
    assert scenes, 'No matching archived scenes'
    yolo = settings['yolo']
    times = {k: [] for k in ('prediction_ms', 'geometry_ms', 'pair_compute_ms',
                            'validation_ms', 'rendering_ms')}
    accepted, measurements = 0, []

    def infer(scene):
        bgr, rgb, context, _ = scene
        start = time.perf_counter()
        check(source)
        result = model.predict(source=bgr, device=torch.device('cpu'), imgsz=yolo['image_size'],
            conf=yolo['confidence'], iou=yolo['iou'], max_det=yolo['max_detections'],
            classes=yolo['class_ids'], half=False, rect=False, batch=1, augment=False,
            agnostic_nms=False, nms=True, quantize=32, save=False, show=False, verbose=False,
            stream=False, retina_masks=args.native_masks)[0]
        predicted = time.perf_counter()
        reply, pixels = predict_trays({'settings': settings, 'plane': plane,
            'camera_context': context, 'returned_only': True, 'render_images': False},
            result, rgb, model.names, cv2, np)
        assert not pixels
        finished = time.perf_counter()
        check(source)
        return reply, (predicted-start)*1000, (finished-predicted)*1000

    for index in range(8):
        infer(scenes[index % len(scenes)])
    wall, cpu = time.perf_counter(), time.process_time()
    for index in range(args.pairs):
        scene = scenes[index % len(scenes)]
        start = time.perf_counter()
        first, a, ga = infer(scene)
        second, b, gb = infer(scene)
        validation_start = time.perf_counter()
        good = first['selected'] is not None and second['selected'] is not None
        if good:
            proof = compare(sample(first['selected'], index*2+1),
                            sample(second['selected'], index*2+2), settings['geometry'])
            good = proof['accepted']
            measurements.append([second['selected']['width_mm'], second['selected']['length_mm']])
        check(source)
        finished = time.perf_counter()
        accepted += int(good)
        times['prediction_ms'].extend((a,b))
        times['geometry_ms'].extend((ga,gb))
        times['validation_ms'].append((finished-validation_start)*1000)
        times['pair_compute_ms'].append((finished-start)*1000)
        render_start = time.perf_counter()
        render_capture({'width': scene[1].shape[1], 'height': scene[1].shape[0],
                        'settings': settings, 'result': second}, scene[1].tobytes(), cv2, np)
        times['rendering_ms'].append((time.perf_counter()-render_start)*1000)
    elapsed, consumed = time.perf_counter()-wall, time.process_time()-cpu
    summary = {key: {'median': statistics.median(values),
                     'p95': float(np.percentile(values,95))} for key, values in times.items()}
    args.output.write_text(json.dumps({'threads': args.threads, 'interop_threads': 1,
        'opencv_threads': 1, 'native_masks': args.native_masks, 'model_sha256': digest,
        'image_size': yolo['image_size'], 'scenes': [s[3] for s in scenes],
        'pairs': args.pairs, 'accepted_repeated_scene_pairs': accepted,
        'cpu_core_percent': consumed / elapsed * 100, 'timings': summary,
        'dimensions_mm': measurements,
        'limitations': 'Repeated archived scene; synthetic timestamps; reconstructed archive camera '
                       'transform; excludes live capture, TF, depth, ROS and physical arrival.'}, indent=2))
    print(json.dumps({'threads':args.threads,'native_masks':args.native_masks,
                      'cpu_core_percent':consumed/elapsed*100,'timings':summary}))


if __name__ == '__main__':
    main()
