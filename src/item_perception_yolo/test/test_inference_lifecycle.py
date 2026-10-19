"""Instrument the real pinned worker to verify model and camera-cache reuse."""
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

from test_item_preview import native_paths, synthetic_model  # noqa: F401


@pytest.mark.parametrize("role", ["item", "tray"])
@pytest.mark.parametrize("change_during_predict", [False, True])
def test_native_model_loaded_once_and_changed_file_never_delivered(
        native_paths, synthetic_model, tmp_path, role, change_during_predict):
    model = tmp_path / "copy.pt"
    shutil.copyfile(synthetic_model, model)
    code = r'''
import contextlib, hashlib, io, json, sys
from pathlib import Path
runtime, path = Path(sys.argv[1]), Path(sys.argv[2])
sys.path.insert(0, str(runtime))
from item_perception_yolo import yolo_worker_native as native
from item_perception_yolo.preview_protocol import send_packet, receive_packet
with contextlib.redirect_stdout(sys.stderr):
    import ultralytics
counts = {"loads": 0, "hashes": 0, "clears": 0, "predict_clears": [], "predictor_ids": []}
old_model, old_digest, old_clear = ultralytics.YOLO, native._digest, native.clear_projection_caches
def load(*a, **kw):
    counts["loads"] += 1
    model = old_model(*a, **kw)
    old_predict = model.predict
    def predict(*a, **kw):
        counts["predict_clears"].append(counts["clears"])
        result = old_predict(*a, **kw)
        counts["predictor_ids"].append(id(model.predictor))
        if sys.argv[4] == "yes":
            with path.open("ab") as f: f.write(b"changed after prediction")
        return result
    model.predict = predict
    return model
def digest(path):
    counts["hashes"] += 1
    return old_digest(path)
def clear():
    counts["clears"] += 1
    old_clear()
ultralytics.YOLO, native._digest, native.clear_projection_caches = load, digest, clear
config = {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
          "task": "detect", "yolo": {"confidence": .9, "iou": .5, "image_size": 32,
                                        "max_detections": 10, "class_ids": [0, 1]}}
incoming, outgoing = io.BytesIO(), io.BytesIO()
send_packet(incoming, {"operation": "inspect", "model": config})
for i, generation in enumerate([1, 1, 2]):
    if i == 1: send_packet(incoming, {"operation": "thread_budget", "threads": 1})
    config["yolo"]["confidence"] = .9 + i * .01
    send_packet(incoming, {"operation": "detect" if sys.argv[3] == "item" else "tray_preview",
        "generation": generation, "camera_generation": 1, "model": config,
        "width": 32, "height": 32, "render_images": False}, bytes(32*32*3))
incoming.seek(0)
manifest = json.loads((runtime / "runtime-lock.json").read_text())
error = None
with contextlib.redirect_stdout(sys.stderr):
    try:
        native.serve(incoming, outgoing, runtime, manifest, path.parent,
            torch_threads=2, predictions={"tray_preview": lambda *a: ({"state": "ok"}, b"")})
    except RuntimeError as exc:
        error = str(exc)
outgoing.seek(0)
states = []
while True:
    try: states.append(receive_packet(outgoing)[0]["state"])
    except EOFError: break
print(json.dumps({**counts, "states": states, "error": error}))
'''
    result = subprocess.run(
        ["/usr/bin/python3", "-c", code, str(native_paths[1]), str(model), role,
         "yes" if change_during_predict else "no"],
        env=dict(os.environ, YOLO_AUTOINSTALL="false", YOLO_OFFLINE="true",
                 YOLO_CONFIG_DIR=str(tmp_path), MPLCONFIGDIR=str(tmp_path)),
        text=True, capture_output=True, timeout=45)
    assert result.returncode == 0, result.stderr
    evidence = json.loads(result.stdout)
    assert evidence["loads"] == 1 and evidence["hashes"] == 2
    if change_during_predict:
        assert evidence["states"] == ["ready", "ok", "error"]
        assert "changed during inference" in evidence["error"]
    else:
        assert evidence["states"] == ["ready", "ok", "ok", "ok", "ok", "ok"]
        first, same, changed = evidence["predict_clears"]
        assert first == same and changed == same + 1
        assert len(set(evidence["predictor_ids"])) == 1
        assert evidence["error"] is None


def test_untuned_client_does_not_hash_each_request(monkeypatch):
    from unittest.mock import MagicMock
    from item_perception_yolo import item_native_client as client
    native = client.NativeClient(MagicMock(), torch_threads=2)
    fail = MagicMock(side_effect=AssertionError("No tuning cache: do not hash request"))
    monkeypatch.setattr(client.cpu_budget, "workload_key", fail)
    native._select_threads({"operation": "detect", "model": {}})
    fail.assert_not_called()
