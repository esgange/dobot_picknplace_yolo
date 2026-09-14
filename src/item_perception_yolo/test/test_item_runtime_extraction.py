import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile

import pytest


def extractor():
    path = Path(__file__).parents[1] / "scripts/extract_yolo_runtime.py"
    spec = importlib.util.spec_from_file_location("extractor_fixture", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.extract


def test_offline_extraction_missing_checksum_and_unsafe_member(tmp_path):
    wheel = tmp_path / "fixture.whl"
    with zipfile.ZipFile(wheel, "w") as archive:
        archive.writestr("runtime/LICENSE.txt", "Synthetic license fixture")
    manifest = {"schema_version": 1, "wheels": {wheel.name:
                hashlib.sha256(wheel.read_bytes()).hexdigest()}}
    lock = tmp_path / "item-yolo-runtime-lock.json"
    lock.write_text(json.dumps(manifest))
    extract = extractor()
    output = tmp_path / "extracted"
    extract(tmp_path, output)
    assert (output / "runtime/LICENSE.txt").read_text() == "Synthetic license fixture"
    wheel.write_bytes(wheel.read_bytes() + b"tamper")
    with pytest.raises(ValueError, match="checksum"):
        extract(tmp_path, tmp_path / "bad")
    assert not (tmp_path / "bad").exists()
    wheel.unlink()
    with pytest.raises(FileNotFoundError):
        extract(tmp_path, tmp_path / "missing")
    with zipfile.ZipFile(wheel, "w") as archive:
        archive.writestr("../escape", "not allowed")
    manifest["wheels"][wheel.name] = hashlib.sha256(wheel.read_bytes()).hexdigest()
    lock.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="Unsafe"):
        extract(tmp_path, tmp_path / "unsafe")
    assert not (tmp_path / "escape").exists()
