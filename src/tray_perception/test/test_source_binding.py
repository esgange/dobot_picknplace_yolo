"""Strong filesystem fences and avoidance of repeated profile/model hashing."""
import os
import time
from unittest.mock import MagicMock

import pytest

from tray_perception import source_binding, requests
from test_requests import backend, artifact, arm, trigger, short_deadline


@pytest.mark.parametrize("edit", ["replace", "same_size_mtime", "delete", "symlink"])
def test_source_change_never_rebinds_silently(tmp_path, edit):
    path = tmp_path / "model.pt"
    path.write_bytes(b"original")
    binding = {"paths": [path], "fingerprints": source_binding.fingerprints([path])}
    before = path.stat()
    time.sleep(.02)  # Separate filesystem timestamp ticks; no mtime change is relied upon.
    source_binding.check(binding)
    if edit == "replace":
        replacement = tmp_path / "new.pt"
        replacement.write_bytes(path.read_bytes())
        os.replace(replacement, path)
    elif edit == "same_size_mtime":
        path.write_bytes(b"modified")
        os.utime(path, ns=(before.st_atime_ns, before.st_mtime_ns))
    elif edit == "delete":
        path.unlink()
    else:
        original = tmp_path / "original.pt"
        path.rename(original)
        path.symlink_to(original)
    with pytest.raises((ValueError, OSError)):
        source_binding.check(binding)


def test_cached_request_checks_metadata_without_rehashing_model(backend, monkeypatch):
    node, path, _ = backend
    arm(backend)
    hashing = MagicMock(side_effect=AssertionError("Unchanged binding must not rehash"))
    monkeypatch.setattr(requests, "file_sha256", hashing)
    monkeypatch.setattr(requests, "load_document", hashing)
    assert trigger(backend).found
    assert trigger(backend).found
    hashing.assert_not_called()
    before = path.stat()
    content = path.read_bytes()
    path.write_bytes(content.replace(b"tray", b"TRAY", 1))
    os.utime(path, ns=(before.st_atime_ns, before.st_mtime_ns))
    assert not trigger(backend).success
    # Restoring bytes/mtime does not restore the original ctime identity.
    path.write_bytes(content)
    os.utime(path, ns=(before.st_atime_ns, before.st_mtime_ns))
    assert not trigger(backend).success
    hashing.assert_not_called()


def test_source_edit_during_diagnostics_revokes_delivery(backend, monkeypatch):
    from tray_perception import diagnostics
    node, path, _ = backend
    arm(backend)
    original = diagnostics.TrayRequestDiagnostics.complete

    def change(self, response, view, sample_success):
        original(self, response, view, sample_success)
        path.write_text(path.read_text() + '\n# edited before delivery\n')
    monkeypatch.setattr(diagnostics.TrayRequestDiagnostics, 'complete', change)
    response = trigger(backend)
    assert not response.success and not response.found
    assert 'source changed' in response.message
