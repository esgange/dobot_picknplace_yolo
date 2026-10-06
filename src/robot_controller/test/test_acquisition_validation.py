"""Verified parsing is shared only within one complete dynamic validation pass."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from robot_controller import configuration, profiles


def test_controller_validates_item_model_once_and_keeps_dynamic_bindings(monkeypatch):
    profile = {"model": {"sha256": "model"}}
    loader = Mock(return_value=(profile, "item"))
    monkeypatch.setattr(configuration, "load_item_profile", loader)
    extra_loader = Mock(side_effect=AssertionError("duplicate item/model validation"))
    monkeypatch.setattr(profiles, "load_item_profile", extra_loader)
    station = Mock()
    camera = Mock()
    monkeypatch.setattr(profiles, "validate_applied_sources", station)
    monkeypatch.setattr(profiles, "validate_robot_camera_calibration", camera)
    digest = Mock(return_value="bin")
    monkeypatch.setattr(profiles, "file_sha256", digest)
    selection = profiles.Selection(Path("item"), profile, "item",
                                   SimpleNamespace(path=Path("bin"), sha256="bin"),
                                   object(), object(), False)
    tray = SimpleNamespace(validate_sources=Mock())
    config = configuration.ControllerConfiguration(Path("item"), Path("bin"), profile,
                                                   "item", (), np.eye(4), "config",
                                                   selection, False, tray)
    config.validate_sources(Path("."))
    loader.assert_called_once()
    extra_loader.assert_not_called()
    station.assert_called_once()
    camera.assert_called_once()
    digest.assert_called_once()
    tray.validate_sources.assert_called_once()
    digest.return_value = "changed"
    with pytest.raises(ValueError, match="Bin Teach changed"):
        config.validate_sources(Path("."))
    assert loader.call_count == 2  # Fresh contents are checked on every pass.
