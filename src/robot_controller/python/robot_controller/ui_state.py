"""One atomic, strict, unapplied controller file-selection prefill store."""

import json
import os
from pathlib import Path
import tempfile

from .placement import validate_target


def load_state(path):
    path = Path(path)
    if not path.exists():
        return None
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        if type(value) is not dict or type(value.get("schema_version")) is not int:
            raise ValueError("Controller UI state requires schema 1, 2 or 3")
        # Explicit schema-1 import preserves the existing Item/Bin selections.
        if value["schema_version"] == 1 and set(value) == {"schema_version", "item", "bin"}:
            value = {**value, "schema_version": 2, "tray": None}
        if value["schema_version"] == 2 and set(value) == {"schema_version", "item", "bin", "tray"}:
            value = {**value, "schema_version": 3, "placement": None}
        if value["schema_version"] != 3 or set(value) != {
                "schema_version", "item", "bin", "tray", "placement"}:
            raise ValueError("Controller UI state requires exact schema 3")
        placement = value["placement"]
        if placement is not None:
            if type(placement) is not list or len(placement) != 3:
                raise ValueError("Placement prefill requires X, Y, Rotation")
            validate_target(*placement)
        for key in ("item", "bin", "tray"):
            if key != "item" and value[key] is None:
                continue
            if (type(value[key]) is not str or Path(value[key]).name != value[key]
                    or not value[key].endswith(".yaml")):
                raise ValueError("Controller UI state requires named YAML files only")
        return value
    except (OSError, json.JSONDecodeError, UnicodeError) as exc:
        raise ValueError(f"Invalid controller UI state: {exc}") from exc


def save_state(path, item, bin_path, tray_path=None, *, placement=None):
    path = Path(path)
    # Validate a present store before replacing it; never hide malformed state.
    previous = load_state(path)
    placement = (previous["placement"] if previous is not None else None) if placement is None else list(
        validate_target(*placement))
    value = {"schema_version": 3, "item": Path(item).name,
             "bin": Path(bin_path).name if bin_path else None,
             "tray": Path(tray_path).name if tray_path else None, "placement": placement}
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent,
                                         prefix=".last_session.", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(value, stream, sort_keys=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
