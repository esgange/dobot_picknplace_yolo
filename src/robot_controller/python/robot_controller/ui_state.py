"""One atomic, strict, unapplied controller file-selection prefill store."""

import json
import os
from pathlib import Path
import tempfile


def load_state(path):
    path = Path(path)
    if not path.exists():
        return None
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        if (type(value) is not dict or set(value) != {"schema_version", "item", "bin"}
                or type(value["schema_version"]) is not int or value["schema_version"] != 1):
            raise ValueError("Controller UI state requires exact schema 1")
        for key in ("item", "bin"):
            if key == "bin" and value[key] is None:
                continue
            if (type(value[key]) is not str or Path(value[key]).name != value[key]
                    or not value[key].endswith(".yaml")):
                raise ValueError("Controller UI state requires named YAML files only")
        return value
    except (OSError, json.JSONDecodeError, UnicodeError) as exc:
        raise ValueError(f"Invalid controller UI state: {exc}") from exc


def save_state(path, item, bin_path):
    path = Path(path)
    # Validate a present store before replacing it; never hide malformed state.
    load_state(path)
    value = {"schema_version": 1, "item": Path(item).name,
             "bin": Path(bin_path).name if bin_path else None}
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
