import pytest

from item_perception_yolo.runtime_teach import runtime_teach_catalog


def _catalog_files(root):
    directory = root / "runtime_teach"
    directory.mkdir()
    for name in ("item_teach_part.yaml", "item_teach_part.pt",
                 "bin_teach_station.yaml"):
        (directory / name).write_bytes(b"fixture")
    return directory


def test_runtime_catalog_selects_one_prefix_classified_item_pair_and_bin(tmp_path):
    directory = _catalog_files(tmp_path)
    (directory / ".atomic-write-temporary").write_bytes(b"ignored")

    selected = runtime_teach_catalog(tmp_path)

    assert selected.directory == directory
    assert selected.item_yaml == directory / "item_teach_part.yaml"
    assert selected.item_model == directory / "item_teach_part.pt"
    assert selected.bin_yaml == directory / "bin_teach_station.yaml"


@pytest.mark.parametrize(
    ("filename", "message"),
    [
        ("item_teach_part.yaml",
         "Missing required item_teach_ YAML in runtime_teach/"),
        ("item_teach_part.pt",
         "Missing required item_teach_ PT model in runtime_teach/"),
        ("bin_teach_station.yaml",
         "Missing required bin_teach_ YAML in runtime_teach/"),
    ],
)
def test_runtime_catalog_reports_each_missing_artifact_kind(
        tmp_path, filename, message):
    directory = _catalog_files(tmp_path)
    (directory / filename).unlink()

    with pytest.raises(ValueError) as raised:
        runtime_teach_catalog(tmp_path)

    assert str(raised.value) == message


@pytest.mark.parametrize(
    ("filename", "message"),
    [
        ("item_teach_other.yaml",
         "Multiple item_teach_ YAML files in runtime_teach/: "
         "item_teach_other.yaml, item_teach_part.yaml"),
        ("item_teach_other.pt",
         "Multiple item_teach_ PT model files in runtime_teach/: "
         "item_teach_other.pt, item_teach_part.pt"),
        ("bin_teach_other.yaml",
         "Multiple bin_teach_ YAML files in runtime_teach/: "
         "bin_teach_other.yaml, bin_teach_station.yaml"),
    ],
)
def test_runtime_catalog_reports_each_duplicate_artifact_kind(
        tmp_path, filename, message):
    directory = _catalog_files(tmp_path)
    (directory / filename).write_bytes(b"duplicate")

    with pytest.raises(ValueError) as raised:
        runtime_teach_catalog(tmp_path)

    assert str(raised.value) == message


@pytest.mark.parametrize(
    ("change", "message"),
    [
        (lambda directory: (directory / "item_teach_part.pt").rename(
            directory / "item_teach_other.pt"), "same filename stem"),
        (lambda directory: (directory / "unknown.yaml").write_bytes(b"x"),
         "filename prefix"),
        (lambda directory: (directory / "tray_teach_future.yaml").write_bytes(b"x"),
         "reserved"),
        (lambda directory: (directory / "bin_teach_notes.txt").write_bytes(b"x"),
         "extension"),
        (lambda directory: (directory / "partition").mkdir(), "regular files"),
    ],
)
def test_runtime_catalog_rejects_mismatched_or_unsupported_entries(
        tmp_path, change, message):
    directory = _catalog_files(tmp_path)
    change(directory)
    with pytest.raises(ValueError, match=message):
        runtime_teach_catalog(tmp_path)


def test_runtime_catalog_rejects_symlinked_artifact(tmp_path):
    directory = _catalog_files(tmp_path)
    target = tmp_path / "outside.yaml"
    target.write_bytes(b"fixture")
    (directory / "bin_teach_station.yaml").unlink()
    (directory / "bin_teach_station.yaml").symlink_to(target)
    with pytest.raises(ValueError, match="regular files"):
        runtime_teach_catalog(tmp_path)


def test_runtime_catalog_requires_the_flat_directory(tmp_path):
    with pytest.raises(ValueError, match="missing or symlinked"):
        runtime_teach_catalog(tmp_path)

    target = tmp_path / "elsewhere"
    target.mkdir()
    (tmp_path / "runtime_teach").symlink_to(target, target_is_directory=True)
    with pytest.raises(ValueError, match="missing or symlinked"):
        runtime_teach_catalog(tmp_path)
