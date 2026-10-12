"""Real temporary-file retention checks; no cameras, ROS nodes or robot commands."""

import os
from pathlib import Path

import pytest

from item_perception_yolo import item_detector as detector


def capture(directory, kind, index, age, *, depth=True):
    stem = (f'pick_20250101T000000_000000Z_{index:032x}' if kind == 'pick'
            else f'tray_{index:032x}')
    paths = [directory / f'{stem}_{channel}.png'
             for channel in (('rgb', 'depth') if depth else ('rgb',))]
    for path in paths:
        path.write_bytes(b'old capture')
        os.utime(path, ns=(age, age))
    return paths


def save_pick(root, identifier='f' * 32):
    return detector.save_pick_debug_pair(root, identifier, {
        'width': 1, 'height': 1, 'rgb': bytes(3), 'depth_rgb': bytes(3)})


def test_item_save_trims_existing_overflow_to_500_pairs(tmp_path):
    folder = tmp_path / 'debug/pick_img'
    folder.mkdir(parents=True)
    old = [capture(folder, 'pick', i, i + 1) for i in range(507)]
    result = save_pick(tmp_path)
    assert not result.get('error')
    assert len(list(folder.glob('*.png'))) == 1000
    assert all(not path.exists() for group in old[:8] for path in group)
    assert all(path.exists() for group in old[8:] for path in group)
    assert all(Path(result[key]).is_file() for key in ('rgb_path', 'depth_path'))


@pytest.mark.parametrize('kind', ['pick', 'tray'])
def test_oldest_group_uses_save_time_and_preserves_current_after_clock_change(
        tmp_path, monkeypatch, kind):
    monkeypatch.setattr(detector, 'MAX_DEBUG_CAPTURES', 2)
    with detector.debug_capture_directory(tmp_path, kind) as folder:
        oldest = capture(folder, kind, 99, 10)
        middle = capture(folder, kind, 98, 20)
        newest = capture(folder, kind, 97, 30)
        # RGB-only tray captures still count as one, even after a clock rollback.
        current = capture(folder, kind, 96, 1, depth=kind == 'pick')
        assert detector.trim_debug_captures(folder, kind, current) == ''
        assert all(not path.exists() for path in oldest + middle)
        assert all(path.exists() for path in newest + current)


@pytest.mark.parametrize('kind', ['pick', 'tray'])
def test_limit_is_per_detector_and_preserves_unrelated_entries(tmp_path, monkeypatch, kind):
    monkeypatch.setattr(detector, 'MAX_DEBUG_CAPTURES', 1)
    with detector.debug_capture_directory(tmp_path, kind) as folder:
        old = capture(folder, kind, 1, 1)
        current = capture(folder, kind, 2, 2)
        other_kind = 'tray' if kind == 'pick' else 'pick'
        unrelated = capture(folder, other_kind, 3, 1)
        for name in ('operator.png', '.inflight.tmp', 'profile.yaml'):
            path = folder / name
            path.write_bytes(b'preserve')
            unrelated.append(path)
        subdir = folder / 'nested'
        subdir.mkdir()
        unrelated += capture(subdir, kind, 4, 1)
        linked = capture(folder, kind, 5, 1)[0]
        linked.unlink()
        linked.symlink_to(unrelated[0])
        other_folder = tmp_path / 'debug' / f'{other_kind}_img'
        other_folder.mkdir()
        unrelated += capture(other_folder, other_kind, 6, 1)
        assert not detector.trim_debug_captures(folder, kind, current)
        assert all(path.exists() for path in current + unrelated)
        assert linked.is_symlink() and linked.read_bytes() == b'old capture'
        assert all(not path.exists() for path in old)


@pytest.mark.parametrize('kind', ['pick', 'tray'])
@pytest.mark.parametrize('link', ['debug', 'folder', 'lock'])
def test_capture_directory_never_follows_symlinks(tmp_path, kind, link):
    outside = tmp_path / 'preserve'
    outside.mkdir()
    keep = outside / 'keep'
    keep.write_text('untouched')
    debug = tmp_path / 'debug'
    if link == 'debug':
        debug.symlink_to(outside, target_is_directory=True)
    else:
        debug.mkdir()
        folder = debug / f'{kind}_img'
        if link == 'folder':
            folder.symlink_to(outside, target_is_directory=True)
        else:
            folder.mkdir()
            (folder / '.capture.lock').symlink_to(keep)
    with pytest.raises((ValueError, OSError)):
        with detector.debug_capture_directory(tmp_path, kind):
            pytest.fail('Linked directory or lock admitted')
    assert keep.read_text() == 'untouched' and list(outside.iterdir()) == [keep]


def test_active_writer_is_not_pruned_or_waited_on(tmp_path):
    with detector.debug_capture_directory(tmp_path, 'pick') as folder:
        with pytest.raises(BlockingIOError):
            save_pick(tmp_path)
        assert not list(folder.glob('*.png'))
    assert Path(save_pick(tmp_path)['rgb_path']).is_file()


def test_failed_item_save_keeps_previous_history(tmp_path, monkeypatch):
    monkeypatch.setattr(detector, 'MAX_DEBUG_CAPTURES', 1)
    previous = save_pick(tmp_path, 'a' * 32)
    encode = detector.encode_rgb_png
    calls = 0

    def fail_depth(*args):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError('Depth write failed')
        return encode(*args)
    monkeypatch.setattr(detector, 'encode_rgb_png', fail_depth)
    with pytest.raises(OSError, match='Depth write failed'):
        save_pick(tmp_path)
    folder = tmp_path / 'debug/pick_img'
    assert set(folder.glob('*.png')) == {Path(path) for path in previous.values()}
    assert not list(folder.glob('*.tmp'))


def test_cleanup_failure_keeps_new_images_and_reports_error(tmp_path, monkeypatch):
    monkeypatch.setattr(detector, 'MAX_DEBUG_CAPTURES', 1)
    previous = save_pick(tmp_path, 'a' * 32)
    unlink = Path.unlink

    def refuse_old(path, *args, **kwargs):
        if str(path) in previous.values():
            raise PermissionError('Old capture is read-only')
        return unlink(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'unlink', refuse_old)
    result = save_pick(tmp_path)
    assert 'cleanup failed' in result['error']
    assert 'read-only' in result['error']
    assert all(Path(result[key]).exists() for key in ('rgb_path', 'depth_path'))
    assert all(Path(path).exists() for path in previous.values())
