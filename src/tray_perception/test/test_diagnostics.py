from datetime import date

from tray_perception.diagnostics import prune_evidence


def test_evidence_retention_preserves_cutoff_operator_files_and_symlinks(tmp_path):
    expired = tmp_path / '2026-09-12' / ('a' * 32)
    retained = tmp_path / '2026-09-13' / ('b' * 32)
    for directory in (expired, retained):
        directory.mkdir(parents=True)
        for name in ('raw.png', 'overlay.svg', 'detections.json'):
            (directory / name).write_text('fixture')
    operator = expired / 'operator-notes.txt'
    operator.write_text('preserve')
    linked = tmp_path / '2020-01-01' / ('c' * 32)
    linked.mkdir(parents=True)
    (linked / 'raw.png').symlink_to(operator)
    prune_evidence(tmp_path, date(2026, 10, 12))
    assert sorted(p.name for p in expired.iterdir()) == ['operator-notes.txt']
    assert len(list(retained.iterdir())) == 3
    assert (linked / 'raw.png').is_symlink()
    assert operator.read_text() == 'preserve'
