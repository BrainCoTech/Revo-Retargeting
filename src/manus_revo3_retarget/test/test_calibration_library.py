import json
from pathlib import Path

import pytest

from manus_revo3_retarget import retarget_tuning_panel as panel


def test_named_snapshots_survive_multiple_exports_in_one_second(tmp_path, monkeypatch):
    ticks = iter([1_700_000_000_100_000_000, 1_700_000_000_200_000_000])
    monkeypatch.setattr(panel.time, 'time_ns', lambda: next(ticks))
    names = []
    for payload in ('first', 'second'):
        name = panel.build_library_filename('operator', 'pinch', 'right')
        names.append(name)
        panel._atomic_write_with_backup(panel.library_side_dir(tmp_path, 'right') / name, payload)
        panel.write_latest_pointer(tmp_path, 'right', name)
    assert names[0] != names[1]
    assert {p.read_text() for p in panel.list_library(tmp_path, 'right')} == {'first', 'second'}
    assert panel.read_latest_pointer(tmp_path, 'right') == names[1]
    assert panel.read_latest_pointer(tmp_path, 'left') is None
    backup = panel.library_side_dir(tmp_path, 'right') / 'latest.json.bak'
    assert json.loads(backup.read_text())['filename'] == names[0]


def test_compatibility_yaml_retains_previous_backup(tmp_path):
    path = tmp_path / 'operator_revo3_right.yaml'
    panel._atomic_write_with_backup(path, 'old')
    panel._atomic_write_with_backup(path, 'new')
    assert path.read_text() == 'new'
    assert Path(str(path) + '.bak').read_text() == 'old'
    assert not list(tmp_path.glob('*.tmp'))


@pytest.mark.parametrize('payload', ['not json', '[]', '{"filename":"missing.yaml"}', '{"filename":"../outside.yaml"}'])
def test_invalid_or_stale_latest_is_ignored(tmp_path, payload):
    side = panel.library_side_dir(tmp_path, 'right')
    side.mkdir(parents=True)
    (side.parent / 'outside.yaml').write_text('outside')
    (side / 'latest.json').write_text(payload)
    assert panel.read_latest_pointer(tmp_path, 'right') is None


def test_invalid_robot_config_uses_defaults(tmp_path):
    config = tmp_path / 'robot.json'
    config.write_text('[]')
    assert panel._load_robot_export_defaults(config) == ('', panel.DEFAULT_CALIBRATION_DIR)
