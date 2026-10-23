"""Regression checks for source/install resource boundaries and run provenance."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from retarget_v2 import paths
from retarget_v2.model import RobotModel
from retarget_v2.viewer import _model_sources, _run_model_version, _run_hand_side


def test_install_description_uses_ament_index(monkeypatch, tmp_path):
    monkeypatch.setattr(paths, "workspace_root", lambda: None)
    monkeypatch.setattr(paths, "package_share", lambda name: tmp_path / name)
    assert paths.description_root() == tmp_path / "revo3_description"


@pytest.mark.parametrize("hand_side", ["left", "right"])
def test_profile_uses_installed_share_when_source_is_absent(monkeypatch, tmp_path, hand_side):
    monkeypatch.setattr(paths, "__file__", str(tmp_path / "site-packages/retarget_v2/paths.py"))
    monkeypatch.setattr(paths, "package_share", lambda name: tmp_path / "share" / name)
    assert paths.default_profile("may3", hand_side) == (
        tmp_path / "share/manus_revo3_vector_retarget/config" / f"{hand_side}_hardware_may3.yaml")


def test_replay_selects_hand_side_and_preserves_old_right_runs(tmp_path):
    for metadata, expected in [({}, "right"), ({"hand_side": "right"}, "right"),
                               ({"hand_side": "left"}, "left")]:
        (tmp_path / "run.json").write_text(json.dumps(metadata))
        assert _run_hand_side(tmp_path) == expected
    (tmp_path / "run.json").write_text(json.dumps({"hand_side": "both"}))
    with pytest.raises(ValueError, match="hand side"):
        _run_hand_side(tmp_path)


def test_source_path_rejects_directory_escape(tmp_path):
    robot = SimpleNamespace(description_root=tmp_path)
    with pytest.raises(ValueError, match="escapes"):
        paths.source_path("description/../outside", robot)


def test_replay_checks_portable_hashes_and_detects_tampering(tmp_path):
    with RobotModel() as robot:
        files = [Path(paths.__file__).with_name("model.py"), *robot.asset_paths]
        hashes = {paths.source_key(p, robot): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
        (tmp_path / "run.json").write_text(json.dumps({"source_hashes": hashes}))
        assert _model_sources(tmp_path, robot) == hashes
        hashes[paths.source_key(robot.urdf_path, robot)] = "changed"
        (tmp_path / "run.json").write_text(json.dumps({"source_hashes": hashes}))
        with pytest.raises(ValueError, match="changed"):
            _model_sources(tmp_path, robot)


def test_replay_requires_and_checks_nested_official_mesh_hashes(tmp_path):
    with RobotModel(model_version="may3") as robot:
        files = [Path(paths.__file__).with_name("model.py"), *robot.asset_paths]
        hashes = {paths.source_key(p, robot): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
        mesh = next(k for k in hashes if k.startswith("description/meshes/") and k.lower().endswith(".stl"))
        hashes[mesh] = "tampered"
        (tmp_path / "run.json").write_text(json.dumps({"source_hashes": hashes}))
        with pytest.raises(ValueError, match="changed"):
            _model_sources(tmp_path, robot)
        del hashes[mesh]
        (tmp_path / "run.json").write_text(json.dumps({"source_hashes": hashes}))
        with pytest.raises(ValueError, match="regenerate"):
            _model_sources(tmp_path, robot)


def test_replay_model_is_selected_from_recorded_provenance(tmp_path):
    for metadata, expected in [({"model_version": "may3"}, "may3")]:
        (tmp_path / "run.json").write_text(json.dumps(metadata))
        assert _run_model_version(tmp_path) == expected
    (tmp_path / "run.json").write_text(json.dumps({"model_version": "wrong"}))
    with pytest.raises(ValueError, match="Unknown"):
        _run_model_version(tmp_path)
    for metadata in ({}, {"model_version": "legacy"}):
        (tmp_path / "run.json").write_text(json.dumps(metadata))
        with pytest.raises(ValueError, match="Unknown"):
            _run_model_version(tmp_path)


def test_replay_rejects_legacy_or_missing_provenance(tmp_path):
    (tmp_path / "run.json").write_text(json.dumps({"source_hashes": {}}))
    with RobotModel() as robot, pytest.raises(ValueError, match="regenerate"):
        _model_sources(tmp_path, robot)
