"""Persistent bone-length preparation without ROS or robot hardware."""
from copy import deepcopy
import json

import numpy as np
import pytest

from retarget_v2 import calibration_session
from retarget_v2.calibration_session import CalibrationSession
from retarget_v2.input import Calibration
from test_input import sdk_hand, transformed


def finish(session, frame, count):
    for _ in range(count - 1):
        assert session.advance(frame) is None
    return session.advance(frame)


def create_cache(directory, frame=None):
    frame = sdk_hand() if frame is None else frame
    session = CalibrationSession(calibration_frames=30, cache_dir=directory,
                                 hand_side=frame["side"].lower())
    assert finish(session, frame, 30) is not None
    return session


def test_default_full_calibration_records_only_bone_lengths_then_restores_after_thirty_frames(tmp_path):
    frame = sdk_hand()
    initial = CalibrationSession(cache_dir=tmp_path)
    assert initial.status()["state"] == "waiting_input"
    assert initial.progress == (0, 120)
    assert finish(initial, frame, 120) is initial.calibration
    assert initial.status()["state"] == "ready"
    assert initial.status()["cache_used"] is False
    value = json.loads(initial.cache_path.read_text())
    assert value["glove_id"] == frame["glove_id"]
    assert value["hand_side"] == "right" and value["normal_sign"] == -1.
    assert value["sample_count"] == 120
    assert np.asarray(value["lengths_m"]).shape == (5, 3)
    assert "raw_nodes" not in value
    original_file = initial.cache_path.read_bytes()

    restored = CalibrationSession(cache_dir=tmp_path)
    # Motion is allowed: these are bone lengths, not a stored reference pose.
    for index in range(29):
        assert restored.advance(sdk_hand(pip_bend_degrees=float(index * 2))) is None
    assert restored.status()["state"] == "checking_cache"
    assert restored.progress == (29, 30)
    assert restored.advance(sdk_hand(pip_bend_degrees=70.)) is restored.calibration
    assert restored.status()["cache_used"] is True
    np.testing.assert_array_equal(restored.calibration.lengths, initial.calibration.lengths)
    assert initial.cache_path.read_bytes() == original_file


def test_cache_verification_uses_cached_lengths_and_not_new_median(tmp_path):
    existing = create_cache(tmp_path)
    frame = transformed(sdk_hand(), np.eye(3) * 1.05, np.zeros(3))
    session = CalibrationSession(cache_dir=tmp_path)
    assert finish(session, frame, 30) is session.calibration
    assert session.status()["cache_used"]
    np.testing.assert_array_equal(session.calibration.lengths, existing.calibration.lengths)


def test_changed_bone_lengths_restart_full_sampling_before_replacing_cache(tmp_path):
    existing = create_cache(tmp_path)
    previous_file = existing.cache_path.read_bytes()
    frame = transformed(sdk_hand(), np.eye(3) * 1.2, np.zeros(3))
    session = CalibrationSession(cache_dir=tmp_path)
    assert finish(session, frame, 30) is None
    assert session.status()["state"] == "calibrating"
    assert session.progress == (0, 120)
    assert "differ" in session.status()["cache_error"]
    assert session.cache_path.read_bytes() == previous_file
    assert finish(session, frame, 120) is session.calibration
    assert session.status()["cache_used"] is False
    assert session.status()["cache_error"] is None
    np.testing.assert_allclose(session.calibration.lengths, existing.calibration.lengths * 1.2)
    assert session.cache_path.read_bytes() != previous_file


@pytest.mark.parametrize("broken", [
    "{incomplete", "null",
    {"schema_version": 100},
    {"schema_version": True},
    {"skeleton_profile": "old_landmarks"},
    {"glove_id": 7},
    {"hand_side": "left"},
    {"normal_sign": 1.},
    {"sample_count": 2},
    {"lengths_m": [[.03]]},
    {"lengths_m": [[float("nan")] * 3] * 5},
    {"lengths_m": [[.2] * 3] * 5},
])
def test_corrupt_or_incompatible_cache_falls_back_without_writing_until_success(tmp_path, broken):
    existing = create_cache(tmp_path)
    if isinstance(broken, dict):
        value = json.loads(existing.cache_path.read_text())
        value.update(broken)
        existing.cache_path.write_text(json.dumps(value))
    else:
        existing.cache_path.write_text(broken)
    corrupted = existing.cache_path.read_bytes()
    session = CalibrationSession(calibration_frames=30, cache_dir=tmp_path)
    assert session.advance(sdk_hand()) is None
    assert session.status()["state"] == "calibrating"
    assert session.status()["cache_error"].startswith("Cache ignored:")
    assert session.cache_path.read_bytes() == corrupted
    assert finish(session, sdk_hand(), 29) is session.calibration
    assert session.status()["cache_used"] is False
    assert json.loads(session.cache_path.read_text())["sample_count"] == 30


@pytest.mark.parametrize("identity", ["side", "glove", "normal", "skeleton"])
def test_cache_keys_separate_hand_glove_normal_and_local_skeleton_version(tmp_path, monkeypatch, identity):
    existing = create_cache(tmp_path)
    frame = sdk_hand()
    settings = dict(cache_dir=tmp_path)
    if identity == "side":
        frame = transformed(frame, np.diag([-1., 1., 1.]), np.zeros(3))
        frame["side"] = "Left"
        settings["hand_side"] = "left"
    elif identity == "glove":
        frame["glove_id"] = -123
    elif identity == "normal":
        settings["normal_sign"] = 1.
    else:
        monkeypatch.setattr(calibration_session, "SKELETON_PROFILE", "manus_sdk_common_25_v2")
    old_path = tmp_path / "12345_right_negative_manus_sdk_common_25_v1.json"
    assert old_path.is_file()
    session = CalibrationSession(**settings)
    assert session.advance(frame) is None
    assert session.status()["state"] == "calibrating"
    assert session.cache_path != old_path
    assert not session.cache_path.exists()


def test_input_gap_discards_pending_verification_but_never_discards_ready_result(tmp_path):
    create_cache(tmp_path)
    session = CalibrationSession(cache_dir=tmp_path)
    frame = sdk_hand()
    for _ in range(18):
        session.advance(frame)
    session.clear_pending()
    assert session.progress == (0, 30)
    result = finish(session, frame, 30)
    assert isinstance(result, Calibration)
    session.clear_pending()
    assert session.calibration is result
    assert session.progress == (30, 30)
    assert session.advance(frame) is result


def test_invalid_frame_clears_pending_without_locking_or_changing_identity(tmp_path):
    session = CalibrationSession(calibration_frames=30, cache_dir=tmp_path)
    frame = sdk_hand()
    bad = deepcopy(frame)
    bad["side"] = "Left"
    with pytest.raises(ValueError, match="right hand"):
        session.advance(bad)
    assert session.glove_id is None
    session.advance(frame)
    bad = deepcopy(frame)
    bad["glove_id"] += 1
    with pytest.raises(ValueError, match="identity changed"):
        session.advance(bad)
    assert session.progress == (0, 30)
    assert session.glove_id == frame["glove_id"]
    assert not session.cache_path.exists()
    assert finish(session, frame, 30) is session.calibration
    original = session.calibration
    bad = transformed(frame, np.eye(3) * 1.2, np.zeros(3))
    with pytest.raises(ValueError, match="Bone length changed"):
        session.advance(bad)
    assert session.calibration is original


def test_recalibration_is_explicit_and_failed_window_leaves_old_cache_intact(tmp_path):
    session = create_cache(tmp_path)
    stored = session.cache_path.read_bytes()
    session.reset()
    assert session.calibration is None
    assert session.status()["state"] == "calibrating"
    assert session.cache_path.read_bytes() == stored
    frame = sdk_hand()
    for _ in range(29):
        session.advance(frame)
    changed = transformed(frame, np.eye(3) * 1.2, np.zeros(3))
    with pytest.raises(ValueError, match="inconsistency"):
        session.advance(changed)
    assert session.progress == (0, 30)
    assert session.calibration is None
    assert session.cache_path.read_bytes() == stored
    assert finish(session, changed, 30) is session.calibration
    assert not session.status()["cache_used"]
    assert session.cache_path.read_bytes() != stored
    session.reset(force=False)
    assert finish(session, changed, 30) is session.calibration
    assert session.status()["cache_used"]


def test_cache_replace_failure_preserves_previous_file_and_cleans_temporary_file(tmp_path, monkeypatch):
    session = create_cache(tmp_path)
    old_file = session.cache_path.read_bytes()
    session.reset()

    def fail_replace(source, destination):
        # A complete JSON file must be written before the atomic replacement.
        assert json.loads(source.read_text())["sample_count"] == 30
        assert destination.read_bytes() == old_file
        raise OSError("read-only cache")

    monkeypatch.setattr(calibration_session.os, "replace", fail_replace)
    changed = transformed(sdk_hand(), np.eye(3) * 1.2, np.zeros(3))
    assert finish(session, changed, 30) is session.calibration
    assert session.status()["state"] == "ready"
    assert "read-only cache" in session.status()["cache_error"]
    assert session.cache_path.read_bytes() == old_file
    assert list(tmp_path.iterdir()) == [session.cache_path]


def test_pending_samples_are_isolated_from_caller_mutation(tmp_path):
    session = CalibrationSession(calibration_frames=30, cache_dir=tmp_path)
    first = sdk_hand()
    session.advance(first)
    first["raw_nodes"][1]["pose"]["position"]["x"] = float("nan")
    assert finish(session, sdk_hand(), 29) is session.calibration


def test_cache_location_uses_xdg_or_home(monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "xdg"))
    assert CalibrationSession().cache_dir == tmp_path / "xdg/manus_revo3_vector_retarget"
    monkeypatch.delenv("XDG_CACHE_HOME")
    monkeypatch.setattr(calibration_session.Path, "home", lambda: tmp_path / "home")
    assert CalibrationSession().cache_dir == tmp_path / "home/.cache/manus_revo3_vector_retarget"


def test_unavailable_cache_directory_never_blocks_fresh_calibration(tmp_path):
    cache_dir = tmp_path / "not_a_directory"
    cache_dir.write_text("unrelated file")
    session = CalibrationSession(calibration_frames=30, cache_dir=cache_dir)
    assert finish(session, sdk_hand(), 30) is session.calibration
    assert session.status()["state"] == "ready"
    assert session.status()["cache_used"] is False
    assert "could not be saved" in session.status()["cache_error"]
    assert cache_dir.read_text() == "unrelated file"


@pytest.mark.parametrize("settings", [
    {"calibration_frames": 29}, {"calibration_frames": True}, {"calibration_frames": 30.},
    {"hand_side": "both"}, {"normal_sign": 0.}, {"normal_sign": True},
])
def test_invalid_preparation_settings_are_rejected(settings, tmp_path):
    with pytest.raises(ValueError):
        CalibrationSession(cache_dir=tmp_path, **settings)
