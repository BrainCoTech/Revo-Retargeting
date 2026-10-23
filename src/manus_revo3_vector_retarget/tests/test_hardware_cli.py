"""Exercise operator options without constructing ROS or hardware transports."""
from pathlib import Path
import sys
import types

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from retarget_v2 import cli


@pytest.fixture
def invocation(monkeypatch):
    calls = []
    module = types.ModuleType("retarget_v2.hardware")
    module.hardware = calls.append
    monkeypatch.setitem(sys.modules, "retarget_v2.hardware", module)

    def run(*arguments):
        monkeypatch.setattr(sys, "argv", ["retarget", "hardware", *arguments])
        cli.main()
        return calls[-1]

    return run, calls


def test_default_is_official_disarmed_with_profile_limits(invocation):
    run, _ = invocation
    args = run()
    assert args.model == "may3"
    assert args.motion_scale is None
    assert args.max_velocity_rad_s is args.max_acceleration_rad_s2 is None
    assert args.hand_side == "right"
    assert args.namespace == "/revo3_right"
    assert args.interpolate is False
    assert args.domain is None


def test_interpolation_requires_explicit_option(invocation):
    run, _ = invocation
    assert run("--interpolate").interpolate is True


def test_left_hand_selects_its_namespace_and_accepts_explicit_override(invocation):
    run, _ = invocation
    args = run("--hand-side", "left")
    assert args.hand_side == "left"
    assert args.namespace == "/revo3_left"
    args = run("--hand-side", "left", "--namespace", "/custom_left")
    assert args.namespace == "/custom_left"


def test_independent_limits_and_explicit_current_model_selection(invocation):
    run, _ = invocation
    args = run("--model", "may3", "--max-velocity-rad-s", "1.2",
               "--max-acceleration-rad-s2", "3")
    assert args.model == "may3"
    assert args.max_velocity_rad_s == 1.2
    assert args.max_acceleration_rad_s2 == 3.
    assert args.motion_scale is None


@pytest.mark.parametrize("velocity, acceleration", [(12.01, 49.), (100., 100.), (1000., 10000.)])
def test_finite_positive_limits_above_former_ceilings_enter_hardware(invocation, velocity, acceleration):
    run, calls = invocation
    args = run("--max-velocity-rad-s", str(velocity),
               "--max-acceleration-rad-s2", str(acceleration))
    assert args.max_velocity_rad_s == velocity
    assert args.max_acceleration_rad_s2 == acceleration
    assert calls == [args]


@pytest.mark.parametrize("option", ["--max-velocity-rad-s", "--max-acceleration-rad-s2"])
@pytest.mark.parametrize("value", ["nan", "inf", "-inf", "0", "-1"])
def test_nonpositive_or_nonfinite_limits_never_enter_hardware(invocation, option, value):
    run, calls = invocation
    with pytest.raises(SystemExit) as error:
        run(f"{option}={value}")
    assert error.value.code == 2
    assert calls == []


@pytest.mark.parametrize("arguments", [
    ("--motion-scale", "1", "--max-velocity-rad-s", "1"),
    ("--motion-scale", "0.2", "--max-acceleration-rad-s2", "2"),
    ("--motion-scale", "0"),
    ("--headless",),
    ("--allow-commands",),
    ("--require-bus-diagnostics",),
    ("--duration", "2"),
    ("--topic", "/custom"),
    ("--output", "/tmp/unused-run"),
    ("--model", "legacy"),
    ("--hand-side", "both"),
])
def test_invalid_motion_or_unattended_output_never_enters_hardware(invocation, arguments):
    run, calls = invocation
    with pytest.raises(SystemExit) as error:
        run(*arguments)
    assert error.value.code == 2
    assert calls == []


def test_cache_directory_and_glove_selection(invocation):
    run, _ = invocation
    assert run().cache_dir is None
    args = run('--cache-dir', '/tmp/operator_alice', '--glove-id', '-123')
    assert args.cache_dir == Path('/tmp/operator_alice')
    assert args.glove_id == -123


def test_bringup_node_name_reaches_hardware(invocation):
    run, _ = invocation
    assert run().node_name is None
    assert run("--node-name", "manus_revo3_retarget").node_name == "manus_revo3_retarget"
