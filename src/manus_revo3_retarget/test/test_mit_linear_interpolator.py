import math

import pytest

from manus_revo3_retarget.mit_linear_interpolator import (
    LinearMitCommandInterpolator,
    duration_from_hz,
)


def assert_close(actual, expected, tol=1e-9):
    assert len(actual) == len(expected)
    for a, e in zip(actual, expected):
        assert abs(a - e) <= tol


def test_no_target_returns_none():
    interp = LinearMitCommandInterpolator(default_duration_s=1.0)

    assert interp.sample(0.0) is None


def test_first_target_publishes_constant_command():
    interp = LinearMitCommandInterpolator(default_duration_s=0.5)

    interp.update_target(["j0", "j1"], [1.0, 2.0], [3.0, 4.0], [0.1, 0.2], now_s=10.0)
    sample = interp.sample(10.1)

    assert sample is not None
    assert sample.joint_names == ["j0", "j1"]
    assert_close(sample.position, [1.0, 2.0])
    assert_close(sample.velocity, [0.0, 0.0])
    assert_close(sample.kp, [3.0, 4.0])
    assert_close(sample.kd, [0.1, 0.2])


def test_linearly_interpolates_between_targets_using_previous_target_interval():
    interp = LinearMitCommandInterpolator(default_duration_s=0.25)

    interp.update_target(["j0"], [0.0], [1.0], [0.1], now_s=0.0)
    interp.update_target(["j0"], [2.0], [1.0], [0.1], now_s=1.0)

    sample = interp.sample(1.5)
    assert sample is not None
    assert_close(sample.position, [1.0])
    assert_close(sample.velocity, [2.0])


def test_velocity_uses_target_delta_not_current_interpolated_delta():
    interp = LinearMitCommandInterpolator(default_duration_s=10.0)

    interp.update_target(["j0"], [0.0], [1.0], [0.1], now_s=0.0)
    interp.update_target(["j0"], [10.0], [1.0], [0.1], now_s=10.0)
    assert_close(interp.sample(12.0).position, [2.0])

    interp.update_target(["j0"], [20.0], [1.0], [0.1], now_s=12.0)
    sample = interp.sample(13.0)

    assert sample is not None
    assert_close(sample.position, [11.0])
    assert_close(sample.velocity, [5.0])


def test_segment_end_clamps_to_target_and_zero_velocity():
    interp = LinearMitCommandInterpolator(default_duration_s=0.25)

    interp.update_target(["j0"], [0.0], [1.0], [0.1], now_s=0.0)
    interp.update_target(["j0"], [2.0], [1.0], [0.1], now_s=1.0)

    sample = interp.sample(2.1)
    assert sample is not None
    assert_close(sample.position, [2.0])
    assert_close(sample.velocity, [0.0])


def test_joint_name_or_size_change_resets_without_cross_interpolation():
    interp = LinearMitCommandInterpolator(default_duration_s=1.0)

    interp.update_target(["j0", "j1"], [0.0, 0.0], [1.0, 1.0], [0.1, 0.1], now_s=0.0)
    interp.update_target(["j0", "j1"], [2.0, 2.0], [1.0, 1.0], [0.1, 0.1], now_s=1.0)
    assert_close(interp.sample(1.25).position, [0.5, 0.5])

    interp.update_target(["j0"], [10.0], [2.0], [0.2], now_s=1.5)
    sample = interp.sample(1.5)
    assert sample is not None
    assert sample.joint_names == ["j0"]
    assert_close(sample.position, [10.0])
    assert_close(sample.velocity, [0.0])
    assert_close(sample.kp, [2.0])
    assert_close(sample.kd, [0.2])


@pytest.mark.parametrize("hz", [0.0, -1.0, math.inf, math.nan])
def test_duration_from_hz_rejects_non_positive_or_nonfinite_values(hz):
    with pytest.raises(ValueError):
        duration_from_hz(hz, name="mit_command_publish_hz")


def test_duration_from_hz_accepts_positive_values():
    assert duration_from_hz(200.0, name="mit_command_publish_hz") == pytest.approx(0.005)
