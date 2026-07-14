from math import inf, nan

import pytest

from go2_base_nav.command_safety import (
    MotionCommand,
    MotionLimits,
    WatchdogState,
    clamp_command,
    is_zero_command,
)


def test_clamp_enforces_real_robot_limits():
    command = clamp_command(0.9, 0.5, -0.8, MotionLimits())
    assert command.linear_x == 0.4
    assert command.linear_y == 0.0
    assert command.angular_z == -0.4


def test_clamp_handles_negative_limits_and_zero():
    command = clamp_command(-0.9, -0.5, 0.8, MotionLimits())
    assert command == MotionCommand(-0.4, 0.0, 0.4)
    assert is_zero_command(clamp_command(0.0, 0.0, 0.0, MotionLimits()))
    assert not is_zero_command(command)


@pytest.mark.parametrize("value", [nan, inf, -inf])
def test_clamp_rejects_non_finite_values(value):
    with pytest.raises(ValueError):
        clamp_command(value, 0.0, 0.0, MotionLimits())


def test_watchdog_stops_once_and_rearms_after_motion():
    watchdog = WatchdogState()
    moving = MotionCommand(0.4, 0.0, 0.0)

    watchdog.observe(moving, now_nanoseconds=1_000_000_000)
    assert not watchdog.expired(1_499_999_999, timeout_seconds=0.5)
    assert watchdog.expired(1_500_000_000, timeout_seconds=0.5)
    assert not watchdog.expired(2_000_000_000, timeout_seconds=0.5)

    watchdog.observe(moving, now_nanoseconds=3_000_000_000)
    assert not watchdog.stopped
    assert watchdog.expired(3_500_000_000, timeout_seconds=0.5)


def test_zero_command_marks_watchdog_stopped_without_timeout_repeat():
    watchdog = WatchdogState()
    watchdog.observe(MotionCommand(0.0, 0.0, 0.0), now_nanoseconds=10)
    assert watchdog.stopped
    assert not watchdog.expired(1_000_000_000, timeout_seconds=0.5)
