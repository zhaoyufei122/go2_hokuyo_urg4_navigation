from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class MotionLimits:
    max_linear_x: float = 0.4
    max_linear_y: float = 0.0
    max_angular_z: float = 0.6
    min_angular_z: float = 0.4

    def __post_init__(self) -> None:
        values = (
            self.max_linear_x,
            self.max_linear_y,
            self.max_angular_z,
            self.min_angular_z,
        )
        if not all(isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("motion limits must be finite and non-negative")
        if self.min_angular_z > self.max_angular_z:
            raise ValueError("minimum angular velocity cannot exceed maximum")


@dataclass(frozen=True)
class MotionCommand:
    linear_x: float
    linear_y: float
    angular_z: float


def _clamp(value: float, maximum: float) -> float:
    return max(-maximum, min(maximum, value))


def _enforce_minimum_magnitude(value: float, minimum: float) -> float:
    if value == 0.0 or abs(value) >= minimum:
        return value
    return minimum if value > 0.0 else -minimum


def clamp_command(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    limits: MotionLimits,
) -> MotionCommand:
    values = (linear_x, linear_y, angular_z)
    if not all(isfinite(value) for value in values):
        raise ValueError("motion command values must be finite")

    clamped_angular_z = _clamp(angular_z, limits.max_angular_z)
    return MotionCommand(
        linear_x=_clamp(linear_x, limits.max_linear_x),
        linear_y=_clamp(linear_y, limits.max_linear_y),
        angular_z=_enforce_minimum_magnitude(
            clamped_angular_z,
            limits.min_angular_z,
        ),
    )


def is_zero_command(command: MotionCommand, *, tolerance: float = 1e-9) -> bool:
    return all(
        abs(value) <= tolerance
        for value in (command.linear_x, command.linear_y, command.angular_z)
    )


@dataclass
class WatchdogState:
    last_command_nanoseconds: int | None = None
    stopped: bool = False

    def observe(self, command: MotionCommand, *, now_nanoseconds: int) -> bool:
        self.last_command_nanoseconds = now_nanoseconds
        was_stopped = self.stopped
        self.stopped = is_zero_command(command)
        return self.stopped and not was_stopped

    def expired(self, now_nanoseconds: int, *, timeout_seconds: float) -> bool:
        if not isfinite(timeout_seconds) or timeout_seconds < 0.0:
            raise ValueError("watchdog timeout must be finite and non-negative")
        if self.stopped or self.last_command_nanoseconds is None:
            return False

        elapsed_nanoseconds = now_nanoseconds - self.last_command_nanoseconds
        if elapsed_nanoseconds < int(timeout_seconds * 1_000_000_000):
            return False

        self.stopped = True
        return True
