from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class MotionLimits:
    max_linear_x: float = 0.4
    max_linear_y: float = 0.0
    max_angular_z: float = 0.6
    # The GO2 sport firmware ignores very small yaw rates when turning on the
    # spot, so an in-place rotation has to be requested at min_angular_z or it
    # simply does not happen.
    min_angular_z: float = 0.4
    # Requests below this magnitude are treated as "no rotation". Without a
    # deadband, tiny heading corrections from the path tracker get promoted
    # to min_angular_z, producing bang-bang yaw oscillation.
    angular_deadband: float = 0.1
    # ...but the minimum only makes sense on the spot. Once the dog is
    # walking it tracks small yaw rates fine, and promoting a path tracker's
    # 0.1-0.4 rad/s correction to 0.4 rad/s turns a gentle curve into a ~1 m
    # radius swerve: the dog snakes, never advances along the path, and the
    # progress checker aborts the goal. Above this translation speed the
    # requested yaw rate is passed through untouched.
    in_place_linear_speed: float = 0.05

    def __post_init__(self) -> None:
        values = (
            self.max_linear_x,
            self.max_linear_y,
            self.max_angular_z,
            self.min_angular_z,
            self.angular_deadband,
            self.in_place_linear_speed,
        )
        if not all(isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("motion limits must be finite and non-negative")
        if self.min_angular_z > self.max_angular_z:
            raise ValueError("minimum angular velocity cannot exceed maximum")
        if self.angular_deadband >= self.min_angular_z:
            raise ValueError("angular deadband must be below the minimum")


@dataclass(frozen=True)
class MotionCommand:
    linear_x: float
    linear_y: float
    angular_z: float


def _clamp(value: float, maximum: float) -> float:
    return max(-maximum, min(maximum, value))


def _enforce_minimum_magnitude(
    value: float,
    minimum: float,
    deadband: float = 0.0,
) -> float:
    if abs(value) <= deadband:
        return 0.0
    if abs(value) >= minimum:
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

    clamped_linear_x = _clamp(linear_x, limits.max_linear_x)
    clamped_linear_y = _clamp(linear_y, limits.max_linear_y)
    clamped_angular_z = _clamp(angular_z, limits.max_angular_z)

    translating = (
        max(abs(clamped_linear_x), abs(clamped_linear_y))
        > limits.in_place_linear_speed
    )
    if translating:
        angular_z_out = clamped_angular_z
    else:
        angular_z_out = _enforce_minimum_magnitude(
            clamped_angular_z,
            limits.min_angular_z,
            limits.angular_deadband,
        )

    return MotionCommand(
        linear_x=clamped_linear_x,
        linear_y=clamped_linear_y,
        angular_z=angular_z_out,
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
