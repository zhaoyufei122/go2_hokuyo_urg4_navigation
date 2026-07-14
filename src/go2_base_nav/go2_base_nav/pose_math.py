from dataclasses import dataclass
from math import asin, atan2, copysign, cos, isclose, isfinite, pi, sin, sqrt


@dataclass(frozen=True)
class QuaternionValue:
    x: float
    y: float
    z: float
    w: float

    def normalized(self) -> "QuaternionValue":
        components = (self.x, self.y, self.z, self.w)
        if not all(isfinite(value) for value in components):
            raise ValueError("quaternion components must be finite")

        norm = sqrt(sum(value * value for value in components))
        if norm < 1e-12:
            raise ValueError("quaternion norm is too small")

        return QuaternionValue(*(value / norm for value in components))

    def conjugate(self) -> "QuaternionValue":
        return QuaternionValue(-self.x, -self.y, -self.z, self.w)

    def roll(self) -> float:
        value = self.normalized()
        sin_roll = 2.0 * (value.w * value.x + value.y * value.z)
        cos_roll = 1.0 - 2.0 * (value.x * value.x + value.y * value.y)
        return atan2(sin_roll, cos_roll)

    def pitch(self) -> float:
        value = self.normalized()
        sin_pitch = 2.0 * (value.w * value.y - value.z * value.x)
        if abs(sin_pitch) >= 1.0:
            return copysign(pi / 2.0, sin_pitch)
        return asin(sin_pitch)

    def yaw(self) -> float:
        value = self.normalized()
        sin_yaw = 2.0 * (value.w * value.z + value.x * value.y)
        cos_yaw = 1.0 - 2.0 * (value.y * value.y + value.z * value.z)
        return atan2(sin_yaw, cos_yaw)

    @classmethod
    def from_rpy(cls, roll: float, pitch: float, yaw: float) -> "QuaternionValue":
        half_roll = roll / 2.0
        half_pitch = pitch / 2.0
        half_yaw = yaw / 2.0
        sin_roll, cos_roll = sin(half_roll), cos(half_roll)
        sin_pitch, cos_pitch = sin(half_pitch), cos(half_pitch)
        sin_yaw, cos_yaw = sin(half_yaw), cos(half_yaw)

        return cls(
            x=sin_roll * cos_pitch * cos_yaw
            - cos_roll * sin_pitch * sin_yaw,
            y=cos_roll * sin_pitch * cos_yaw
            + sin_roll * cos_pitch * sin_yaw,
            z=cos_roll * cos_pitch * sin_yaw
            - sin_roll * sin_pitch * cos_yaw,
            w=cos_roll * cos_pitch * cos_yaw
            + sin_roll * sin_pitch * sin_yaw,
        ).normalized()

    def is_equivalent(
        self,
        other: "QuaternionValue",
        *,
        abs_tol: float = 1e-9,
    ) -> bool:
        left = self.normalized()
        right = other.normalized()
        direct = all(
            isclose(a, b, abs_tol=abs_tol)
            for a, b in zip(
                (left.x, left.y, left.z, left.w),
                (right.x, right.y, right.z, right.w),
            )
        )
        negated = all(
            isclose(a, -b, abs_tol=abs_tol)
            for a, b in zip(
                (left.x, left.y, left.z, left.w),
                (right.x, right.y, right.z, right.w),
            )
        )
        return direct or negated


def multiply(a: QuaternionValue, b: QuaternionValue) -> QuaternionValue:
    return QuaternionValue(
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    )


def split_planar_orientation(
    full: QuaternionValue,
) -> tuple[QuaternionValue, QuaternionValue]:
    normalized = full.normalized()
    planar = QuaternionValue.from_rpy(0.0, 0.0, normalized.yaw())
    residual = multiply(planar.conjugate(), normalized).normalized()
    return planar, residual
