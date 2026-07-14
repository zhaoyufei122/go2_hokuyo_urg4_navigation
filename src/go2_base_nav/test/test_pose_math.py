from math import isclose, pi

from go2_base_nav.pose_math import (
    QuaternionValue,
    multiply,
    split_planar_orientation,
)


def test_split_recomposes_full_orientation():
    full = QuaternionValue(
        x=0.0843056797421489,
        y=-0.07285182744658007,
        z=0.2506948010244541,
        w=0.961632611936709,
    )
    planar, residual = split_planar_orientation(full)
    recomposed = multiply(planar, residual)
    assert isclose(recomposed.x, full.x, abs_tol=1e-9)
    assert isclose(recomposed.y, full.y, abs_tol=1e-9)
    assert isclose(recomposed.z, full.z, abs_tol=1e-9)
    assert isclose(recomposed.w, full.w, abs_tol=1e-9)


def test_planar_part_contains_yaw_only():
    full = QuaternionValue.from_rpy(0.2, -0.1, pi / 3.0)
    planar, residual = split_planar_orientation(full)
    assert isclose(planar.yaw(), pi / 3.0, abs_tol=1e-9)
    assert isclose(planar.roll(), 0.0, abs_tol=1e-9)
    assert isclose(planar.pitch(), 0.0, abs_tol=1e-9)
    recomposed = multiply(planar, residual)
    assert recomposed.is_equivalent(full, abs_tol=1e-9)
