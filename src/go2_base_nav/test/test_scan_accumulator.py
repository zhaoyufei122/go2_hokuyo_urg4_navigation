from collections import deque

import pytest
from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points_numpy
from std_msgs.msg import Header

from go2_base_nav.scan_accumulator import (
    concatenate_clouds,
    prune_frame_buffer,
    quaternion_to_matrix,
    transform_cloud,
    voxel_consistency_filter,
)


FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]


def _make_cloud(points, frame_id="odom"):
    header = Header()
    header.frame_id = frame_id
    return create_cloud(header, FIELDS, points)


def test_prune_drops_frames_older_than_window():
    buffer = deque((stamp, object()) for stamp in (0, 2, 4, 6, 8))

    prune_frame_buffer(buffer, latest_stamp_ns=8, window_ns=5, max_frames=10)

    assert [stamp for stamp, _ in buffer] == [4, 6, 8]


def test_prune_caps_frame_count():
    buffer = deque((stamp, object()) for stamp in range(10))

    prune_frame_buffer(buffer, latest_stamp_ns=9, window_ns=100, max_frames=3)

    assert [stamp for stamp, _ in buffer] == [7, 8, 9]


def test_prune_rejects_invalid_limits():
    buffer = deque()
    with pytest.raises(ValueError):
        prune_frame_buffer(buffer, 0, -1, 3)
    with pytest.raises(ValueError):
        prune_frame_buffer(buffer, 0, 1, 0)


def test_concatenate_merges_points_and_preserves_layout():
    first = _make_cloud([(1.0, 2.0, 0.3), (2.0, 0.0, 0.4)])
    second = _make_cloud([(3.0, 1.0, 0.2)])

    merged = concatenate_clouds([first, second])

    points = read_points_numpy(merged)
    assert merged.width == 3
    assert merged.height == 1
    assert merged.row_step == merged.point_step * 3
    assert len(points) == 3
    assert merged.fields == second.fields


def test_concatenate_rejects_empty_input():
    with pytest.raises(ValueError):
        concatenate_clouds([])


def test_quaternion_to_matrix_identity_and_rotation():
    identity = quaternion_to_matrix((0.0, 0.0, 0.0, 1.0))
    assert identity.flatten().tolist() == pytest.approx(
        [1, 0, 0, 0, 1, 0, 0, 0, 1]
    )

    # 90 degrees about z: x axis maps to y axis.
    import math

    rotated = quaternion_to_matrix(
        (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    )
    assert (rotated @ [1.0, 0.0, 0.0]) == pytest.approx([0.0, 1.0, 0.0])


def test_quaternion_to_matrix_rejects_zero_quaternion():
    with pytest.raises(ValueError):
        quaternion_to_matrix((0.0, 0.0, 0.0, 0.0))


def test_transform_cloud_rotates_translates_and_preserves_layout():
    import math

    cloud = _make_cloud([(1.0, 0.0, 0.5), (0.0, 2.0, -0.5)])
    transformed = transform_cloud(
        cloud,
        translation=(10.0, 0.0, 1.0),
        quaternion=(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)),
        target_frame="odom",
    )

    points = read_points_numpy(transformed)
    assert transformed.header.frame_id == "odom"
    assert transformed.width == cloud.width
    assert transformed.point_step == cloud.point_step
    assert list(points[0]) == pytest.approx([10.0, 1.0, 1.5])
    # (0, 2, -0.5) rotated 90 deg about z -> (-2, 0, -0.5), then translated.
    assert list(points[1]) == pytest.approx([8.0, 0.0, 0.5])


def test_transform_cloud_rejects_non_finite_translation():
    cloud = _make_cloud([(1.0, 0.0, 0.5)])
    with pytest.raises(ValueError):
        transform_cloud(
            cloud,
            translation=(float("nan"), 0.0, 0.0),
            quaternion=(0.0, 0.0, 0.0, 1.0),
            target_frame="odom",
        )


def test_voxel_filter_drops_isolated_noise_points():
    # Two frames hit the same wall voxel, plus one isolated noise outlier.
    cloud = _make_cloud(
        [
            (1.00, 2.00, 0.30),
            (1.01, 2.00, 0.31),  # same 5 cm voxel as the first point
            (5.00, 5.00, 0.30),  # isolated outlier, single hit
        ]
    )

    filtered = voxel_consistency_filter(cloud, voxel_size=0.05, min_hits=2)

    points = read_points_numpy(filtered)
    assert filtered.width == 2
    assert len(points) == 2
    assert all(abs(point[0] - 1.0) < 0.05 for point in points)


def test_voxel_filter_drops_non_finite_points():
    cloud = _make_cloud(
        [(float("nan"), 2.0, 0.3), (1.0, 2.0, 0.3), (1.01, 2.0, 0.3)]
    )

    filtered = voxel_consistency_filter(cloud, voxel_size=0.05, min_hits=2)

    assert filtered.width == 2
    assert filtered.is_dense is True


def test_voxel_filter_min_hits_one_is_passthrough():
    cloud = _make_cloud([(1.0, 2.0, 0.3), (5.0, 5.0, 0.3)])
    assert voxel_consistency_filter(cloud, 0.05, 1) is cloud


def test_voxel_filter_rejects_invalid_parameters():
    cloud = _make_cloud([(1.0, 2.0, 0.3)])
    with pytest.raises(ValueError):
        voxel_consistency_filter(cloud, 0.0, 2)
    with pytest.raises(ValueError):
        voxel_consistency_filter(cloud, 0.05, 0)
