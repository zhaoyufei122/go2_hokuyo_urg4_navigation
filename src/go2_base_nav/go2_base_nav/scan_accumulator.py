"""Accumulate multiple deskewed GO2 point clouds into one denser cloud.

The Unitree onboard LiDAR (L1/L2) uses a non-repetitive scan pattern, so a
single frame is very sparse. Feeding sparse frames directly into
``pointcloud_to_laserscan`` leaves most angular bins empty, which previously
forced ``use_inf: true`` and let SLAM Toolbox / costmaps ray-trace through
real walls and furniture.

This node keeps a short sliding window of deskewed clouds (expressed in a
fixed accumulation frame, ``odom`` by default) and republishes their union.
Because the deskewed clouds are registered in a static frame, concatenating
them is exact and additionally removes the motion distortion that a
single-timestamp rigid projection used to re-introduce.
"""

from collections import deque
from math import isfinite
import signal
from typing import Deque, List, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import dtype_from_fields
import tf2_ros


FrameBuffer = Deque[Tuple[int, PointCloud2]]


def quaternion_to_matrix(
    quaternion: Sequence[float],
) -> np.ndarray:
    """Return the 3x3 rotation matrix for a quaternion (x, y, z, w)."""
    x, y, z, w = (float(value) for value in quaternion)
    norm_sq = x * x + y * y + z * z + w * w
    if not isfinite(norm_sq) or norm_sq == 0.0:
        raise ValueError("transform quaternion must be finite and non-zero")
    scale = 2.0 / norm_sq
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    )


def transform_cloud(
    cloud: PointCloud2,
    translation: Sequence[float],
    quaternion: Sequence[float],
    target_frame: str,
) -> PointCloud2:
    """Rigidly transform a cloud into target_frame, in place on a copy.

    tf2_sensor_msgs.do_transform_cloud is broken on Jazzy (its create_cloud
    call asserts a structured dtype that read_points_numpy no longer
    returns), so the x/y/z fields are transformed directly in the raw byte
    buffer, preserving every other field and the point layout exactly.
    """
    rotation = quaternion_to_matrix(quaternion)
    offset = np.array([float(value) for value in translation])
    if not np.isfinite(offset).all():
        raise ValueError("transform translation must be finite")

    dtype = dtype_from_fields(cloud.fields, point_step=cloud.point_step)
    points = np.frombuffer(bytes(cloud.data), dtype=dtype).copy()
    xyz = np.vstack((points["x"], points["y"], points["z"]))
    transformed = rotation @ xyz.astype(np.float64) + offset.reshape(3, 1)
    points["x"] = transformed[0]
    points["y"] = transformed[1]
    points["z"] = transformed[2]

    output = PointCloud2()
    output.header = cloud.header
    output.header.frame_id = target_frame
    output.fields = cloud.fields
    output.is_bigendian = cloud.is_bigendian
    output.point_step = cloud.point_step
    output.row_step = cloud.row_step
    output.width = cloud.width
    output.height = cloud.height
    output.is_dense = False
    output.data = points.tobytes()
    return output


def voxel_consistency_filter(
    cloud: PointCloud2,
    voxel_size: float,
    min_hits: int,
) -> PointCloud2:
    """Drop points whose voxel was hit fewer than min_hits times.

    Within a short accumulation window, real surfaces (walls, furniture)
    are hit by several frames and land in the same voxel, while LiDAR
    outliers, leg ghosts and odometry jitter land in isolated voxels.
    Requiring repeated hits per voxel removes those noise points cheaply.
    """
    if not isfinite(voxel_size) or voxel_size <= 0.0:
        raise ValueError("voxel_size must be finite and > 0")
    if min_hits < 1:
        raise ValueError("min_hits must be >= 1")
    if min_hits == 1:
        return cloud

    dtype = dtype_from_fields(cloud.fields, point_step=cloud.point_step)
    points = np.frombuffer(bytes(cloud.data), dtype=dtype)
    xyz = np.vstack((points["x"], points["y"], points["z"])).T
    finite = np.isfinite(xyz).all(axis=1)
    finite_points = points[finite]
    if finite_points.shape[0] == 0:
        kept = finite_points
    else:
        keys = np.floor(xyz[finite] / voxel_size).astype(np.int64)
        keys = np.ascontiguousarray(keys)
        keys_1d = keys.view(np.dtype((np.void, keys.dtype.itemsize * 3)))
        keys_1d = keys_1d.ravel()
        _, inverse, counts = np.unique(
            keys_1d, return_inverse=True, return_counts=True
        )
        kept = finite_points[counts[inverse] >= min_hits]

    output = PointCloud2()
    output.header = cloud.header
    output.fields = cloud.fields
    output.is_bigendian = cloud.is_bigendian
    output.point_step = cloud.point_step
    output.height = 1
    output.width = int(kept.shape[0])
    output.row_step = output.point_step * output.width
    output.is_dense = True
    output.data = kept.tobytes()
    return output


def prune_frame_buffer(
    buffer: FrameBuffer,
    latest_stamp_ns: int,
    window_ns: int,
    max_frames: int,
) -> FrameBuffer:
    """Drop frames older than the sliding window or beyond the frame cap."""
    if window_ns < 0 or max_frames < 1:
        raise ValueError("window_ns must be >= 0 and max_frames must be >= 1")
    oldest_allowed = latest_stamp_ns - window_ns
    while buffer and buffer[0][0] < oldest_allowed:
        buffer.popleft()
    while len(buffer) > max_frames:
        buffer.popleft()
    return buffer


def _field_layout(cloud: PointCloud2) -> tuple:
    return (
        tuple(
            (field.name, field.offset, field.datatype, field.count)
            for field in cloud.fields
        ),
        cloud.point_step,
        cloud.is_bigendian,
    )


def concatenate_clouds(clouds: List[PointCloud2]) -> PointCloud2:
    """Merge clouds that share the same frame and field layout.

    All input clouds must already be expressed in the same frame. Layouts
    are identical for clouds from a single driver, so merging is a cheap
    byte-level concatenation. The output keeps the newest cloud's header
    and field layout; if layouts differ, the newest cloud is returned.
    """
    if not clouds:
        raise ValueError("cannot concatenate an empty cloud list")
    newest = clouds[-1]
    layout = _field_layout(newest)
    if any(_field_layout(cloud) != layout for cloud in clouds):
        return newest

    output = PointCloud2()
    output.header = newest.header
    output.fields = newest.fields
    output.is_bigendian = newest.is_bigendian
    output.point_step = newest.point_step
    output.height = 1
    output.width = int(sum(cloud.width * cloud.height for cloud in clouds))
    output.row_step = output.point_step * output.width
    output.is_dense = all(cloud.is_dense for cloud in clouds)
    output.data = b"".join(bytes(cloud.data) for cloud in clouds)
    return output


class ScanAccumulator(Node):
    def __init__(self) -> None:
        super().__init__("scan_accumulator")
        self.declare_parameter("input_topic", "/cloud_self_filtered")
        self.declare_parameter("output_topic", "/cloud_accumulated")
        self.declare_parameter("accumulation_frame", "odom")
        self.declare_parameter("window_sec", 0.5)
        self.declare_parameter("max_frames", 8)
        self.declare_parameter("transform_timeout", 0.1)
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("min_voxel_hits", 2)

        self._accumulation_frame = self.get_parameter(
            "accumulation_frame"
        ).value
        self._window_sec = float(self.get_parameter("window_sec").value)
        self._max_frames = int(self.get_parameter("max_frames").value)
        self._transform_timeout = float(
            self.get_parameter("transform_timeout").value
        )
        if not isfinite(self._window_sec) or self._window_sec < 0.0:
            raise ValueError("window_sec must be finite and >= 0")
        if self._max_frames < 1:
            raise ValueError("max_frames must be >= 1")
        self._voxel_size = float(self.get_parameter("voxel_size").value)
        self._min_voxel_hits = int(
            self.get_parameter("min_voxel_hits").value
        )
        if not isfinite(self._voxel_size) or self._voxel_size <= 0.0:
            raise ValueError("voxel_size must be finite and > 0")
        if self._min_voxel_hits < 1:
            raise ValueError("min_voxel_hits must be >= 1")

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._frames: FrameBuffer = deque()

        self._publisher = self.create_publisher(
            PointCloud2,
            self.get_parameter("output_topic").value,
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("input_topic").value,
            self._handle_cloud,
            qos_profile_sensor_data,
        )

    def _to_accumulation_frame(self, cloud: PointCloud2) -> PointCloud2 | None:
        source_frame = cloud.header.frame_id
        if not source_frame or source_frame == self._accumulation_frame:
            cloud.header.frame_id = self._accumulation_frame
            return cloud
        try:
            transform = self._tf_buffer.lookup_transform(
                self._accumulation_frame,
                source_frame,
                Time.from_msg(cloud.header.stamp),
                timeout=Duration(seconds=self._transform_timeout),
            )
        except tf2_ros.ExtrapolationException:
            # The GO2 onboard clock is not synced to the PC and the lidar
            # pipeline stamps clouds slightly ahead of the odometry TF
            # stream, so a stamped lookup can require extrapolation into
            # the future. Falling back to the latest transform costs at
            # most a few tens of milliseconds of motion, which is
            # negligible at GO2 walking speeds.
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._accumulation_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self._transform_timeout),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as error:
                self.get_logger().warning(
                    f"Skipping cloud: no transform {source_frame} -> "
                    f"{self._accumulation_frame} ({error})",
                    throttle_duration_sec=2.0,
                )
                return None
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
        ) as error:
            self.get_logger().warning(
                f"Skipping cloud: no transform {source_frame} -> "
                f"{self._accumulation_frame} ({error})",
                throttle_duration_sec=2.0,
            )
            return None
        try:
            transformed = transform_cloud(
                cloud,
                (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ),
                (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ),
                self._accumulation_frame,
            )
        except ValueError as error:
            self.get_logger().error(
                f"Skipping cloud: invalid transform ({error})",
                throttle_duration_sec=2.0,
            )
            return None
        return transformed

    def _handle_cloud(self, message: PointCloud2) -> None:
        cloud = self._to_accumulation_frame(message)
        if cloud is None:
            return

        stamp_ns = Time.from_msg(message.header.stamp).nanoseconds
        self._frames.append((stamp_ns, cloud))
        prune_frame_buffer(
            self._frames,
            stamp_ns,
            int(self._window_sec * 1_000_000_000),
            self._max_frames,
        )

        merged = concatenate_clouds([cloud for _, cloud in self._frames])
        merged.header.stamp = self._frames[-1][1].header.stamp
        merged.header.frame_id = self._accumulation_frame
        filtered = voxel_consistency_filter(
            merged, self._voxel_size, self._min_voxel_hits
        )
        self._publisher.publish(filtered)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ScanAccumulator()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        finally:
            if rclpy.ok():
                rclpy.shutdown()
