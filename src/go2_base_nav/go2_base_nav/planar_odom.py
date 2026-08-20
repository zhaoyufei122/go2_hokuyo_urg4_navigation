from copy import deepcopy
from math import isfinite
import signal
from typing import Iterable

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions
from tf2_ros import TransformBroadcaster

from go2_base_nav.pose_math import QuaternionValue, split_planar_orientation


def _quaternion_value(message: Quaternion) -> QuaternionValue:
    return QuaternionValue(
        x=message.x,
        y=message.y,
        z=message.z,
        w=message.w,
    )


def _set_quaternion(message: Quaternion, value: QuaternionValue) -> None:
    message.x = value.x
    message.y = value.y
    message.z = value.z
    message.w = value.w


def _require_finite(values: Iterable[float]) -> None:
    if not all(isfinite(value) for value in values):
        raise ValueError("odometry pose and planar twist values must be finite")


def planarize_odometry(
    message: Odometry,
    odom_frame: str = "odom",
    footprint_frame: str = "base_footprint",
    base_frame: str = "base_link",
) -> tuple[Odometry, TransformStamped, TransformStamped]:
    position = message.pose.pose.position
    twist = message.twist.twist
    _require_finite(
        (
            position.x,
            position.y,
            position.z,
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
        )
    )
    planar, residual = split_planar_orientation(
        _quaternion_value(message.pose.pose.orientation)
    )

    output = deepcopy(message)
    output.header.frame_id = odom_frame
    output.child_frame_id = footprint_frame
    output.pose.pose.position.z = 0.0
    _set_quaternion(output.pose.pose.orientation, planar)
    output.twist.twist.linear.z = 0.0
    output.twist.twist.angular.x = 0.0
    output.twist.twist.angular.y = 0.0

    odom_to_footprint = TransformStamped()
    odom_to_footprint.header.stamp = deepcopy(message.header.stamp)
    odom_to_footprint.header.frame_id = odom_frame
    odom_to_footprint.child_frame_id = footprint_frame
    odom_to_footprint.transform.translation.x = position.x
    odom_to_footprint.transform.translation.y = position.y
    odom_to_footprint.transform.translation.z = 0.0
    _set_quaternion(odom_to_footprint.transform.rotation, planar)

    footprint_to_base = TransformStamped()
    footprint_to_base.header.stamp = deepcopy(message.header.stamp)
    footprint_to_base.header.frame_id = footprint_frame
    footprint_to_base.child_frame_id = base_frame
    footprint_to_base.transform.translation.x = 0.0
    footprint_to_base.transform.translation.y = 0.0
    footprint_to_base.transform.translation.z = position.z
    _set_quaternion(footprint_to_base.transform.rotation, residual)

    return output, odom_to_footprint, footprint_to_base


class PlanarOdomNode(Node):
    def __init__(self) -> None:
        super().__init__("planar_odom")
        self.declare_parameter("input_odom_topic", "/utlidar/robot_odom")
        self.declare_parameter("output_odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("footprint_frame", "base_footprint")
        self.declare_parameter("base_frame", "base_link")
        # The onboard odometry arrives at ~150 Hz; republishing every
        # message floods /tf and every subscriber's buffer for no benefit.
        self.declare_parameter("output_rate_hz", 50.0)
        # The GO2 onboard clock is never perfectly synced with this computer.
        # Restamping on arrival removes residual clock skew from TF lookups.
        self.declare_parameter("restamp", True)
        self._restamp = bool(self.get_parameter("restamp").value)

        self._odom_frame = self.get_parameter("odom_frame").value
        self._footprint_frame = self.get_parameter("footprint_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        output_rate_hz = float(self.get_parameter("output_rate_hz").value)
        if not isfinite(output_rate_hz) or output_rate_hz <= 0.0:
            raise ValueError("output_rate_hz must be finite and > 0")
        self._min_period_ns = int(1_000_000_000 / output_rate_hz)
        self._last_publish_ns: int | None = None

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            Odometry,
            self.get_parameter("output_odom_topic").value,
            10,
        )
        self._transform_broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(
            Odometry,
            self.get_parameter("input_odom_topic").value,
            self._handle_odometry,
            input_qos,
        )

    def _handle_odometry(self, message: Odometry) -> None:
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if (
            self._last_publish_ns is not None
            and stamp_ns - self._last_publish_ns < self._min_period_ns
        ):
            return
        try:
            output, odom_to_footprint, footprint_to_base = planarize_odometry(
                message,
                odom_frame=self._odom_frame,
                footprint_frame=self._footprint_frame,
                base_frame=self._base_frame,
            )
        except ValueError as error:
            self.get_logger().error(f"Dropping invalid GO2 odometry: {error}")
            return

        self._last_publish_ns = stamp_ns
        if self._restamp:
            now = self.get_clock().now().to_msg()
            output.header.stamp = now
            odom_to_footprint.header.stamp = now
            footprint_to_base.header.stamp = now
        self._publisher.publish(output)
        self._transform_broadcaster.sendTransform(
            [odom_to_footprint, footprint_to_base]
        )


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PlanarOdomNode()
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
