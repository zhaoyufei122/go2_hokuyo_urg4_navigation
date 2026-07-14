from math import isclose

from nav_msgs.msg import Odometry

from go2_base_nav.planar_odom import planarize_odometry
from go2_base_nav.pose_math import QuaternionValue, multiply


def _value(message_quaternion):
    return QuaternionValue(
        x=message_quaternion.x,
        y=message_quaternion.y,
        z=message_quaternion.z,
        w=message_quaternion.w,
    )


def test_planarize_odometry_splits_pose_and_preserves_measurements():
    message = Odometry()
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456_000_000
    message.header.frame_id = "raw_odom"
    message.child_frame_id = "base_link"
    message.pose.pose.position.x = 1.2
    message.pose.pose.position.y = -0.7
    message.pose.pose.position.z = 0.31

    full = QuaternionValue.from_rpy(0.2, -0.1, 0.8)
    message.pose.pose.orientation.x = full.x
    message.pose.pose.orientation.y = full.y
    message.pose.pose.orientation.z = full.z
    message.pose.pose.orientation.w = full.w
    message.twist.twist.linear.x = 0.35
    message.twist.twist.linear.y = 0.0
    message.twist.twist.angular.z = -0.2
    message.pose.covariance = [float(index) for index in range(36)]
    message.twist.covariance = [float(index + 100) for index in range(36)]

    odom, odom_to_footprint, footprint_to_base = planarize_odometry(message)

    assert odom.header.stamp == message.header.stamp
    assert odom.header.frame_id == "odom"
    assert odom.child_frame_id == "base_footprint"
    assert isclose(odom.pose.pose.position.x, 1.2)
    assert isclose(odom.pose.pose.position.y, -0.7)
    assert isclose(odom.pose.pose.position.z, 0.0)
    planar = _value(odom.pose.pose.orientation)
    assert isclose(planar.roll(), 0.0, abs_tol=1e-9)
    assert isclose(planar.pitch(), 0.0, abs_tol=1e-9)
    assert isclose(planar.yaw(), 0.8, abs_tol=1e-9)
    assert list(odom.pose.covariance) == list(message.pose.covariance)
    assert list(odom.twist.covariance) == list(message.twist.covariance)
    assert odom.twist == message.twist

    assert odom_to_footprint.header.stamp == message.header.stamp
    assert odom_to_footprint.header.frame_id == "odom"
    assert odom_to_footprint.child_frame_id == "base_footprint"
    assert isclose(odom_to_footprint.transform.translation.x, 1.2)
    assert isclose(odom_to_footprint.transform.translation.y, -0.7)
    assert isclose(odom_to_footprint.transform.translation.z, 0.0)
    assert _value(odom_to_footprint.transform.rotation).is_equivalent(planar)

    assert footprint_to_base.header.stamp == message.header.stamp
    assert footprint_to_base.header.frame_id == "base_footprint"
    assert footprint_to_base.child_frame_id == "base_link"
    assert isclose(footprint_to_base.transform.translation.x, 0.0)
    assert isclose(footprint_to_base.transform.translation.y, 0.0)
    assert isclose(footprint_to_base.transform.translation.z, 0.31)
    residual = _value(footprint_to_base.transform.rotation)
    assert multiply(planar, residual).is_equivalent(full, abs_tol=1e-9)
