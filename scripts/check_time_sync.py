#!/usr/bin/env python3
"""Measure the clock skew and transport jitter of every off-board sensor.

The Hokuyo driver runs on the GO2 Jetson and the odometry comes from the GO2
main computer, so both carry timestamps from clocks that are not this
computer's. scan_filter and planar_odom currently paper over that by restamping
every message with "now" on arrival, which trades a fixed clock offset for a
*variable* latency error -- and a variable error between the scan and the
odometry is exactly what makes the scan swing away from the map on turns.

This script quantifies it. For each topic it reports (arrival - header.stamp):

  offset  the minimum over the window. This is clock skew + the best-case
          transport latency, and it is the part a real time sync removes.
  jitter  max - min. This is the part restamping CANNOT fix: it lands
          directly on the pose that AMCL pairs with each scan.

What matters for localisation is the *difference* in offset between /scan_raw
and the odometry, and the jitter on each. Rules of thumb for this robot:

  jitter < 20 ms and |scan offset - odom offset| < 20 ms
      Good. Turn restamping off (restamp:=false on both nodes) and use the
      real timestamps.
  jitter of 50-200 ms
      This is worth 3-7 degrees of yaw error at 0.6 rad/s. Fix the Jetson
      clock (chrony/timesyncd against this computer) before chasing anything
      else in the localisation stack.

Run it with the lidar driver and the GO2 up, but with navigation stopped:

    source /home/yufei/Desktop/unitree_ros2/setup.sh
    python3 scripts/check_time_sync.py
"""

import argparse

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def _stamp_to_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class TimeSyncProbe(Node):
    def __init__(self, scan_topic: str, odom_topic: str) -> None:
        super().__init__("check_time_sync")
        self._deltas: dict[str, list[float]] = {}
        # The Foxy urg_node publishes reliable; the GO2 bridge is best effort.
        # Subscribe best effort so we are compatible with either.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            LaserScan, scan_topic, self._make_handler(scan_topic), qos
        )
        self.create_subscription(
            Odometry, odom_topic, self._make_handler(odom_topic), qos
        )
        self._topics = (scan_topic, odom_topic)

    def _make_handler(self, topic: str):
        def handler(message) -> None:
            arrival = self.get_clock().now().nanoseconds * 1e-9
            delta = arrival - _stamp_to_seconds(message.header.stamp)
            self._deltas.setdefault(topic, []).append(delta)

        return handler

    def report(self) -> None:
        print()
        print(f"{'topic':<28}{'msgs':>6}{'offset':>12}{'jitter':>10}")
        print("-" * 56)
        offsets = {}
        for topic in self._topics:
            samples = self._deltas.get(topic, [])
            if not samples:
                print(f"{topic:<28}{0:>6}{'no messages':>12}")
                continue
            offset = min(samples)
            jitter = max(samples) - offset
            offsets[topic] = offset
            print(
                f"{topic:<28}{len(samples):>6}"
                f"{offset * 1e3:>10.1f}ms{jitter * 1e3:>8.1f}ms"
            )

        if len(offsets) == 2:
            scan_topic, odom_topic = self._topics
            skew = abs(offsets[scan_topic] - offsets[odom_topic])
            print("-" * 56)
            print(f"{'scan vs odom offset gap':<34}{skew * 1e3:>10.1f}ms")
            print()
            if skew < 0.02:
                print("Clocks agree. restamp:=false is safe to try.")
            else:
                print(
                    "Every scan is paired with an odometry pose "
                    f"{skew * 1e3:.0f} ms away from it."
                )
                print(
                    "At 0.6 rad/s that is "
                    f"{skew * 0.6 * 57.3:.1f} degrees of yaw error per scan."
                )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scan-topic", default="/scan_raw")
    parser.add_argument("--odom-topic", default="/utlidar/robot_odom")
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = TimeSyncProbe(args.scan_topic, args.odom_topic)
    print(
        f"Listening to {args.scan_topic} and {args.odom_topic} "
        f"for {args.seconds:.0f} s..."
    )
    deadline = node.get_clock().now().nanoseconds + int(args.seconds * 1e9)
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
