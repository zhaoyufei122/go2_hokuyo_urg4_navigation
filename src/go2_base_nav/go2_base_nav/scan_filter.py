"""Filter fake near-zero range readings from a 2D laser scan.

The Hokuyo URG-04LX-UG01 leaks driver error codes as 0.007--0.019 m fake
ranges. Nav2's costmap would inflate them into obstacles sitting exactly on
the robot, which makes every goal abort immediately. This node replaces
out-of-range readings with +inf so SLAM and Nav2 ignore them (and can still
ray-clear through them).
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def filter_ranges(ranges, min_range, max_range):
    """Return a new list where out-of-range readings become +inf."""
    filtered = []
    for value in ranges:
        if math.isnan(value) or math.isinf(value):
            filtered.append(value)
        elif value <= min_range or value >= max_range:
            filtered.append(math.inf)
        else:
            filtered.append(value)
    return filtered


class ScanFilter(Node):
    def __init__(self):
        super().__init__("scan_filter")
        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("min_range", 0.06)
        self.declare_parameter("max_range", 4.0)
        # The driver may run on the Jetson, whose clock is never perfectly
        # synced with this computer. Restamping on arrival removes any
        # residual clock skew from TF lookups (collision_monitor, Nav2).
        self.declare_parameter("restamp", True)
        self._restamp = bool(self.get_parameter("restamp").value)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._min_range = self.get_parameter("min_range").value
        self._max_range = self.get_parameter("max_range").value

        # The foxy urg_node publishes with the default reliable QoS; use the
        # same on both sides. Reliable publishers are still compatible with
        # best-effort subscribers (RViz, slam_toolbox).
        self._publisher = self.create_publisher(LaserScan, output_topic, 10)
        self._subscription = self.create_subscription(
            LaserScan, input_topic, self._on_scan, 10
        )
        self.get_logger().info(
            f"Filtering {input_topic} -> {output_topic}, "
            f"keeping ({self._min_range}, {self._max_range}) m"
        )

    def _on_scan(self, msg):
        output = LaserScan()
        output.header = msg.header
        if self._restamp:
            output.header.stamp = self.get_clock().now().to_msg()
        output.angle_min = msg.angle_min
        output.angle_max = msg.angle_max
        output.angle_increment = msg.angle_increment
        output.time_increment = msg.time_increment
        output.scan_time = msg.scan_time
        output.range_min = self._min_range
        output.range_max = self._max_range
        output.ranges = filter_ranges(
            msg.ranges, self._min_range, self._max_range
        )
        output.intensities = list(msg.intensities)
        self._publisher.publish(output)


def main():
    rclpy.init()
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
