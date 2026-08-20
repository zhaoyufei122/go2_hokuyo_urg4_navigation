#!/usr/bin/env python3
"""Measure how many degrees the Hokuyo is actually rotated on its mount.

Why this matters
----------------
laser_yaw is currently 0.0, i.e. "the lidar's forward axis is the dog's
forward axis". It was never measured -- the README calls the mount pose an
approximation. If the real mount is rotated by phi, AMCL ends up in a fight it
cannot win:

  * the measurement model wants a base pose rotated by +phi, because only then
    does the (mis-rotated) scan land on the map;
  * the motion model insists the dog travels along base +x, which the
    compensated pose no longer points along.

AMCL settles on a compromise between the two and sits there. That is exactly
the symptom: a scan that would fit the map beautifully after a ~10 deg twist,
and an AMCL that refuses to make the twist no matter how often you re-drop the
2D Pose Estimate.

How to use it
-------------
Park the dog with one **side of its body parallel to a flat, clear wall** --
sight along the body panels, a few degrees of eyeball accuracy is plenty to
find a 10 degree error. Keep 0.5-3 m of clear wall in view and nobody standing
in front of the lidar. Then:

    source /home/yufei/Desktop/unitree_ros2/setup.sh
    source install/setup.bash
    python3 scripts/check_laser_yaw.py --parallel

Or point the dog square at a wall and use --perpendicular. The script fits the
dominant straight line in the scan and reports how far it is from where a
correctly mounted lidar would put it. That offset is laser_yaw.

Nothing else needs to change: the saved map stays valid, because the map is a
faithful copy of the room either way -- only the sensor-to-body transform is
wrong.
"""

import argparse
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def fit_dominant_line(
    points: np.ndarray,
    tolerance: float,
    iterations: int,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray] | None:
    """RANSAC the longest straight run of points. Returns (direction, inliers)."""
    if len(points) < 20:
        return None

    best_inliers = None
    for _ in range(iterations):
        i, j = rng.choice(len(points), size=2, replace=False)
        segment = points[j] - points[i]
        length = np.hypot(*segment)
        if length < 0.3:  # too short to define a wall
            continue
        normal = np.array([-segment[1], segment[0]]) / length
        distances = np.abs((points - points[i]) @ normal)
        inliers = distances < tolerance
        if best_inliers is None or inliers.sum() > best_inliers.sum():
            best_inliers = inliers

    if best_inliers is None or best_inliers.sum() < 20:
        return None

    # Total-least-squares refit on the inliers.
    selected = points[best_inliers]
    centred = selected - selected.mean(axis=0)
    _, _, vt = np.linalg.svd(centred, full_matrices=False)
    return vt[0], selected


def wrap_to_90(degrees: float) -> float:
    """Fold an undirected line angle into (-90, 90]."""
    folded = (degrees + 90.0) % 180.0 - 90.0
    return 90.0 if folded == -90.0 else folded


class LaserYawProbe(Node):
    def __init__(self, args) -> None:
        super().__init__("check_laser_yaw")
        self._args = args
        self._rng = np.random.default_rng(0)
        self._estimates: list[float] = []
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, args.scan_topic, self._on_scan, qos)
        stance = "parallel to" if args.parallel else "square in front of"
        print(
            f"Fitting the dominant wall in {args.scan_topic}.\n"
            f"The dog's body must be {stance} that wall.\n"
            f"Collecting {args.samples} scans...\n"
        )

    def _on_scan(self, scan: LaserScan) -> None:
        if len(self._estimates) >= self._args.samples:
            return

        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        usable = np.isfinite(ranges)
        usable &= ranges > max(scan.range_min, self._args.min_range)
        usable &= ranges < min(scan.range_max, self._args.max_range)
        if usable.sum() < 20:
            return

        points = np.stack(
            [ranges[usable] * np.cos(angles[usable]),
             ranges[usable] * np.sin(angles[usable])],
            axis=1,
        )
        result = fit_dominant_line(
            points, self._args.tolerance, self._args.iterations, self._rng
        )
        if result is None:
            return
        direction, inliers = result

        line_deg = wrap_to_90(math.degrees(math.atan2(direction[1], direction[0])))
        # A correctly mounted lidar sees a wall the body is parallel to as a
        # line along its own x axis (0 deg); square-on it sees 90 deg.
        expected = 0.0 if self._args.parallel else 90.0
        error = wrap_to_90(line_deg - expected)
        self._estimates.append(error)
        span = np.hypot(*(inliers.max(axis=0) - inliers.min(axis=0)))
        print(
            f"  scan {len(self._estimates):3d}/{self._args.samples}: "
            f"wall at {line_deg:+6.2f} deg, {len(inliers):3d} inliers "
            f"over {span:.2f} m  ->  offset {error:+6.2f} deg"
        )

    def done(self) -> bool:
        return len(self._estimates) >= self._args.samples

    def report(self) -> None:
        if len(self._estimates) < 3:
            print("\nNot enough clean line fits. Move closer to a bare wall.")
            return
        values = np.array(self._estimates)
        median = float(np.median(values))
        spread = float(np.percentile(values, 90) - np.percentile(values, 10))
        print("\n" + "=" * 60)
        print(f"mount offset : {median:+.2f} deg   (10-90% spread {spread:.2f} deg)")
        print(f"             = {math.radians(median):+.4f} rad")
        if spread > 3.0:
            print(
                "\nThe fits disagree by more than 3 deg -- the lidar is probably "
                "seeing furniture, not a bare wall. Re-park and run again."
            )
            return
        if abs(median) < 2.0:
            print("\nThe mount is square. laser_yaw: 0.0 is correct; look elsewhere.")
            return
        print(
            f"\nThe lidar is rotated {abs(median):.1f} deg on its mount. Try it "
            "without rebuilding or re-mapping anything:\n\n"
            f"    ./scripts/start_hokuyo_navigation.sh "
            f"laser_yaw:={math.radians(median):.4f}\n\n"
            "Then re-drop the 2D Pose Estimate and drive. If this was it, the "
            "scan will stay on the walls instead of sitting ~10 deg off, and "
            "AMCL will stop fighting its own motion model."
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--min-range", type=float, default=0.3)
    parser.add_argument("--max-range", type=float, default=3.5)
    parser.add_argument("--tolerance", type=float, default=0.02)
    parser.add_argument("--iterations", type=int, default=400)
    stance = parser.add_mutually_exclusive_group()
    stance.add_argument("--parallel", action="store_true", default=True)
    stance.add_argument(
        "--perpendicular", dest="parallel", action="store_false"
    )
    args = parser.parse_args()

    rclpy.init()
    node = LaserYawProbe(args)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
