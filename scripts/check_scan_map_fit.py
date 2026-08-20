#!/usr/bin/env python3
"""Report, live and in centimetres, how well /scan sits on the saved map.

"The white dots do not line up with my map" is the symptom; this turns it into
a number. Every second it prints how far the scan endpoints land from the
nearest occupied cell of the map, plus the pose AMCL is publishing and how
much it is correcting.

    fit      median / p90 distance from a scan endpoint to the nearest wall.
             Below ~8 cm is as good as a 5 cm map gets. Above ~25 cm the
             localisation is wrong, not just noisy.
    pose     where AMCL thinks base_footprint is, in the map frame.
    corr     how far map->odom moved since the last line. This is AMCL
             actively correcting. If it stays at 0.000 while you drive the
             dog around, AMCL is not updating at all and the scan is being
             dead-reckoned off the odometry.

Read it like this:

    fit good, corr changing         localisation is working.
    fit good standing still,
      fit bad while turning,
      fit good again when stopped   timestamp/latency problem, not geometry.
      (run scripts/check_time_sync.py)
    fit bad and corr always 0.000   AMCL never updates. Check that the
                                    odometry actually moves when you move the
                                    dog: ros2 topic echo /odom --field pose
    fit bad and corr changing       AMCL is locked onto the wrong place.
                                    Re-set the 2D Pose Estimate.

Run it alongside a live navigation stack:

    source /home/yufei/Desktop/unitree_ros2/setup.sh
    source install/setup.bash
    python3 scripts/check_scan_map_fit.py
"""

import argparse
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def load_occupied_cells(map_yaml: Path) -> tuple[np.ndarray, float]:
    """Return the map's occupied cell centres in metres, plus the resolution."""
    meta = yaml.safe_load(map_yaml.read_text())
    image_path = Path(meta["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    resolution = float(meta["resolution"])
    origin_x, origin_y = float(meta["origin"][0]), float(meta["origin"][1])
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    negate = int(meta.get("negate", 0))

    from PIL import Image

    pixels = np.array(Image.open(image_path).convert("L")).astype(float)
    # map_server convention: p = (255 - value) / 255, inverted when negate.
    occupancy = pixels / 255.0 if negate else (255.0 - pixels) / 255.0
    rows, cols = np.nonzero(occupancy >= occupied_thresh)
    height = pixels.shape[0]
    # Row 0 of the image is the TOP of the map, i.e. the highest y.
    xs = origin_x + (cols + 0.5) * resolution
    ys = origin_y + (height - rows - 0.5) * resolution
    return np.stack([xs, ys], axis=1), resolution


class ScanMapFit(Node):
    def __init__(self, map_yaml: Path, scan_topic: str, period: float) -> None:
        super().__init__("check_scan_map_fit")
        self._walls, resolution = load_occupied_cells(map_yaml)
        if len(self._walls) == 0:
            raise ValueError(f"{map_yaml} has no occupied cells")
        from scipy.spatial import cKDTree

        self._tree = cKDTree(self._walls)
        self.get_logger().info(
            f"{map_yaml.name}: {len(self._walls)} occupied cells "
            f"@ {resolution:.3f} m"
        )

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._latest: LaserScan | None = None
        self._previous_correction: np.ndarray | None = None
        self._stale_transform = 0

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos)
        self.create_timer(period, self._report)

    def _on_scan(self, message: LaserScan) -> None:
        self._latest = message

    def _lookup(self, target: str, source: str, stamp):
        """Transform at the message stamp, falling back to the latest one."""
        try:
            return self._buffer.lookup_transform(target, source, stamp), True
        except Exception:
            try:
                return self._buffer.lookup_transform(target, source, Time()), False
            except Exception:
                return None, False

    @staticmethod
    def _as_xy_yaw(transform) -> tuple[float, float, float]:
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return t.x, t.y, yaw

    def _report(self) -> None:
        scan = self._latest
        if scan is None:
            self.get_logger().warning("no scan received yet")
            return

        stamp = Time.from_msg(scan.header.stamp)
        transform, fresh = self._lookup("map", scan.header.frame_id, stamp)
        if transform is None:
            self.get_logger().warning(
                f"no map -> {scan.header.frame_id} transform "
                "(is navigation running and the initial pose set?)"
            )
            return
        if not fresh:
            self._stale_transform += 1

        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        usable = np.isfinite(ranges) & (ranges > scan.range_min)
        usable &= ranges < scan.range_max
        if usable.sum() < 20:
            self.get_logger().warning(f"only {usable.sum()} usable beams")
            return

        x, y, yaw = self._as_xy_yaw(transform)
        local = np.stack(
            [ranges[usable] * np.cos(angles[usable]),
             ranges[usable] * np.sin(angles[usable])],
            axis=1,
        )
        rotation = np.array(
            [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
        )
        points = local @ rotation.T + np.array([x, y])
        distances, _ = self._tree.query(points)

        correction = "    n/a"
        map_to_odom, _ = self._lookup("map", "odom", Time())
        if map_to_odom is not None:
            current = np.array(self._as_xy_yaw(map_to_odom))
            if self._previous_correction is not None:
                moved = np.hypot(*(current[:2] - self._previous_correction[:2]))
                turned = abs(
                    np.arctan2(
                        np.sin(current[2] - self._previous_correction[2]),
                        np.cos(current[2] - self._previous_correction[2]),
                    )
                )
                correction = f"{moved:.3f}m/{np.degrees(turned):4.1f}deg"
            self._previous_correction = current

        base, _ = self._lookup("map", "base_footprint", Time())
        pose = "unknown"
        if base is not None:
            bx, by, byaw = self._as_xy_yaw(base)
            pose = f"{bx:6.2f} {by:6.2f} {np.degrees(byaw):7.1f}deg"

        flag = "" if fresh else "  [TF stale: scan stamp outside cache]"
        print(
            f"fit p50={np.median(distances) * 100:5.1f}cm "
            f"p90={np.percentile(distances, 90) * 100:5.1f}cm "
            f"({usable.sum():3d} beams)  pose {pose}  corr {correction}{flag}",
            flush=True,
        )

    def summarise(self) -> None:
        if self._stale_transform:
            print(
                f"\n{self._stale_transform} report(s) had to fall back to the "
                "latest transform because the scan timestamp was outside the "
                "TF cache. That is a clock/latency problem -- run "
                "scripts/check_time_sync.py."
            )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--map",
        default=str(Path(__file__).resolve().parents[1] / "maps" / "hokuyo_room.yaml"),
    )
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--period", type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = ScanMapFit(Path(args.map), args.scan_topic, args.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summarise()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
