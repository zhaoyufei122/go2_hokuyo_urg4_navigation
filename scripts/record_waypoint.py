#!/usr/bin/env python3
"""Print the GO2's current pose (x, y, yaw) in the map frame.

Drive the dog to the desired spot with the physical remote, then run:

    source /home/yufei/Desktop/unitree_ros2/setup.sh
    source /home/yufei/Desktop/go2_base_navi/install/setup.bash
    ./scripts/record_waypoint.py

It just prints the pose. To also append it to a waypoint file for later
replay with follow_waypoints.py, pass a file path:

    ./scripts/record_waypoint.py missions/door_mission.yaml
"""

import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


REPO_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    save_path = Path(sys.argv[1]) if len(sys.argv) > 1 else None

    rclpy.init()
    node = Node("record_waypoint")
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    print("Waiting for map -> base_footprint transform ...", flush=True)
    deadline = time.time() + 20
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if tf_buffer.can_transform(
            "map", "base_footprint", rclpy.time.Time()
        ):
            break
    else:
        print("No map transform. Is navigation running?", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    tf = tf_buffer.lookup_transform(
        "map", "base_footprint", rclpy.time.Time()
    )
    node.destroy_node()
    rclpy.shutdown()

    x = tf.transform.translation.x
    y = tf.transform.translation.y
    q = tf.transform.rotation
    yaw_deg = math.degrees(
        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    )

    waypoint = {
        "x": round(x, 3),
        "y": round(y, 3),
        "yaw_deg": round(yaw_deg, 1),
    }

    print(
        f"current pose: x={waypoint['x']} y={waypoint['y']} "
        f"yaw={waypoint['yaw_deg']} deg"
    )

    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        waypoints = []
        if save_path.exists():
            waypoints = yaml.safe_load(save_path.read_text()) or []
        waypoints.append(waypoint)
        save_path.write_text(yaml.safe_dump(waypoints, allow_unicode=True))
        print(f"Appended to {save_path} ({len(waypoints)} waypoints total)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
