#!/usr/bin/env python3
"""Drive the GO2 through a list of waypoints programmatically via Nav2.

Prerequisite: navigation must be running, e.g.

    ./scripts/start_hokuyo_navigation.sh localization:=slam_toolbox \
      slam_posegraph:=/home/yufei/Desktop/go2_base_navi/maps/hokuyo_room

Then in another terminal:

    source /home/yufei/Desktop/unitree_ros2/setup.sh
    source /home/yufei/Desktop/go2_base_navi/install/setup.bash
    ./scripts/follow_waypoints.py

Waypoints are (x, y, yaw_deg) in the *map* frame. To read coordinates from
RViz: select the "Publish Point" tool (add it via the "+" in the toolbar if
missing), click a spot on the map, and read /clicked_point with:

    ros2 topic echo /clicked_point --once

Keep the physical remote in hand: it always overrides the software.
"""

import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# Default waypoint file; override with: ./scripts/follow_waypoints.py my.yaml
WAYPOINT_FILE = (
    Path(__file__).resolve().parents[1] / "missions" / "waypoints.yaml"
)

# Fallback example if the file does not exist: (x [m], y [m], yaw [deg])
# in the map frame.
WAYPOINTS = [
    (1.0, 0.0, 0.0),
    (2.0, 1.0, 90.0),
    (0.0, 0.0, 180.0),
]


def load_waypoints(path):
    if not path.exists():
        return WAYPOINTS
    data = yaml.safe_load(path.read_text())
    return [(float(w["x"]), float(w["y"]), float(w["yaw_deg"])) for w in data]


def make_pose(navigator: BasicNavigator, x: float, y: float, yaw_deg: float):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main() -> int:
    waypoint_file = Path(sys.argv[1]) if len(sys.argv) > 1 else WAYPOINT_FILE
    waypoints = load_waypoints(waypoint_file)

    rclpy.init()
    navigator = BasicNavigator()

    # NOTE: waitUntilNav2Active() hardcodes waiting for amcl/get_state;
    # this pipeline localizes with slam_toolbox, so wait for the navigate
    # action server directly instead.
    print("Waiting for Nav2 action server ...", flush=True)
    while not navigator.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
        print("  ... still waiting for navigate_to_pose server", flush=True)
    print(f"Nav2 active. Waypoints from {waypoint_file}:", flush=True)

    pause_sec = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

    for i, (x, y, yaw) in enumerate(waypoints):
        print(
            f"  -> waypoint {i}: x={x:.2f} y={y:.2f} yaw={yaw:.1f} deg",
            flush=True,
        )
        navigator.goToPose(make_pose(navigator, x, y, yaw))
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(
                    f"     ... distance remaining "
                    f"{feedback.distance_remaining:.2f} m",
                    flush=True,
                )
            time.sleep(1.0)
        result = navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            print(
                f"Waypoint {i} failed/canceled: {result}", flush=True
            )
            navigator.lifecycleShutdown()
            rclpy.shutdown()
            return 1
        print(f"  waypoint {i} reached.", flush=True)
        if pause_sec > 0 and i < len(waypoints) - 1:
            print(f"  pausing {pause_sec:.0f} s ...", flush=True)
            time.sleep(pause_sec)

    navigator.lifecycleShutdown()
    rclpy.shutdown()
    print("All waypoints reached.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
