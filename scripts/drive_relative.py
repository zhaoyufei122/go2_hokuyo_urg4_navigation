#!/usr/bin/env python3
"""Drive the GO2 a relative distance straight forward/backward (no Nav2).

Intended as a mission-building block, e.g. backing out from the cabinet
after the arm grabs something:

    ./scripts/drive_relative.py -0.5          # back up 0.5 m
    ./scripts/drive_relative.py 0.3 0.2       # forward 0.3 m at 0.2 m/s

Publishes /cmd_vel through the existing go2_cmd_vel_bridge (same safety
limits and StopMove watchdog as navigation). Uses odometry feedback to stop
at the requested distance. Do NOT run while a Nav2 goal is active.

Safety: the lidar blind zone is at the REAR. Only reverse over short
distances you have just seen to be clear.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


MAX_SPEED = 0.4  # bridge safety limit
MIN_SPEED = 0.3  # below this the GO2 gait does not really move


class RelativeDrive(Node):
    def __init__(self):
        super().__init__("drive_relative")
        self.odom = None
        self.create_subscription(
            Odometry, "/odom", lambda m: setattr(self, "odom", m), 10
        )
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def xy(self):
        p = self.odom.pose.pose.position
        return p.x, p.y


def main() -> int:
    distance = float(sys.argv[1])  # metres; negative = backward
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
    if abs(speed) < MIN_SPEED or abs(speed) > MAX_SPEED:
        print(f"speed must be within [{MIN_SPEED}, {MAX_SPEED}] m/s")
        return 1
    if distance == 0.0:
        print("nothing to do")
        return 0

    rclpy.init()
    node = RelativeDrive()
    t0 = time.time()
    while node.odom is None and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.5)
    if node.odom is None:
        print("No /odom. Is navigation running?", file=sys.stderr)
        return 1

    x0, y0 = node.xy()
    msg = Twist()
    msg.linear.x = math.copysign(abs(speed), distance)
    print(f"driving {distance:+.2f} m at {msg.linear.x:+.2f} m/s ...", flush=True)

    cmd_end = time.time() + abs(distance) / abs(speed) + 2.0  # slack
    while time.time() < cmd_end:
        rclpy.spin_once(node, timeout_sec=0.02)
        x, y = node.xy()
        if math.hypot(x - x0, y - y0) >= abs(distance):
            break
        node.pub.publish(msg)
        time.sleep(0.05)

    for _ in range(5):  # explicit stop; bridge watchdog backs this up
        node.pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.05)

    x, y = node.xy()
    done = math.hypot(x - x0, y - y0)
    print(f"done, moved {done:.2f} m", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main())
