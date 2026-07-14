from math import isfinite

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from go2_base_nav.command_safety import (
    MotionCommand,
    MotionLimits,
    WatchdogState,
    clamp_command,
    is_zero_command,
)
from go2_base_nav.sport_client import Go2SportClient


class Go2CmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_cmd_vel_bridge")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("sport_request_topic", "/api/sport/request")
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("max_linear_x", 0.4)
        self.declare_parameter("max_linear_y", 0.0)
        self.declare_parameter("max_angular_z", 0.4)

        self._timeout = float(self.get_parameter("cmd_timeout").value)
        if not isfinite(self._timeout) or self._timeout <= 0.0:
            raise ValueError("cmd_timeout must be finite and greater than zero")
        self._limits = MotionLimits(
            max_linear_x=float(self.get_parameter("max_linear_x").value),
            max_linear_y=float(self.get_parameter("max_linear_y").value),
            max_angular_z=float(self.get_parameter("max_angular_z").value),
        )
        self._watchdog = WatchdogState()
        self._sport_client = Go2SportClient(
            self,
            request_topic=self.get_parameter("sport_request_topic").value,
        )
        self._subscription = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self._handle_command,
            10,
        )
        self._watchdog_timer = self.create_timer(0.05, self._check_watchdog)

    def _now_nanoseconds(self) -> int:
        return self.get_clock().now().nanoseconds

    def _stop_for_command(self, command: MotionCommand, now: int) -> None:
        if self._watchdog.observe(command, now_nanoseconds=now):
            self._sport_client.stop_move()

    def _handle_command(self, message: Twist) -> None:
        now = self._now_nanoseconds()
        try:
            command = clamp_command(
                message.linear.x,
                message.linear.y,
                message.angular.z,
                self._limits,
            )
        except ValueError as error:
            self.get_logger().error(f"Rejecting invalid cmd_vel: {error}")
            self._stop_for_command(MotionCommand(0.0, 0.0, 0.0), now)
            return

        if is_zero_command(command):
            self._stop_for_command(command, now)
            return

        self._watchdog.observe(command, now_nanoseconds=now)
        self._sport_client.move(
            command.linear_x,
            command.linear_y,
            command.angular_z,
        )

    def _check_watchdog(self) -> None:
        if self._watchdog.expired(
            self._now_nanoseconds(),
            timeout_seconds=self._timeout,
        ):
            self.get_logger().warning("cmd_vel timeout; sending StopMove")
            self._sport_client.stop_move()

    def destroy_node(self) -> bool:
        try:
            self._sport_client.stop_move()
        except Exception as error:  # noqa: BLE001 - shutdown must continue safely
            self.get_logger().error(f"Failed to send final StopMove: {error}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
