"""Nav2 action relay（CycloneDDS 侧）。

task_planner（FastDDS）直连 Nav2 action 会触发跨 RMW 类型大小不匹配
（RTPS_READER_HISTORY: payload 32 > 19 bytes，2026-08-19 实测），所以 action
只在 Cyclone 侧本地调用，对外暴露两个纯话题接口（跨 RMW 话题已实证可靠）：

  订阅 /task_nav/goal   (std_msgs/String): "x,y,yaw_deg"（map 系）或 "cancel"
  发布 /task_nav/status (std_msgs/String): "NAV <剩余米数>" / "SUCCEEDED" /
                                           "FAILED" / "REJECTED" / "CANCELED"
"""

import math

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class NavRelay(Node):

    def __init__(self):
        super().__init__('nav_relay')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(String, '/task_nav/goal', self._on_goal, 10)
        self.status_pub = self.create_publisher(String, '/task_nav/status', 10)
        self._send_future = None
        # 已有目标在跑：先取消再发新的（巡逻 NAV_APPROACH 会按最新 TF
        # 周期重发目标点，2026-08-24）
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._result_future = None
        self._last_remaining = -1.0
        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            'nav_relay 就绪: /task_nav/goal("x,y,yaw_deg") -> navigate_to_pose')

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _on_goal(self, msg):
        text = msg.data.strip()
        if text == 'cancel':
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
                self._publish_status('CANCELED')
            self._goal_handle = None
            self._result_future = None
            return
        try:
            x, y, yaw_deg = (float(v) for v in text.split(','))
        except ValueError:
            self.get_logger().warn(f'非法目标格式: {text!r}（需要 "x,y,yaw_deg"）')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        yaw = math.radians(yaw_deg)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_handle = None
        self._result_future = None
        self._last_remaining = -1.0
        self._send_future = self.client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        self.get_logger().info(f'新目标: ({x:.2f}, {y:.2f}, {yaw_deg:.0f}°)')
        self._publish_status('ACCEPTED_PENDING')

    def _on_feedback(self, fb):
        self._last_remaining = fb.feedback.distance_remaining

    def _tick(self):
        if self._goal_handle is None:
            if self._send_future is not None and self._send_future.done():
                handle = self._send_future.result()
                self._send_future = None
                if handle is None or not handle.accepted:
                    self._publish_status('REJECTED')
                    return
                self._goal_handle = handle
                self._result_future = handle.get_result_async()
            return
        if self._result_future is not None and self._result_future.done():
            status = self._result_future.result().status
            self._publish_status('SUCCEEDED' if status == 4 else 'FAILED')
            self._result_future = None
            self._goal_handle = None
        elif self._goal_handle is not None and self._last_remaining >= 0.0:
            self._publish_status(f'NAV {self._last_remaining:.2f}')


def main():
    rclpy.init()
    node = NavRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
