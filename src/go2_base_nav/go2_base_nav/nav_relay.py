"""Nav2 action relay（CycloneDDS 侧）。

task_planner（FastDDS）直连 Nav2 action 会触发跨 RMW 类型大小不匹配
（RTPS_READER_HISTORY: payload 32 > 19 bytes，2026-08-19 实测），所以 action
只在 Cyclone 侧本地调用，对外暴露两个纯话题接口（跨 RMW 话题已实证可靠）：

  订阅 /task_nav/goal   (std_msgs/String): "x,y,yaw_deg"（map 系）或 "cancel"
  发布 /task_nav/status (std_msgs/String): "NAV <剩余米数>" / "SUCCEEDED" /
                                           "FAILED" / "REJECTED" / "CANCELED"

代际（epoch）设计（2026-08-24 test7 后修复）：
巡逻 NAV_APPROACH 会按最新 TF 重发目标。每个目标分配递增 epoch；
只有当前 epoch 的 handle/future 才能发布状态——旧目标的迟到结果
（CANCELED/FAILED）不会污染新目标的状态流（此前旧 CANCELED 会让
task_planner 误判新目标失败）。
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
        # 当前代际的目标状态；epoch 之外的全部是过期目标，静默丢弃
        self._epoch = 0
        self._send_future = None
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
            self._epoch += 1  # 过期当前代：旧 future 落地时不再发状态
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
                self._publish_status('CANCELED')
            self._goal_handle = None
            self._result_future = None
            self._send_future = None
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
        # 已有目标在跑：取消并换代（2026-08-24：巡逻 NAV_APPROACH 按最新
        # TF 重发目标点）
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._epoch += 1
        epoch = self._epoch
        self._goal_handle = None
        self._result_future = None
        self._last_remaining = -1.0
        future = self.client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        future.add_done_callback(
            lambda fut, ep=epoch: self._on_send_done(fut, ep))
        self._send_future = future
        self.get_logger().info(f'新目标 #{epoch}: ({x:.2f}, {y:.2f}, {yaw_deg:.0f}°)')
        self._publish_status('ACCEPTED_PENDING')

    def _on_send_done(self, future, epoch):
        """goal accept/reject 回调。过期代的 goal 即使被接受也立刻取消。"""
        if epoch != self._epoch:
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        handle = future.result()
        if handle is None or not handle.accepted:
            self._publish_status('REJECTED')
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, ep=epoch: self._on_result(fut, ep))
        self._result_future = result_future

    def _on_result(self, future, epoch):
        """结果回调。过期代的结果直接丢弃（取消旧目标时的迟到 CANCELED）。"""
        if epoch != self._epoch:
            return
        status = future.result().status
        self._publish_status('SUCCEEDED' if status == 4 else 'FAILED')
        self._result_future = None
        self._goal_handle = None

    def _on_feedback(self, fb):
        self._last_remaining = fb.feedback.distance_remaining

    def _tick(self):
        if self._goal_handle is not None and self._last_remaining >= 0.0:
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
