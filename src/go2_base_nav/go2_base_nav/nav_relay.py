"""CycloneDDS-side Nav2 action relay.

task_planner（FastDDS）直连 Nav2 action 会触发跨 RMW 类型大小不匹配
（RTPS_READER_HISTORY: payload 32 > 19 bytes，2026-08-19 实测），所以 action
只在 Cyclone 侧本地调用，对外暴露两个纯话题接口（跨 RMW 话题已实证可靠）：

  订阅 /task_nav/goal   (std_msgs/String):
      "request_id|x,y,yaw_deg"（map 系）或 "request_id|cancel"
      "request_id|PLAN,x,y,yaw_deg,planner_id" 仅计算路径，不执行导航
      兼容旧格式 "x,y,yaw_deg" / "cancel"
  发布 /task_nav/status (std_msgs/String):
      新格式回显 "request_id|NAV <剩余米数>" / "request_id|SUCCEEDED" 等；
      规划回显 "request_id|PLAN_PENDING" / "request_id|PATH,<JSON>" /
               "request_id|PLAN_FAILED,<error_code>"；
      旧格式目标仍发布无前缀状态

代际（epoch）设计（2026-08-24 test7 后修复）：
巡逻 NAV_APPROACH 会按最新 TF 重发目标。每个目标分配递增 epoch；
只有当前 epoch 的 handle/future 才能发布状态——旧目标的迟到结果
（CANCELED/FAILED）不会污染新目标的状态流（此前旧 CANCELED 会让
task_planner 误判新目标失败）。
"""

import json
import math

from nav2_msgs.action import ComputePathToPose, NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def _point_segment_distance(point, start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 0.0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    fraction = ((point[0] - start[0]) * dx
                + (point[1] - start[1]) * dy) / length_sq
    fraction = max(0.0, min(1.0, fraction))
    projected_x = start[0] + fraction * dx
    projected_y = start[1] + fraction * dy
    return math.hypot(point[0] - projected_x, point[1] - projected_y)


def _simplify_polyline(points, tolerance):
    """Ramer-Douglas-Peucker simplification with a geometric error bound."""
    if len(points) <= 2:
        return points
    farthest_index = 0
    farthest_distance = -1.0
    for index, point in enumerate(points[1:-1], start=1):
        distance = _point_segment_distance(point, points[0], points[-1])
        if distance > farthest_distance:
            farthest_distance = distance
            farthest_index = index
    if farthest_distance <= tolerance:
        return [points[0], points[-1]]
    first = _simplify_polyline(points[:farthest_index + 1], tolerance)
    second = _simplify_polyline(points[farthest_index:], tolerance)
    return first[:-1] + second


def _compact_path_points(path, max_deviation=0.04, max_points=80):
    """Return a bounded path without cutting farther than 4 cm from Nav2.

    Index/spacing downsampling can replace a door-frame corner with a diagonal
    chord.  Geometry-bounded simplification keeps corners; if an unusually
    complex path cannot fit in the cross-RMW payload safely, reject it instead
    of silently returning an unsafe shortcut.
    """
    if max_points < 2:
        raise ValueError('max_points must be at least 2')
    if (not math.isfinite(max_deviation) or max_deviation < 0.0):
        raise ValueError('max_deviation must be finite and non-negative')

    raw_points = []
    for pose_stamped in path.poses:
        x = float(pose_stamped.pose.position.x)
        y = float(pose_stamped.pose.position.y)
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('non-finite path coordinate')
        point = [round(x, 4), round(y, 4)]
        if not raw_points or point != raw_points[-1]:
            raw_points.append(point)

    if len(raw_points) <= 1:
        return raw_points

    compact = _simplify_polyline(raw_points, max_deviation)
    if len(compact) > max_points:
        raise ValueError(
            f'path too complex to encode safely ({len(compact)} points > '
            f'{max_points})')
    return compact


class NavRelay(Node):

    def __init__(self):
        super().__init__('nav_relay')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.plan_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose')
        self.create_subscription(String, '/task_nav/goal', self._on_goal, 10)
        self.status_pub = self.create_publisher(String, '/task_nav/status', 10)
        # 当前代际的目标状态；epoch 之外的全部是过期目标，静默丢弃
        self._epoch = 0
        self._send_future = None
        self._goal_handle = None
        self._result_future = None
        self._request_id = None
        self._last_remaining = -1.0
        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            'nav_relay 就绪: /task_nav/goal("x,y,yaw_deg") -> navigate_to_pose')

    def _publish_status(self, text, request_id=None):
        msg = String()
        msg.data = (
            f'{request_id}|{text}' if request_id is not None else text)
        self.status_pub.publish(msg)

    def _on_goal(self, msg):
        raw = msg.data.strip()
        request_id = None
        text = raw
        if '|' in raw:
            request_text, text = raw.split('|', 1)
            try:
                request_id = int(request_text)
            except ValueError:
                self.get_logger().warn(
                    f'非法 request_id: {request_text!r}')
                return
        if text == 'cancel':
            # 带 ID 的取消只能取消同一请求；旧版裸 cancel 保持无条件取消，
            # 以兼容现有客户端。
            if (request_id is not None
                    and request_id != self._request_id):
                self.get_logger().warn(
                    f'忽略过期取消 request_id={request_id}；'
                    f'当前 request_id={self._request_id}')
                return
            active_request_id = self._request_id
            self._epoch += 1  # 过期当前代：旧 future 落地时不再发状态
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
                self._publish_status('CANCELED', active_request_id)
            self._goal_handle = None
            self._result_future = None
            self._send_future = None
            self._request_id = None
            self._last_remaining = -1.0
            return
        if text.startswith('PLAN,'):
            fields = text.split(',')
            if len(fields) != 5:
                self.get_logger().warn(
                    f'非法规划格式: {text!r}（需要 '
                    '"PLAN,x,y,yaw_deg,planner_id"）')
                return
            _, x_text, y_text, yaw_text, planner_id = fields
            try:
                x = float(x_text)
                y = float(y_text)
                yaw_deg = float(yaw_text)
            except ValueError:
                self.get_logger().warn(f'非法规划目标数值: {text!r}')
                return
            if (not all(math.isfinite(value) for value in (x, y, yaw_deg))
                    or not planner_id):
                self.get_logger().warn(f'非法规划目标: {text!r}')
                return

            goal = ComputePathToPose.Goal()
            goal.goal.header.frame_id = 'map'
            goal.goal.header.stamp = self.get_clock().now().to_msg()
            goal.goal.pose.position.x = x
            goal.goal.pose.position.y = y
            yaw = math.radians(yaw_deg)
            goal.goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.goal.pose.orientation.w = math.cos(yaw / 2.0)
            goal.planner_id = planner_id
            goal.use_start = False

            old_handle = self._goal_handle
            self._epoch += 1
            epoch = self._epoch
            self._request_id = request_id
            self._goal_handle = None
            self._result_future = None
            self._last_remaining = -1.0
            if old_handle is not None:
                old_handle.cancel_goal_async()
            future = self.plan_client.send_goal_async(goal)
            future.add_done_callback(
                lambda fut, ep=epoch, rid=request_id:
                    self._on_plan_send_done(fut, ep, rid))
            self._send_future = future
            self.get_logger().info(
                f'规划请求 #{epoch}: ({x:.2f}, {y:.2f}, '
                f'{yaw_deg:.0f}°), planner={planner_id}')
            self._publish_status('PLAN_PENDING', request_id)
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
        # 先换代再取消旧目标，确保任何同步/迟到回调都已过期。
        old_handle = self._goal_handle
        self._epoch += 1
        epoch = self._epoch
        self._request_id = request_id
        self._goal_handle = None
        self._result_future = None
        self._last_remaining = -1.0
        if old_handle is not None:
            old_handle.cancel_goal_async()
        future = self.client.send_goal_async(
            goal,
            feedback_callback=(
                lambda fb, ep=epoch: self._on_feedback(fb, ep)))
        future.add_done_callback(
            lambda fut, ep=epoch, rid=request_id:
                self._on_send_done(fut, ep, rid))
        self._send_future = future
        self.get_logger().info(f'新目标 #{epoch}: ({x:.2f}, {y:.2f}, {yaw_deg:.0f}°)')
        self._publish_status('ACCEPTED_PENDING', request_id)

    def _on_send_done(self, future, epoch, request_id):
        """Handle goal acceptance while isolating callbacks by epoch."""
        if epoch != self._epoch:
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        handle = future.result()
        if handle is None or not handle.accepted:
            self._publish_status('REJECTED', request_id)
            self._send_future = None
            self._request_id = None
            return
        self._goal_handle = handle
        self._send_future = None
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, ep=epoch, rid=request_id:
                self._on_result(fut, ep, rid))
        self._result_future = result_future

    def _on_result(self, future, epoch, request_id):
        """Discard navigation results from superseded epochs."""
        if epoch != self._epoch:
            return
        status = future.result().status
        self._publish_status(
            'SUCCEEDED' if status == 4 else 'FAILED', request_id)
        self._result_future = None
        self._goal_handle = None
        self._request_id = None

    def _on_plan_send_done(self, future, epoch, request_id):
        """Handle planner acceptance while isolating callbacks by epoch."""
        if epoch != self._epoch:
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        handle = future.result()
        if handle is None or not handle.accepted:
            self._publish_status('PLAN_FAILED,200', request_id)
            self._send_future = None
            self._request_id = None
            return
        self._goal_handle = handle
        self._send_future = None
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, ep=epoch, rid=request_id:
                self._on_plan_result(fut, ep, rid))
        self._result_future = result_future

    def _on_plan_result(self, future, epoch, request_id):
        """Publish a bounded path payload or the Nav2 planner error code."""
        if epoch != self._epoch:
            return

        wrapped_result = future.result()
        result = wrapped_result.result
        error_code = int(result.error_code)
        if wrapped_result.status != 4 or error_code != 0:
            self._publish_status(
                f'PLAN_FAILED,{error_code or 200}', request_id)
        else:
            try:
                points = _compact_path_points(result.path)
            except (TypeError, ValueError):
                self._publish_status('PLAN_FAILED,200', request_id)
            else:
                if not points:
                    self._publish_status('PLAN_FAILED,208', request_id)
                else:
                    payload = json.dumps(
                        points, separators=(',', ':'), allow_nan=False)
                    self._publish_status(f'PATH,{payload}', request_id)

        self._result_future = None
        self._goal_handle = None
        self._request_id = None

    def _on_feedback(self, fb, epoch):
        if epoch != self._epoch:
            return
        self._last_remaining = fb.feedback.distance_remaining

    def _tick(self):
        if self._goal_handle is not None and self._last_remaining >= 0.0:
            self._publish_status(
                f'NAV {self._last_remaining:.2f}', self._request_id)


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
