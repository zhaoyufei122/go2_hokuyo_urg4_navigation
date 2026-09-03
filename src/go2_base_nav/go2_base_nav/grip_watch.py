"""grip_watch: 用 2D 雷达判断 walker 是否还被夹着（电机无力反馈的替代方案）。

原理（用户 2026-08-21 提出，2026-08-24 A/B 标定实测定论）：
  * 夹住时 walker 在正前方 ~0.38m 处有稳定近读数（随狗一起动）；
  * 脱手后 walker 被留在原地，狗继续倒开 → 读数变远（0.5m 后 ~0.62m）。
  * 实测两簇分布：夹住 0.384±0.003m / 脱手 0.616±0.002m，间距 232mm。
    阈值 0.52m 为看过 development data 后冻结的 prospective configuration
    （2026-09-03 冻结）；L1/L2 全部 4237 帧离线回放证明 0.483/0.49/0.52
    判定结果完全一致（准确率均 100%），冻结值不改变任何已有实验结论。

机制：
  * /walker_gripped = true（拖行中）才监测；
  * hold_polarity 参数决定判定方向：
      'present'（默认，标定结论）：扇区持续【没有】近读数（最近 >
                hold_max_m 或全空）超过 lost_grace_s → LOST；
      'absent'（备用假设）：扇区持续【出现】读数（< appear_max_m）
                超过 lost_grace_s → LOST；
  * 发布 /walker_lost (Bool, latched 风格周期重发)，task_planner 的倒车
    任务收到 LOST 立即停车报错。
"""

import math

import rclpy
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


def sector_min_range(ranges, angle_min, angle_increment, half_rad,
                     min_range=0.06, percentile=0.2):
    """扇区内所有束的【分位数】距离（默认 20%）；无有效读数返回 inf。

    2026-09-02 Codex 修正：必须在【全部束】上排序（无效束/噪声束按 inf
    计入），否则臂是仅有回波时仍被当成 walker。只在有效回波里取分位
    是错的（实测 L2 T01 RELEASED 仅 2/101 帧正确）。

    min_range 与 scan_filter 一致（0.06m）：/scan_raw 里有 ~0.015m 的
    机身自身噪声读数，不滤掉会误判（2026-07-18 实测）。"""
    vals = []
    for i, value in enumerate(ranges):
        angle = angle_min + i * angle_increment
        wrapped = math.atan2(math.sin(angle), math.cos(angle))
        if abs(wrapped) > half_rad:
            continue
        # 所有束参与排序；无效/噪声按 inf 处理（排到尾部）
        if math.isfinite(value) and value > min_range:
            vals.append(value)
        else:
            vals.append(math.inf)
    if not vals:
        return math.inf
    vals.sort()
    idx = max(0, math.ceil(len(vals) * percentile) - 1)
    return vals[idx]


def sector_band_fraction(ranges, angle_min, angle_increment, half_rad,
                         lower_m, upper_m):
    """Fraction of all sector beams inside the expected walker range band."""
    total = 0
    inside = 0
    for i, value in enumerate(ranges):
        angle = angle_min + i * angle_increment
        wrapped = math.atan2(math.sin(angle), math.cos(angle))
        if abs(wrapped) > half_rad:
            continue
        total += 1
        if math.isfinite(value) and lower_m <= value < upper_m:
            inside += 1
    return inside / total if total else 0.0


class GripWatch(Node):

    def __init__(self):
        super().__init__('grip_watch')
        self.declare_parameter('scan_topic', '/scan_raw')  # 必须看未遮蔽的原始数据
        # （/scan 在 gripped=true 时被 scan_filter 把正前方 ±45° 置 inf，
        # grip_watch 盯的 ±20° 全在里面，用 /scan 会立即误报脱手）
        self.declare_parameter('watch_half_angle_deg', 10.0)  # 2026-09-02 L1 扫描实测：10° 分离度最优
        self.declare_parameter('hold_max_m', 0.52)   # 2026-09-03 冻结的 prospective 配置
        # （2026-08-24 A/B 标定：夹住 0.384±0.003m / 脱手 0.616±0.002m，取中点）
        self.declare_parameter('lost_grace_s', 0.5)
        # 启动宽限：gripped 变 true 后等臂到位稳定再开始监控
        # （2026-08-25 实测：CATCH 完成仅 0.65s 就误报脱手，臂还没到位）
        self.declare_parameter('startup_grace_s', 2.0)
        self.declare_parameter('scan_timeout_s', 1.0)
        self.declare_parameter('min_range', 0.06)   # 与 scan_filter 一致，滤机身噪声
        # absent 极性（夹住=扇区无读数）：出现 < appear_max_m 的读数 → 疑似脱手
        # 2026-08-24 标定结论：实测夹住时有 0.38m 近读数 → 用 'present'
        self.declare_parameter('hold_polarity', 'present')  # 'absent' | 'present'
        self.declare_parameter('appear_max_m', 2.0)  # absent 极性：多远以内出现才算 walker
        # walker 特征距离带（标定 0.384m）：只认这个范围内的读数才算
        # "还夹着"。臂在前伸位（~0.2m）或墙（>0.6m）不会误判。
        # 2026-08-25 用户要求更严格
        self.declare_parameter('hold_min_m', 0.25)  # 带的下限
        self.declare_parameter(
            'hold_min_fraction', 0.2)  # 至少 20% 束命中 walker 距离带
        # hold_max_m 就是带的上限（0.52）

        self._half = math.radians(
            float(self.get_parameter('watch_half_angle_deg').value))
        self._hold_max = float(self.get_parameter('hold_max_m').value)
        self._grace = float(self.get_parameter('lost_grace_s').value)
        self._startup_grace = float(
            self.get_parameter('startup_grace_s').value)
        self._scan_timeout = float(
            self.get_parameter('scan_timeout_s').value)
        if not math.isfinite(self._scan_timeout) or self._scan_timeout <= 0.0:
            raise ValueError(
                'scan_timeout_s must be finite and greater than zero')
        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._min_range = float(self.get_parameter('min_range').value)
        self._polarity = str(self.get_parameter('hold_polarity').value)
        self._appear_max = float(self.get_parameter('appear_max_m').value)
        self._hold_min = float(self.get_parameter('hold_min_m').value)
        self._hold_min_fraction = float(
            self.get_parameter('hold_min_fraction').value)

        self._gripped = False
        self._gripped_since = None  # 启动宽限用
        self._lost_since = None
        self._lost = False
        self._ready = False
        self._last_scan_at = None

        gripped_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value, self._on_scan, 10)
        self.create_subscription(
            Bool, '/walker_gripped', self._on_gripped, gripped_qos)
        self.lost_pub = self.create_publisher(Bool, '/walker_lost', 10)
        self.ready_pub = self.create_publisher(
            Bool, '/grip_watch/ready', gripped_qos)
        # 前向扇区最近距离始终发布：脱手后自动恢复（NavigateToPoint RECOVER
        # 阶段）需要它闭环开回 walker 旁
        self.range_pub = self.create_publisher(Float32, '/walker_front_range', 10)
        self._publish_ready(False)
        self.create_timer(0.1, self._tick, clock=self._watchdog_clock)
        self.get_logger().info(
            f'grip_watch 就绪: 正前方 ±{math.degrees(self._half):.0f}° 扇区, '
            f'极性 {self._polarity}（absent=出现<{self._appear_max}m 判脱手 / '
            f'present=最近>{self._hold_max}m 判脱手）, 宽限 {self._grace} s')

    def _on_gripped(self, msg):
        was_gripped = self._gripped
        self._gripped = bool(msg.data)
        if not self._gripped:
            self._lost = False
            self._ready = False
            self._lost_since = None
            self._gripped_since = None
            self._last_scan_at = None
            self._publish(False)
            self._publish_ready(False)
        elif not was_gripped:
            self._ready = False
            self._gripped_since = self._watchdog_clock.now()
            self._last_scan_at = None
            self._publish_ready(False)

    def _on_scan(self, msg):
        now = self._watchdog_clock.now()
        self._last_scan_at = now
        nearest = sector_min_range(msg.ranges, msg.angle_min,
                                   msg.angle_increment, self._half,
                                   self._min_range)
        hold_fraction = sector_band_fraction(
            msg.ranges, msg.angle_min, msg.angle_increment, self._half,
            self._hold_min, self._hold_max)
        range_msg = Float32()
        range_msg.data = float(nearest)
        self.range_pub.publish(range_msg)
        if not self._gripped or self._lost:
            return
        # 启动宽限：gripped 变 true 后等臂到位稳定再监控
        if self._gripped_since is not None:
            elapsed = (now - self._gripped_since).nanoseconds / 1e9
            if elapsed < self._startup_grace:
                self._ready = False
                self._publish_ready(False)
                return
        if self._polarity == 'absent':
            # 夹住 = 扇区无读数；出现近读数 = walker 被留下 → 疑似脱手
            suspect = nearest < self._appear_max
        else:
            # 至少 20% 的扇区束必须落在 walker 标定距离带。这样低于
            # hold_min_m 的机械臂近场回波不能冒充 walker；同时保留
            # walker 宽目标，不依赖单个最小值。
            suspect = hold_fraction < self._hold_min_fraction
        if not suspect:
            self._lost_since = None
            self._ready = True
            self._publish_ready(True)
            return
        # A sub-grace suspect is classification evidence, not a sensor-health
        # failure. Keep an already-confirmed grip ready until loss is accepted.
        # Before the first holding scan, _ready is still false as required.
        self._publish_ready(self._ready)
        if self._lost_since is None:
            self._lost_since = now
            return
        if (now - self._lost_since).nanoseconds / 1e9 >= self._grace:
            self._lost = True
            self._ready = False
            self.get_logger().warn(
                f'grip_watch: 脱手！极性={self._polarity} '
                f'扇区q20={nearest:.2f}m, '
                f'walker距离带占比={hold_fraction:.0%} '
                f'持续>{self._grace}s')
            self._publish(True)
            self._publish_ready(False)

    def _publish(self, lost):
        msg = Bool()
        msg.data = lost
        self.lost_pub.publish(msg)

    def _publish_ready(self, ready):
        msg = Bool()
        msg.data = bool(ready)
        self.ready_pub.publish(msg)

    def _tick(self):
        if not self._gripped:
            return

        now = self._watchdog_clock.now()
        startup_elapsed = (
            self._gripped_since is None
            or (now - self._gripped_since).nanoseconds / 1e9
            >= self._startup_grace
        )
        if not self._lost and startup_elapsed:
            scan_age = (
                None if self._last_scan_at is None
                else (now - self._last_scan_at).nanoseconds / 1e9
            )
            if scan_age is None or scan_age >= self._scan_timeout:
                self._lost = True
                self._ready = False
                self._lost_since = None
                reason = (
                    '从未收到 scan'
                    if scan_age is None
                    else f'scan 已 {scan_age:.2f}s 无更新'
                )
                self.get_logger().warn(
                    f'grip_watch: {reason}，fail-closed 判定脱手')

        self._publish(self._lost)
        self._publish_ready(self._ready and not self._lost)


def main():
    rclpy.init()
    node = GripWatch()
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
