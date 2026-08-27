import math
from types import SimpleNamespace

from go2_base_nav import grip_watch, scan_filter
import pytest
from rclpy.clock_type import ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class _FakeTime:

    def __init__(self, seconds):
        self.nanoseconds = round(seconds * 1_000_000_000)

    def __sub__(self, other):
        return SimpleNamespace(
            nanoseconds=self.nanoseconds - other.nanoseconds)


class _FakeClock:

    def __init__(self, seconds=0.0):
        self._seconds = seconds

    def now(self):
        return _FakeTime(self._seconds)

    def advance(self, seconds):
        self._seconds += seconds


class _Publisher:

    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(message.data)


class _Logger:

    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)

    def info(self, _message):
        pass


def _bool(value):
    message = Bool()
    message.data = value
    return message


def _watch(watchdog_clock, *, ros_clock=None, polarity='present'):
    if ros_clock is None:
        ros_clock = watchdog_clock
    lost_publications = []
    ready_publications = []
    logger = _Logger()
    return SimpleNamespace(
        _gripped=False,
        _gripped_since=None,
        _lost_since=None,
        _lost=False,
        _ready=False,
        _last_scan_at=None,
        _startup_grace=2.0,
        _scan_timeout=1.0,
        _half=math.radians(20.0),
        _min_range=0.06,
        _polarity=polarity,
        _appear_max=2.0,
        _hold_min=0.25,
        _hold_max=0.5,
        _grace=0.5,
        _watchdog_clock=watchdog_clock,
        range_pub=_Publisher(),
        get_clock=lambda: ros_clock,
        get_parameter=lambda name: SimpleNamespace(
            value={'startup_grace_s': 2.0}[name]),
        get_logger=lambda: logger,
        _publish=lambda value: lost_publications.append(bool(value)),
        _publish_ready=lambda value: ready_publications.append(bool(value)),
        lost_publications=lost_publications,
        ready_publications=ready_publications,
        logger=logger,
    )


def _scan(nearest):
    return SimpleNamespace(
        ranges=[nearest], angle_min=0.0, angle_increment=1.0)


def _capture_constructor(
        monkeypatch, module, node_class, *, parameter_overrides=None):
    parameters = dict(parameter_overrides or {})
    subscriptions = []
    timers = []

    class _ConstructorLogger:

        def info(self, _message):
            pass

    monkeypatch.setattr(module.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(
        module.Node,
        'declare_parameter',
        lambda self, name, default: parameters.setdefault(name, default),
    )
    monkeypatch.setattr(
        module.Node,
        'get_parameter',
        lambda self, name: SimpleNamespace(value=parameters[name]),
    )
    monkeypatch.setattr(
        module.Node,
        'create_subscription',
        lambda self, message_type, topic, callback, qos: subscriptions.append(
            (message_type, topic, callback, qos)),
    )
    monkeypatch.setattr(
        module.Node,
        'create_publisher',
        lambda self, *args: _Publisher(),
    )
    monkeypatch.setattr(
        module.Node,
        'create_timer',
        lambda self, *args, **kwargs: timers.append((args, kwargs)),
    )
    monkeypatch.setattr(
        module.Node, 'get_logger', lambda self: _ConstructorLogger())

    node = node_class()
    return node, parameters, subscriptions, timers


@pytest.mark.parametrize(
    ('module', 'node_class'),
    ((grip_watch, grip_watch.GripWatch),
     (scan_filter, scan_filter.ScanFilter)),
)
def test_walker_gripped_subscriptions_are_latched_depth_one(
        monkeypatch, module, node_class):
    _, _, subscriptions, _ = _capture_constructor(
        monkeypatch, module, node_class)

    qos = next(qos for _, topic, _, qos in subscriptions
               if topic == '/walker_gripped')
    assert isinstance(qos, QoSProfile)
    assert qos.depth == 1
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_scan_timeout_parameter_defaults_to_one_second(monkeypatch):
    _, parameters, _, _ = _capture_constructor(
        monkeypatch, grip_watch, grip_watch.GripWatch)

    assert parameters.get('scan_timeout_s') == 1.0


@pytest.mark.parametrize(
    'timeout', (0.0, -0.1, math.inf, -math.inf, math.nan))
def test_invalid_scan_timeout_is_rejected(monkeypatch, timeout):
    with pytest.raises(ValueError, match='scan_timeout_s'):
        _capture_constructor(
            monkeypatch,
            grip_watch,
            grip_watch.GripWatch,
            parameter_overrides={'scan_timeout_s': timeout},
        )


def test_watchdog_timer_uses_the_steady_clock(monkeypatch):
    node, _, _, timers = _capture_constructor(
        monkeypatch, grip_watch, grip_watch.GripWatch)

    timer_args, timer_kwargs = timers[0]
    timer_clock = timer_kwargs.get('clock')
    assert timer_args[0] == 0.1
    assert timer_clock is not None
    assert timer_clock is node._watchdog_clock
    assert timer_clock.clock_type == ClockType.STEADY_TIME


def test_no_scan_fails_closed_after_startup_grace():
    clock = _FakeClock()
    watch = _watch(clock)
    grip_watch.GripWatch._on_gripped(watch, _bool(True))

    clock.advance(1.9)
    grip_watch.GripWatch._tick(watch)
    assert watch._lost is False
    assert watch.lost_publications[-1] is False

    clock.advance(0.2)
    grip_watch.GripWatch._tick(watch)
    assert watch._lost is True
    assert watch.lost_publications[-1] is True


def test_stale_scan_fails_closed_and_latches_lost():
    clock = _FakeClock()
    watch = _watch(clock)
    grip_watch.GripWatch._on_gripped(watch, _bool(True))
    clock.advance(2.1)
    grip_watch.GripWatch._on_scan(watch, _scan(0.4))

    clock.advance(1.01)
    grip_watch.GripWatch._tick(watch)
    assert watch._lost is True
    assert watch.lost_publications[-1] is True

    grip_watch.GripWatch._on_scan(watch, _scan(0.4))
    assert watch._lost is True


def test_scan_is_stale_at_exact_timeout_boundary():
    clock = _FakeClock()
    watch = _watch(clock)
    grip_watch.GripWatch._on_gripped(watch, _bool(True))
    clock.advance(2.0)
    grip_watch.GripWatch._on_scan(watch, _scan(0.4))

    clock.advance(1.0)
    grip_watch.GripWatch._tick(watch)

    assert watch._lost is True
    assert watch.lost_publications[-1] is True


def test_scan_filter_acknowledges_only_after_a_masked_scan_is_published():
    ready_pub = _Publisher()
    scan_outputs = []
    node = SimpleNamespace(
        _mask_enabled=False,
        _mask_ready=False,
        _mask_center=0.0,
        _mask_half=math.radians(45.0),
        _min_range=0.06,
        _max_range=4.0,
        _restamp=False,
        _ready_pub=ready_pub,
        _publisher=SimpleNamespace(publish=scan_outputs.append),
        get_logger=lambda: _Logger(),
    )
    node._publish_mask_ready = lambda ready: (
        scan_filter.ScanFilter._publish_mask_ready(node, ready))
    scan_filter.ScanFilter._on_gripped(node, _bool(True))
    assert ready_pub.values[-1] is False

    scan = LaserScan()
    scan.angle_min = -math.pi / 2
    scan.angle_max = math.pi / 2
    scan.angle_increment = math.pi / 2
    scan.range_min = 0.06
    scan.range_max = 4.0
    scan.ranges = [1.0, 0.38, 1.2]
    scan_filter.ScanFilter._on_scan(node, scan)

    assert math.isfinite(scan_outputs[-1].ranges[0])
    assert math.isinf(scan_outputs[-1].ranges[1])
    assert math.isfinite(scan_outputs[-1].ranges[2])
    assert ready_pub.values[-1] is True


def test_grip_watch_ready_requires_a_post_grace_holding_scan():
    clock = _FakeClock()
    watch = _watch(clock)
    grip_watch.GripWatch._on_gripped(watch, _bool(True))

    grip_watch.GripWatch._on_scan(watch, _scan(0.38))
    assert watch.ready_publications[-1] is False

    clock.advance(2.1)
    grip_watch.GripWatch._on_scan(watch, _scan(0.38))
    assert watch.ready_publications[-1] is True

    grip_watch.GripWatch._on_scan(watch, _scan(0.70))
    assert watch.ready_publications[-1] is False


def test_fresh_holding_scan_does_not_fail_closed():
    clock = _FakeClock()
    watch = _watch(clock)
    grip_watch.GripWatch._on_gripped(watch, _bool(True))
    clock.advance(2.1)
    grip_watch.GripWatch._on_scan(watch, _scan(0.4))

    clock.advance(0.99)
    grip_watch.GripWatch._tick(watch)
    assert watch._lost is False
    assert watch.lost_publications[-1] is False


def test_repeated_true_does_not_restart_startup_grace():
    clock = _FakeClock()
    watch = _watch(clock)
    grip_watch.GripWatch._on_gripped(watch, _bool(True))

    clock.advance(1.9)
    grip_watch.GripWatch._on_gripped(watch, _bool(True))
    clock.advance(0.2)
    grip_watch.GripWatch._tick(watch)

    assert watch._lost is True
    assert watch.lost_publications[-1] is True


def test_new_grip_epoch_ignores_old_and_ungripped_scans():
    clock = _FakeClock()
    watch = _watch(clock)
    watch._scan_timeout = 10.0

    grip_watch.GripWatch._on_gripped(watch, _bool(True))
    grip_watch.GripWatch._on_scan(watch, _scan(0.4))
    grip_watch.GripWatch._on_gripped(watch, _bool(False))
    assert watch._last_scan_at is None

    clock.advance(0.1)
    grip_watch.GripWatch._on_scan(watch, _scan(0.4))
    grip_watch.GripWatch._on_gripped(watch, _bool(True))
    assert watch._last_scan_at is None

    clock.advance(2.0)
    grip_watch.GripWatch._tick(watch)
    assert watch._lost is True
    assert watch.lost_publications[-1] is True


def test_watchdog_progresses_while_ros_clock_is_frozen():
    watchdog_clock = _FakeClock()
    ros_clock = _FakeClock(100.0)
    watch = _watch(watchdog_clock, ros_clock=ros_clock)
    grip_watch.GripWatch._on_gripped(watch, _bool(True))

    watchdog_clock.advance(2.0)
    grip_watch.GripWatch._tick(watch)

    assert watch._lost is True
    assert watch.lost_publications[-1] is True


def test_scan_detection_uses_steady_time_while_ros_clock_is_frozen():
    watchdog_clock = _FakeClock()
    ros_clock = _FakeClock(100.0)
    watch = _watch(watchdog_clock, ros_clock=ros_clock)
    grip_watch.GripWatch._on_gripped(watch, _bool(True))

    watchdog_clock.advance(2.0)
    grip_watch.GripWatch._on_scan(watch, _scan(0.8))
    assert watch._last_scan_at.nanoseconds == 2_000_000_000
    assert watch._lost_since.nanoseconds == 2_000_000_000

    watchdog_clock.advance(0.5)
    grip_watch.GripWatch._on_scan(watch, _scan(0.8))
    assert watch._lost is True


def test_false_clears_lost_and_monitoring_state():
    clock = _FakeClock(3.0)
    watch = _watch(clock)
    watch._gripped = True
    watch._gripped_since = _FakeTime(0.0)
    watch._lost_since = _FakeTime(2.0)
    watch._lost = True

    grip_watch.GripWatch._on_gripped(watch, _bool(False))

    assert watch._gripped is False
    assert watch._gripped_since is None
    assert watch._lost_since is None
    assert watch._lost is False
    assert watch.lost_publications == [False]


@pytest.mark.parametrize(
    ('polarity', 'holding_range', 'suspect_range'),
    (('present', 0.4, 0.8), ('absent', math.inf, 1.0)),
)
def test_fresh_scans_keep_existing_polarity_detection(
        polarity, holding_range, suspect_range):
    clock = _FakeClock()
    watch = _watch(clock, polarity=polarity)
    grip_watch.GripWatch._on_gripped(watch, _bool(True))
    clock.advance(2.1)

    grip_watch.GripWatch._on_scan(watch, _scan(holding_range))
    clock.advance(0.6)
    grip_watch.GripWatch._on_scan(watch, _scan(holding_range))
    assert watch._lost is False

    grip_watch.GripWatch._on_scan(watch, _scan(suspect_range))
    clock.advance(0.6)
    grip_watch.GripWatch._on_scan(watch, _scan(suspect_range))
    assert watch._lost is True
    assert watch.lost_publications[-1] is True


def test_present_polarity_keeps_hold_min_unused():
    clock = _FakeClock()
    watch = _watch(clock)
    grip_watch.GripWatch._on_gripped(watch, _bool(True))
    clock.advance(2.1)

    grip_watch.GripWatch._on_scan(watch, _scan(0.2))
    clock.advance(0.6)
    grip_watch.GripWatch._on_scan(watch, _scan(0.2))

    assert watch._lost is False
    assert watch._lost_since is None
