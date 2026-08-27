import json
import math
from types import MethodType, SimpleNamespace

from go2_base_nav import nav_relay as nav_relay_module
from go2_base_nav.nav_relay import NavRelay
import pytest
from std_msgs.msg import String


class _Future:

    def __init__(self):
        self._callbacks = []
        self._result = None

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def result(self):
        return self._result

    def complete(self, result):
        self._result = result
        for callback in list(self._callbacks):
            callback(self)


class _GoalHandle:

    def __init__(self, *, accepted=True):
        self.accepted = accepted
        self.result_future = _Future()
        self.cancel_count = 0

    def cancel_goal_async(self):
        self.cancel_count += 1

    def get_result_async(self):
        return self.result_future


class _ActionClient:

    def __init__(self):
        self.requests = []

    def send_goal_async(self, goal, *, feedback_callback=None):
        request = SimpleNamespace(
            goal=goal,
            feedback_callback=feedback_callback,
            send_future=_Future(),
        )
        self.requests.append(request)
        return request.send_future


class _Publisher:

    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(message.data)


class _Logger:

    def __init__(self):
        self.warnings = []

    def info(self, _message):
        pass

    def warn(self, message):
        self.warnings.append(message)


class _Time:

    def to_msg(self):
        return SimpleNamespace()


def _relay():
    logger = _Logger()
    relay = SimpleNamespace(
        client=_ActionClient(),
        plan_client=_ActionClient(),
        status_pub=_Publisher(),
        _epoch=0,
        _request_id=None,
        _send_future=None,
        _goal_handle=None,
        _result_future=None,
        _last_remaining=-1.0,
        get_clock=lambda: SimpleNamespace(now=lambda: _Time()),
        get_logger=lambda: logger,
        logger=logger,
    )
    for method_name in (
            '_publish_status', '_on_feedback', '_on_send_done', '_on_result',
            '_on_plan_send_done', '_on_plan_result'):
        if not hasattr(NavRelay, method_name):
            continue
        setattr(
            relay,
            method_name,
            MethodType(getattr(NavRelay, method_name), relay),
        )
    return relay


def _message(text):
    message = String()
    message.data = text
    return message


def _feedback(distance_remaining):
    return SimpleNamespace(
        feedback=SimpleNamespace(distance_remaining=distance_remaining))


def _send_and_accept(relay, text):
    NavRelay._on_goal(relay, _message(text))
    request = relay.client.requests[-1]
    handle = _GoalHandle()
    request.send_future.complete(handle)
    return request, handle


def _path(*points):
    return SimpleNamespace(poses=[
        SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y)))
        for x, y in points
    ])


def _plan_result(*points, status=4, error_code=0):
    return SimpleNamespace(
        status=status,
        result=SimpleNamespace(
            error_code=error_code,
            path=_path(*points),
        ),
    )


def _send_plan_and_accept(relay, text):
    NavRelay._on_goal(relay, _message(text))
    request = relay.plan_client.requests[-1]
    handle = _GoalHandle()
    request.send_future.complete(handle)
    return request, handle


def test_tagged_goal_echoes_request_id_on_status_feedback_and_result():
    relay = _relay()

    request, handle = _send_and_accept(relay, '17|1.0,2.0,90.0')
    request.feedback_callback(_feedback(1.25))
    NavRelay._tick(relay)
    handle.result_future.complete(SimpleNamespace(status=4))

    assert relay.status_pub.values == [
        '17|ACCEPTED_PENDING',
        '17|NAV 1.25',
        '17|SUCCEEDED',
    ]


def test_legacy_goal_keeps_bare_status_protocol():
    relay = _relay()

    request, handle = _send_and_accept(relay, '1.0,2.0,90.0')
    request.feedback_callback(_feedback(2.5))
    NavRelay._tick(relay)
    handle.result_future.complete(SimpleNamespace(status=5))

    assert relay.status_pub.values == [
        'ACCEPTED_PENDING',
        'NAV 2.50',
        'FAILED',
    ]


def test_tagged_and_legacy_cancel_are_both_supported():
    tagged = _relay()
    _, tagged_handle = _send_and_accept(tagged, '17|1.0,2.0,90.0')

    NavRelay._on_goal(tagged, _message('17|cancel'))

    assert tagged_handle.cancel_count == 1
    assert tagged.status_pub.values[-1] == '17|CANCELED'
    assert tagged._goal_handle is None

    legacy = _relay()
    _, legacy_handle = _send_and_accept(legacy, '1.0,2.0,90.0')

    NavRelay._on_goal(legacy, _message('cancel'))

    assert legacy_handle.cancel_count == 1
    assert legacy.status_pub.values[-1] == 'CANCELED'
    assert legacy._goal_handle is None


def test_stale_tagged_cancel_does_not_cancel_current_request():
    relay = _relay()
    _, first_handle = _send_and_accept(relay, '17|1.0,2.0,90.0')
    _, current_handle = _send_and_accept(relay, '18|2.0,3.0,0.0')
    assert first_handle.cancel_count == 1
    epoch = relay._epoch

    NavRelay._on_goal(relay, _message('17|cancel'))

    assert current_handle.cancel_count == 0
    assert relay._goal_handle is current_handle
    assert relay._epoch == epoch
    assert relay.status_pub.values[-1] == '18|ACCEPTED_PENDING'


def test_feedback_from_old_epoch_cannot_pollute_current_request():
    relay = _relay()
    old_request, old_handle = _send_and_accept(
        relay, '17|1.0,2.0,90.0')
    current_request, current_handle = _send_and_accept(
        relay, '18|2.0,3.0,0.0')

    current_request.feedback_callback(_feedback(2.0))
    old_request.feedback_callback(_feedback(99.0))
    NavRelay._tick(relay)
    old_handle.result_future.complete(SimpleNamespace(status=4))

    assert relay._last_remaining == 2.0
    assert relay.status_pub.values[-1] == '18|NAV 2.00'
    assert '17|SUCCEEDED' not in relay.status_pub.values

    current_handle.result_future.complete(SimpleNamespace(status=4))
    assert relay.status_pub.values[-1] == '18|SUCCEEDED'


def test_tagged_plan_uses_requested_planner_and_returns_compact_path_json():
    relay = _relay()

    _, handle = _send_plan_and_accept(
        relay, '41|PLAN,2.0,3.0,90.0,TowGrid')
    request = relay.plan_client.requests[-1]
    handle.result_future.complete(_plan_result(
        (0.0, 0.0), (0.05, 0.0), (0.21, 0.0), (0.40, 0.0)))

    assert request.goal.planner_id == 'TowGrid'
    assert request.goal.use_start is False
    assert request.goal.goal.header.frame_id == 'map'
    assert request.goal.goal.pose.position.x == pytest.approx(2.0)
    assert request.goal.goal.pose.position.y == pytest.approx(3.0)
    assert request.goal.goal.pose.orientation.z == pytest.approx(
        math.sin(math.pi / 4.0))
    assert relay.status_pub.values[0] == '41|PLAN_PENDING'
    prefix, payload = relay.status_pub.values[-1].split('PATH,', 1)
    assert prefix == '41|'
    assert ' ' not in payload
    assert json.loads(payload) == [[0.0, 0.0], [0.4, 0.0]]


def test_rejected_plan_reports_unknown_error_code():
    relay = _relay()

    NavRelay._on_goal(
        relay, _message('42|PLAN,2.0,3.0,0.0,TowGrid'))
    request = relay.plan_client.requests[-1]
    request.send_future.complete(_GoalHandle(accepted=False))

    assert relay.status_pub.values == [
        '42|PLAN_PENDING',
        '42|PLAN_FAILED,200',
    ]


def test_empty_successful_plan_reports_no_valid_path():
    relay = _relay()

    _, handle = _send_plan_and_accept(
        relay, '43|PLAN,2.0,3.0,0.0,TowGrid')
    handle.result_future.complete(_plan_result())

    assert relay.status_pub.values[-1] == '43|PLAN_FAILED,208'


def test_failed_plan_preserves_compute_path_error_code():
    relay = _relay()

    _, handle = _send_plan_and_accept(
        relay, '44|PLAN,2.0,3.0,0.0,TowGrid')
    handle.result_future.complete(_plan_result(
        status=6, error_code=204))

    assert relay.status_pub.values[-1] == '44|PLAN_FAILED,204'


def test_new_navigation_goal_cancels_plan_and_stale_plan_result_is_ignored():
    relay = _relay()
    _, plan_handle = _send_plan_and_accept(
        relay, '45|PLAN,2.0,3.0,0.0,TowGrid')

    _, nav_handle = _send_and_accept(relay, '46|1.0,2.0,0.0')
    plan_handle.result_future.complete(_plan_result(
        (0.0, 0.0), (2.0, 3.0)))

    assert plan_handle.cancel_count == 1
    assert not any(value.startswith('45|PATH,')
                   for value in relay.status_pub.values)
    nav_handle.result_future.complete(SimpleNamespace(status=4))
    assert relay.status_pub.values[-1] == '46|SUCCEEDED'


def test_tagged_cancel_cancels_active_plan_and_invalidates_late_result():
    relay = _relay()
    _, handle = _send_plan_and_accept(
        relay, '47|PLAN,2.0,3.0,0.0,TowGrid')

    NavRelay._on_goal(relay, _message('47|cancel'))
    handle.result_future.complete(_plan_result(
        (0.0, 0.0), (2.0, 3.0)))

    assert handle.cancel_count == 1
    assert relay.status_pub.values[-1] == '47|CANCELED'
    assert not any(value.startswith('47|PATH,')
                   for value in relay.status_pub.values)


def test_path_compaction_preserves_endpoints_and_bounds_payload():
    path = _path(*[(index * 0.21, 0.0) for index in range(120)])

    points = nav_relay_module._compact_path_points(path)

    assert 2 <= len(points) <= 80
    assert points[0] == [0.0, 0.0]
    assert points[-1] == [24.99, 0.0]


def test_path_compaction_rejects_non_finite_coordinates():
    path = _path((0.0, 0.0), (float('nan'), 1.0))

    with pytest.raises(ValueError, match='non-finite'):
        nav_relay_module._compact_path_points(path)


def test_path_compaction_preserves_a_tight_right_angle_corner():
    raw = ([(index * 0.05, 0.0) for index in range(21)]
           + [(1.0, index * 0.05) for index in range(1, 21)])

    points = nav_relay_module._compact_path_points(_path(*raw))

    assert [1.0, 0.0] in points
    assert points == [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]


def test_path_compaction_rejects_geometry_too_complex_for_safe_payload():
    raw = [(index * 0.05, 0.2 if index % 2 else 0.0)
           for index in range(200)]

    with pytest.raises(ValueError, match='too complex'):
        nav_relay_module._compact_path_points(_path(*raw))
