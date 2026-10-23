"""Hardware transport tests use in-process ROS doubles and never real DDS."""
from copy import deepcopy
import json
from types import SimpleNamespace
import threading
import time

import pytest
import numpy as np

from retarget_v2 import ros_hardware
from retarget_v2.ros_hardware import HardwareTransport


class FakeControl:
    def __init__(self, profile, interpolate=False):
        self.interpolate = interpolate
        self.watchdog_calls = 0
        self.enabled, self.reason, self.sample_calls = False, 'disarmed', 0
        self.feedback = self.target = None
        self.closed = False

    def update_feedback(self, *values):
        self.feedback = values
        return True

    def update_target(self, q, received_at):
        if self.target is not None and received_at <= self.target[1]:
            return False
        self.target = (deepcopy(q), received_at)
        return True

    def arm(self, now):
        self.enabled, self.reason = True, 'armed'
        return True

    def pause(self, reason):
        self.enabled, self.reason = False, reason

    def clear_target(self, reason):
        self.pause(reason)
        self.target = None

    def shutdown(self):
        self.closed = True
        self.pause('shutdown')

    def sample(self, now):
        self.sample_calls += 1
        if self.enabled:
            return [i * .01 for i in range(21)], [-i * .02 for i in range(21)]
        return None

    def check_watchdog(self, now):
        self.watchdog_calls += 1
        if self.enabled and self.target is not None and now - self.target[1] > .25:
            self.pause("stale_target")

    def status(self, now):
        return dict(state='tracking' if self.enabled else 'waiting', enabled=self.enabled,
                    reason=self.reason, feedback_model_q=None, feedback_velocity_hardware=None,
                    ready_to_arm=True, feedback_age_s=None,
                    target_age_s=None if self.target is None else now-self.target[1], last_error=None)


def endpoint(name, namespace='/revo3_right', gid=1):
    return SimpleNamespace(node_name=name, node_namespace=namespace, endpoint_gid=bytes([gid]*24),
                           topic_type='revo3_mit_controller_msgs/msg/Revo3MITCommand')


@pytest.fixture
def fixture(monkeypatch):
    profile = SimpleNamespace(hardware_names=tuple(f'joint_{i}' for i in range(21)),
                              kp=[.6]*21, kd=[.12]*21, command=SimpleNamespace(publish_hz=200.))
    state = dict(publishers=[], subscribers=[endpoint('joint_forward_mit_controller')], nodes=[],
                 messages=[], statuses=[], contexts=[], executors=[], publisher_creations=0)

    class Context:
        def __init__(self):
            self.active = False
            state['contexts'].append(self)

        def init(self, args, domain_id):
            import os
            self.active = True
            self.domain_id = int(os.environ.get('ROS_DOMAIN_ID', '0')) if domain_id is None else domain_id

        def get_domain_id(self):
            return self.domain_id

        def ok(self):
            return self.active

        def shutdown(self):
            self.active = False

    class Now:
        nanoseconds = 123_000_000_045

        def to_msg(self):
            return SimpleNamespace(sec=123, nanosec=45)

    class Publisher:
        def __init__(self, is_command):
            self.key = 'messages' if is_command else 'statuses'

        def publish(self, message):
            state[self.key].append(deepcopy(message))

    class Timer:
        def __init__(self, period, callback):
            self.period, self.callback, self.cancelled = period, callback, False

        def cancel(self):
            self.cancelled = True

    class Node:
        def __init__(self, name, **kwargs):
            self.name, self.kwargs, self.destroyed = name, kwargs, False
            self.subscriptions = {}
            self.services = {}
            self.timers = []
            state['nodes'].append(self)

        def create_subscription(self, typ, topic, callback, qos):
            self.subscriptions[topic] = callback
            return object()

        def create_publisher(self, typ, topic, qos):
            is_command = topic.endswith('/commands')
            if is_command:
                state['publisher_creations'] += 1
                state['publishers'].append(endpoint(self.name, self.kwargs['namespace'], gid=9))
            return Publisher(is_command)

        def create_service(self, typ, name, callback):
            self.services[name] = callback
            return object()

        def create_timer(self, period, callback):
            timer = Timer(period, callback)
            self.timers.append(timer)
            return timer

        def get_clock(self):
            return SimpleNamespace(now=lambda: Now())

        def get_publishers_info_by_topic(self, topic):
            return list(state['publishers'])

        def get_subscriptions_info_by_topic(self, topic):
            return list(state['subscribers'])

        def destroy_node(self):
            self.destroyed = True
            state['publishers'][:] = [p for p in state['publishers'] if p.node_name != self.name]

    class Executor:
        def __init__(self, context):
            self.event, self.closed = threading.Event(), False
            state['executors'].append(self)

        def add_node(self, node):
            self.node = node

        def spin_once(self, timeout_sec):
            self.event.wait(timeout_sec)
            self.event.clear()

        def wake(self):
            self.event.set()

        def shutdown(self, timeout_sec):
            self.closed = True

    class Command:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None)

    monkeypatch.setattr(ros_hardware, '_new_controller', FakeControl)
    monkeypatch.setattr(ros_hardware, '_dependencies', lambda: (
        Context, Executor, Node, 'feedback_qos', 'command_qos', 'status_qos', object, Command, SimpleNamespace, object, object))
    monkeypatch.setattr(HardwareTransport, 'DISCOVERY_WAIT_S', 0.)
    return profile, state


def ready(transport, generation=0):
    assert transport.set_target([0.]*21, time.monotonic(), generation)
    assert transport.update_retarget_status(dict(phase='READY', calibrated=True,
        calibration_count=120, calibration_frames=120, glove_id=7, input_error=None), generation)


def call_enable(transport, value):
    return transport._enable_callback(SimpleNamespace(data=value), SimpleNamespace())


@pytest.mark.parametrize("name", ["manus_revo3_retarget", "manus_revo3_retarget_left",
                                  "manus_revo3_retarget_right"])
def test_compatible_node_name_keeps_ownership_and_enable_contract(fixture, name):
    profile, state = fixture
    with HardwareTransport(profile, domain_id=197, node_name=name) as transport:
        assert state['nodes'][0].name == name
        assert transport.snapshot()['enable_service'] == f'/{name}/enable_retarget'
        assert transport.status_topic == f'/{name}/status'
        assert state['messages'] == []
        assert not call_enable(transport, True).success
        ready(transport)
        assert call_enable(transport, True).success
        transport.set_target([.1] * 21, time.monotonic())
        assert len(state['messages']) == 1
        assert call_enable(transport, False).success
        transport.set_target([.2] * 21, time.monotonic())
        assert len(state['messages']) == 1
        assert transport.snapshot()['calibrated']


@pytest.mark.parametrize("name", ["", "/node", "a/b", "1node", "bad-name"])
def test_invalid_node_name_rejected_before_ros_starts(fixture, name):
    profile, state = fixture
    with pytest.raises(ValueError, match="node_name"):
        HardwareTransport(profile, node_name=name)
    assert not state['contexts']


def test_default_has_services_but_does_not_publish_until_calibrated_and_enabled(fixture):
    profile, state = fixture
    with HardwareTransport(profile, domain_id=197) as transport:
        assert transport.start() is transport
        assert state['publisher_creations'] == 1
        assert set(state['nodes'][0].services) == {'~/enable_retarget', '~/recalibrate'}
        assert set(state['nodes'][0].subscriptions) == {transport.feedback_topic}
        transport._tick()
        assert not call_enable(transport, True).success
        assert state['messages'] == []
        ready(transport)
        transport._tick()
        assert state['messages'] == []
        assert transport.snapshot()['phase'] == 'READY'
        transport._publish_status()
        assert json.loads(state['statuses'][-1].data)['phase'] == 'READY'
        assert [timer.period for timer in state['nodes'][0].timers] == [.05, .5]
    assert state['nodes'][0].destroyed
    assert all(timer.cancelled for timer in state['nodes'][0].timers)
    assert state['executors'][0].closed
    assert not state['contexts'][0].active


def test_direct_publishes_on_new_target_only_never_on_timer_feedback_or_enable(fixture):
    profile, state = fixture
    with HardwareTransport(profile, domain_id=197) as transport:
        ready(transport)
        assert transport.arm()
        assert state['messages'] == []
        received = time.monotonic()
        assert transport.set_target([.1]*21, received, generation=0)
        assert len(state['messages']) == 1
        assert not transport.set_target([.1]*21, received, generation=0)
        transport._feedback_callback(SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=123, nanosec=0)),
            name=list(profile.hardware_names), position=[.2]*21, velocity=[0.]*21))
        for _ in range(5):
            transport._tick()
        assert len(state['messages']) == 1
        assert transport.control.watchdog_calls == 5
        assert transport.snapshot()['interpolate'] is False
        assert transport.snapshot()['output_mode'] == 'direct'
        assert transport.set_target([.3]*21, time.monotonic(), generation=0)
        assert len(state['messages']) == 2
        assert call_enable(transport, False).success
        assert transport.set_target([.4]*21, time.monotonic(), generation=0)
        transport._tick()
        assert len(state['messages']) == 2


def test_direct_watchdog_and_conflict_checks_never_resend_cached_targets(fixture, monkeypatch):
    profile, state = fixture
    clock = [10.]
    monkeypatch.setattr(ros_hardware, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    with HardwareTransport(profile, domain_id=197) as transport:
        assert transport.set_target([0.]*21, clock[0], generation=0)
        assert transport.update_retarget_status(dict(calibrated=True, input_error=None), 0)
        assert transport.arm()
        clock[0] += .02
        assert transport.set_target([.1]*21, clock[0], generation=0)
        assert len(state['messages']) == 1
        clock[0] += .3
        transport._tick()
        assert not transport.snapshot()['enabled']
        assert transport.snapshot()['controller']['reason'] == 'stale_target'
        assert transport.set_target([.2]*21, clock[0], generation=0)
        assert len(state['messages']) == 1
        assert transport.arm()
        state['publishers'].append(endpoint('competing', gid=5))
        clock[0] += .11
        assert transport.set_target([.3]*21, clock[0], generation=0)
        assert not transport.snapshot()['enabled']
        assert 'Competing' in transport.snapshot()['reason']
        assert len(state['messages']) == 1


def test_direct_recalibration_discards_inflight_result_before_publication(fixture):
    profile, state = fixture
    with HardwareTransport(profile, domain_id=197) as transport:
        ready(transport)
        assert transport.arm()
        assert transport.set_target([.1]*21, time.monotonic(), generation=0)
        assert len(state['messages']) == 1
        assert transport._recalibrate_callback(None, SimpleNamespace()).success
        assert not transport.set_target([.2]*21, time.monotonic(), generation=0)
        assert transport.take_recalibration_request() == 1
        ready(transport, generation=1)
        transport._tick()
        assert len(state['messages']) == 1
        assert transport.arm()
        assert len(state['messages']) == 1
        assert transport.set_target([.3]*21, time.monotonic(), generation=1)
        assert len(state['messages']) == 2


def test_direct_publication_failure_latches_pause(fixture, monkeypatch):
    profile, state = fixture
    with HardwareTransport(profile, domain_id=197) as transport:
        ready(transport)
        assert transport.arm()
        def fail(message):
            raise RuntimeError('publish failed')
        monkeypatch.setattr(transport._publisher, 'publish', fail)
        assert not transport.set_target([.1]*21, time.monotonic(), generation=0)
        assert not transport.snapshot()['enabled']
        assert 'publish failed' in transport.snapshot()['last_error']
        assert state['messages'] == []


@pytest.mark.parametrize('interpolate', ['false', 0, 1, None])
def test_interpolate_requires_a_boolean(fixture, interpolate):
    profile, _ = fixture
    with pytest.raises(ValueError, match='interpolate'):
        HardwareTransport(profile, interpolate=interpolate)


def test_explicit_output_remains_disarmed_until_arm_and_has_complete_named_command(fixture):
    profile, state = fixture
    with HardwareTransport(profile, domain_id=197, interpolate=True) as transport:
        transport._tick()
        assert state['messages'] == []
        ready(transport)
        assert transport.arm()
        transport._tick()
        assert len(state['messages']) == 1
        command = state['messages'][0]
        assert command.joint_names == list(profile.hardware_names)
        assert command.position == [i*.01 for i in range(21)]
        assert command.velocity == [-i*.02 for i in range(21)]
        assert command.kp == profile.kp and command.kd == profile.kd
        assert command.effort == [0.]*21
        assert command.header.stamp.sec == 123
        assert command.header.frame_id == 'right_hand_base_link'
        assert transport.snapshot()['published'] == 1
        transport.pause('user requested pause')
        transport._tick()
        assert len(state['messages']) == 1
        ready(transport)
        assert transport.arm()
        transport._tick()
        assert len(state['messages']) == 2


def test_existing_publisher_rejected_before_output_created(fixture):
    profile, state = fixture
    state['publishers'].append(endpoint('old_pipeline'))
    transport = HardwareTransport(profile, domain_id=197, interpolate=True)
    with pytest.raises(RuntimeError, match='existing publishers'):
        transport.start()
    assert state['publisher_creations'] == 0
    assert transport.snapshot()['closed']


def test_competitor_latches_pause_and_no_sample_until_explicit_rearm(fixture):
    profile, state = fixture
    with HardwareTransport(profile, domain_id=197, interpolate=True) as transport:
        ready(transport)
        assert transport.arm()
        transport._tick()
        calls = transport.control.sample_calls
        state['publishers'].append(endpoint('old_pipeline', gid=2))
        transport._last_graph_check = float('-inf')
        transport._tick()
        assert transport.control.sample_calls == calls
        assert transport.snapshot()['state'] == 'fault'
        assert 'Competing' in transport.snapshot()['reason']
        state['publishers'].pop()
        transport._last_graph_check = float('-inf')
        transport._tick()
        assert not transport.snapshot()['enabled']
        calls = transport.control.sample_calls
        ready(transport)
        assert transport.arm()
        transport._tick()
        assert transport.control.sample_calls == calls + 1


def test_duplicate_self_named_publisher_is_competition_by_gid(fixture):
    profile, state = fixture
    with HardwareTransport(profile, domain_id=197, interpolate=True) as transport:
        ready(transport)
        assert transport.arm()
        state['publishers'].append(endpoint(transport.node_name, namespace='/', gid=7))
        transport._last_graph_check = float('-inf')
        transport._tick()
        assert transport.snapshot()['state'] == 'fault'
        assert not transport.snapshot()['enabled']


def test_controller_subscriber_required_and_loss_latches_pause(fixture):
    profile, state = fixture
    with HardwareTransport(profile, domain_id=197, interpolate=True) as transport:
        ready(transport)
        state['subscribers'] = [endpoint('observer_only')]
        assert not transport.arm()
        assert 'subscriber is missing' in transport.snapshot()['reason']
        state['subscribers'] = [endpoint('joint_forward_mit_controller', namespace='/other_hand')]
        assert not transport.arm()
        assert transport.snapshot()['controller_subscriber_count'] == 0
        bad_type = endpoint('joint_forward_mit_controller')
        bad_type.topic_type = 'std_msgs/msg/Float64MultiArray'
        state['subscribers'] = [bad_type]
        assert not transport.arm()
        state['subscribers'] = [endpoint('joint_forward_mit_controller')]
        transport._last_graph_check = float('-inf')
        transport._tick()
        assert state['messages'] == []
        ready(transport)
        assert transport.arm()
        state['subscribers'] = []
        transport._last_graph_check = float('-inf')
        transport._tick()
        assert state['messages'] == []
        assert not transport.snapshot()['enabled']


def test_target_receipt_timestamp_is_preserved(fixture):
    profile, _ = fixture
    with HardwareTransport(profile, domain_id=197, interpolate=True) as transport:
        transport.set_target([.01]*21, 3.25)
        assert transport.control.target[1] == 3.25
        transport._tick()
        assert transport.control.target[1] == 3.25


def test_feedback_passes_names_and_header_time_to_pure_controller(fixture):
    profile, state = fixture
    with HardwareTransport(profile, domain_id=197, interpolate=True) as transport:
        node = state['nodes'][0]
        header = SimpleNamespace(stamp=SimpleNamespace(sec=120, nanosec=25))
        joint_message = SimpleNamespace(header=header, name=list(profile.hardware_names), position=[.1]*21, velocity=[.2]*21)
        node.subscriptions[transport.feedback_topic](joint_message)
        feedback = transport.control.feedback
        assert feedback[0] == list(profile.hardware_names)
        assert feedback[-3] <= time.monotonic()
        assert feedback[-2] == pytest.approx(123.000000045, abs=1e-12)
        assert feedback[-1] == feedback[-3]
        assert feedback[3] == 120.000000025


def test_close_stops_publishing_and_is_not_an_extra_hold_command(fixture):
    profile, state = fixture
    transport = HardwareTransport(profile, domain_id=197, interpolate=True).start()
    ready(transport)
    assert transport.arm()
    transport._tick()
    transport.close()
    transport._tick()
    assert len(state['messages']) == 1
    assert not transport.set_target([0.]*21, time.monotonic())
    assert not transport.arm()
    assert transport.control.closed
    transport.close()


def test_timer_failure_stops_output_and_latches_fault(fixture, monkeypatch):
    profile, state = fixture
    with HardwareTransport(profile, domain_id=197, interpolate=True) as transport:
        ready(transport)
        assert transport.arm()
        def fail(now):
            raise ValueError('test control failure')
        monkeypatch.setattr(transport.control, 'sample', fail)
        transport._tick()
        assert 'test control failure' in transport.snapshot()['last_error']
        assert not transport.snapshot()['enabled']
        assert state['messages'] == []


def test_real_controller_snapshot_is_strict_json_without_ros():
    from retarget_v2.model import RobotModel
    from retarget_v2.hardware_profile import HardwareProfile
    with RobotModel() as robot:
        profile = HardwareProfile(robot)
        transport = HardwareTransport(profile, domain_id=197, interpolate=True)
        now = time.monotonic()
        q = np.clip(np.zeros(21), profile.lower, profile.upper)
        zeros = np.zeros(21)
        assert transport.control.update_feedback(profile.names, q, zeros, 2000., now, 2000., now)
        transport.set_target(np.zeros(21), now)
        encoded = json.dumps(transport.snapshot(), allow_nan=False)
        decoded = json.loads(encoded)
        assert not decoded['enabled']
        assert len(decoded['feedback_model_q']) == 21
        assert decoded['feedback_source'] == 'joint_state'
        assert 'current_a' not in decoded and 'motor_state' not in decoded
        transport.close()


def test_services_pause_and_recalibrate_invalidate_inflight_solve(fixture):
    profile, state = fixture
    with HardwareTransport(profile, domain_id=197, interpolate=True) as transport:
        ready(transport)
        assert call_enable(transport, True).success
        transport._tick()
        assert len(state['messages']) == 1
        assert call_enable(transport, False).success
        transport._tick()
        assert len(state['messages']) == 1
        assert transport.snapshot()['calibrated']
        assert call_enable(transport, True).success
        response = transport._recalibrate_callback(None, SimpleNamespace())
        assert response.success
        assert not transport.control.enabled
        assert transport.control.target is None
        assert not call_enable(transport, True).success
        # A solve or status update started before the service cannot restore READY.
        assert not transport.set_target([0.]*21, time.monotonic(), generation=0)
        assert not transport.update_retarget_status(dict(calibrated=True), generation=0)
        assert transport.take_recalibration_request() == 1
        assert transport.take_recalibration_request() is None
        ready(transport, generation=1)
        transport._tick()
        assert len(state['messages']) == 1
        assert call_enable(transport, True).success
        transport._tick()
        assert len(state['messages']) == 2


def test_fixed_per_hand_services_are_independent(fixture):
    profile, state = fixture
    right = HardwareTransport(profile, domain_id=197, interpolate=True)
    left_profile = deepcopy(profile)
    left_profile.hand_side = 'left'
    left = HardwareTransport(left_profile, domain_id=197)
    assert right.service_prefix == '/manus_revo3_vector_retarget_right'
    assert left.service_prefix == '/manus_revo3_vector_retarget_left'
    right._recalibrate_callback(None, SimpleNamespace())
    assert right.take_recalibration_request() == 1
    assert left.take_recalibration_request() is None


@pytest.mark.parametrize('domain', [0, 188])
def test_default_domain_inherits_ros_environment(fixture, monkeypatch, domain):
    profile, _ = fixture
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    with HardwareTransport(profile) as transport:
        assert transport.domain_id == domain
        assert transport.snapshot()['domain_id'] == domain


@pytest.mark.parametrize('invalid', [-1, 233, True, '197'])
def test_invalid_domain_rejected(fixture, invalid):
    profile, _ = fixture
    with pytest.raises(ValueError, match='domain'):
        HardwareTransport(profile, domain_id=invalid)
