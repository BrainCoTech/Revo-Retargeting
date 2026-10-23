"""Pure-Python buffer/conversion tests plus deterministic ROS lifecycle doubles."""
from copy import deepcopy
import importlib
import sys
import threading
import time
from types import ModuleType, SimpleNamespace

import pytest

from retarget_v2 import ros_input
from retarget_v2.ros_input import LatestMessageBuffer, ManusSubscriber, ros_message_to_dict


def sdk_message(glove_id=-794976691, side="Right"):
    nodes = []

    def append(chain, part, parent):
        node_id = len(nodes)
        nodes.append(dict(node_id=node_id, parent_node_id=parent, chain_type=chain, joint_type=part,
                          pose=dict(position=dict(x=node_id * .001, y=0., z=node_id * .002),
                                    orientation=dict(x=0., y=0., z=0., w=1.))))
        return node_id

    append("Hand", "Invalid", 0)
    for finger in ("Thumb", "Index", "Middle", "Ring", "Pinky"):
        parent = 0
        for part in (("Metacarpal", "Proximal", "Distal", "Tip") if finger == "Thumb" else
                     ("Metacarpal", "Proximal", "Intermediate", "Distal", "Tip")):
            parent = append(finger, part, parent)
    return dict(glove_id=glove_id, side=side, raw_node_count=25, raw_nodes=nodes,
                ergonomics_count=1, ergonomics=[dict(type="IndexMCPStretch", value=14.25)],
                raw_sensor_orientation=dict(x=.1, y=.2, z=.3, w=.9),
                raw_sensor_count=1, raw_sensor=[deepcopy(nodes[3]["pose"])])


def object_message(value):
    if isinstance(value, dict):
        return SimpleNamespace(**{key: object_message(item) for key, item in value.items()})
    if isinstance(value, list):
        return [object_message(item) for item in value]
    return value


def test_ros_conversion_preserves_all_raw_fields_and_negative_glove_id():
    source = sdk_message()
    assert ros_message_to_dict(object_message(source)) == source


def test_latest_only_sequence_and_copy_isolation():
    buffer = LatestMessageBuffer()
    assert buffer.latest() is None
    source = sdk_message()
    assert buffer.put(source, 1.)
    source["raw_nodes"][0]["pose"]["position"]["x"] = 100.
    sample = buffer.latest()
    assert sample.sequence == 1
    assert sample.received_monotonic == 1.
    assert sample.message["raw_nodes"][0]["pose"]["position"]["x"] == 0.
    sample.message["glove_id"] = 22
    assert buffer.latest().message["glove_id"] == -794976691
    for i in range(2, 10):
        assert buffer.put(sdk_message(), float(i))
    assert buffer.latest(after_sequence=1).sequence == 9
    assert buffer.latest(after_sequence=9) is None
    assert buffer.status()["accepted"] == 9


def start_buffer_waiter(monkeypatch, buffer, after_sequence=None):
    """Start a reader and wait until it is about to release the condition lock."""
    entered = threading.Event()
    result = []
    original_wait = buffer._condition.wait

    def observed_wait(timeout=None):
        entered.set()
        return original_wait(timeout)

    monkeypatch.setattr(buffer._condition, "wait", observed_wait)
    worker = threading.Thread(
        target=lambda: result.append(buffer.wait_for_sample(after_sequence)), daemon=True)
    worker.start()
    assert entered.wait(timeout=1.)
    return worker, result


def test_wait_reads_existing_latest_without_lost_notification_or_backlog():
    buffer = LatestMessageBuffer()
    for sequence in range(1, 10):
        assert buffer.put(sdk_message(), float(sequence))
    sample = buffer.wait_for_sample(after_sequence=1, timeout=0.)
    assert sample.sequence == 9
    sample.message["glove_id"] = 23
    assert buffer.latest().message["glove_id"] == -794976691
    assert buffer.wait_for_sample(after_sequence=9, timeout=0.) is None
    assert buffer.wait_for_sample(timeout=0.).sequence == 9


def test_wait_wakes_for_new_frame(monkeypatch):
    buffer = LatestMessageBuffer()
    assert buffer.put(sdk_message(), 1.)
    worker, result = start_buffer_waiter(monkeypatch, buffer, after_sequence=1)
    assert buffer.put(sdk_message(), 2.)
    worker.join(timeout=1.)
    assert not worker.is_alive()
    assert result[0].sequence == 2
    assert result[0].received_monotonic == 2.


def test_invalid_and_wrong_hand_frames_do_not_notify_or_refresh_waiter(monkeypatch):
    buffer = LatestMessageBuffer()
    assert buffer.put(sdk_message(), 1.)
    worker, result = start_buffer_waiter(monkeypatch, buffer, after_sequence=1)
    notifications = []
    original_notify = buffer._condition.notify_all

    def observed_notify():
        notifications.append(True)
        original_notify()

    monkeypatch.setattr(buffer._condition, "notify_all", observed_notify)
    assert not buffer.put(sdk_message(side="Left"), 2.)
    invalid = sdk_message()
    invalid["raw_nodes"][8]["parent_node_id"] = 16
    assert not buffer.put(invalid, 3.)
    assert not buffer.put(sdk_message(glove_id=32), 4.)
    assert not buffer.put(sdk_message(), 1.)
    assert notifications == []
    assert result == []
    assert buffer.latest().sequence == 1
    assert buffer.latest().received_monotonic == 1.
    assert buffer.put(sdk_message(), 5.)
    worker.join(timeout=1.)
    assert not worker.is_alive()
    assert len(notifications) == 1
    assert result[0].sequence == 2


def test_wait_times_out_without_new_frame():
    buffer = LatestMessageBuffer()
    started = time.monotonic()
    assert buffer.wait_for_sample(timeout=.01) is None
    assert time.monotonic() - started >= .009


def test_close_wakes_waiters_and_permanently_rejects_new_frames(monkeypatch):
    buffer = LatestMessageBuffer()
    assert buffer.put(sdk_message(), 1.)
    worker, result = start_buffer_waiter(monkeypatch, buffer, after_sequence=1)
    buffer.close()
    worker.join(timeout=1.)
    assert not worker.is_alive()
    assert result == [None]
    assert buffer.wait_for_sample(timeout=0.) is None
    assert not buffer.put(sdk_message(), 2.)
    assert buffer.latest().sequence == 1
    assert buffer.status()["accepted"] == 1
    buffer.close()


def test_simultaneous_wait_and_arrival_never_lose_a_frame():
    buffer = LatestMessageBuffer()
    turn = threading.Barrier(2)
    failures = []

    def producer():
        try:
            for sequence in range(1, 51):
                turn.wait(timeout=2.)
                assert buffer.put(sdk_message(), float(sequence))
                turn.wait(timeout=2.)
        except BaseException as exc:
            failures.append(exc)

    worker = threading.Thread(target=producer, daemon=True)
    worker.start()
    for sequence in range(1, 51):
        turn.wait(timeout=2.)
        sample = buffer.wait_for_sample(after_sequence=sequence - 1, timeout=1.)
        assert sample is not None
        assert sample.sequence == sequence
        turn.wait(timeout=2.)
    worker.join(timeout=1.)
    assert not worker.is_alive()
    assert not failures


def test_only_valid_right_hand_can_establish_identity():
    buffer = LatestMessageBuffer()
    bad = sdk_message(glove_id=7)
    bad["raw_nodes"][8]["parent_node_id"] = 16
    assert not buffer.put(bad, 1.)
    assert not buffer.put(sdk_message(glove_id=8, side="Left"), 2.)
    assert buffer.status()["selected_glove_id"] is None
    assert buffer.put(sdk_message(glove_id=-45), 3.)
    assert not buffer.put(sdk_message(glove_id=-46), 4.)
    sample = buffer.latest()
    assert sample.received_monotonic == 3.
    assert sample.message["glove_id"] == -45
    assert sample.sequence == 1
    state = buffer.status()
    assert (state["received"], state["accepted"], state["rejected"]) == (4, 1, 3)
    assert "mismatch" in state["last_error"]


def test_explicit_negative_glove_filter_and_backward_time():
    buffer = LatestMessageBuffer(glove_id=-45)
    assert not buffer.put(sdk_message(glove_id=45), 1.)
    assert buffer.put(sdk_message(glove_id=-45), 2.)
    assert not buffer.put(sdk_message(glove_id=-45), 1.9)
    assert not buffer.put(sdk_message(glove_id=-45), 2.)
    assert buffer.latest().received_monotonic == 2.
    assert "Out-of-order" in buffer.status()["last_error"]
    assert buffer.put(sdk_message(glove_id=-45), 3.)
    assert buffer.status()["last_error"] is None
    assert buffer.status()["rejected"] == 3


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -1., True, "123"])
def test_invalid_receipt_time_rejected_without_identity_lock(value):
    buffer = LatestMessageBuffer()
    assert not buffer.put(sdk_message(), value)
    assert buffer.latest() is None
    assert buffer.status()["selected_glove_id"] is None


@pytest.mark.parametrize("value", [True, 2**31, -(2**31)-1, 1.5, "-3"])
def test_invalid_glove_filter_rejected(value):
    with pytest.raises(ValueError, match="signed 32-bit"):
        LatestMessageBuffer(glove_id=value)


def test_buffer_uses_monotonic_clock(monkeypatch):
    monkeypatch.setattr(ros_input.time, "monotonic", lambda: 15.25)
    buffer = LatestMessageBuffer()
    assert buffer.put(sdk_message())
    assert buffer.latest().received_monotonic == 15.25


def test_threaded_latest_reads_are_consistent():
    buffer = LatestMessageBuffer()
    finished = threading.Event()
    failure = []

    def writer():
        try:
            for i in range(1, 301):
                frame = sdk_message()
                frame["ergonomics"][0]["value"] = float(i)
                assert buffer.put(frame, float(i))
        except BaseException as exc:
            failure.append(exc)
        finally:
            finished.set()

    worker = threading.Thread(target=writer)
    worker.start()
    previous = 0
    while not finished.is_set():
        sample = buffer.latest()
        if sample:
            assert sample.sequence >= previous
            assert sample.sequence == sample.received_monotonic == sample.message["ergonomics"][0]["value"]
            previous = sample.sequence
    worker.join(timeout=1.)
    assert not failure
    assert buffer.latest().sequence == 300


@pytest.fixture
def fake_ros(monkeypatch):
    state = dict(nodes=[], contexts=[], executors=[], qos=[])

    class Context:
        def __init__(self):
            self.active = False
            state["contexts"].append(self)

        def init(self, args, domain_id):
            self.active, self.domain = True, 25 if domain_id is None else domain_id

        def get_domain_id(self):
            return self.domain

        def ok(self):
            return self.active

        def shutdown(self):
            self.active = False

    class Node:
        def __init__(self, name, **kwargs):
            self.kwargs, self.destroyed = kwargs, False
            self.subscriptions = {}
            state["nodes"].append(self)

        def create_subscription(self, message_type, topic, callback, qos):
            self.message_type, self.topic, self.callback, self.qos = message_type, topic, callback, qos
            self.subscriptions[topic] = callback
            return object()

        def create_publisher(self, *args, **kwargs):
            raise AssertionError("Subscriber code must not create publishers")

        def destroy_node(self):
            self.destroyed = True

    class Executor:
        def __init__(self, context):
            self.wakeup, self.closed, self.failure = threading.Event(), False, None
            state["executors"].append(self)

        def add_node(self, node):
            self.node = node

        def spin_once(self, timeout_sec):
            self.wakeup.wait(timeout_sec)
            self.wakeup.clear()
            if self.failure:
                raise self.failure

        def wake(self):
            self.wakeup.set()

        def shutdown(self, timeout_sec):
            self.closed = True

    class QoSProfile:
        def __init__(self, **kwargs):
            self.values = kwargs
            state["qos"].append(self)

    modules = {
        "rclpy": {}, "rclpy.context": dict(Context=Context),
        "rclpy.executors": dict(SingleThreadedExecutor=Executor), "rclpy.node": dict(Node=Node),
        "rclpy.qos": dict(QoSProfile=QoSProfile, ReliabilityPolicy=SimpleNamespace(BEST_EFFORT="be"),
                          HistoryPolicy=SimpleNamespace(KEEP_LAST="last"), DurabilityPolicy=SimpleNamespace(VOLATILE="volatile")),
        "manus_ros2_msgs": {}, "manus_ros2_msgs.msg": dict(ManusGlove=SimpleNamespace),
    }
    for name, members in modules.items():
        module = ModuleType(name)
        vars(module).update(members)
        monkeypatch.setitem(sys.modules, name, module)
    return state


def test_subscriber_context_lifecycle_qos_and_callback(fake_ros):
    subscriber = ManusSubscriber(topic="/custom_manus", glove_id=-12, domain_id=27)
    assert subscriber.status()["running"] is False
    with subscriber as active:
        assert active.start() is active
        node = fake_ros["nodes"][0]
        assert node.kwargs["enable_rosout"] is False
        assert node.kwargs["start_parameter_services"] is False
        assert node.topic == "/custom_manus"
        assert node.qos.values == dict(history="last", depth=1, reliability="be", durability="volatile")
        node.callback(object_message(sdk_message(glove_id=-12)))
        sample = active.latest()
        assert sample.message["glove_id"] == -12
        assert sample.received_monotonic <= time.monotonic()
        assert active.status()["domain_id"] == 27
        assert active.status()["running"] is True
        assert active.status()["robot_command_publishers"] == 0
        assert active.wait_for_sample(timeout=0.) == sample
    assert fake_ros["executors"][0].closed
    assert fake_ros["nodes"][0].destroyed
    assert not fake_ros["contexts"][0].active
    assert subscriber.status()["closed"]
    subscriber.close()
    with pytest.raises(RuntimeError, match="closed"):
        subscriber.start()


def test_conversion_failure_is_visible_and_does_not_refresh_latest(fake_ros):
    with ManusSubscriber() as subscriber:
        callback = fake_ros["nodes"][0].callback
        callback(object_message(sdk_message()))
        previous = subscriber.latest()
        callback(SimpleNamespace(glove_id=1))
        assert subscriber.latest() == previous
        state = subscriber.status()
        assert (state["received"], state["accepted"], state["rejected"]) == (2, 1, 1)
        assert "conversion failed" in state["last_error"]


def test_executor_error_is_reported_and_close_still_cleans_up(fake_ros):
    with ManusSubscriber() as subscriber:
        executor = fake_ros["executors"][0]
        executor.failure = RuntimeError("test DDS failure")
        executor.wake()
        deadline = time.monotonic() + 2.
        while subscriber.status()["thread_error"] is None and time.monotonic() < deadline:
            time.sleep(.005)
        assert "test DDS failure" in subscriber.status()["thread_error"]
    assert executor.closed


def test_subscriber_close_wakes_reader(fake_ros, monkeypatch):
    subscriber = ManusSubscriber().start()
    worker, result = start_buffer_waiter(monkeypatch, subscriber.buffer)
    subscriber.close()
    worker.join(timeout=1.)
    assert not worker.is_alive()
    assert result == [None]
    assert subscriber.status()["closed"]


def test_executor_error_wakes_reader_with_error_available(fake_ros, monkeypatch):
    with ManusSubscriber() as subscriber:
        worker, result = start_buffer_waiter(monkeypatch, subscriber.buffer)
        executor = fake_ros["executors"][0]
        executor.failure = RuntimeError("test DDS wait failure")
        executor.wake()
        worker.join(timeout=1.)
        assert not worker.is_alive()
        assert result == [None]
        assert "test DDS wait failure" in subscriber.status()["thread_error"]
        assert subscriber.wait_for_sample(timeout=0.) is None


def test_unexpected_context_shutdown_wakes_reader_with_error(fake_ros, monkeypatch):
    with ManusSubscriber() as subscriber:
        worker, result = start_buffer_waiter(monkeypatch, subscriber.buffer)
        fake_ros["contexts"][0].shutdown()
        fake_ros["executors"][0].wake()
        worker.join(timeout=1.)
        assert not worker.is_alive()
        assert result == [None]
        assert "stopped unexpectedly" in subscriber.status()["thread_error"]


def test_module_does_not_import_ros_until_start(monkeypatch):
    for name in list(sys.modules):
        if name == "rclpy" or name.startswith("rclpy.") or name == "manus_ros2_msgs" or name.startswith("manus_ros2_msgs."):
            monkeypatch.delitem(sys.modules, name)
    importlib.reload(ros_input)
    assert "rclpy" not in sys.modules
    buffer = ros_input.LatestMessageBuffer()
    assert buffer.put(sdk_message())


def test_missing_ros_error_is_actionable(monkeypatch):
    monkeypatch.setitem(sys.modules, "rclpy", None)
    subscriber = ManusSubscriber()
    with pytest.raises(RuntimeError, match="Source /opt/ros/humble/setup.bash"):
        subscriber.start()
    assert subscriber.status()["closed"]
    assert subscriber.status()["thread_error"]


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('topic', ['/manus_glove_0', '/manus_glove_1'])
def test_fixed_topics_route_by_message_side_and_ignore_other_hand(fake_ros, side, topic):
    other = 'left' if side == 'right' else 'right'
    with ManusSubscriber(hand_side=side) as subscriber:
        callbacks = fake_ros['nodes'][0].subscriptions
        assert set(callbacks) == {'/manus_glove_0', '/manus_glove_1'}
        callbacks[topic](object_message(sdk_message(side=other.title())))
        assert subscriber.latest() is None
        assert subscriber.status()['selected_glove_id'] is None
        callbacks[topic](object_message(sdk_message(side=side.title())))
        previous = subscriber.latest()
        assert previous.message['side'].lower() == side
        for callback in callbacks.values():
            callback(object_message(sdk_message(side=other.title())))
        assert subscriber.latest() == previous
        assert subscriber.status()['last_error'] is None
        assert subscriber.status()['topics'] == ['/manus_glove_0', '/manus_glove_1']
