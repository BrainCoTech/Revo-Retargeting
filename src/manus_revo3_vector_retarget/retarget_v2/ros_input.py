"""Receive validated selected-hand SDK skeletons without robot command publishers.

Importing this module needs no ROS installation. ``LatestMessageBuffer`` is
also usable with a synthetic source. Receipt time is host ``time.monotonic``;
the Manus interface does not supply a hardware acquisition timestamp.
"""
from __future__ import annotations

from collections import Counter
from copy import deepcopy
from dataclasses import dataclass
import math
import threading
import time
from uuid import uuid4

from .input import decode, normalize_hand_side


MANUS_TOPICS = ("/manus_glove_0", "/manus_glove_1")


def _glove_id(value):
    if isinstance(value, bool) or not isinstance(value, int) or not -(2**31) <= value < 2**31:
        raise ValueError("glove_id must be a signed 32-bit integer; negative Manus IDs are valid")
    return value


@dataclass(frozen=True)
class LatestSample:
    sequence: int
    received_monotonic: float
    message: dict


class LatestMessageBuffer:
    """One validated sample, never an accumulated queue.

    ``glove_id=None`` locks onto the first valid skeleton for ``hand_side``. Different
    gloves and invalid samples do not replace the latest sample or refresh its
    freshness timestamp. Each returned message is an isolated deep copy.
    """

    def __init__(self, glove_id=None, hand_side="right"):
        self.hand_side = normalize_hand_side(hand_side)
        self._selected_glove_id = None if glove_id is None else _glove_id(glove_id)
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._closed = False
        self._sample = None
        self._received = self._accepted = self._rejected = 0
        self._reasons = Counter()
        self._last_error = None

    def reject(self, reason):
        """Count a callback conversion failure as a rejected received message."""
        with self._lock:
            if self._closed:
                return
            self._received += 1
            self._reject_locked(str(reason))

    def _reject_locked(self, reason):
        self._rejected += 1
        self._reasons[reason] += 1
        self._last_error = reason

    def put(self, message, received_monotonic=None):
        received = time.monotonic() if received_monotonic is None else received_monotonic
        try:
            if isinstance(received, bool) or not isinstance(received, (int, float)) or not math.isfinite(received) or received < 0:
                raise ValueError("Receipt time must be finite nonnegative monotonic seconds")
            frame = deepcopy(message)
            glove_id = _glove_id(frame["glove_id"])
            if not isinstance(frame["side"], str) or frame["side"].lower() != self.hand_side:
                raise ValueError(f"Wrong hand side: expected {self.hand_side.title()}")
            decode(frame, self.hand_side)  # Validate topology before first-frame identity lock.
        except (KeyError, TypeError, ValueError, AttributeError) as exc:
            self.reject(f"Invalid Manus frame: {exc}")
            return False
        with self._lock:
            if self._closed:
                return False
            self._received += 1
            if self._selected_glove_id is not None and glove_id != self._selected_glove_id:
                self._reject_locked(f"Glove ID mismatch: expected {self._selected_glove_id}, received {glove_id}")
                return False
            if self._sample is not None and received <= self._sample.received_monotonic:
                self._reject_locked("Out-of-order monotonic receipt time")
                return False
            self._selected_glove_id = glove_id
            self._accepted += 1
            self._sample = LatestSample(self._accepted, float(received), frame)
            self._last_error = None
            self._condition.notify_all()
            return True

    def _has_sample_locked(self, after_sequence):
        return self._sample is not None and (after_sequence is None or self._sample.sequence > after_sequence)

    def _copy_sample_locked(self):
        return LatestSample(self._sample.sequence, self._sample.received_monotonic,
                            deepcopy(self._sample.message))

    def latest(self, after_sequence=None):
        with self._lock:
            return self._copy_sample_locked() if self._has_sample_locked(after_sequence) else None

    def wait_for_sample(self, after_sequence=None, timeout=None):
        """Wait for a newer accepted frame and return an isolated latest sample.

        A frame received before this call returns immediately. Sequence checking
        and waiting share the producer's lock, so an arrival cannot be lost
        between those steps. Bursts replace the sample instead of queueing it.
        Timeout or final closure returns ``None``; closure takes precedence over
        an unread sample. ``latest()`` remains available for inspection afterward.
        """
        with self._condition:
            ready = self._condition.wait_for(
                lambda: self._closed or self._has_sample_locked(after_sequence), timeout=timeout)
            if not ready or self._closed:
                return None
            return self._copy_sample_locked()

    def close(self):
        """Permanently stop accepting frames and wake every blocked reader."""
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    def status(self):
        with self._lock:
            return dict(received=self._received, accepted=self._accepted, rejected=self._rejected,
                        selected_glove_id=self._selected_glove_id, hand_side=self.hand_side,
                        latest_sequence=None if self._sample is None else self._sample.sequence,
                        last_received_monotonic=None if self._sample is None else self._sample.received_monotonic,
                        rejected_reasons=dict(self._reasons), last_error=self._last_error)


def ros_message_to_dict(message):
    """Explicit conversion of manus_ros2_msgs/msg/ManusGlove and its raw fields."""
    def quaternion(q):
        return {axis: float(getattr(q, axis)) for axis in "xyzw"}

    def pose(p):
        return dict(position={axis: float(getattr(p.position, axis)) for axis in "xyz"},
                    orientation=quaternion(p.orientation))

    return dict(
        glove_id=int(message.glove_id), side=str(message.side), raw_node_count=int(message.raw_node_count),
        raw_nodes=[dict(node_id=int(n.node_id), parent_node_id=int(n.parent_node_id),
                        joint_type=str(n.joint_type), chain_type=str(n.chain_type), pose=pose(n.pose))
                   for n in message.raw_nodes],
        ergonomics_count=int(message.ergonomics_count),
        ergonomics=[dict(type=str(e.type), value=float(e.value)) for e in message.ergonomics],
        raw_sensor_orientation=quaternion(message.raw_sensor_orientation),
        raw_sensor_count=int(message.raw_sensor_count), raw_sensor=[pose(p) for p in message.raw_sensor],
    )


def _ros_dependencies():
    try:
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from manus_ros2_msgs.msg import ManusGlove
    except (ImportError, OSError) as exc:
        raise RuntimeError(
            "ROS2 Manus input is unavailable. Source /opt/ros/humble/setup.bash and the "
            "Manus workspace install/setup.bash, and use its Python 3.10 environment. "
            f"Required message: manus_ros2_msgs/msg/ManusGlove. Import error: {exc}"
        ) from exc
    return Context, SingleThreadedExecutor, Node, QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE), ManusGlove


class ManusSubscriber:
    """Fixed MANUS input topics, private ROS context and one executor thread.

    Both numbered topics are subscribed by default, as in the legacy backend.
    The message side chooses the hand; the SDK discovery index does not.
    An explicit topic remains available for embedded callers and ROS tests.

    Start is idempotent while running. Close is idempotent and final; make a new
    instance to reconnect. Executor failures are available as ``thread_error``
    in status; callers must still enforce sample age with their own watchdog.
    The standard ROS Node may publish /parameter_events internally. No robot
    command publisher is created; rosout and parameter services are disabled.
    """

    def __init__(self, topic=None, glove_id=None, domain_id=None, hand_side="right"):
        if topic is not None and (not isinstance(topic, str) or not topic.strip()):
            raise ValueError("Manus topic must be a nonempty string")
        if domain_id is not None and (isinstance(domain_id, bool) or not isinstance(domain_id, int) or domain_id < 0):
            raise ValueError("ROS domain ID must be a nonnegative integer")
        self.topic, self.domain_id = topic, domain_id
        self.topics = MANUS_TOPICS if topic is None else (topic,)
        self.buffer = LatestMessageBuffer(glove_id, hand_side)
        self._lifecycle = threading.RLock()
        self._stop = threading.Event()
        self._context = self._node = self._executor = self._thread = None
        self._subscriptions = []
        self._closed = False
        self._thread_error = None

    def start(self):
        with self._lifecycle:
            if self._closed:
                raise RuntimeError("Manus subscriber is closed; create a new instance to reconnect")
            if self._thread is not None:
                if self._thread.is_alive():
                    return self
                raise RuntimeError(f"Manus executor stopped: {self._thread_error or 'unknown reason'}")
            try:
                Context, Executor, Node, qos, Message = _ros_dependencies()
                self._context = Context()
                # Context.init avoids installing process-wide ROS signal handlers.
                self._context.init(args=[], domain_id=self.domain_id)
                self.domain_id = self._context.get_domain_id()
                self._node = Node(f"independent_manus_input_{uuid4().hex[:10]}", context=self._context,
                                  use_global_arguments=False, enable_rosout=False, start_parameter_services=False)
                self._subscriptions = [self._node.create_subscription(Message, topic, self._callback, qos)
                                       for topic in self.topics]
                self._executor = Executor(context=self._context)
                self._executor.add_node(self._node)
                self._thread = threading.Thread(target=self._spin, name="manus-latest-input", daemon=True)
                self._thread.start()
            except Exception as exc:
                self._thread_error = f"ROS startup failed: {exc}"
                self.close()
                raise RuntimeError(self._thread_error) from exc
            return self

    def _callback(self, message):
        received = time.monotonic()  # Stamp immediately, before conversion.
        try:
            side = getattr(message, 'side', '')
            # Other-hand traffic is expected on the fixed pair of topics.
            # Ignore it without locking identity or refreshing input freshness.
            if isinstance(side, str) and side.lower() in ('left', 'right') and side.lower() != self.buffer.hand_side:
                return
            converted = ros_message_to_dict(message)
        except (AttributeError, TypeError, ValueError) as exc:
            self.buffer.reject(f"Manus message conversion failed: {exc}")
            return
        self.buffer.put(converted, received)

    def _spin(self):
        try:
            while not self._stop.is_set() and self._context.ok():
                self._executor.spin_once(timeout_sec=.1)
        except Exception as exc:
            if not self._stop.is_set():
                with self._lifecycle:
                    self._thread_error = f"Manus ROS executor failed: {type(exc).__name__}: {exc}"
        finally:
            with self._lifecycle:
                if not self._stop.is_set() and self._thread_error is None:
                    self._thread_error = "Manus ROS executor stopped unexpectedly"
            self.buffer.close()

    def latest(self, after_sequence=None):
        return self.buffer.latest(after_sequence)

    def wait_for_sample(self, after_sequence=None, timeout=None):
        return self.buffer.wait_for_sample(after_sequence, timeout)

    def status(self):
        result = self.buffer.status()
        with self._lifecycle:
            result.update(topic=self.topic, topics=list(self.topics), domain_id=self.domain_id, closed=self._closed,
                          running=self._thread is not None and self._thread.is_alive() and not self._stop.is_set(),
                          thread_error=self._thread_error, message_type="manus_ros2_msgs/msg/ManusGlove",
                          qos=dict(reliability="best_effort", depth=1, durability="volatile"),
                          robot_command_publishers=0, standard_ros_parameter_events=True)
        return result

    def close(self):
        # No waiting while holding the lifecycle lock: the worker may need it
        # to report a concurrent executor failure before it exits.
        with self._lifecycle:
            if self._closed:
                return
            self._closed = True
            self._stop.set()
            executor, thread = self._executor, self._thread
        self.buffer.close()
        if executor is not None:
            executor.wake()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.)
            if thread.is_alive():
                raise RuntimeError("Manus executor did not stop within 2 seconds")
        with self._lifecycle:
            if self._executor is not None:
                self._executor.shutdown(timeout_sec=1.)
            if self._node is not None:
                self._node.destroy_node()
            if self._context is not None and self._context.ok():
                self._context.shutdown()
            self._subscriptions.clear()
            self._node = self._executor = self._context = None

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
