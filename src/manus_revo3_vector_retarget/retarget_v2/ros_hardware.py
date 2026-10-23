"""Service-controlled MIT output with JointState feedback and a private executor.

Starts disabled. Pause stops publishing; the MIT controller timeout applies its
configured hold behavior. Calibration and IK run outside this executor.
"""
from __future__ import annotations

from copy import deepcopy
import json
import re
import threading
import time


def _new_controller(profile, interpolate=False):
    from .hardware_control import HardwareController
    return HardwareController(profile, interpolate=interpolate)


def _dependencies():
    try:
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, Trigger
        from revo3_mit_controller_msgs.msg import Revo3MITCommand
    except (ImportError, OSError) as exc:
        raise RuntimeError("ROS Revo3 interfaces unavailable. Source ROS Humble and the original workspace "
                           "install/setup.bash with Python 3.10. Need sensor_msgs/JointState, "
                           "std_msgs/String, std_srvs and revo3_mit_controller_msgs/Revo3MITCommand. "
                           f"Import error: {exc}") from exc
    feedback_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    command_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    status_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                           reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    return (Context, SingleThreadedExecutor, Node, feedback_qos, command_qos, status_qos,
            JointState, Revo3MITCommand, String, SetBool, Trigger)


class HardwareTransport:
    """Latest JointState feedback and direct or interpolated MIT publication.

    The solver only calls ``set_target(model_q, received_at)``; the original
    Manus receipt time is preserved for the controller's input-age check.
    By default, each accepted target publishes synchronously in that call using
    the latest feedback cache. With interpolation enabled a 200 Hz timer owns
    publication instead. The direct-mode timer only checks graph/watchdogs.
    ``~/enable_retarget`` enables output only after calibration and a valid target.
    Publisher conflicts and a missing controller subscriber latch a pause;
    recovery always requires an explicit new ``arm()`` attempt.
    """

    GRAPH_PERIOD_S = .1
    MAINTENANCE_PERIOD_S = .05
    DISCOVERY_WAIT_S = 1.

    def __init__(self, profile, domain_id=None, namespace=None, interpolate=False, node_name=None):
        if type(interpolate) is not bool:
            raise ValueError("interpolate must be a boolean")
        if domain_id is not None and (isinstance(domain_id, bool) or not isinstance(domain_id, int) or not 0 <= domain_id <= 232):
            raise ValueError("ROS domain ID must be an integer in 0..232")
        self.hand_side = getattr(profile, "hand_side", "right")
        if self.hand_side not in ("left", "right"):
            raise ValueError("Hardware transport hand_side must be left or right")
        if namespace is None:
            namespace = f"/revo3_{self.hand_side}"
        if not isinstance(namespace, str) or not namespace.strip('/'):
            raise ValueError("Hardware namespace must be nonempty")
        self.profile, self.domain_id = profile, domain_id
        self.interpolate = interpolate
        self.namespace = '/' + namespace.strip('/')
        self.control = _new_controller(profile, interpolate=interpolate)
        self.node_name = node_name if node_name is not None else f"manus_revo3_vector_retarget_{self.hand_side}"
        if not isinstance(self.node_name, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", self.node_name):
            raise ValueError("node_name must be a nonempty ROS node name without a namespace")
        self.node_namespace = "/"
        self.service_prefix = "/" + self.node_name
        self.status_topic = self.service_prefix + "/status"
        self.command_topic = self.namespace + '/joint_forward_mit_controller/commands'
        self.feedback_topic = self.namespace + '/revo3_joint_state/joint_states'
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._context = self._node = self._executor = self._thread = self._timer = self._publisher = None
        self._subscriptions = []
        self._services = []
        self._status_publisher = self._status_timer = self._status_type = None
        self._generation = 0
        self._recalibration_pending = False
        self._retarget_status = dict(phase="WAITING_INPUT", calibrated=False,
                                     calibration_count=0, calibration_frames=120,
                                     glove_id=None, hand_side=self.hand_side, input_error=None,
                                     reason="Waiting for glove input")
        self._message_type = None
        self._closed = self._starting = False
        self._thread_error = self._last_error = self._blocked_reason = None
        self._publisher_count = self._subscriber_count = self._controller_subscriber_count = 0
        self._publisher_nodes = []
        self._own_gid = None
        self._published = 0
        self._last_publish_monotonic = None
        self._last_graph_check = float('-inf')

    def start(self):
        with self._lock:
            if self._closed:
                raise RuntimeError("Hardware transport is closed")
            if self._starting:
                raise RuntimeError("Hardware transport is already starting")
            if self._thread is not None:
                if self._thread.is_alive():
                    return self
                raise RuntimeError(f"Hardware executor stopped: {self._thread_error}")
            self._starting = True
        try:
            (Context, Executor, Node, feedback_qos, command_qos, status_qos,
             JointState, Command, String, SetBool, Trigger) = _dependencies()
            self._context = Context()
            self._context.init(args=[], domain_id=self.domain_id)
            self.domain_id = self._context.get_domain_id()
            self._node = Node(self.node_name, namespace=self.node_namespace, context=self._context,
                              use_global_arguments=False, enable_rosout=False, start_parameter_services=False)
            self._subscriptions = [
                self._node.create_subscription(JointState, self.feedback_topic, self._feedback_callback, feedback_qos),
            ]
            self._executor = Executor(context=self._context)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._spin, name='revo3-hardware-transport', daemon=True)
            self._thread.start()
            # Allow DDS discovery before checking ownership and creating output.
            self._stop.wait(self.DISCOVERY_WAIT_S)
            with self._lock:
                if self._thread_error:
                    raise RuntimeError(self._thread_error)
                self._check_graph_locked(time.monotonic(), before_publisher=True)
                if self._publisher_count:
                    raise RuntimeError(f"Refusing command publisher: existing publishers {self._publisher_nodes}")
                self._message_type = Command
                self._publisher = self._node.create_publisher(Command, self.command_topic, command_qos)
                self._status_type = String
                self._status_publisher = self._node.create_publisher(String, self.status_topic, status_qos)
                self._services = [
                    self._node.create_service(SetBool, '~/enable_retarget', self._enable_callback),
                    self._node.create_service(Trigger, '~/recalibrate', self._recalibrate_callback),
                ]
                period = (1. / self.profile.command.publish_hz if self.interpolate
                          else self.MAINTENANCE_PERIOD_S)
                self._timer = self._node.create_timer(period, self._tick)
                self._status_timer = self._node.create_timer(.5, self._publish_status)
                self._starting = False
            return self
        except Exception as exc:
            with self._lock:
                self._last_error = f"Hardware ROS startup failed: {exc}"
                self._starting = False
            self.close()
            raise RuntimeError(self._last_error) from exc

    @staticmethod
    def _stamp(header):
        return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9

    def _clocks(self):
        received = time.monotonic()
        now_ros = self._node.get_clock().now().nanoseconds * 1e-9
        return received, now_ros

    def _feedback_callback(self, message):
        received, now_ros = self._clocks()
        with self._lock:
            try:
                accepted = self.control.update_feedback(message.name, message.position, message.velocity,
                                                        self._stamp(message.header), received, now_ros, received)
                if not accepted:
                    self._last_error = self.control.status(received).get('last_error') or 'JointState feedback rejected'
            except Exception as exc:
                self._pause_locked(f"JointState callback failed: {type(exc).__name__}: {exc}")

    def _enable_callback(self, request, response):
        with self._lock:
            if request.data:
                response.success = self.arm()
                state = self.snapshot()
                response.message = "Retargeting enabled" if response.success else str(
                    self._last_error or state.get('reason'))
            else:
                self.pause('operator_pause')
                response.success = True
                response.message = "Retargeting paused; calibration retained"
        return response

    def _recalibrate_callback(self, request, response):
        # Invalidates any solve already in progress without waiting for IK.
        with self._lock:
            self.control.clear_target('recalibration')
            self._generation += 1
            self._recalibration_pending = True
            self._last_error = None
            self._retarget_status.update(phase='CALIBRATING', calibrated=False,
                                         calibration_count=0, input_error=None,
                                         reason='Recalibration requested; waiting for new glove frames')
            response.success = True
            response.message = "Recalibration accepted; output paused. Enable again after READY."
        return response

    def take_recalibration_request(self):
        with self._lock:
            if not self._recalibration_pending:
                return None
            self._recalibration_pending = False
            return self._generation

    def update_retarget_status(self, status, generation):
        with self._lock:
            if generation != self._generation or self._closed:
                return False
            self._retarget_status = deepcopy(status)
            return True

    def _publish_status(self):
        with self._lock:
            if self._closed or self._status_publisher is None:
                return
            message = self._status_type()
            message.data = json.dumps(self.snapshot(), allow_nan=False)
            self._status_publisher.publish(message)

    def _pause_locked(self, reason):
        self._blocked_reason = self._last_error = str(reason)
        self.control.pause(str(reason))

    def _check_graph_locked(self, now, before_publisher=False):
        publishers = self._node.get_publishers_info_by_topic(self.command_topic)
        subscribers = self._node.get_subscriptions_info_by_topic(self.command_topic)
        self._last_graph_check = now
        self._publisher_count, self._subscriber_count = len(publishers), len(subscribers)
        self._publisher_nodes = [p.node_namespace.rstrip('/') + '/' + p.node_name for p in publishers]
        self._controller_subscriber_count = sum(s.node_name == 'joint_forward_mit_controller'
                                                and s.node_namespace.rstrip('/') == self.namespace
                                                and s.topic_type == 'revo3_mit_controller_msgs/msg/Revo3MITCommand'
                                                for s in subscribers)
        if before_publisher:
            return
        foreign = []
        own = []
        for endpoint in publishers:
            gid = bytes(endpoint.endpoint_gid)
            if endpoint.node_name == self.node_name and endpoint.node_namespace.rstrip('/') == self.node_namespace.rstrip('/'):
                own.append(gid)
                if self._own_gid is not None and gid != self._own_gid:
                    foreign.append(endpoint)
            else:
                foreign.append(endpoint)
        if len(own) > 1:
            foreign.extend(publishers)
        if self._own_gid is None and len(own) == 1:
            self._own_gid = own[0]
        if foreign:
            names = [p.node_namespace.rstrip('/') + '/' + p.node_name for p in foreign]
            self._pause_locked(f"Competing command publisher detected: {names}; explicit rearm required")
        elif self._controller_subscriber_count < 1:
            self._pause_locked("MIT controller command subscriber is missing; explicit rearm required")
        else:
            # Clearing a graph problem never enables control; a new service call is required.
            if self._blocked_reason is not None:
                self._last_error = None
            self._blocked_reason = None

    def _tick(self):
        with self._lock:
            now = time.monotonic()
            if self._closed:
                return
            try:
                if now - self._last_graph_check >= self.GRAPH_PERIOD_S:
                    self._check_graph_locked(now)
                if not self.interpolate:
                    self.control.check_watchdog(now)
                    return
                if self._publisher is None or self._blocked_reason is not None:
                    return
                self._publish_command_locked(self.control.sample(now), now)
            except Exception as exc:
                self._pause_locked(f"Hardware timer failed: {type(exc).__name__}: {exc}")

    def _publish_command_locked(self, command, now):
        if command is None:
            return
        q, dq = command
        message = self._message_type()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = f'{self.hand_side}_hand_base_link'
        message.joint_names = list(self.profile.hardware_names)
        message.position = [float(value) for value in q]
        message.velocity = [float(value) for value in dq]
        message.effort = [0.] * len(message.joint_names)
        message.kp = [float(value) for value in self.profile.kp]
        message.kd = [float(value) for value in self.profile.kd]
        self._publisher.publish(message)
        self._published += 1
        self._last_publish_monotonic = now

    def _spin(self):
        try:
            while not self._stop.is_set() and self._context.ok():
                self._executor.spin_once(timeout_sec=.05)
        except Exception as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._thread_error = f"Hardware ROS executor failed: {type(exc).__name__}: {exc}"
                    self._pause_locked(self._thread_error)

    def set_target(self, model_q, received_at, generation=None):
        with self._lock:
            if self._closed or (generation is not None and generation != self._generation):
                return False
            accepted = self.control.update_target(model_q, received_at)
            if accepted and not self.interpolate and self._publisher is not None:
                try:
                    now = time.monotonic()
                    if now - self._last_graph_check >= self.GRAPH_PERIOD_S:
                        self._check_graph_locked(now)
                    if self._blocked_reason is None and self.control.enabled:
                        self._publish_command_locked(self.control.sample(now), now)
                except Exception as exc:
                    self._pause_locked(f"Direct MIT publication failed: {type(exc).__name__}: {exc}")
                    return False
            return accepted

    def arm(self):
        with self._lock:
            now = time.monotonic()
            if self._closed or self._publisher is None or self._thread_error:
                self._last_error = "Hardware output is unavailable or closed"
                return False
            if not self._retarget_status.get('calibrated') or self._retarget_status.get('input_error'):
                progress = self._retarget_status
                self._last_error = (f"{progress.get('phase', 'WAITING_INPUT')}: "
                                    f"{progress.get('calibration_count', 0)}/{progress.get('calibration_frames', 120)}; "
                                    f"{progress.get('input_error') or progress.get('reason', 'Calibration not ready')}")
                return False
            # Clear only for this explicit request, then re-check live ownership.
            self._blocked_reason = None
            self._check_graph_locked(now)
            if self._blocked_reason is not None:
                return False
            accepted = self.control.arm(now)
            self._last_error = None if accepted else self.control.status(now).get('last_error') or self.control.status(now).get('reason')
            return accepted

    def pause(self, reason='paused by user'):
        with self._lock:
            self.control.pause(str(reason))
            self._last_error = None

    def snapshot(self):
        with self._lock:
            now = time.monotonic()
            controller = self.control.status(now)
            result = dict(controller)
            if self._blocked_reason is not None:
                result.update(state='fault', enabled=False, reason=self._blocked_reason)
            if self._closed:
                result.update(state='hold', enabled=False, reason='Transport closed; command publishing stopped')
            result.update(controller=controller,
                          interpolate=self.interpolate,
                          output_mode='interpolated' if self.interpolate else 'direct',
                          published=self._published, command_topic=self.command_topic,
                          command_publisher_count=self._publisher_count, command_subscriber_count=self._subscriber_count,
                          controller_subscriber_count=self._controller_subscriber_count,
                          command_publisher_nodes=list(self._publisher_nodes),
                          last_publish_monotonic=self._last_publish_monotonic,
                          transport_blocked_reason=self._blocked_reason,
                          last_error=self._last_error or controller.get('last_error'), thread_error=self._thread_error,
                          closed=self._closed, domain_id=self.domain_id, namespace=self.namespace,
                          hand_side=self.hand_side, node_name=self.node_name,
                          enable_service=self.service_prefix + '/enable_retarget',
                          recalibrate_service=self.service_prefix + '/recalibrate', status_topic=self.status_topic)
            front = deepcopy(self._retarget_status)
            result.update(retarget=front, calibrated=front.get('calibrated', False),
                          calibration_count=front.get('calibration_count', 0),
                          calibration_frames=front.get('calibration_frames', 120),
                          glove_id=front.get('glove_id'), cache_used=front.get('cache_used', False))
            if result['enabled']:
                phase, reason = 'RUNNING', result.get('reason')
            elif not result['calibrated']:
                phase, reason = front.get('phase', 'WAITING_INPUT'), front.get('reason')
            elif front.get('input_error'):
                phase, reason = 'WAITING_INPUT', front['input_error']
            elif not result.get('ready_to_arm'):
                phase, reason = 'BLOCKED', result.get('arming_blocked_by') or result.get('reason')
            elif self._blocked_reason is not None or self._closed:
                phase, reason = 'BLOCKED', result.get('reason')
            elif self._controller_subscriber_count < 1:
                phase, reason = 'BLOCKED', 'Waiting for MIT controller command subscriber'
            else:
                phase, reason = 'READY', 'Ready; call enable_retarget with data: true'
            result.update(phase=phase, reason=reason,
                          ready_to_arm=bool(result.get('ready_to_arm') and result['calibrated']
                                            and not front.get('input_error') and self._blocked_reason is None
                                            and self._controller_subscriber_count >= 1
                                            and not self._closed and not self._thread_error))
            return result

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._stop.set()
            self.control.shutdown()
            for timer in (self._timer, self._status_timer):
                if timer is not None:
                    timer.cancel()
            thread, executor = self._thread, self._executor
        if executor is not None:
            executor.wake()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.)
            if thread.is_alive():
                raise RuntimeError("Hardware executor did not stop within 2 seconds")
        with self._lock:
            if self._executor is not None:
                self._executor.shutdown(timeout_sec=1.)
            if self._node is not None:
                self._node.destroy_node()
            if self._context is not None and self._context.ok():
                self._context.shutdown()
            self._timer = self._publisher = self._node = self._executor = self._context = None
            self._status_timer = self._status_publisher = None
            self._subscriptions.clear()
            self._services.clear()

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
