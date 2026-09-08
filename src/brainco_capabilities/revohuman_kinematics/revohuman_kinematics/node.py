"""Convert one SDK raw observation into canonical hand kinematics."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from hand_teleop_msgs.msg import HandKinematics
from revohuman_msgs.msg import RawFrame
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .core import HandModel, load_config


class FrameGate:
    """Reject invalid, duplicate, stale or reordered observations."""
    def __init__(self, side, max_age_sec):
        self.side = side
        self.max_age_ns = int(max_age_sec * 1e9)
        if self.max_age_ns <= 0:
            raise ValueError("max_age_sec must be positive")
        self.last_stamp = 0
        self.last_key = None

    def check(self, msg, now_ns):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if msg.side != self.side:
            raise ValueError("wrong hand side")
        if stamp <= 0 or stamp <= self.last_stamp or not 0 <= now_ns - stamp <= self.max_age_ns:
            raise ValueError("stale, future, zero or reordered timestamp")
        mask = (1 << 21) - 1
        if (msg.encoder_valid_mask & mask != mask or
                (msg.encoder_offline_mask | msg.encoder_reconnecting_mask) & mask):
            raise ValueError("invalid/offline/reconnecting encoder")
        key = (msg.session_id, msg.source_generation, msg.encoder_sequence, msg.encoder_device_tick)
        if key == self.last_key:
            raise ValueError("encoder sample has not updated")
        return stamp, key

    def accept(self, stamp, key):
        self.last_stamp, self.last_key = stamp, key


class KinematicsNode(Node):
    def __init__(self):
        super().__init__("revohuman_kinematics")
        share = Path(get_package_share_directory("revohuman_kinematics"))
        for name, default in (("side", "left"), ("config_file", str(share / "config/kinematics.yaml")),
                              ("urdf_path", str(share / "urdf/revohuman_dv1_kinematics.urdf")),
                              ("max_age_sec", 0.5)):
            self.declare_parameter(name, default)
        self.side = str(self.get_parameter("side").value)
        self.model = HandModel(self.get_parameter("urdf_path").value,
                               load_config(self.get_parameter("config_file").value), self.side)
        self.gate = FrameGate(self.side, float(self.get_parameter("max_age_sec").value))
        self.pub = self.create_publisher(HandKinematics, f"/hand_kinematics/{self.side}", 10)
        self.joint_pub = self.create_publisher(JointState, f"/revohuman/{self.side}/joint_states", 10)
        self.create_subscription(RawFrame, f"/revohuman/{self.side}/raw", self.on_frame, 10)
        self.get_logger().info(f"Waiting for {self.side} raw frames; DV1 FK, tip offsets from YAML")

    def on_frame(self, raw):
        try:
            stamp, key = self.gate.check(raw, self.get_clock().now().nanoseconds)
            q, joints, points = self.model.compute(raw.encoder_angles_deg)
        except ValueError as exc:
            self.get_logger().warning(f"Dropping RevoHuman frame: {exc}", throttle_duration_sec=2.0)
            return
        canonical = HandKinematics()
        canonical.header.stamp = raw.header.stamp
        canonical.header.frame_id = f"revohuman_{self.side}_retarget"
        canonical.side = HandKinematics.LEFT if self.side == "left" else HandKinematics.RIGHT
        canonical.source = "revohuman"
        canonical.joint_names = list(joints)
        canonical.joint_positions_rad = list(joints.values())
        for name, xyz in points.items():
            canonical.landmark_names.append(name)
            canonical.landmarks_m.append(Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])))
        measured = JointState()
        measured.header.stamp = raw.header.stamp
        measured.header.frame_id = f"{self.side}_palm_link"
        measured.name = list(self.model.joint_names)
        measured.position = q.tolist()
        self.gate.accept(stamp, key)
        self.joint_pub.publish(measured)
        self.pub.publish(canonical)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KinematicsNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
