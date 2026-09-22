"""Synchronize HumanDex joint/FK topics into side-specific HandKinematics."""

from __future__ import annotations

from collections import OrderedDict
import math

from geometry_msgs.msg import Pose, PoseArray
from hand_teleop_msgs.msg import HandKinematics
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from .palm_transform import PalmTransform

from .common import (
    append_joint,
    append_landmark,
    finite_point,
    humandex_joint_name,
    selected_sides,
    side_value,
    stamp_key,
    validate_message,
)


FINGERTIP_ORDER = ("index_tip", "middle_tip", "ring_tip", "little_tip", "thumb_tip")


class HumanDexHandAdapter(Node):
    """Pair legacy mux outputs, or calculate DV1 FK directly from JointState."""

    def __init__(self) -> None:
        super().__init__("humandex_hand_adapter")
        self.declare_parameter("input_mode", "paired")
        self.declare_parameter("hand_mode", "right")
        self.declare_parameter("joint_topic", "/joint_states")
        self.declare_parameter("pose_topic", "/humandex_eef_pose")
        self.declare_parameter("left_output_topic", "/hand_kinematics/left")
        self.declare_parameter("right_output_topic", "/hand_kinematics/right")
        self.declare_parameter("left_frame_id", "hand_retarget_left")
        self.declare_parameter("right_frame_id", "hand_retarget_right")
        self.declare_parameter("sync_queue_size", 50)
        self.sides = selected_sides(self.get_parameter("hand_mode").value)
        self.frames = {
            side: str(self.get_parameter(f"{side}_frame_id").value)
            for side in self.sides
        }
        self.palm_transforms = {}
        self.input_frames = {}
        for side in self.sides:
            for key, value in (("palm_translation_m", [0.0, 0.0, 0.0]),
                               ("palm_rpy_rad", [0.0, 0.0, 0.0]), ("input_frame_id", "")):
                self.declare_parameter(f"{side}_{key}", value)
            self.palm_transforms[side] = PalmTransform(
                self.get_parameter(f"{side}_palm_translation_m").value,
                self.get_parameter(f"{side}_palm_rpy_rad").value)
            self.input_frames[side] = self.get_parameter(f"{side}_input_frame_id").value
        self.output_publishers = {
            side: self.create_publisher(
                HandKinematics,
                str(self.get_parameter(f"{side}_output_topic").value),
                20,
            )
            for side in self.sides
        }
        self.queue_size = max(2, int(self.get_parameter("sync_queue_size").value))
        self.joint_cache: OrderedDict[int, JointState] = OrderedDict()
        self.pose_cache: OrderedDict[int, PoseArray] = OrderedDict()
        self.published = {side: 0 for side in self.sides}

        self.joint_fk = None
        input_mode = self.get_parameter("input_mode").value
        if input_mode == "dv1_joint_states":
            if len(self.sides) != 1:
                raise ValueError("DV1 direct input requires one hand per adapter process")
            from revohuman_kinematics.joint_fk import JointStateFK
            from .dv1_joint_state import DV1JointStateInput
            side = self.sides[0]
            for key, value in (("urdf_path", ""), ("tip_offsets_m", [0.0] * 15),
                               ("fk_ema_alpha", 0.2), ("max_age_sec", 0.5),
                               ("joint_state_layout", "sdk_single"), ("source_frame_id", ""),
                               ("fk_joint_topic", f"/humandex_{side}/fk_joint_states"),
                               ("fk_pose_topic", f"/humandex_{side}/eef_pose")):
                self.declare_parameter(key, value)
            self.joint_fk = JointStateFK(
                self.get_parameter("urdf_path").value, side,
                self.get_parameter("tip_offsets_m").value,
                self.get_parameter("fk_ema_alpha").value,
                self.get_parameter("max_age_sec").value)
            self.joint_input = DV1JointStateInput(
                side, self.get_parameter("joint_state_layout").value,
                self.get_parameter("source_frame_id").value)
            self.fk_joint_pub = self.create_publisher(
                JointState, self.get_parameter("fk_joint_topic").value, 20)
            self.fk_pose_pub = self.create_publisher(
                PoseArray, self.get_parameter("fk_pose_topic").value, 20)
        elif input_mode != "paired":
            raise ValueError("input_mode must be paired or dv1_joint_states")

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_topic").value),
            self._joint_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)
            if self.joint_fk is not None else 20,
        )
        if input_mode == "paired":
            self.create_subscription(
                PoseArray,
                str(self.get_parameter("pose_topic").value),
                self._pose_callback,
                20,
            )
        self.get_logger().info(
            f"HumanDex adapter input_mode={input_mode}, "
            f"hand_mode={','.join(self.sides)}"
        )
        if self.joint_fk is not None:
            self.get_logger().info(
                f"DV1 FK from JointState radians: {self.get_parameter('urdf_path').value}; "
                f"layout={self.get_parameter('joint_state_layout').value}, "
                f"source_frame={self.joint_input.source_frame_id}, "
                f"base={self.joint_fk.base_link}, tips=DIP_Link + local offsets")

    def _joint_callback(self, message: JointState) -> None:
        if self.joint_fk is not None:
            self._publish_from_joint_state(message)
            return
        self._cache(self.joint_cache, stamp_key(message.header), message)
        self._try_publish(stamp_key(message.header))

    def _publish_from_joint_state(self, message: JointState) -> None:
        from revohuman_kinematics.joint_fk import quaternion_xyzw
        try:
            names, positions = self.joint_input.normalize(
                message.name, message.position, message.header.frame_id)
            q, transforms = self.joint_fk.compute(
                names, positions, stamp_key(message.header),
                self.get_clock().now().nanoseconds, self.joint_fk.base_link)
        except ValueError as exc:
            self.get_logger().warning(f"Dropping DV1 JointState: {exc}", throttle_duration_sec=2.0)
            return
        measured = JointState()
        measured.header.stamp = message.header.stamp
        measured.header.frame_id = self.joint_fk.base_link
        measured.name = list(self.joint_fk.joint_names)
        measured.position = q.tolist()
        poses = PoseArray()
        poses.header = measured.header
        for transform in transforms:
            pose = Pose()
            pose.position = finite_point(*transform[:3, 3])
            quat = quaternion_xyzw(transform[:3, :3]).tolist()
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
            poses.poses.append(pose)
        self.fk_joint_pub.publish(measured)
        self.fk_pose_pub.publish(poses)
        self._publish_pair(measured, poses)

    def _pose_callback(self, message: PoseArray) -> None:
        self._cache(self.pose_cache, stamp_key(message.header), message)
        self._try_publish(stamp_key(message.header))

    def _cache(self, cache: OrderedDict, key: int, message) -> None:
        if key <= 0:
            self.get_logger().warning(
                "Dropping HumanDex frame without a source timestamp.",
                throttle_duration_sec=2.0,
            )
            return
        cache[key] = message
        cache.move_to_end(key)
        while len(cache) > self.queue_size:
            cache.popitem(last=False)

    def _try_publish(self, key: int) -> None:
        joint_state = self.joint_cache.get(key)
        pose_array = self.pose_cache.get(key)
        if joint_state is None or pose_array is None:
            return
        del self.joint_cache[key]
        del self.pose_cache[key]
        self._publish_pair(joint_state, pose_array)

    @staticmethod
    def _present_sides(joint_state: JointState) -> tuple[str, ...]:
        names = tuple(str(name) for name in joint_state.name)
        return tuple(
            side for side in ("left", "right")
            if any(name.startswith(side + "_") for name in names)
        )

    def _publish_pair(self, joint_state: JointState, pose_array: PoseArray) -> None:
        if len(joint_state.position) < len(joint_state.name):
            self.get_logger().warning(
                "Dropping HumanDex pair with incomplete JointState positions.",
                throttle_duration_sec=2.0,
            )
            return

        present = self._present_sides(joint_state)
        expected_pose_count = 5 * len(present)
        if not present or len(pose_array.poses) != expected_pose_count:
            self.get_logger().warning(
                "Dropping HumanDex pair: joint sides and PoseArray layout disagree "
                f"(sides={present}, poses={len(pose_array.poses)}).",
                throttle_duration_sec=2.0,
            )
            return

        pose_offset = 0
        poses_by_side = {}
        for side in present:
            poses_by_side[side] = pose_array.poses[pose_offset:pose_offset + 5]
            pose_offset += 5

        for side in self.sides:
            if side not in poses_by_side:
                continue
            expected_frame = self.input_frames[side]
            if expected_frame and pose_array.header.frame_id != expected_frame:
                self.get_logger().warning(
                    f"Dropping {side} poses: expected frame {expected_frame}, "
                    f"got {pose_array.header.frame_id!r}.", throttle_duration_sec=2.0)
                continue
            output = HandKinematics()
            output.header.stamp = pose_array.header.stamp
            output.header.frame_id = self.frames[side]
            output.side = side_value(side)
            output.source = "humandex"

            for name, value in zip(joint_state.name, joint_state.position):
                canonical = humandex_joint_name(name, side)
                if canonical is not None and math.isfinite(value):
                    append_joint(output, canonical, value)

            for name, pose in zip(FINGERTIP_ORDER, poses_by_side[side]):
                append_landmark(
                    output,
                    name,
                    finite_point(*self.palm_transforms[side].apply(
                        (pose.position.x, pose.position.y, pose.position.z))),
                )

            reason = validate_message(output)
            if reason is not None or "thumb_tip" not in output.landmark_names:
                self.get_logger().warning(
                    f"Dropping invalid HumanDex {side} frame: {reason or 'missing thumb_tip'}",
                    throttle_duration_sec=2.0,
                )
                continue
            self.output_publishers[side].publish(output)
            self.published[side] += 1
            if self.published[side] == 1:
                self.get_logger().info(
                    f"Published first canonical HumanDex {side} frame with "
                    f"{len(output.joint_names)} joints and {len(output.landmark_names)} landmarks."
                )


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = HumanDexHandAdapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        # SIGINT can invalidate the context while spin() creates its wait set.
        # Runtime errors while the context is still active must remain visible.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
