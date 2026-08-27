"""Synchronize HumanDex joint/FK topics into side-specific HandKinematics."""

from __future__ import annotations

from collections import OrderedDict
import math

from geometry_msgs.msg import PoseArray
from hand_teleop_msgs.msg import HandKinematics
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

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
    """Pair mux outputs by exact source stamp and publish one message per hand."""

    def __init__(self) -> None:
        super().__init__("humandex_hand_adapter")
        self.declare_parameter("hand_mode", "right")
        self.declare_parameter("joint_topic", "/joint_states")
        self.declare_parameter("pose_topic", "/humandex_eef_pose")
        self.declare_parameter("left_output_topic", "/hand_kinematics/left")
        self.declare_parameter("right_output_topic", "/hand_kinematics/right")
        self.declare_parameter("left_frame_id", "hand_retarget_left")
        self.declare_parameter("right_frame_id", "hand_retarget_right")
        self.declare_parameter("sync_queue_size", 50)
        self.declare_parameter("four_finger_open_rad", 0.20943951023931956)
        self.declare_parameter("four_finger_closed_rad", -0.20943951023931956)
        self.declare_parameter("four_finger_flexion_range_rad", 1.4661)

        self.sides = selected_sides(self.get_parameter("hand_mode").value)
        self.frames = {
            side: str(self.get_parameter(f"{side}_frame_id").value)
            for side in self.sides
        }
        self.output_publishers = {
            side: self.create_publisher(
                HandKinematics,
                str(self.get_parameter(f"{side}_output_topic").value),
                20,
            )
            for side in self.sides
        }
        self.queue_size = max(2, int(self.get_parameter("sync_queue_size").value))
        self.four_open = float(self.get_parameter("four_finger_open_rad").value)
        self.four_closed = float(self.get_parameter("four_finger_closed_rad").value)
        self.four_range = float(self.get_parameter("four_finger_flexion_range_rad").value)
        if abs(self.four_closed - self.four_open) < 1e-9 or self.four_range <= 0.0:
            raise ValueError("invalid HumanDex four-finger calibration range")
        self.joint_cache: OrderedDict[int, JointState] = OrderedDict()
        self.pose_cache: OrderedDict[int, PoseArray] = OrderedDict()
        self.published = {side: 0 for side in self.sides}

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_topic").value),
            self._joint_callback,
            20,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("pose_topic").value),
            self._pose_callback,
            20,
        )
        self.get_logger().info(
            "HumanDex adapter waiting for exact-stamp JointState/PoseArray pairs: "
            f"hand_mode={','.join(self.sides)}"
        )

    def _joint_callback(self, message: JointState) -> None:
        self._cache(self.joint_cache, stamp_key(message.header), message)
        self._try_publish(stamp_key(message.header))

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
            output = HandKinematics()
            output.header.stamp = pose_array.header.stamp
            output.header.frame_id = self.frames[side]
            output.side = side_value(side)
            output.source = "humandex"

            for name, value in zip(joint_state.name, joint_state.position):
                canonical = humandex_joint_name(name, side)
                if canonical is not None and math.isfinite(value):
                    append_joint(output, canonical, value)

            measured = dict(zip(output.joint_names, output.joint_positions_rad))
            for finger in ("index", "middle", "ring", "little"):
                pip = measured.get(f"{finger}_pip")
                if pip is None:
                    continue
                normalized = min(
                    1.0,
                    max(0.0, (pip - self.four_open) / (self.four_closed - self.four_open)),
                )
                append_joint(output, f"{finger}_flexion", normalized * self.four_range)

            for name, pose in zip(FINGERTIP_ORDER, poses_by_side[side]):
                append_landmark(
                    output,
                    name,
                    finite_point(pose.position.x, pose.position.y, pose.position.z),
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
    node = HumanDexHandAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
