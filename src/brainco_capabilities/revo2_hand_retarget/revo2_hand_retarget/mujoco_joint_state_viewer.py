"""MuJoCo viewer driven by retargeted Revo2 joint targets."""

import argparse
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from ros2_stark_interfaces.msg import SetMotorMulti
except ImportError:  # Optional legacy input mode only.
    SetMotorMulti = None


VALID_HAND_MODES = {"left", "right"}
VALID_INPUT_MODES = {"joint_state", "motor_command"}
DEFAULT_MOTOR_TOPICS = {
    "left": "/set_motor_multi_126",
    "right": "/set_motor_multi_127",
}
MOTOR_ORDER = (
    "thumb_prox",
    "thumb_meta",
    "index_prox",
    "middle_prox",
    "ring_prox",
    "pinky_prox",
)
MOTOR_TO_JOINT_SUFFIX = {
    "thumb_prox": "thumb_proximal_joint",
    "thumb_meta": "thumb_metacarpal_joint",
    "index_prox": "index_proximal_joint",
    "middle_prox": "middle_proximal_joint",
    "ring_prox": "ring_proximal_joint",
    "pinky_prox": "pinky_proximal_joint",
}
MIMIC_SUFFIX = {
    "thumb_distal_joint": ("thumb_proximal_joint", 1.0),
    "index_distal_joint": ("index_proximal_joint", 1.155),
    "middle_distal_joint": ("middle_proximal_joint", 1.155),
    "ring_distal_joint": ("ring_proximal_joint", 1.155),
    "pinky_distal_joint": ("pinky_proximal_joint", 1.155),
}


def _package_root():
    return Path(__file__).resolve().parent


def _default_urdf(hand_mode):
    return _package_root() / "brainco_hand" / f"brainco_{hand_mode}.urdf"


def _joint_qposadr(model, joint_name):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        return None
    return int(model.jnt_qposadr[joint_id])


def _joint_range(model, joint_name):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        return None
    lo, hi = float(model.jnt_range[joint_id, 0]), float(model.jnt_range[joint_id, 1])
    if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
        return 0.0, 0.0
    return lo, hi


def _motor_to_rad(value, joint_range):
    lo, hi = joint_range
    normalized = float(np.clip(value, 0.0, 1000.0)) / 1000.0
    return lo + normalized * (hi - lo)


class MujocoJointStateViewer(Node):
    """Visualizes one Revo2 hand from retargeted joint targets."""

    def __init__(self, hand_mode, urdf_path=None, input_mode="joint_state", topic_name=None):
        super().__init__(f"mujoco_{hand_mode}_hand_viewer")
        self.hand_mode = hand_mode
        self.prefix = f"{hand_mode}_"
        self.input_mode = input_mode
        self.urdf_path = Path(urdf_path).expanduser() if urdf_path else _default_urdf(hand_mode)
        if not self.urdf_path.exists():
            raise FileNotFoundError(f"URDF not found: {self.urdf_path}")

        self.model = mujoco.MjModel.from_xml_path(str(self.urdf_path))
        self.data = mujoco.MjData(self.model)
        self.motor_joint_map = self._build_motor_joint_map()
        self.mimic_map = self._build_mimic_map()

        if input_mode == "joint_state":
            topic_name = topic_name or f"/set_{hand_mode}_hand_joints"
            self.create_subscription(JointState, topic_name, self._joint_state_callback, 10)
        elif input_mode == "motor_command":
            if SetMotorMulti is None:
                raise RuntimeError(
                    "motor_command mode requires ros2_stark_interfaces; "
                    "use joint_state mode with the revo2_driver path."
                )
            topic_name = topic_name or DEFAULT_MOTOR_TOPICS[hand_mode]
            self.create_subscription(SetMotorMulti, topic_name, self._motor_command_callback, 10)
        else:
            raise ValueError(
                f"Invalid input_mode: {input_mode}, expected one of {sorted(VALID_INPUT_MODES)}."
            )

        self.get_logger().info(f"Loaded MuJoCo model: {self.urdf_path}")
        self.get_logger().info(f"Listening to {topic_name} ({input_mode})")

    def _build_motor_joint_map(self):
        mapping = {}
        for short_suffix, joint_suffix in MOTOR_TO_JOINT_SUFFIX.items():
            msg_name = f"{self.hand_mode}_{short_suffix}"
            joint_name = f"{self.prefix}{joint_suffix}"
            qposadr = _joint_qposadr(self.model, joint_name)
            joint_range = _joint_range(self.model, joint_name)
            if qposadr is None or joint_range is None:
                self.get_logger().warning(f"Joint not found in MuJoCo model: {joint_name}")
                continue
            mapping[msg_name] = {
                "joint_name": joint_name,
                "qposadr": qposadr,
                "range": joint_range,
            }
        return mapping

    def _build_mimic_map(self):
        mapping = []
        for mimic_suffix, (source_suffix, multiplier) in MIMIC_SUFFIX.items():
            mimic_name = f"{self.prefix}{mimic_suffix}"
            source_name = f"{self.prefix}{source_suffix}"
            mimic_adr = _joint_qposadr(self.model, mimic_name)
            source_adr = _joint_qposadr(self.model, source_name)
            mimic_range = _joint_range(self.model, mimic_name)
            if mimic_adr is None or source_adr is None or mimic_range is None:
                continue
            mapping.append((mimic_adr, source_adr, float(multiplier), mimic_range))
        return mapping

    def _joint_state_callback(self, msg):
        for name, value in zip(msg.name, msg.position):
            self._apply_joint_rad_value(name, value)

        self._apply_mimic_and_forward()

    def _motor_command_callback(self, msg):
        for short_suffix, value in zip(MOTOR_ORDER, msg.positions):
            self._apply_legacy_motor_value(f"{self.hand_mode}_{short_suffix}", value)

        self._apply_mimic_and_forward()

    def _apply_joint_rad_value(self, name, value):
        joint_info = self.motor_joint_map.get(name)
        if joint_info is None:
            return
        lo, hi = joint_info["range"]
        self.data.qpos[joint_info["qposadr"]] = float(np.clip(value, lo, hi))

    def _apply_legacy_motor_value(self, name, value):
        joint_info = self.motor_joint_map.get(name)
        if joint_info is None:
            return
        self.data.qpos[joint_info["qposadr"]] = _motor_to_rad(value, joint_info["range"])

    def _apply_mimic_and_forward(self):
        for mimic_adr, source_adr, multiplier, mimic_range in self.mimic_map:
            lo, hi = mimic_range
            self.data.qpos[mimic_adr] = np.clip(
                self.data.qpos[source_adr] * multiplier,
                lo,
                hi,
            )

        mujoco.mj_forward(self.model, self.data)


def parse_cli_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Open a MuJoCo viewer for retargeted Revo2 output."
    )
    parser.add_argument(
        "--hand-mode",
        "--hand_mode",
        dest="hand_mode",
        default="right",
        choices=sorted(VALID_HAND_MODES),
        help="Which hand model to visualize.",
    )
    parser.add_argument(
        "--urdf",
        default=None,
        help="Optional explicit URDF path. Defaults to the package brainco_hand URDF.",
    )
    parser.add_argument(
        "--input-mode",
        "--input_mode",
        dest="input_mode",
        default="joint_state",
        choices=sorted(VALID_INPUT_MODES),
        help=(
            "Input topic type. joint_state listens to /set_<hand>_hand_joints; "
            "motor_command is a legacy SetMotorMulti input mode and requires ros2_stark_interfaces."
        ),
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="Optional explicit input topic name.",
    )
    return parser.parse_known_args(argv)


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    cli_args, ros_args = parse_cli_args(argv)

    rclpy.init(args=ros_args)
    node = MujocoJointStateViewer(
        hand_mode=cli_args.hand_mode,
        urdf_path=cli_args.urdf,
        input_mode=cli_args.input_mode,
        topic_name=cli_args.topic,
    )

    try:
        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            while rclpy.ok() and viewer.is_running():
                loop_start = time.monotonic()
                rclpy.spin_once(node, timeout_sec=0.0)
                viewer.sync()
                elapsed = time.monotonic() - loop_start
                time.sleep(max(0.0, 1.0 / 60.0 - elapsed))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
