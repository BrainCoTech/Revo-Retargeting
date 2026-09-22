"""ROS transport smoke check, opt in on a dedicated non-hardware domain."""

import os
from pathlib import Path
import time
import signal
import subprocess

import pytest


SDK_NAMES = (
    'index_DIP_joint', 'index_PIP_joint', 'index_MCP_joint', 'index_MPR_joint',
    'middle_DIP_joint', 'middle_PIP_joint', 'middle_MCP_joint', 'middle_MPR_joint',
    'ring_DIP_joint', 'ring_PIP_joint', 'ring_MCP_joint', 'ring_MPR_joint',
    'little_DIP_joint', 'little_PIP_joint', 'little_MCP_joint', 'little_MPR_joint',
    'thumb_DIP_joint', 'thumb_PIP_joint', 'thumb_MCP_joint', 'thumb_CMR_joint', 'thumb_CMP_joint',
)


def sdk_sample(side, layout, value):
    if layout == 'sdk_single':
        return [f'{side}_{name}' for name in SDK_NAMES], [value] * 21, f'revohuman_{side}'
    names = [f'{hand}_{name}' for hand in ('left', 'right') for name in SDK_NAMES]
    positions = ([value] * 21 + [-.7] * 21 if side == 'left'
                 else [-.7] * 21 + [value] * 21)
    return names, positions, 'revohuman_pair'


@pytest.mark.skipif(os.environ.get('DV1_ROS_GRAPH_TEST') != '1', reason='Opt-in ROS graph check')
@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('layout', ['sdk_single', 'sdk_pair'])
def test_sdk_joint_state_reaches_canonical_without_pose_publisher(side, layout):
    assert os.environ.get('ROS_DOMAIN_ID') == '126', 'Use isolated test domain 126'
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseArray
    from hand_teleop_msgs.msg import HandKinematics
    from hand_input_adapters.humandex_adapter_node import HumanDexHandAdapter

    root = Path(__file__).resolve().parents[1]
    urdf = Path(os.environ['DV1_SDK_PATH']) / 'description/urdf/Revo_Human_DV1_URDF_Bimanual.urdf'
    topic = f'/revohuman/{"pair" if layout == "sdk_pair" else side}/joint_states'
    rclpy.init(args=['--ros-args', '--params-file', str(root / f'config/dv1_{side}.yaml'),
                    '-p', f'urdf_path:={urdf}', '-p', f'joint_state_layout:={layout}',
                    '-p', f'joint_topic:={topic}'])
    worker = HumanDexHandAdapter()
    probe = rclpy.create_node('dv1_synthetic_sdk', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(worker)
    executor.add_node(probe)
    canonical, poses, joints = [], [], []
    probe.create_subscription(HandKinematics, f'/hand_kinematics/{side}', canonical.append, 10)
    probe.create_subscription(PoseArray, f'/humandex_{side}/eef_pose', poses.append, 10)
    probe.create_subscription(JointState, f'/humandex_{side}/fk_joint_states', joints.append, 10)
    pub = probe.create_publisher(JointState, topic, QoSProfile(
        depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE))
    sent_stamps = set()
    stamp_key = lambda message: (message.header.stamp.sec, message.header.stamp.nanosec)
    matched_canonical = None
    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            msg = JointState()
            msg.header.stamp = probe.get_clock().now().to_msg()
            msg.name, msg.position, msg.header.frame_id = sdk_sample(side, layout, .3)
            sent_stamps.add((msg.header.stamp.sec, msg.header.stamp.nanosec))
            pub.publish(msg)
            for _ in range(5):
                executor.spin_once(timeout_sec=.01)
            # The three DDS subscriptions can finish discovery on different samples.
            poses_by_stamp = {stamp_key(p): p for p in poses}
            joints_by_stamp = {stamp_key(j): j for j in joints}
            matched_canonical = next((c for c in canonical
                                      if stamp_key(c) in poses_by_stamp and stamp_key(c) in joints_by_stamp), None)
            if matched_canonical is not None:
                break
        assert matched_canonical is not None
        stamp = stamp_key(matched_canonical)
        assert stamp in sent_stamps
        matched_pose, matched_joint = poses_by_stamp[stamp], joints_by_stamp[stamp]
        assert len(matched_pose.poses) == 5 and len(matched_joint.position) == 21
        assert len(matched_canonical.joint_names) == 21
        assert matched_canonical.side == (HandKinematics.LEFT if side == 'left' else HandKinematics.RIGHT)
        assert matched_pose.header.frame_id == matched_joint.header.frame_id == f'{side}_palm_link'
        assert list(matched_joint.name) == [f'{side}_{name}' for name in SDK_NAMES]
        assert list(matched_joint.position) == pytest.approx([.3] * 21)
        assert worker.pose_cache == {} and worker.joint_cache == {}
    finally:
        executor.shutdown()
        worker.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(os.environ.get('DV1_ROS_GRAPH_TEST') != '1', reason='Opt-in ROS graph check')
@pytest.mark.parametrize('layout', ['sdk_single', 'sdk_pair'])
def test_installed_launch_produces_left_retarget_commands(tmp_path, layout):
    assert os.environ.get('ROS_DOMAIN_ID') == '126', 'Use isolated test domain 126'
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PointStamped

    rclpy.init(args=[])
    probe = rclpy.create_node('dv1_launch_probe')
    targets, ik_targets = [], []
    topic = f'/revohuman/{"pair" if layout == "sdk_pair" else "left"}/joint_states'
    pub = probe.create_publisher(JointState, topic, QoSProfile(
        depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE))
    probe.create_subscription(JointState, '/revo2_left/revo2_pid_controller/target_joint_states', targets.append, 10)
    probe.create_subscription(PointStamped, '/revo2_left/retarget/debug/thumb_ik_target', ik_targets.append, 10)
    logfile = tmp_path / 'launch.log'
    with logfile.open('w') as output:
        process = subprocess.Popen([
            'ros2', 'launch', 'revo2_teleop_bringup', 'dv1_sdk.launch.py',
            'hand_mode:=left', f'urdf_path:={os.environ["DV1_SDK_PATH"]}/description/urdf/Revo_Human_DV1_URDF_Bimanual.urdf',
            f'joint_state_layout:={layout}',
            'launch_retarget:=true', 'launch_revo2_driver:=false'],
            stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline and not (targets and ik_targets):
                if process.poll() is not None:
                    break
                msg = JointState()
                msg.header.stamp = probe.get_clock().now().to_msg()
                msg.name, msg.position, msg.header.frame_id = sdk_sample('left', layout, .4)
                pub.publish(msg)
                rclpy.spin_once(probe, timeout_sec=.02)
            assert targets and ik_targets, logfile.read_text()[-8000:]
            target = targets[-1]
            assert len(target.position) == 6
            assert all(n.startswith('left_') for n in target.name)
            assert all(0 < float(v) <= 1.4661 for v in target.position[2:])
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
            probe.destroy_node()
            rclpy.shutdown()
