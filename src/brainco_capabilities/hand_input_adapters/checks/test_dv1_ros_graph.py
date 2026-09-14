"""ROS transport smoke check, opt in on a dedicated non-hardware domain."""

import os
from pathlib import Path
import time
import signal
import subprocess

import pytest


@pytest.mark.skipif(os.environ.get('DV1_ROS_GRAPH_TEST') != '1', reason='Opt-in ROS graph check')
def test_left_joint_state_reaches_canonical_without_pose_publisher():
    assert os.environ.get('ROS_DOMAIN_ID') == '126', 'Use isolated test domain 126'
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseArray
    from hand_teleop_msgs.msg import HandKinematics
    from hand_input_adapters.humandex_adapter_node import HumanDexHandAdapter

    root = Path(__file__).resolve().parents[1]
    urdf = Path(os.environ['DV1_SDK_PATH']) / 'description/urdf/Revo_Human_DV1_URDF_Bimanual.urdf'
    rclpy.init(args=['--ros-args', '--params-file', str(root / 'config/dv1_left.yaml'),
                    '-p', f'urdf_path:={urdf}'])
    worker = HumanDexHandAdapter()
    probe = rclpy.create_node('dv1_synthetic_sdk')
    executor = SingleThreadedExecutor()
    executor.add_node(worker)
    executor.add_node(probe)
    canonical, poses, joints = [], [], []
    probe.create_subscription(HandKinematics, '/hand_kinematics/left', canonical.append, 10)
    probe.create_subscription(PoseArray, '/humandex_left/eef_pose', poses.append, 10)
    probe.create_subscription(JointState, '/humandex_left/fk_joint_states', joints.append, 10)
    pub = probe.create_publisher(JointState, '/humandex_left/joint_states', 10)
    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not (canonical and poses and joints):
            msg = JointState()
            msg.header.stamp = probe.get_clock().now().to_msg()
            msg.header.frame_id = 'left_palm_link'
            msg.name = list(worker.joint_fk.joint_names)
            msg.position = [0.3] * 21
            pub.publish(msg)
            for _ in range(5):
                executor.spin_once(timeout_sec=.01)
        assert canonical and poses and joints
        stamp = canonical[0].header.stamp
        matched_pose = next(p for p in poses if p.header.stamp == stamp)
        matched_joint = next(j for j in joints if j.header.stamp == stamp)
        assert len(matched_pose.poses) == 5 and len(matched_joint.position) == 21
        assert len(canonical[0].joint_names) == 25
        assert canonical[0].side == HandKinematics.LEFT
        assert worker.pose_cache == {} and worker.joint_cache == {}
    finally:
        executor.shutdown()
        worker.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(os.environ.get('DV1_ROS_GRAPH_TEST') != '1', reason='Opt-in ROS graph check')
def test_installed_launch_produces_left_retarget_commands(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '126', 'Use isolated test domain 126'
    import rclpy
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PointStamped
    from revohuman_kinematics.core import JOINT_KEYS

    rclpy.init(args=[])
    probe = rclpy.create_node('dv1_launch_probe')
    targets, ik_targets = [], []
    pub = probe.create_publisher(JointState, '/humandex_left/joint_states', 10)
    probe.create_subscription(JointState, '/revo2_left/revo2_pid_controller/target_joint_states', targets.append, 10)
    probe.create_subscription(PointStamped, '/revo2_left/retarget/debug/thumb_ik_target', ik_targets.append, 10)
    logfile = tmp_path / 'launch.log'
    with logfile.open('w') as output:
        process = subprocess.Popen([
            'ros2', 'launch', 'revo2_teleop_bringup', 'dv1_sdk.launch.py',
            'hand_mode:=left', f'urdf_path:={os.environ["DV1_SDK_PATH"]}/description/urdf/Revo_Human_DV1_URDF_Bimanual.urdf',
            'launch_retarget:=true', 'launch_revo2_driver:=false'],
            stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline and not (targets and ik_targets):
                if process.poll() is not None:
                    break
                msg = JointState()
                msg.header.stamp = probe.get_clock().now().to_msg()
                msg.header.frame_id = 'left_palm_link'
                msg.name = [f'left_{k.rsplit("_", 1)[0]}_{k.rsplit("_", 1)[1].upper()}_joint' for k in JOINT_KEYS]
                msg.position = [0.4] * 21
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
