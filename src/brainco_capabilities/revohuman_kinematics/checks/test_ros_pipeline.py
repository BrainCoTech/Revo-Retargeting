"""Publish actual ROS RawFrame and observe canonical kinematics + JointState."""
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from hand_teleop_msgs.msg import HandKinematics
from revohuman_msgs.msg import RawFrame
from sensor_msgs.msg import JointState

from revohuman_kinematics.node import KinematicsNode


def test_ros_raw_to_kinematics():
    rclpy.init(args=['--ros-args', '-p', 'side:=left'])
    worker = KinematicsNode()
    probe = Node('revohuman_test_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(worker)
    executor.add_node(probe)
    messages, joints = [], []
    probe.create_subscription(HandKinematics, '/hand_kinematics/left', messages.append, 10)
    probe.create_subscription(JointState, '/revohuman/left/joint_states', joints.append, 10)
    pub = probe.create_publisher(RawFrame, '/revohuman/left/raw', 10)
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and pub.get_subscription_count() == 0:
            executor.spin_once(timeout_sec=0.05)
        assert pub.get_subscription_count() > 0
        raw = RawFrame()
        raw.side = 'left'
        raw.encoder_valid_mask = 0x1fffff
        raw.encoder_angles_deg = [45.0] * 21
        while time.monotonic() < deadline and not (messages and joints):
            raw.encoder_sequence += 1
            raw.header.stamp = probe.get_clock().now().to_msg()
            pub.publish(raw)
            for _ in range(4):
                executor.spin_once(timeout_sec=0.02)
        assert messages and joints
        msg = messages[-1]
        assert len(msg.landmarks_m) == 5
        assert len(joints[-1].position) == 21
        angles = dict(zip(msg.joint_names, msg.joint_positions_rad))
        assert np.isclose(angles['index_flexion'], 1.4661 / 2)
        assert msg.header.stamp == joints[-1].header.stamp
        assert msg.source == 'revohuman'
        assert all(np.isfinite([p.x, p.y, p.z]).all() for p in msg.landmarks_m)
    finally:
        executor.shutdown()
        worker.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()
