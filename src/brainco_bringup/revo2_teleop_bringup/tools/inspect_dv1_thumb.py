#!/usr/bin/env python3
"""Read-only thumb monitor: native FK, canonical point, IK target, command FK.

The reported error uses commanded joint angles (including output tuning),
not measured hardware feedback and not the solver's internal residual.
IK/command stamps must be within 50 ms; this is an approximate debug pairing.
"""

import argparse
import csv
from pathlib import Path
import time

from geometry_msgs.msg import PointStamped, PoseArray
from hand_teleop_msgs.msg import HandKinematics
import mujoco
import numpy as np
import rclpy
from sensor_msgs.msg import JointState

import revo2_hand_retarget.retargeters.pose_thumb_retargeter as thumb


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=['left', 'right'], default='left')
    parser.add_argument('--csv', type=Path)
    args = parser.parse_args()
    side = args.side
    path = Path(thumb.__file__).parents[1] / 'brainco_hand' / f'brainco_{side}.urdf'
    model, data = thumb.PoseThumbRetargeter._load_mujoco(path)
    idx = thumb.PoseThumbRetargeter._thumb_indices(model, side)
    rclpy.init(args=[])
    node = rclpy.create_node('dv1_thumb_inspector')
    received = {}
    subscriptions = []
    for key, typ, topic in [
        ('native', PoseArray, f'/humandex_{side}/eef_pose'),
        ('canonical', HandKinematics, f'/hand_kinematics/{side}'),
        ('ik', PointStamped, f'/revo2_{side}/retarget/debug/thumb_ik_target'),
        ('command', JointState, f'/revo2_{side}/revo2_pid_controller/target_joint_states'),
    ]:
        subscriptions.append(node.create_subscription(
            typ, topic, lambda msg, k=key: received.update({k: (msg, time.monotonic())}), 10))
    stream = None
    writer = None
    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        stream = args.csv.open('w', newline='')
        writer = csv.writer(stream)
        writer.writerow(['command_stamp_ns', 'native_x', 'native_y', 'native_z',
                         'canonical_x', 'canonical_y', 'canonical_z', 'ik_x', 'ik_y', 'ik_z',
                         'proximal_rad', 'metacarpal_rad', 'command_tip_error_mm'])

    def xyz(p):
        return np.array([p.x, p.y, p.z])

    def stamp(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def report():
        now = time.monotonic()
        if len(received) != 4 or any(now - t > .5 for _, t in received.values()):
            print('Waiting for fresh FK/canonical/IK/command topics (enable launch_retarget:=true).', flush=True)
            return
        native, canonical, ik, command = [received[k][0] for k in ('native', 'canonical', 'ik', 'command')]
        if (len(native.poses) != 5 or 'thumb_tip' not in canonical.landmark_names
                or abs(stamp(ik) - stamp(command)) > 50_000_000):
            return
        angles = dict(zip(command.name, command.position))
        try:
            proximal = angles[f'{side}_thumb_proximal_joint']
            metacarpal = angles[f'{side}_thumb_metacarpal_joint']
        except KeyError:
            return
        target = xyz(ik.point)
        if not np.isfinite([*target, proximal, metacarpal]).all():
            return
        data.qpos[:] = 0
        data.qpos[idx['proximal_adr']] = proximal
        data.qpos[idx['distal_adr']] = proximal
        data.qpos[idx['metacarpal_adr']] = metacarpal
        mujoco.mj_forward(model, data)
        error = np.linalg.norm(data.xpos[idx['tip_body_id']] - target) * 1000
        n = xyz(native.poses[4].position)
        c = xyz(canonical.landmarks_m[canonical.landmark_names.index('thumb_tip')])
        print(f'native={np.round(n, 4)} canonical={np.round(c, 4)} IK={np.round(target, 4)} m; '
              f'command(prox,meta)={proximal:.3f},{metacarpal:.3f} rad; command_tip_error={error:.1f} mm', flush=True)
        if writer:
            writer.writerow([stamp(command), *n, *c, *target, proximal, metacarpal, error])
            stream.flush()

    node.create_timer(.5, report)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if stream:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
