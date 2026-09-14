"""Record open/fist endpoints from ROS; writes a local Revo2 flexion YAML only."""

import argparse
from datetime import datetime, timezone
import math
from pathlib import Path
import statistics
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from sensor_msgs.msg import JointState
import yaml

from .finger_flexion import CalibratedFingerFlexion, JOINTS


def endpoints(rows):
    if len(rows) < 30:
        raise ValueError(f"Only {len(rows)} valid samples; need at least 30")
    columns = list(zip(*rows))
    unwrapped = [[c[0] + (v - c[0] + math.pi) % (2 * math.pi) - math.pi for v in c]
                 for c in columns]
    if max(statistics.pstdev(c) for c in unwrapped) > math.radians(5):
        raise ValueError("Hand moved during capture (>5 degree std); repeat with a steady pose")
    return [statistics.median(c) for c in unwrapped]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=['left', 'right'], default='left')
    parser.add_argument('--topic', help='Default /humandex_<side>/joint_states')
    parser.add_argument('--seconds', type=float, default=2.0)
    parser.add_argument('--config', type=Path, help='Template Revo2 flexion YAML; defaults to flexion_dv1_<side>.yaml')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 1:
        parser.error('--seconds must be at least 1')
    template = args.config or Path(get_package_share_directory('revo2_hand_retarget')) / 'config' / f'flexion_dv1_{args.side}.yaml'
    document = yaml.safe_load(template.read_text())
    params = document
    if params.get('hand_mode') != args.side:
        parser.error('Template hand mode does not match the requested DV1 hand')
    names = [f'{args.side}_{k.rsplit("_", 1)[0]}_{k.rsplit("_", 1)[1].upper()}_joint' for k in JOINTS]
    rclpy.init(args=[])
    node = rclpy.create_node('dv1_finger_calibration')
    rows = []
    last_stamp = 0

    def receive(msg):
        nonlocal last_stamp
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = node.get_clock().now().nanoseconds - stamp
        if (msg.header.frame_id != f'{args.side}_palm_link' or stamp <= last_stamp
                or not 0 <= age < 500_000_000 or len(msg.name) != len(msg.position)
                or len(set(msg.name)) != len(msg.name)):
            return
        measured = dict(zip(msg.name, msg.position))
        if any(n not in measured or not math.isfinite(measured[n]) for n in names):
            return
        rows.append([measured[n] for n in names])
        last_stamp = stamp

    node.create_subscription(JointState, args.topic or f'/humandex_{args.side}/joint_states', receive, 10)
    captured = {}
    try:
        for pose, label in [('open', '张开四指'), ('closed', '握拳')]:
            input(f'{args.side}: {label}并保持，准备好后按 Enter，采集 {args.seconds:g} 秒：')
            rows.clear()
            last_stamp = node.get_clock().now().nanoseconds
            deadline = time.monotonic() + args.seconds
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            captured[pose] = endpoints(rows)
            print(f'{pose}: {len(rows)} frames; angles deg = {[round(math.degrees(v), 1) for v in captured[pose]]}')
        spans = [(c-o+math.pi) % (2*math.pi)-math.pi
                 for o, c in zip(captured['open'], captured['closed'])]
        if any(abs(v) < math.radians(10) for v in spans):
            raise ValueError('At least one bending joint moved <10 degrees; check motion/joints and repeat')
        CalibratedFingerFlexion(captured['open'], captured['closed'], [1.] * 3, 1.4661, True)
        for pose in ('open', 'closed'):
            params[f'{args.side}_four_finger_{pose}_rad'] = captured[pose]
        params['four_finger_mapping'] = 'calibrated'
        params['four_finger_wrap_angles'] = True
        params['four_finger_calibration_label'] = f'measured {args.side} {datetime.now(timezone.utc).isoformat()}'
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text('# DV1 open/fist ranges; firmware zero and FK angles are unchanged.\n'
                               + yaml.safe_dump(document, sort_keys=False))
        print(f'Saved {args.output.resolve()}; restart Revo2 with finger_flexion_config:=this_file')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
