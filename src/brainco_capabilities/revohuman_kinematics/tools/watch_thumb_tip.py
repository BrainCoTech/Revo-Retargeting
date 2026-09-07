#!/usr/bin/env python3
"""Read-only monitor for the upstream right-hand-only HumanDex PoseArray."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseArray


class Monitor(Node):
    def __init__(self):
        super().__init__('watch_humandex_thumb_tip')
        self.last_print = -math.inf
        self.last_received = None
        self.last_xyz = None
        self.create_subscription(PoseArray, '/humandex_eef_pose', self.receive, 10)
        self.create_timer(2.0, self.check_receiving)
        print('Right-hand-only input expected: 5 poses, thumb at index 4. Units: metres.', flush=True)

    def receive(self, msg):
        self.last_received = time.monotonic()
        if len(msg.poses) != 5:
            self.get_logger().warning('Expected exactly 5 poses from the right-hand-only upstream.',
                                      throttle_duration_sec=2.0)
            return
        p = msg.poses[4].position
        xyz = (p.x, p.y, p.z)
        if not all(math.isfinite(v) for v in xyz):
            self.get_logger().warning('Non-finite thumb position', throttle_duration_sec=2.0)
            return
        now = time.monotonic()
        if now - self.last_print < 0.2:
            return
        delta = 0.0 if self.last_xyz is None else math.dist(xyz, self.last_xyz) * 1000
        stamp = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
        print(f'{stamp} frame={msg.header.frame_id} '
              f'x={p.x:+.5f} y={p.y:+.5f} z={p.z:+.5f} m '
              f'delta={delta:.2f} mm', flush=True)
        self.last_print, self.last_xyz = now, xyz

    def check_receiving(self):
        if self.last_received is None or time.monotonic() - self.last_received > 2:
            print('No recent /humandex_eef_pose messages; check upstream and ROS_DOMAIN_ID.', flush=True)


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
