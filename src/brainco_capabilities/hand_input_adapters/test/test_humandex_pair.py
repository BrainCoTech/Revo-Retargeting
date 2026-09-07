"""Exercise adapter message conversion without a ROS graph or hardware."""

from types import SimpleNamespace
import unittest

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import JointState

from hand_input_adapters.humandex_adapter_node import HumanDexHandAdapter
from test_finger_flexion import OPEN, CLOSED, upstream, load_calibration


class HumanDexPairTest(unittest.TestCase):
    def test_ros_message_conversion_preserves_joints_and_tip(self):
        sent = []
        warnings = []
        harness = SimpleNamespace(
            sides=('right',), frames={'right': 'hand_retarget_right'},
            finger_calibrations={'right': load_calibration()},
            output_publishers={'right': SimpleNamespace(publish=sent.append)},
            published={'right': 0},
            _present_sides=HumanDexHandAdapter._present_sides,
            get_logger=lambda: SimpleNamespace(
                info=lambda *args: None,
                warning=lambda text, **kw: warnings.append(text)),
        )
        poses = PoseArray()
        poses.header.stamp.sec = 123
        poses.poses = [Pose() for _ in range(5)]
        poses.poses[4].position.x = 0.13061
        poses.poses[4].position.z = -0.08358
        for raw, expected in ((OPEN, 0), (CLOSED, 1.4661)):
            measured = upstream(raw)
            measured.update({f'{f}_mpr': 0.42 for f in ('index', 'middle', 'ring', 'little')})
            measured.update({f'thumb_{j}': 0.21 for j in ('dip', 'pip', 'mcp', 'cmr', 'cmp')})
            joint = JointState()
            joint.header.stamp = poses.header.stamp
            joint.name = [f'right_{n.rsplit("_", 1)[0]}_{n.rsplit("_", 1)[1].upper()}_joint'
                          for n in measured]
            joint.position = list(measured.values())
            HumanDexHandAdapter._publish_pair(harness, joint, poses)
            result = sent[-1]
            values = dict(zip(result.joint_names, result.joint_positions_rad))
            self.assertEqual(len(values), 25)
            for name, value in measured.items():
                self.assertEqual(values[name], value)
            for finger in ('index', 'middle', 'ring', 'little'):
                self.assertAlmostEqual(values[f'{finger}_flexion'], expected, places=9)
            self.assertEqual(result.landmarks_m[4], poses.poses[4].position)
            self.assertEqual(result.header.stamp, poses.header.stamp)
        self.assertEqual(len(sent), 2)
        joint.name.pop(0)
        joint.position.pop(0)
        HumanDexHandAdapter._publish_pair(harness, joint, poses)
        self.assertEqual(len(sent), 2)
        self.assertEqual(len(warnings), 1)


if __name__ == '__main__':
    unittest.main()
