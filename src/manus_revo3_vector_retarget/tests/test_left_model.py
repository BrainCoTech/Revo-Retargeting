"""Left CAD import, handed geometry, and analytic FK/Jacobian regressions."""

from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from retarget_v2.model import FINGERS, RobotModel
from test_official_urdf import _xml_forward_kinematics


class LeftModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.left = RobotModel(hand_side="left")
        cls.right = RobotModel()
        cls.root = ET.parse(cls.left.urdf_path).getroot()

    @classmethod
    def tearDownClass(cls):
        cls.left.close()
        cls.right.close()

    def _poses(self):
        yield np.zeros(21)
        yield self.left.lower.copy()
        yield self.left.upper.copy()
        rng = np.random.default_rng(11102026)
        for _ in range(4):
            yield rng.uniform(self.left.lower, self.left.upper)

    def test_side_selects_assets_and_preserves_named_joint_semantics(self):
        left, right = self.left, self.right
        self.assertEqual(right.hand_side, "right")
        self.assertEqual(left.hand_side, "left")
        self.assertEqual(left.urdf_path.name, "revo3_left_may3.urdf")
        self.assertEqual(left.model_metadata["hand_side"], "left")
        self.assertEqual(left.model_metadata["urdf"], "description/urdf/revo3_left_may3.urdf")
        self.assertEqual(left.joint_names, tuple(
            "left_" + name.removeprefix("right_") for name in right.joint_names
        ))
        np.testing.assert_array_equal(left.lower, right.lower)
        np.testing.assert_array_equal(left.upper, right.upper)
        self.assertTrue(all(path.is_file() for path in left.asset_paths))
        for report in left.geometry_report.values():
            self.assertEqual(report["hand_side"], "left")
            self.assertTrue(all("revo3_left_hand/" in path for path in report["meshes"]))

    def test_left_thumb_uses_negative_y_distal_cap(self):
        # The left thumb mesh grows down -Y; +Y would select its proximal end.
        for finger in FINGERS:
            report = self.left.geometry_report[finger]
            tip = np.asarray(report["tip_local_m"])
            low, high = np.asarray(report["mesh_bounds_m"])
            if finger == "thumb":
                self.assertEqual(report["axis"], "-Y")
                self.assertLess(tip[1], -0.03)
                self.assertGreaterEqual(tip[1], low[1])
                self.assertLessEqual(tip[1], low[1] + report["cap_depth_m"])
            else:
                self.assertEqual(report["axis"], "+Z")
                self.assertGreaterEqual(tip[2], high[2] - report["cap_depth_m"])
            self.assertGreater(report["cap_area_m2"], 0)
        # May3 sides are separate CAD exports, with micrometre length differences.
        np.testing.assert_allclose(self.left.finger_lengths, self.right.finger_lengths, atol=1e-5)

    def test_left_palm_basis_is_a_right_handed_rigid_frame(self):
        basis = self.left.palm_basis
        np.testing.assert_allclose(basis.T @ basis, np.eye(3), atol=1e-14)
        self.assertAlmostEqual(np.linalg.det(basis), 1.0, places=14)

    def test_left_link_poses_and_joint_axes_match_independent_xml_fk(self):
        # This evaluator uses scipy/XML, independently of MuJoCo/model helpers.
        robot = self.left
        for q in self._poses():
            poses, anchors, axes = _xml_forward_kinematics(
                self.root, dict(zip(robot.joint_names, q)),
            )
            robot.set_q(q)
            for name, expected in poses.items():
                body = mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_BODY, name)
                self.assertGreaterEqual(body, 0, name)
                np.testing.assert_allclose(robot.data.xpos[body], expected[:3, 3], atol=2e-6)
                np.testing.assert_allclose(
                    robot.data.xmat[body].reshape(3, 3), expected[:3, :3], atol=1e-5,
                )
            for name in robot.joint_names:
                joint = mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                np.testing.assert_allclose(robot.data.xanchor[joint], anchors[name], atol=2e-6)
                np.testing.assert_allclose(robot.data.xaxis[joint], axes[name], atol=1e-5)

    def test_left_point_jacobian_matches_finite_difference(self):
        robot = self.left
        rng = np.random.default_rng(1121)
        for _ in range(2):
            q = rng.uniform(robot.lower, robot.upper)
            _, analytic = robot.points(q, jacobian=True)
            numeric = np.empty_like(analytic)
            for axis in range(21):
                delta = np.zeros(21)
                delta[axis] = 1e-6
                numeric[:, :, axis] = (robot.points(q + delta) - robot.points(q - delta)) / 2e-6
            np.testing.assert_allclose(analytic, numeric, atol=2e-9, rtol=2e-6)

    def test_unknown_side_is_rejected(self):
        for side in ("Left", "both", "", None):
            with self.subTest(side=side):
                with self.assertRaisesRegex(ValueError, "hand side"):
                    RobotModel(hand_side=side)


if __name__ == "__main__":
    unittest.main()
