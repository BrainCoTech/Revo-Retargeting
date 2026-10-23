"""Check the official model import against an independent URDF tree evaluator.

The evaluator below uses scipy rotations and XML only; it does not call the
production FK, URDF conversion helpers, or MuJoCo to obtain its expected poses.
These are software/asset consistency checks, not physical-hand calibration.
"""

from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from retarget_v2.model import FINGERS, RobotModel


def _vector(element, name, default):
    return np.array([float(value) for value in element.get(name, default).split()])


def _xml_forward_kinematics(root, named_angles):
    """URDF parent * origin(RPY/XYZ) * motion, preserving all fixed links."""
    joints = list(root.findall("joint"))
    child_names = {joint.find("child").get("link") for joint in joints}
    roots = {link.get("name") for link in root.findall("link")} - child_names
    if len(roots) != 1:
        raise AssertionError("Test requires a single connected URDF tree")
    poses = {roots.pop(): np.eye(4)}
    anchors, axes = {}, {}
    while joints:
        remaining = []
        for joint in joints:
            parent = joint.find("parent").get("link")
            if parent not in poses:
                remaining.append(joint)
                continue
            origin = joint.find("origin")
            offset = np.eye(4)
            if origin is not None:
                offset[:3, 3] = _vector(origin, "xyz", "0 0 0")
                offset[:3, :3] = Rotation.from_euler(
                    "xyz", _vector(origin, "rpy", "0 0 0")
                ).as_matrix()
            joint_pose = poses[parent] @ offset
            motion = np.eye(4)
            if joint.get("type") == "revolute":
                name = joint.get("name")
                local_axis = _vector(joint.find("axis"), "xyz", "1 0 0")
                local_axis /= np.linalg.norm(local_axis)
                axes[name] = joint_pose[:3, :3] @ local_axis
                anchors[name] = joint_pose[:3, 3].copy()
                motion[:3, :3] = Rotation.from_rotvec(
                    local_axis * named_angles[name]
                ).as_matrix()
            elif joint.get("type") != "fixed":
                raise AssertionError(f"Unhandled URDF joint type: {joint.get('type')}")
            poses[joint.find("child").get("link")] = joint_pose @ motion
        if len(remaining) == len(joints):
            raise AssertionError("Disconnected or cyclic URDF tree")
        joints = remaining
    return poses, anchors, axes


class May3UrdfTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = RobotModel(model_version="may3")
        cls.root = ET.parse(cls.robot.urdf_path).getroot()
        cls.joints = {joint.get("name"): joint for joint in cls.root.findall("joint")}

    @classmethod
    def tearDownClass(cls):
        cls.robot.close()

    def _poses(self):
        robot = self.robot
        rng = np.random.default_rng(10082026)
        yield np.zeros(21)
        yield robot.lower.copy()
        yield robot.upper.copy()
        for _ in range(8):
            yield rng.uniform(robot.lower, robot.upper)

    def test_named_vector_uses_may3_order_and_limits(self):
        expected = tuple(
            f"right_{finger}_{kind}_joint"
            for finger, kinds in (
                ("thumb", ("CMP", "CMR", "MCP", "PIP", "DIP")),
                ("index", ("MPR", "MCP", "PIP", "DIP")),
                ("middle", ("MPR", "MCP", "PIP", "DIP")),
                ("ring", ("MPR", "MCP", "PIP", "DIP")),
                ("little", ("MPR", "MCP", "PIP", "DIP")),
            )
            for kind in kinds
        )
        self.assertEqual(self.robot.joint_names, expected)
        expected_ranges = {
            "right_index_MPR_joint": (-0.2618, 0.2618),
            "right_middle_MPR_joint": (-0.2618, 0.2618),
            "right_ring_MPR_joint": (-0.2618, 0.2618),
            "right_little_MPR_joint": (-0.2618, 0.2618),
            "right_thumb_MCP_joint": (0., .8727),
            "right_thumb_CMR_joint": (0., 2.0071),
        }
        for name, bounds in expected_ranges.items():
            index = self.robot.joint_names.index(name)
            np.testing.assert_allclose(
                [self.robot.lower[index], self.robot.upper[index]], bounds, atol=1e-12
            )
        for name in expected:
            limit = self.joints[name].find("limit")
            self.assertEqual(float(limit.get("effort")), .35)


    def test_all_link_poses_and_joint_axes_match_independent_tree_fk(self):
        robot = self.robot
        for pose_number, q in enumerate(self._poses()):
            with self.subTest(pose=pose_number):
                poses, anchors, axes = _xml_forward_kinematics(
                    self.root, dict(zip(robot.joint_names, q))
                )
                robot.set_q(q)
                for name, expected in poses.items():
                    body = mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_BODY, name)
                    self.assertGreaterEqual(body, 0, name)
                    # MuJoCo's URDF -> saved MJCF serialization can round values.
                    np.testing.assert_allclose(
                        robot.data.xpos[body], expected[:3, 3], atol=2e-6, rtol=0,
                        err_msg=f"{name}: world translation",
                    )
                    np.testing.assert_allclose(
                        robot.data.xmat[body].reshape(3, 3), expected[:3, :3],
                        atol=1e-5, rtol=0, err_msg=f"{name}: world rotation",
                    )
                for name in robot.joint_names:
                    joint = mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                    self.assertGreaterEqual(joint, 0, name)
                    np.testing.assert_allclose(
                        robot.data.xanchor[joint], anchors[name], atol=2e-6, rtol=0,
                        err_msg=f"{name}: world anchor",
                    )
                    np.testing.assert_allclose(
                        robot.data.xaxis[joint], axes[name], atol=1e-5, rtol=0,
                        err_msg=f"{name}: world axis",
                    )

    def test_task_points_match_xml_link_frames_at_random_poses(self):
        robot = self.robot
        for q in self._poses():
            poses, _, _ = _xml_forward_kinematics(
                self.root, dict(zip(robot.joint_names, q))
            )
            expected = []
            for finger in FINGERS:
                cad = "little" if finger == "pinky" else finger
                for kind in ("MCP", "PIP", "DIP"):
                    link = self.joints[f"right_{cad}_{kind}_joint"].find("child").get("link")
                    expected.append(poses[link][:3, 3])
                tip_joint = self.joints[f"right_{cad}_tip_joint"]
                distal_pose = poses[tip_joint.find("parent").get("link")]
                # The local cap proxy is CAD-derived; the independent tree
                # evaluator verifies its world transformation, not its physical
                # suitability as a contact point.
                local_tip = np.asarray(robot.geometry_report[finger]["tip_local_m"])
                expected.append(distal_pose[:3, :3] @ local_tip + distal_pose[:3, 3])
                expected.append(poses[tip_joint.find("child").get("link")][:3, 3])
            np.testing.assert_allclose(robot.points(q), expected, atol=2e-6, rtol=0)

    def test_mpr_world_axes_keep_may3_positive_x_direction(self):
        _, _, axes = _xml_forward_kinematics(
            self.root, dict.fromkeys(self.robot.joint_names, 0.0)
        )
        for finger in ("index", "middle", "ring", "little"):
            axis = axes[f"right_{finger}_MPR_joint"]
            self.assertGreater(axis[0], 0.998)
            self.assertAlmostEqual(np.linalg.norm(axis), 1.0, places=12)

    def test_point_jacobians_match_independent_joint_tree_cross_products(self):
        robot = self.robot
        parent_joint = {
            joint.find("child").get("link"): joint for joint in self.joints.values()
        }
        attached_links = []
        local_points = []
        for finger in FINGERS:
            cad = "little" if finger == "pinky" else finger
            for kind in ("MCP", "PIP", "DIP"):
                attached_links.append(
                    self.joints[f"right_{cad}_{kind}_joint"].find("child").get("link")
                )
                local_points.append(np.zeros(3))
            attached_links.append(self.joints[f"right_{cad}_DIP_joint"].find("child").get("link"))
            local_points.append(np.asarray(robot.geometry_report[finger]["tip_local_m"]))
            attached_links.append(self.joints[f"right_{cad}_tip_joint"].find("child").get("link"))
            local_points.append(np.zeros(3))
        joint_indices = {name: index for index, name in enumerate(robot.joint_names)}
        for q in self._poses():
            poses, anchors, axes = _xml_forward_kinematics(
                self.root, dict(zip(robot.joint_names, q))
            )
            expected = np.zeros((25, 3, 21))
            for row, (link, local) in enumerate(zip(attached_links, local_points)):
                point = poses[link][:3, :3] @ local + poses[link][:3, 3]
                while link in parent_joint:
                    joint = parent_joint[link]
                    name = joint.get("name")
                    if name in joint_indices:
                        expected[row, :, joint_indices[name]] = np.cross(
                            axes[name], point - anchors[name]
                        )
                    link = joint.find("parent").get("link")
            _, actual = robot.points(q, jacobian=True)
            np.testing.assert_allclose(actual, expected, atol=2e-6, rtol=0)

    def test_geometry_has_finite_metre_scale_and_fixed_bone_lengths(self):
        robot = self.robot
        reference = robot.finger_lengths
        self.assertEqual(reference.shape, (5, 3))
        self.assertTrue(np.all((reference > 0.02) & (reference < 0.06)))
        for q in self._poses():
            points = robot.points(q).reshape(5, 5, 3)
            self.assertTrue(np.isfinite(points).all())
            self.assertLess(np.max(np.abs(points)), 0.4)
            np.testing.assert_allclose(
                np.linalg.norm(np.diff(points[:, :4], axis=1), axis=2),
                reference, atol=1e-12, rtol=0,
            )
        for report in robot.geometry_report.values():
            low, high = np.asarray(report["mesh_bounds_m"])
            tip = np.asarray(report["tip_local_m"])
            self.assertTrue(np.isfinite([low, high, tip]).all())
            self.assertTrue(np.all(tip >= low - 1e-12))
            self.assertTrue(np.all(tip <= high + 1e-12))
            self.assertLess(np.linalg.norm(high - low), 0.1)
            self.assertGreater(report["cap_area_m2"], 0)


if __name__ == "__main__":
    unittest.main()
