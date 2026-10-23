"""Kinematic checks independent of the retarget optimization objective."""

from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from retarget_v2.model import FINGERS, POINT_KINDS, RobotModel, _distal_cap_centroid


class ModelTests(unittest.TestCase):
    MODEL_VERSION = "may3"

    @classmethod
    def setUpClass(cls):
        cls.robot = RobotModel(model_version=cls.MODEL_VERSION)

    @classmethod
    def tearDownClass(cls):
        cls.robot.close()

    def test_model_names_and_limits_match_source_urdf(self):
        robot = self.robot
        root = ET.parse(robot.urdf_path).getroot()
        joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
        self.assertEqual(len(joints), 21)
        self.assertEqual(robot.joint_names, tuple(j.attrib["name"] for j in joints))
        self.assertEqual(robot.point_names, tuple(f"{f}_{p}" for f in FINGERS for p in POINT_KINDS))
        for index, joint in enumerate(joints):
            limit = joint.find("limit")
            self.assertEqual(robot.lower[index], float(limit.attrib["lower"]))
            self.assertEqual(robot.upper[index], float(limit.attrib["upper"]))
            self.assertEqual(robot.velocity[index], float(limit.attrib["velocity"]))
            model_id = mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_JOINT, joint.attrib["name"])
            np.testing.assert_allclose(robot.model.jnt_range[model_id], [robot.lower[index], robot.upper[index]], atol=1e-8)
        self.assertEqual(robot.model.nq, 21)
        self.assertEqual(robot.model.nv, 21)
        self.assertFalse(np.any(robot.model.geom_contype))
        self.assertFalse(np.any(robot.model.geom_conaffinity))

    def test_source_assets_and_geometry_provenance_cover_used_visuals(self):
        robot = self.robot
        self.assertEqual(robot.model_version, self.MODEL_VERSION)
        self.assertEqual(robot.model_metadata["model_version"], self.MODEL_VERSION)
        root = ET.parse(robot.urdf_path).getroot()
        expected = {robot.urdf_path.resolve()}
        for mesh in root.findall("./link/visual/geometry/mesh"):
            filename = mesh.get("filename")
            if filename.startswith("package://revo3_description/"):
                expected.add((robot.description_root / filename.split("package://revo3_description/", 1)[1]).resolve())
            else:
                expected.add((robot.urdf_path.parent / filename).resolve())
        for texture in root.findall(".//texture"):
            if texture.get("filename"):
                expected.add(robot._mesh_path(texture.get("filename")))
        expected |= {robot._import_mesh_path(path) for path in tuple(expected)}
        self.assertEqual(set(robot.asset_paths), expected)
        self.assertTrue(all(path.is_file() for path in robot.asset_paths))
        for finger, report in robot.geometry_report.items():
            self.assertFalse(report["pad_verified"])
            self.assertEqual(len(report["source_visuals"]), 1)
        self.assertEqual(robot.ignored_fixed_mimics, ())
        self.assertTrue(np.all(robot.effort == .35))

    def test_cosmetic_meshes_do_not_invent_mass_or_inertia(self):
        robot = self.robot
        root = ET.parse(robot.urdf_path).getroot()
        for link in root.findall("link"):
            name = link.get("name")
            body_id = mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_BODY, name)
            # The URDF world dummy is the MuJoCo world body.
            self.assertGreaterEqual(body_id, 0)
            inertial = link.find("inertial")
            mass = 0.0 if inertial is None else float(inertial.find("mass").get("value"))
            self.assertAlmostEqual(robot.model.body_mass[body_id], mass, places=12)
        self.assertAlmostEqual(float(robot.model.body_mass.sum()), 1.105, places=12)
        for joint_id in robot._joint_ids:
            self.assertGreater(robot.model.body_mass[robot.model.jnt_bodyid[joint_id]], 0)
        robot.set_q((robot.lower + robot.upper) / 2)
        inertia = np.empty((21, 21))
        try:
            mujoco.mj_fullM(robot.model, robot.data, inertia)
        except TypeError:  # MuJoCo before 3.10.
            mujoco.mj_fullM(robot.model, inertia, robot.data.qM)
        self.assertGreater(np.linalg.eigvalsh(inertia).min(), 0)

    def test_point_jacobian_against_central_finite_differences(self):
        robot = self.robot
        rng = np.random.default_rng(8201)
        for _ in range(3):
            q = robot.lower + (0.15 + 0.7 * rng.random(21)) * (robot.upper - robot.lower)
            _, analytic = robot.points(q, jacobian=True)
            numeric = np.empty_like(analytic)
            epsilon = 1e-6
            for axis in range(21):
                delta = np.zeros(21)
                delta[axis] = epsilon
                numeric[:, :, axis] = (robot.points(q + delta) - robot.points(q - delta)) / (2 * epsilon)
            np.testing.assert_allclose(analytic, numeric, atol=2e-9, rtol=2e-6)

    def test_anatomical_tip_is_on_distal_cap_and_separate_from_pad(self):
        robot = self.robot
        points = robot.points(np.zeros(21)).reshape(5, 5, 3)
        for index, finger in enumerate(FINGERS):
            report = robot.geometry_report[finger]
            local = np.array(report["tip_local_m"])
            lower, upper = np.array(report["mesh_bounds_m"])
            axis = 1 if finger == "thumb" else 2
            self.assertGreater(report["cap_area_m2"], 0)
            self.assertGreaterEqual(local[axis], upper[axis] - report["cap_depth_m"] - 1e-12)
            self.assertLessEqual(local[axis], upper[axis] + 1e-12)
            self.assertTrue(np.all(local >= lower - 1e-12))
            self.assertTrue(np.all(local <= upper + 1e-12))
            self.assertGreater(np.linalg.norm(local - report["pad_local_m"]), 0.018)
            self.assertAlmostEqual(np.linalg.norm(points[index, 3] - points[index, 2]), np.linalg.norm(local), places=10)
        self.assertEqual(robot.finger_lengths.shape, (5, 3))
        self.assertTrue(np.all(robot.finger_lengths > 0.02))
        self.assertTrue(np.all(robot.finger_lengths < 0.06))

    def test_palm_basis_is_a_right_handed_rigid_frame(self):
        robot = self.robot
        np.testing.assert_allclose(robot.palm_basis.T @ robot.palm_basis, np.eye(3), atol=1e-14)
        self.assertAlmostEqual(np.linalg.det(robot.palm_basis), 1.0, places=14)
        self.assertGreater(robot.palm_basis[1, 0], 0.98)
        self.assertGreater(robot.palm_basis[2, 1], 0.98)

    def test_invalid_input_rejected_before_forward_kinematics(self):
        for value in (np.zeros(20), np.full(21, np.nan), np.full(21, np.inf)):
            with self.assertRaises(ValueError):
                self.robot.points(value)


class ImportValidationTests(unittest.TestCase):
    def test_fixed_descendant_visual_scale_and_transforms_reach_distal_cap(self):
        with tempfile.TemporaryDirectory() as directory:
            root_path = Path(directory)
            mesh_path = root_path / "surface.stl"
            mesh_path.write_text('''solid plate
facet normal 0 -1 0
outer loop
vertex 0 1 0
vertex 1 1 0
vertex 0 1 1
endloop
endfacet
endsolid plate
''')
            root = ET.fromstring(f'''<robot><link name="dip"><visual><geometry>
              <mesh filename="{mesh_path}"/></geometry></visual></link>
              <link name="rubber"><visual><origin xyz="0 1 0"/>
              <geometry><mesh filename="{mesh_path}" scale="2 2 2"/></geometry>
              </visual></link><joint name="cover" type="fixed">
              <parent link="dip"/><child link="rubber"/>
              <origin xyz="2 3 4" rpy="{np.pi / 2} 0 0"/>
              </joint></robot>''')
            robot = RobotModel.__new__(RobotModel)
            robot.description_root = root_path
            robot.urdf_path = root_path / "model.urdf"
            surface, sources = robot._distal_visuals(root, "dip")
            centroid, report = _distal_cap_centroid(surface, axis=2)
            # The transformed cover lies in z=7; base visual ends at z=1.
            np.testing.assert_allclose(centroid, [2 + 2 / 3, 3 - 2 / 3, 7], atol=1e-14)
            self.assertAlmostEqual(report["cap_area_m2"], 2)
            self.assertEqual({source["link"] for source in sources}, {"dip", "rubber"})

    def test_unknown_version_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Unknown Revo3 model version"):
            RobotModel(model_version="unversioned")

    def test_actuated_mimic_is_not_silently_discarded(self):
        root = ET.fromstring('''<robot><joint name="driven" type="revolute">
          <parent link="base"/><child link="distal"/>
          <mimic joint="other"/></joint></robot>''')
        with self.assertRaisesRegex(ValueError, "actuated"):
            RobotModel.__new__(RobotModel)._validate_mimics(root)

    def test_only_redundant_identity_fixed_mimic_is_accepted(self):
        source = '''<robot><joint name="driven" type="revolute">
          <parent link="base"/><child link="distal"/></joint>
          <joint name="rubber" type="fixed"><parent link="distal"/>
          <child link="rubber"/><mimic joint="driven" multiplier="1" offset="0"/>
          </joint></robot>'''
        robot = RobotModel.__new__(RobotModel)
        good = ET.fromstring(source)
        robot._validate_mimics(good)
        self.assertEqual(robot.ignored_fixed_mimics, ("rubber",))
        self.assertEqual(good.findall(".//mimic"), [])
        for old, new in (("multiplier=\"1\"", "multiplier=\"2\""),
                         ("offset=\"0\"", "offset=\"0.1\""),
                         ("joint=\"driven\"", "joint=\"missing\""),
                         ("parent link=\"distal\"", "parent link=\"base\"")):
            with self.subTest(old=old):
                with self.assertRaisesRegex(ValueError, "fixed mimic"):
                    robot._validate_mimics(ET.fromstring(source.replace(old, new)))


if __name__ == "__main__":
    unittest.main()
