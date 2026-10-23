"""Independent Revo3 hand kinematics, derived only from CAD/URDF assets.

The solver coordinates are URDF coordinates, not verified motor coordinates.
In particular, the real hand's thumb motor mapping requires separate calibration.
"""

from __future__ import annotations

import struct
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

from .paths import description_root, workspace_root, source_key

import mujoco
import numpy as np


FINGERS = ("thumb", "index", "middle", "ring", "pinky")
POINT_KINDS = ("root", "pip", "dip", "tip", "pad")
_CAD_FINGER = {finger: "little" if finger == "pinky" else finger for finger in FINGERS}
_CAP_DEPTH_M = 0.0005
_MODEL_URDFS = {"may3": "urdf/revo3_{hand_side}_may3.urdf"}


def _rpy_matrix(rpy: str = "0 0 0") -> np.ndarray:
    roll, pitch, yaw = np.fromstring(rpy, sep=" ")
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _origin(element: ET.Element) -> tuple[np.ndarray, np.ndarray]:
    origin = element.find("origin")
    if origin is None:
        return np.eye(3), np.zeros(3)
    return (_rpy_matrix(origin.get("rpy", "0 0 0")),
            np.fromstring(origin.get("xyz", "0 0 0"), sep=" "))


def _read_stl_triangles(path: Path) -> np.ndarray:
    """Read binary or ASCII STL without introducing a mesh-processing dependency."""
    raw = path.read_bytes()
    if len(raw) >= 84:
        count = struct.unpack_from("<I", raw, 80)[0]
        if len(raw) == 84 + 50 * count:
            dtype = np.dtype([
                ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)),
                ("attribute", "<u2"),
            ])
            return np.frombuffer(raw, dtype=dtype, count=count, offset=84)[
                "vertices"
            ].astype(np.float64)
    vertices = []
    for line in raw.decode("ascii").splitlines():
        fields = line.strip().split()
        if fields and fields[0] == "vertex":
            vertices.append([float(value) for value in fields[1:4]])
    if not vertices or len(vertices) % 3:
        raise ValueError(f"Invalid STL: {path}")
    return np.asarray(vertices, dtype=float).reshape(-1, 3, 3)


def _distal_cap_centroid(
    triangles: np.ndarray, axis: int, direction: int = 1,
) -> tuple[np.ndarray, dict]:
    """Area centroid of the visual surface in its distal 0.5 mm band.

    Each triangle is clipped against the distal extreme minus 0.0005 m. The area
    centroid of the remaining triangles avoids weighting CAD tessellation
    density as a vertex average would. Four fingers point along local +Z;
    the right/left thumb points along local +Y/-Y. This is a geometric endpoint proxy, not
    a measured tactile contact point or a claim about the physical fingertip.
    """
    if direction not in (-1, 1):
        raise ValueError("Distal axis direction must be -1 or 1")
    flat = triangles.reshape(-1, 3)
    lower, upper = flat.min(axis=0), flat.max(axis=0)
    extreme = upper[axis] if direction == 1 else lower[axis]
    threshold = float(extreme - direction * _CAP_DEPTH_M)
    weighted = np.zeros(3)
    total_area = 0.0
    candidates = triangles[(direction * triangles[:, :, axis]).max(axis=1) >= direction * threshold]
    for triangle in candidates:
        clipped = []
        for index in range(3):
            start, end = triangle[index - 1], triangle[index]
            start_inside = direction * start[axis] >= direction * threshold
            end_inside = direction * end[axis] >= direction * threshold
            if start_inside != end_inside:
                alpha = (threshold - start[axis]) / (end[axis] - start[axis])
                clipped.append(start + alpha * (end - start))
            if end_inside:
                clipped.append(end)
        for index in range(1, len(clipped) - 1):
            a, b, c = clipped[0], clipped[index], clipped[index + 1]
            area = 0.5 * np.linalg.norm(np.cross(b - a, c - a))
            total_area += area
            weighted += area * (a + b + c) / 3.0
    if total_area <= 0.0:
        raise ValueError("Visual STL has no non-degenerate distal cap surface")
    centroid = weighted / total_area
    return centroid, {
        "method": "area-weighted centroid of visual STL triangles clipped to distal 0.5 mm",
        "axis": ("+" if direction == 1 else "-") + "XYZ"[axis],
        "cap_depth_m": _CAP_DEPTH_M,
        "cap_threshold_m": threshold,
        "cap_area_m2": float(total_area),
        "mesh_bounds_m": [lower.tolist(), upper.tolist()],
        "tip_local_m": centroid.tolist(),
        "meaning": "geometric anatomical endpoint proxy; separate from URDF pad marker",
    }


class RobotModel:
    """Kinematic MuJoCo model with named task sites and analytic point Jacobians.

    ``q`` follows ``joint_names`` (the 21 revolute joints in URDF order).
    Points and Jacobians are expressed in the fixed hand-base/world frame.
    ``palm_basis`` columns are palm x/y/z expressed in that same frame.
    The instance owns one MuJoCo data buffer and is not thread safe.
    """

    def __init__(self, repo_root: str | Path | None = None, model_version: str = "may3",
                 hand_side: str = "right"):
        if model_version not in _MODEL_URDFS:
            raise ValueError(f"Unknown Revo3 model version: {model_version!r}; use may3 (workspace revo3_description)")
        if hand_side not in ("left", "right"):
            raise ValueError(f"Unknown Revo3 hand side: {hand_side!r}; use left or right")
        self.model_version = model_version
        self.hand_side = hand_side
        self.repo_root = Path(repo_root) if repo_root is not None else (workspace_root() or description_root().parent)
        self.repo_root = self.repo_root.resolve()
        self.description_root = self.repo_root / "src/brainco_revo3_ros2/revo3_description" if repo_root is not None else description_root()
        self.urdf_path = self.description_root / _MODEL_URDFS[model_version].format(hand_side=hand_side)
        root = ET.parse(self.urdf_path).getroot()
        self._validate_mimics(root)
        joints = [joint for joint in root.findall("joint") if joint.get("type") == "revolute"]
        expected_names = {f"{hand_side}_{cad}_{kind}_joint" for cad in _CAD_FINGER.values()
                          for kind in (("CMP", "CMR", "MCP", "PIP", "DIP") if cad == "thumb"
                                       else ("MPR", "MCP", "PIP", "DIP"))}
        if (len(joints) != 21 or {j.get("name") for j in joints} != expected_names
                or any(j.get("type") not in ("fixed", "revolute") for j in root.findall("joint"))):
            raise ValueError("Expected exactly the 21 named Revo3 revolute joints and fixed links")
        self.joint_names = tuple(joint.attrib["name"] for joint in joints)
        self.lower = np.array([float(joint.find("limit").attrib["lower"]) for joint in joints])
        self.upper = np.array([float(joint.find("limit").attrib["upper"]) for joint in joints])
        # Source metadata only. Official CAD exports zeros for velocity/effort;
        # independently configured simulation/control bounds must remain separate.
        self.velocity = np.array([float(joint.find("limit").attrib["velocity"]) for joint in joints])
        self.effort = np.array([float(joint.find("limit").attrib["effort"]) for joint in joints])
        if not np.all(np.isfinite([self.lower, self.upper])) or np.any(self.lower >= self.upper):
            raise ValueError("Invalid URDF position limits")
        visual_assets = {
            self._mesh_path(mesh.attrib["filename"])
            for mesh in root.findall("./link/visual/geometry/mesh")
        }
        self.asset_paths = tuple(sorted({self.urdf_path.resolve()} | visual_assets | {
            self._import_mesh_path(path) for path in visual_assets
        } | {
            self._mesh_path(texture.attrib["filename"])
            for texture in root.findall(".//texture") if "filename" in texture.attrib
        }))
        self.model_metadata = {
            "model_version": model_version,
            "hand_side": hand_side,
            "description_package": "revo3_description",
            "description_root": str(self.description_root),
            "urdf": source_key(self.urdf_path, self),
            "joint_order": "URDF revolute declaration order; commands must map by name",
            "velocity_effort_metadata": "source values, not validated control limits",
            "fixed_mimics_ignored": list(self.ignored_fixed_mimics),
            "contact_geometry_validated": False,
        }
        self.point_names = tuple(f"{finger}_{kind}" for finger in FINGERS for kind in POINT_KINDS)
        self.geometry_report = {}
        self._temporary = tempfile.TemporaryDirectory(prefix="revo3_independent_model_")

        try:
            sites = self._derive_sites(root)
            self._derive_palm(root)
            self._compile(root, sites)
            self.data = mujoco.MjData(self.model)
            self._joint_ids = np.array([
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                for name in self.joint_names
            ])
            if self.model.nq != 21 or self.model.nv != 21 or np.any(self._joint_ids < 0):
                raise ValueError("URDF import changed the expected 21-joint kinematics")
            self._qpos_indices = self.model.jnt_qposadr[self._joint_ids].copy()
            self._dof_indices = self.model.jnt_dofadr[self._joint_ids].copy()
            self._site_ids = np.array([
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)
                for name in self.point_names
            ])
            if np.any(self._site_ids < 0):
                raise ValueError("Task sites missing after MuJoCo compilation")
            rest = self.points(np.zeros(21)).reshape(5, 5, 3)
            self.finger_lengths = np.linalg.norm(np.diff(rest[:, :4], axis=1), axis=2)
        except Exception:
            self.close()
            raise

    def _mesh_path(self, filename: str) -> Path:
        prefix = "package://revo3_description/"
        if filename.startswith(prefix):
            return (self.description_root / filename[len(prefix):]).resolve()
        path = Path(filename)
        return path.resolve() if path.is_absolute() else (self.urdf_path.parent / path).resolve()

    @staticmethod
    def _import_mesh_path(path: Path) -> Path:
        """MuJoCo cannot import DAE; use the matching description STL in memory."""
        if path.suffix.lower() == ".dae":
            fallback = path.with_suffix(".STL")
            if not fallback.is_file():
                raise ValueError(f"MuJoCo needs an STL sibling for DAE visual: {path}")
            return fallback
        return path

    def _validate_mimics(self, root: ET.Element) -> None:
        """Ignore only redundant metadata on rigid cosmetic child links.

        A fixed joint cannot also move with a mimic constraint. The official
        exporter attaches identity mimic tags to covers already fixed to the
        mimicked joint's child. Remove that redundant tag in memory only.
        Actuated mimic chains or any other arrangement need an explicit model.
        """
        joints = {joint.get("name"): joint for joint in root.findall("joint")}
        ignored = []
        for joint in joints.values():
            mimics = joint.findall("mimic")
            if not mimics:
                continue
            if joint.get("type") != "fixed" or len(mimics) != 1:
                raise ValueError(f"Unsupported actuated or multiple mimic: {joint.get('name')}")
            mimic = mimics[0]
            target = joints.get(mimic.get("joint"))
            if (target is None or target.get("type") != "revolute"
                    or joint.find("parent").get("link") != target.find("child").get("link")
                    or float(mimic.get("multiplier", "1")) != 1.0
                    or float(mimic.get("offset", "0")) != 0.0):
                raise ValueError(f"Unsupported fixed mimic metadata: {joint.get('name')}")
            ignored.append(joint.get("name"))
            joint.remove(mimic)
        self.ignored_fixed_mimics = tuple(ignored)

    def _distal_visuals(self, root: ET.Element, dip_name: str) -> tuple[np.ndarray, list[dict]]:
        """Transform every DIP/fixed-descendant visual surface into DIP axes."""
        links = {link.get("name"): link for link in root.findall("link")}
        fixed_children = {}
        for joint in root.findall("joint"):
            if joint.get("type") == "fixed":
                fixed_children.setdefault(joint.find("parent").get("link"), []).append(joint)
        pending = [(dip_name, np.eye(3), np.zeros(3))]
        visited, triangles, provenance = set(), [], []
        while pending:
            name, rotation, offset = pending.pop()
            if name in visited:
                raise ValueError(f"Cycle or duplicate parent in distal fixed links: {name}")
            visited.add(name)
            for visual in links[name].findall("visual"):
                mesh = visual.find("geometry/mesh")
                if mesh is None:
                    raise ValueError(f"Distal surface is not a supported STL mesh: {name}")
                mesh_path = self._mesh_path(mesh.attrib["filename"])
                visual_rotation, visual_offset = _origin(visual)
                dip_rotation = rotation @ visual_rotation
                dip_offset = offset + rotation @ visual_offset
                scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
                surface = _read_stl_triangles(mesh_path) * scale
                triangles.append(surface @ dip_rotation.T + dip_offset)
                provenance.append({
                    "link": name, "mesh": source_key(mesh_path, self),
                    "scale": scale.tolist(), "dip_from_visual_rotation": dip_rotation.tolist(),
                    "dip_from_visual_translation_m": dip_offset.tolist(),
                })
            for joint in fixed_children.get(name, ()):
                child_rotation, child_offset = _origin(joint)
                pending.append((joint.find("child").get("link"), rotation @ child_rotation,
                                offset + rotation @ child_offset))
        if not triangles:
            raise ValueError(f"No distal visual surfaces: {dip_name}")
        return np.concatenate(triangles), provenance

    def _derive_sites(self, root: ET.Element) -> dict[str, tuple[str, np.ndarray]]:
        sites = {}
        for finger in FINGERS:
            cad = _CAD_FINGER[finger]
            for kind, link_suffix in (("root", "MCP"), ("pip", "PIP"), ("dip", "DIP")):
                joint = root.find(f"joint[@name='{self.hand_side}_{cad}_{link_suffix}_joint']")
                sites[f"{finger}_{kind}"] = (joint.find("child").get("link"), np.zeros(3))
            dip_name = sites[f"{finger}_dip"][0]
            triangles, provenance = self._distal_visuals(root, dip_name)
            # The left May3 thumb distal surface extends along negative local Y.
            # its thumb grows along -Y; selecting +Y would pick the proximal cap.
            endpoint, report = _distal_cap_centroid(
                triangles, axis=1 if finger == "thumb" else 2,
                direction=-1 if finger == "thumb" and self.hand_side == "left" else 1,
            )
            pad_joint = root.find(f"joint[@name='{self.hand_side}_{cad}_tip_joint']")
            pad = np.fromstring(pad_joint.find("origin").get("xyz", "0 0 0"), sep=" ")
            parent_link = pad_joint.find("parent").attrib["link"]
            if parent_link != dip_name:
                raise ValueError(f"Unexpected URDF tip parent for {finger}: {parent_link}")
            sites[f"{finger}_tip"] = (parent_link, endpoint)
            sites[f"{finger}_pad"] = (parent_link, pad)
            report.update({
                "mesh": provenance[0]["mesh"],  # Legacy report readers.
                "meshes": [item["mesh"] for item in provenance],
                "source_visuals": provenance, "dip_link": dip_name,
                "model_version": self.model_version, "hand_side": self.hand_side,
                "pad_local_m": pad.tolist(),
                "pad_verified": False,
                "pad_meaning": "unverified URDF marker, not measured contact; thumb marker coordinates were reused across frame changes",
            })
            self.geometry_report[finger] = report
        return sites

    def _derive_palm(self, root: ET.Element) -> None:
        anchors = {}
        for finger in ("index", "middle", "ring", "little"):
            joint = root.find(f"joint[@name='{self.hand_side}_{finger}_MPR_joint']")
            if joint.find("parent").attrib["link"] != f"{self.hand_side}_hand_base_link":
                raise ValueError("Palm anchors must be expressed directly in the hand-base frame")
            anchors[finger] = np.fromstring(joint.find("origin").attrib["xyz"], sep=" ")
        self.palm_origin = np.mean(list(anchors.values()), axis=0)
        x_axis = anchors["index"] - anchors["little"]
        x_axis /= np.linalg.norm(x_axis)
        y_axis = self.palm_origin - x_axis * np.dot(x_axis, self.palm_origin)
        y_axis /= np.linalg.norm(y_axis)
        z_axis = np.cross(x_axis, y_axis)
        self.palm_basis = np.column_stack((x_axis, y_axis, z_axis))

    def _compile(self, root: ET.Element, sites: dict[str, tuple[str, np.ndarray]]) -> None:
        # Geometry is visual-only: no collision inference or contact dynamics.
        # All rigid links remain so site bodies survive URDF conversion.
        for link in root.findall("link"):
            for collision in link.findall("collision"):
                link.remove(collision)
        for mesh in root.findall(".//mesh"):
            mesh.set("filename", str(self._import_mesh_path(self._mesh_path(mesh.attrib["filename"]))))
        extension = root.find("mujoco")
        if extension is None:
            extension = ET.SubElement(root, "mujoco")
        compiler = extension.find("compiler")
        if compiler is None:
            compiler = ET.SubElement(extension, "compiler")
        compiler.attrib.update({
            "fusestatic": "false", "balanceinertia": "true", "discardvisual": "false",
            "strippath": "false", "inertiafromgeom": "false",
        })
        imported = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        self.xml_path = Path(self._temporary.name) / "revo3_kinematic.xml"
        mujoco.mj_saveLastXML(str(self.xml_path), imported)
        mjcf = ET.parse(self.xml_path).getroot()
        option = mjcf.find("option")
        if option is None:
            option = ET.SubElement(mjcf, "option")
        flag = option.find("flag")
        if flag is None:
            flag = ET.SubElement(option, "flag")
        flag.set("contact", "disable")
        for geom in mjcf.findall(".//geom"):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
        for name, (link_name, position) in sites.items():
            body = mjcf.find(f".//body[@name='{link_name}']")
            if body is None:
                raise ValueError(f"Missing task-site body after import: {link_name}")
            ET.SubElement(body, "site", {
                "name": name,
                "pos": " ".join(f"{value:.12g}" for value in position),
                "size": "0.0015",
                "rgba": "0.2 0.8 0.3 0.9" if name.endswith("_tip") else "0.9 0.35 0.15 0.6",
                "group": "3",
            })
        ET.ElementTree(mjcf).write(self.xml_path, encoding="unicode")
        self.model = mujoco.MjModel.from_xml_path(str(self.xml_path))

    def set_q(self, q: np.ndarray) -> None:
        q = np.asarray(q, dtype=float)
        if q.shape != (21,) or not np.all(np.isfinite(q)):
            raise ValueError("q must contain exactly 21 finite URDF joint angles in radians")
        self.data.qpos[self._qpos_indices] = q
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def points(self, q: np.ndarray, jacobian: bool = False):
        self.set_q(q)
        positions = self.data.site_xpos[self._site_ids].copy()
        if not jacobian:
            return positions
        jacobians = np.empty((len(self.point_names), 3, 21))
        jacp = np.empty((3, self.model.nv))
        jacr = np.empty_like(jacp)
        for index, site in enumerate(self._site_ids):
            mujoco.mj_jacSite(self.model, self.data, jacp, jacr, int(site))
            jacobians[index] = jacp[:, self._dof_indices]
        return positions, jacobians

    def close(self) -> None:
        if hasattr(self, "_temporary"):
            self._temporary.cleanup()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
