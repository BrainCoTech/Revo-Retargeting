"""Unchanged official kinematics, explicit simulation PD, and named landmark FK."""
import difflib
from pathlib import Path
import xml.etree.ElementTree as ET
from lab_common import checked, configure, digest, write_json
configure()
import mujoco
import numpy as np
from itertools import combinations

FINGERS = ['thumb', 'index', 'middle', 'ring', 'little']
MODEL_ID = 'right_official_pd_v1'


def prepare_model(model_id=MODEL_ID):
    source = checked('repos/brainco-description/revo3_system/mjcf/revo3_right.xml')
    if model_id not in [MODEL_ID, 'right_official_pd_v2']:
        raise ValueError('Unknown model version')
    folder = checked('models/revo3') / model_id
    folder.mkdir(parents=True, exist_ok=True)
    dest = folder / 'scene.xml'
    tree = ET.parse(source)
    root = tree.getroot()
    option = root.find('option')
    option.set('timestep', '0.002')
    option.set('integrator', 'implicitfast')
    option.set('gravity', '0 0 0')
    # These are software evaluation settings; no claim of hardware controller identification.
    for mesh in root.findall('.//asset/mesh'):
        mesh.set('file', str(checked(source.parent / mesh.attrib['file']).resolve()))
    defaults = root.find('default')
    ET.SubElement(defaults, 'joint', damping='0.03', armature='0.00005')
    coll = defaults.find("default[@class='collision']/geom")
    coll.set('friction', '0.8 0.005 0.0001')
    coll.set('condim', '4')
    coll.set('margin', '0')
    actuators = ET.SubElement(root, 'actuator')
    for joint in root.findall('.//worldbody//joint'):
        ET.SubElement(actuators, 'position', name=joint.attrib['name']+'_pd',
                      joint=joint.attrib['name'], kp='3', kv='0.08',
                      ctrlrange=joint.attrib['range'], forcerange=joint.attrib['actuatorfrcrange'])
    for finger in FINGERS:
        tip = root.find(f".//body[@name='right_{finger}_tip_Link']")
        ET.SubElement(tip, 'site', name=finger+'_tip', size='0.002', rgba='0 0.7 1 1')
        base_name = 'right_thumb_MCP_Link' if finger == 'thumb' else f'right_{finger}_MCP_Link'
        ET.SubElement(root.find(f".//body[@name='{base_name}']"), 'site', name=finger+'_base', size='0.001')
    exclusions = []
    if model_id == 'right_official_pd_v2':
        contact = ET.SubElement(root, 'contact')
        base = root.find(".//body[@name='right_hand_base_link']")
        for child in base.findall('body'):
            if child.find('joint') is not None:
                pair = ['right_hand_base_link', child.attrib['name']]
                exclusions.append(pair)
                ET.SubElement(contact, 'exclude', body1=pair[0], body2=pair[1])
    xml = ET.tostring(root, encoding='unicode')
    if dest.exists() and dest.read_text() != xml:
        raise RuntimeError('Model ID already exists with different settings')
    dest.write_text(xml)
    (folder/'source.diff').write_text(''.join(difflib.unified_diff(source.read_text().splitlines(True), xml.splitlines(True),
                                                               fromfile='official/revo3_right.xml', tofile='derived/scene.xml')))
    model = mujoco.MjModel.from_xml_path(str(dest))
    if (model.nq, model.nv, model.nu) != (21,21,21):
        raise RuntimeError('Expected 21 independent controlled hinge joints')
    receipt = {'id': model_id, 'adjacent_fixed_base_exclusions': exclusions,
               'exclusion_reason': 'Restore direct parent-child filtering for fixed base welded to world; all non-adjacent and inter-finger contacts retained',
               'collision_reference': 'https://mujoco.readthedocs.io/en/3.3.5/computation/#collision-detection', 'source_sha256': digest(source), 'derived_sha256': digest(dest),
               'joint_names': [model.joint(i).name for i in range(model.njnt)],
               'joint_limits_rad': model.jnt_range.tolist(), 'torque_limits_nm': model.actuator_forcerange.tolist(),
               'timestep_s': .002, 'kp': 3., 'kv': .08, 'gravity': [0,0,0],
               'friction': [.8,.005,.0001], 'hardware_capability': 'confirmed_by_user',
               'hardware_capability_test': 'skipped_by_user',
               'controller_status': 'software comparison controller; not identified from hardware',
               'landmarks': 'official tip_Link origins; distal collision contacts are proxies, not verified fingerpad contacts'}
    if not (folder/'model.json').exists():
        write_json(folder/'model.json', receipt)
    return dest


class Hand:
    def __init__(self, path):
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self.tip_ids = np.array([self.model.site(f+'_tip').id for f in FINGERS])
        self.base_ids = np.array([self.model.site(f+'_base').id for f in FINGERS])
        self.lo, self.hi = self.model.jnt_range.T.copy()
        self.names = [self.model.joint(i).name for i in range(self.model.nq)]
        self.q0 = np.clip(np.zeros(21), self.lo, self.hi)
        tips, bases = self.fk(self.q0)
        wrist = self.data.body('right_hand_base_link').xpos.copy()
        z = bases[2]-wrist; z /= np.linalg.norm(z)
        y = bases[1]-bases[4]; y -= y@z*z; y /= np.linalg.norm(y)
        self.basis = np.stack([np.cross(y,z),y,z],axis=1)
        self.wrist = wrist
        self.rest_tips = (tips-wrist)@self.basis
        self.rest_bases = (bases-wrist)@self.basis
        self.robot_width = np.linalg.norm(bases[1]-bases[4])
        self.distal_geoms = {i: finger for i in range(self.model.ngeom) for finger in range(5)
                             if self.model.geom(i).name == f'right_{FINGERS[finger]}_DIP_Link_collision_0'}
        bodies = sorted(set(int(self.model.geom_bodyid[i]) for i in range(self.model.ngeom)
                            if self.model.geom_contype[i]))
        self.collision_pairs = {pair: i for i, pair in enumerate(combinations(bodies, 2))}

    def fk(self, q, jac=False):
        self.data.qpos[:] = q
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        tips = self.data.site_xpos[self.tip_ids].copy()
        bases = self.data.site_xpos[self.base_ids].copy()
        if not jac:
            return tips, bases
        jp = np.zeros((5,3,21)); jr = np.zeros((3,21))
        for i, sid in enumerate(self.tip_ids):
            mujoco.mj_jacSite(self.model,self.data,jp[i],jr,int(sid))
        return tips, bases, jp

    def collision_penalty(self, allowed_target_penetration_m=0.0005, target_fingers=(0,1), allowed_pairs=None):
        """One deepest-contact residual per body pair, with a separation Jacobian.

        Call after fk(). Target distal contact may touch; interpenetration beyond
        the explicit tolerance and every other inter-body penetration are penalized.
        """
        m,d = self.model,self.data
        mujoco.mj_collision(m,d)
        residual = np.zeros(len(self.collision_pairs))
        jac = np.zeros((len(residual),21))
        j1,j2,jr = [np.zeros((3,21)) for _ in range(3)]
        for contact in d.contact:
            b1,b2 = int(m.geom_bodyid[contact.geom1]),int(m.geom_bodyid[contact.geom2])
            if b1 == b2:
                continue
            row = self.collision_pairs[tuple(sorted((b1,b2)))]
            fingers = {self.distal_geoms.get(int(contact.geom1),-1),self.distal_geoms.get(int(contact.geom2),-1)}
            allowed = (allowed_pairs.get(frozenset(fingers),0.) if allowed_pairs is not None else
                       allowed_target_penetration_m if fingers == set(target_fingers) else 0.)
            value = max(0.,-float(contact.dist)-allowed)/.01
            if value <= residual[row]:
                continue
            mujoco.mj_jac(m,d,j1,jr,contact.pos,b1)
            mujoco.mj_jac(m,d,j2,jr,contact.pos,b2)
            residual[row] = value
            jac[row] = -(contact.frame[:3]@(j2-j1))/.01
        return residual,jac
