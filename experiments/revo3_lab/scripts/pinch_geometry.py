"""Anatomical surface patches on unchanged official distal meshes and pad-target IK."""
import json
from pathlib import Path
import struct
from lab_common import checked,configure,digest
configure()
import mujoco
import numpy as np
from scipy.optimize import least_squares
from revo3_model import Hand
from ingest_manus import NAMES


def unit(v):
    return np.asarray(v)/max(float(np.linalg.norm(v)),1e-12)


class AllPadGeometry:
    """One fixed anatomical pad definition per finger, shared by every motion.

    Geometry/Jacobians only; no partner selection or task solver lives here.
    """
    def __init__(self,hand):
        self.hand=hand;self.bodies=[];self.positions=[];self.normals=[]
        self.axes=np.array([[0.,1.,0.]]+[[0.,0.,1.]]*4)
        for finger in ['thumb','index','middle','ring','little']:
            thumb=finger=='thumb'
            definition=dict(normal_axis=2 if thumb else 0,long_axis=1 if thumb else 2,
                width_axis=0 if thumb else 1,long_center_m=.020 if thumb else .018,width_center_m=0.)
            path=checked(f'repos/brainco-description/revo3_system/meshes/hands/collision/right/right_{finger}_DIP_Link.STL')
            pad=surface_patch(path,definition)
            self.bodies.append(hand.model.body(f'right_{finger}_DIP_Link').id)
            self.positions.append(pad['position_m']);self.normals.append(pad['normal'])

    def evaluate(self):
        h=self.hand;positions=[];normals=[];axes=[];jps=[];jns=[];jas=[]
        for body,p,n,a in zip(self.bodies,self.positions,self.normals,self.axes):
            rot=h.data.xmat[body].reshape(3,3)
            p=h.data.xpos[body]+rot@p;n=rot@n;a=rot@a
            jp,jr=np.zeros((3,h.model.nv)),np.zeros((3,h.model.nv))
            mujoco.mj_jac(h.model,h.data,jp,jr,p,body)
            positions.append(p);normals.append(n);axes.append(a)
            jps.append(jp);jns.append(np.cross(jr.T,n).T);jas.append(np.cross(jr.T,a).T)
        return tuple(np.array(x) for x in [positions,normals,axes,jps,jns,jas])


def surface_patch(path,definition):
    raw=path.read_bytes();count=struct.unpack_from('<I',raw,80)[0]
    dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
    tris=np.frombuffer(raw,dtype=dtype,count=count,offset=84)['vertices'].astype(float)
    normal=definition['normal_axis']; axes=[definition['width_axis'],definition['long_axis']]
    target=np.array([definition['width_center_m'],definition['long_center_m']])
    hits=[]
    for tri in tris:
        face=np.cross(tri[1]-tri[0],tri[2]-tri[0]);length=np.linalg.norm(face)
        if length<1e-14:continue
        face/=length
        if face[normal]<.5:continue
        projected=(tri[1:,axes]-tri[0,axes]).T
        if abs(np.linalg.det(projected))<1e-15:continue
        uv=np.linalg.solve(projected,target-tri[0,axes])
        if uv.min()>=-1e-7 and uv.sum()<=1+1e-7:
            point=tri[0]+uv@(tri[1:]-tri[0]);hits.append((point,face))
    if not hits:raise RuntimeError('Pad surface ray missed the official mesh')
    point,face=max(hits,key=lambda x:x[0][normal])
    return {'position_m':point.tolist(),'normal':face.tolist(),'mesh_sha256':digest(path),
            'mesh':str(path),'definition':definition,'surface_method':'positive-facing STL triangle ray intersection at fixed tangential patch center'}


class PadHand(Hand):
    def __init__(self,path,config):
        super().__init__(path);self.pad_config=config['pads'];self.pads={}
        self.target_finger=config.get('target_finger','index')
        partners=['index','middle','ring','little']
        if self.target_finger not in partners:raise ValueError('Unknown opposition partner')
        self.target_number=partners.index(self.target_finger)+1
        self.source_finger=['Index','Middle','Ring','Pinky'][self.target_number-1]
        self.pad_fingers=['thumb',self.target_finger]
        start=5+4*(self.target_number-1)
        self.active_dofs=np.r_[np.arange(5),np.arange(start,start+4)]
        self.inactive_dofs=np.setdiff1d(np.arange(21),self.active_dofs)
        self.source_tip_index=NAMES.index(self.source_finger+'_TIP')
        self.pad_body_ids=[];self.pad_geom_ids=[]
        for finger in self.pad_fingers:
            mesh=checked(f'repos/brainco-description/revo3_system/meshes/hands/collision/right/right_{finger}_DIP_Link.STL')
            self.pads[finger]=surface_patch(mesh,config['pads'][finger])
            self.pad_body_ids.append(self.model.body(f'right_{finger}_DIP_Link').id)
            self.pad_geom_ids.append(self.model.geom(f'right_{finger}_DIP_Link_collision_0').id)
        self.pad_positions=np.array([p['position_m'] for p in self.pads.values()])
        self.pad_normals=np.array([p['normal'] for p in self.pads.values()])
        self.pad_axis=np.array([[0,1,0],[0,0,1]])

    def pad_fk(self,q,jacobian=False):
        self.fk(q)
        pos=[];norm=[];axis=[];jpos=[];jnorm=[];jaxis=[]
        for i,bid in enumerate(self.pad_body_ids):
            rotation=self.data.xmat[bid].reshape(3,3)
            p=self.data.xpos[bid]+rotation@self.pad_positions[i]
            n=rotation@self.pad_normals[i];a=rotation@self.pad_axis[i]
            pos.append(p);norm.append(n);axis.append(a)
            if jacobian:
                jp,jr=np.zeros((3,21)),np.zeros((3,21))
                mujoco.mj_jac(self.model,self.data,jp,jr,p,bid)
                jpos.append(jp);jnorm.append(np.cross(jr.T,n).T);jaxis.append(np.cross(jr.T,a).T)
        if jacobian:return np.array(pos),np.array(norm),np.array(axis),np.array(jpos),np.array(jnorm),np.array(jaxis)
        return np.array(pos),np.array(norm),np.array(axis)

    def point_in_pad(self,finger,position,data):
        i=self.pad_fingers.index(finger);bid=self.pad_body_ids[i]
        local=data.xmat[bid].reshape(3,3).T@(position-data.xpos[bid])
        cfg=self.pad_config[finger]
        return bool(cfg['long_range_m'][0]<=local[cfg['long_axis']]<=cfg['long_range_m'][1] and
                    cfg['width_range_m'][0]<=local[cfg['width_axis']]<=cfg['width_range_m'][1] and
                    local[cfg['normal_axis']]>0.003)

    def collision_penalty(self,allowed_target_penetration_m=.0005):
        return super().collision_penalty(allowed_target_penetration_m,(0,self.target_number))


def source_target(hand,points):
    ids=[NAMES.index(n) for n in ['Thumb_DIP','Thumb_TIP',hand.source_finger+'_DIP',hand.source_finger+'_TIP']]
    thd,th,ind,ix=points[ids]
    width=np.linalg.norm(points[NAMES.index('Index_MCP')]-points[NAMES.index('Pinky_MCP')])
    scale=hand.robot_width/width
    center=((th+ix)*.5*scale)@hand.basis.T+hand.wrist
    directions=np.array([unit(th-thd),unit(ix-ind)])@hand.basis.T
    toward=(th+thd-ix-ind)*.5
    toward_world=toward@hand.basis.T
    normal=unit(toward_world-(toward_world@directions[1])*directions[1])
    gap=float(np.linalg.norm(th-ix))
    return {'center':center,'directions':directions,'index_normal':normal,'source_gap_m':gap,'scale':float(scale)}


def pad_tangent(axis,normal,jaxis,jnormal):
    tangent=axis-normal*(normal@axis)
    length=np.linalg.norm(tangent)
    if length<1e-8:raise ValueError('Degenerate pad sliding direction')
    jac=jaxis-jnormal*(normal@axis)-np.outer(normal,axis@jnormal+normal@jaxis)
    tangent/=length
    jac=(np.eye(3)-np.outer(tangent,tangent))@jac/length
    return tangent,jac


def solve_pads(hand,points,previous,config,close=True,slide_m=0.,posture_target=None):
    target=source_target(hand,points);cfg=config['solver'];cache={}
    posture_weight=cfg.get('partner_posture_weight',0.)
    if posture_weight:
        from vector_solver import finger_angles
        posture=finger_angles(points,hand)
    desired_gap=-cfg['preload_m'] if close else max(0.,target['source_gap_m']-.015)*target['scale']*cfg.get('open_gap_scale',1.)
    active=hand.active_dofs
    def evaluate(x):
        if 'x' in cache and np.array_equal(x,cache['x']):return cache['r'],cache['j']
        q=np.zeros(21);q[active]=x
        p,n,a,jp,jn,ja=hand.pad_fk(q,True)
        # Index longitudinal axis projected into its actual mesh tangent plane.
        # Zero offset preserves the existing opposition objective exactly.
        tangent,jt=pad_tangent(a[1],n[1],ja[1],jn[1])
        r=[(p[0]-p[1]-n[1]*desired_gap-tangent*slide_m)/cfg['pad_position_scale_m'],
           (n[0]+n[1])/cfg['opposed_normal_scale'],
           ((p[0]+p[1])*.5-target['center'])/cfg['center_scale_m'],
           (a-target['directions']).ravel()/cfg['source_direction_scale'],
           .3*(n[1]-target['index_normal']),np.sqrt(cfg['temporal_weight'])*(x-previous[active])]
        j=[(jp[0]-jp[1]-jn[1]*desired_gap-jt*slide_m)/cfg['pad_position_scale_m'],
           (jn[0]+jn[1])/cfg['opposed_normal_scale'],
           .5*(jp[0]+jp[1])/cfg['center_scale_m'],ja.reshape(6,21)/cfg['source_direction_scale'],
           .3*jn[1],np.sqrt(cfg['temporal_weight'])*np.eye(21)[active]]
        cr,cj=hand.collision_penalty(max(cfg['preload_m'],.0005))
        r.append(np.sqrt(cfg['collision_weight'])*cr);j.append(np.sqrt(cfg['collision_weight'])*cj)
        if posture_weight:
            r.append(np.sqrt(posture_weight)*(q[active[5:]]-posture[active[5:]]))
            j.append(np.sqrt(posture_weight)*np.eye(21)[active[5:]])
        # Optional input-relative flexion tracking resolves redundant contact IK.
        # Defaults preserve every existing opposition solve.
        if posture_target is not None:
            dofs=np.asarray(cfg['rubbing_posture_dofs'],dtype=int)
            weight=np.sqrt(cfg['rubbing_posture_weight'])
            r.append(weight*(q[dofs]-posture_target[dofs]))
            j.append(np.broadcast_to(weight,len(dofs))[:,None]*np.eye(21)[dofs])
        residual=np.concatenate(r);jac=np.vstack(j)[:,active]
        cache.update(x=x.copy(),r=residual,j=jac)
        return residual,jac
    fit=least_squares(lambda x:evaluate(x)[0],np.clip(previous[active],hand.lo[active]+1e-7,hand.hi[active]-1e-7),
                      jac=lambda x:evaluate(x)[1],bounds=(hand.lo[active],hand.hi[active]),max_nfev=cfg['max_nfev'],
                      ftol=1e-8,xtol=1e-8,gtol=1e-8)
    q=np.zeros(21);q[active]=fit.x
    p,n,_=hand.pad_fk(q)
    return q,{'success':fit.success,'nfev':fit.nfev,'cost':float(fit.cost),'pad_center_gap_m':float(np.linalg.norm(p[0]-p[1])),
              'opposed_angle_deg':float(np.degrees(np.arccos(np.clip(-n[0]@n[1],-1,1)))),
              'source_gap_m':target['source_gap_m'],'desired_gap_m':desired_gap,'center_target_m':target['center'].tolist()}
