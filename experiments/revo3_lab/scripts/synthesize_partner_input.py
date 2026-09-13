"""Bone-preserving human skeleton IK fixtures, explicitly synthetic and unvalidated anatomically."""
import argparse
import copy
import json
from pathlib import Path
from lab_common import checked,configure,run,write_json,digest
configure()
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from ingest_manus import NAMES,FINGERS
from pinch_geometry import PadHand
from run_pinch import source_data
from vector_solver import Solver


def unit(x):return x/max(float(np.linalg.norm(x)),1e-12)
def hinge(a,b):
    axis=np.cross(a,b)
    return unit(axis) if np.linalg.norm(axis)>1e-6 else np.array([0.,1.,0.])
def angle(a,b):return float(np.arccos(np.clip(unit(a)@unit(b),-1,1)))
def rotation(axis,value):return Rotation.from_rotvec(axis*value).as_matrix()


class SkeletonIK:
    def __init__(self,points,finger):
        self.points=points.copy();source={'index':'Index','middle':'Middle','ring':'Ring','little':'Pinky'}[finger]
        self.thumb=[NAMES.index('Thumb_'+j) for j in ['CMC','MCP','DIP','TIP']]
        self.target=[NAMES.index(source+'_'+j) for j in ['MCP','PIP','DIP','TIP']]
        self.tv=np.diff(points[self.thumb],axis=0);self.fv=np.diff(points[self.target],axis=0)
        meta=points[self.target[0]]-points[NAMES.index(source+'_CMC')]
        self.axes=[hinge(self.tv[0],self.tv[1]),hinge(self.tv[1],self.tv[2]),
                   hinge(meta,self.fv[0]),hinge(self.fv[0],self.fv[1]),hinge(self.fv[1],self.fv[2])]
        old=np.array([angle(self.tv[0],self.tv[1]),angle(self.tv[1],self.tv[2]),
                      angle(meta,self.fv[0]),angle(self.fv[0],self.fv[1]),angle(self.fv[1],self.fv[2])])
        self.lo=np.array([-.8,-.8,-.8,-old[0],-old[1],-.35,-old[2],-old[3],-old[4]])
        self.hi=np.array([.8,.8,.8,1.7-old[0],1.7-old[1],.35,1.8-old[2],1.9-old[3],1.6-old[4]])
        if np.any(self.hi<=self.lo):raise RuntimeError('Invalid fixture bounds')

    def fk(self,q):
        p=self.points.copy();r=Rotation.from_rotvec(q[:3]).as_matrix()
        for i,vector in enumerate(self.tv):
            if i:r=r@rotation(self.axes[i-1],q[2+i])
            p[self.thumb[i+1]]=p[self.thumb[i]]+r@vector
        r=rotation(np.array([1.,0.,0.]),q[5])@rotation(self.axes[2],q[6])
        for i,vector in enumerate(self.fv):
            if i:r=r@rotation(self.axes[2+i],q[6+i])
            p[self.target[i+1]]=p[self.target[i]]+r@vector
        return p

    def solve(self,target_gap_m=.0035):
        a,b=self.thumb[-1],self.target[-1];center=(self.points[a]+self.points[b])*.5
        separation=target_gap_m*unit(self.points[a]-self.points[b])
        # A modest curled posture resolves the underdetermined point-contact fixture.
        prior=np.clip(np.array([0.,0.,0.,.15,.15,0.,.1,.5,.3]),self.lo+1e-7,self.hi-1e-7)
        if target_gap_m>.02:prior=np.clip(np.zeros(9),self.lo+1e-7,self.hi-1e-7)
        def residual(q):
            p=self.fk(q)
            return np.r_[(p[a]-p[b]-separation)/.001,((p[a]+p[b])*.5-center)/.04,.3*(q-prior)]
        fit=least_squares(residual,np.clip(np.zeros(9),self.lo+1e-7,self.hi-1e-7),bounds=(self.lo,self.hi),max_nfev=400,ftol=1e-10,xtol=1e-10,gtol=1e-10)
        p=self.fk(fit.x);gap=float(np.linalg.norm(p[a]-p[b]))
        if not fit.success or abs(gap-target_gap_m)>.0015:raise RuntimeError('Human skeleton IK did not create the requested separation')
        return fit.x,{'converged':fit.success,'nfev':fit.nfev,'gap_m':gap,'cost':float(fit.cost),'q':fit.x.tolist(),
                      'joint_delta_bounds_rad':[self.lo.tolist(),self.hi.tolist()],
                      'bounds_status':'synthetic engineering bounds, not calibrated anatomical limits',
                      'bone_lengths':'exactly retained from source skeleton','anatomical_or_surface_contact_validation':False}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--finger',choices=['middle','ring','little'],required=True);ap.add_argument('--robot-config',required=True)
    ap.add_argument('--open-gap-m',type=float,default=0.)
    args=ap.parse_args();cfg=json.loads(checked(args.robot_config).read_text())
    data,ps,path=source_data(cfg);idx=int(np.argmin(abs(data['timestamps']-cfg['source_pose_s'])));base=ps[idx]
    with run('synthetic_partner_input',{'finger':args.finger,'template_config':cfg,'duration_s':3.5,'fps':30}) as (out,mf):
        ik=SkeletonIK(base,args.finger);goal,details=ik.solve();ts=np.arange(0,3.5+1e-8,1/30)
        open_goal,open_details=ik.solve(args.open_gap_m) if args.open_gap_m else (np.zeros(9),None)
        details['open_ik']=open_details
        qs=[];points=[]
        for t in ts:
            u=min(1.,t/.75) if t<=2.75 else max(0.,(3.5-t)/.75)
            blend=10*u**3-15*u**4+6*u**5;q=open_goal+blend*(goal-open_goal);qs.append(q);points.append(ik.fk(q))
        points=np.array(points);parents=data['parent'];errors=[]
        for child,parent in enumerate(parents):
            if parent<0:continue
            length=np.linalg.norm(points[:,child]-points[:,parent],axis=1)
            errors.append(float(np.max(abs(length-np.linalg.norm(base[child]-base[parent])))))
        if max(errors)>1e-10:raise RuntimeError('Synthetic input changed bone lengths')
        stored=points.copy();stored[:,:,0]*=cfg['source_palm_x_sign']
        source=out/'source.npz';np.savez_compressed(source,timestamps=ts,names=data['names'],parent=parents,palm_positions_m=stored,
                       synthetic_joint_deltas=np.array(qs),synthetic_goal=goal)
        details.update(max_bone_length_error_m=max(errors),source_frame=idx,source_time_s=float(data['timestamps'][idx]),source_sha256=digest(path))
        write_json(out/'synthesis.json',details)
        cfg.update(source_kind='synthetic_ik',normalized_source=str(source),sequence='synthetic_'+args.finger+'_ik',source_pose_s=1.75,
                   source_window_s=[0.,3.5],synthetic_hold_window_s=[.75,2.75],explicit_contact_command=False,
                   source_annotation='Bone-preserving synthetic human IK close-hold-release fixture; not a recorded MANUS contact sequence.',
                   evaluation_label='Synthetic human skeleton IK input; no real-recording or hardware capability claim.')
        h=PadHand(checked(f"models/revo3/{cfg['model_id']}/scene.xml"),cfg)
        legacy=json.loads(checked('repos/Revo-Retargeting/experiments/revo3_lab/configs/vector_batch_03.json').read_text())
        candidate=next(c for c in legacy['candidates'] if c['name']=='vector_balanced')
        solver=Solver(h,{**legacy['common'],**candidate},1/30);baseline=[]
        for p in points:
            scale=h.robot_width/np.linalg.norm(p[NAMES.index('Index_MCP')]-p[NAMES.index('Pinky_MCP')])
            targets=(p[[NAMES.index(f+'_TIP') for f in FINGERS]]*scale)@h.basis.T+h.wrist
            q,_=solver.solve(p,targets);baseline.append(q)
        baseline_path=out/'legacy_baseline.npz';np.savez_compressed(baseline_path,timestamps=ts,q_target=np.array(baseline))
        cfg['legacy_baseline_npz']=str(baseline_path);write_json(out/'config.json',cfg)
        mf.update(template={'path':str(path),'sha256':digest(path),'frame':idx},synthesis=details,source_kind='synthetic_ik')
        print(json.dumps({'config':str(out/'config.json'),'synthesis':details}),flush=True)

if __name__=='__main__':main()
