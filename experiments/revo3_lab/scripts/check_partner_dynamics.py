"""Numerical robustness of the three selected task mappings; fixed input and fixed goal."""
import json
from lab_common import checked,configure,run,write_json,digest
configure()
import numpy as np
from pinch_geometry import PadHand
from run_pinch import rollout

CASES={'middle':'20260913T095510Z_pinch_hold_70f0baed','ring':'20260913T094258Z_pinch_clip_277adcf4','little':'20260913T095459Z_pinch_clip_5f4a14aa'}

def main():
    variants=[('dt_1ms',.001,1.,1e-8),('friction_80pct',.002,.8,1e-8),('friction_120pct_tight_solver',.002,1.2,1e-10)]
    with run('partner_dynamics_check',{'cases':CASES,'variants':variants,'scope':'fixed retargeted goals under numerical perturbations; hardware capability tests skipped'}) as (out,mf):
        results={}
        for finger,run_id in CASES.items():
            source=checked('runs/'+run_id);cfg=json.loads((source/'config.json').read_text());path=source/'pad_vector/trajectory.npz';tr=np.load(path,allow_pickle=False)
            goal=tr['goal'] if 'goal' in tr.files else tr['mapped_goals'][np.argmin(abs(tr['source_clip_times']-cfg['source_pose_s']))]
            results[finger]={};mf.setdefault('source_hashes',{})[run_id]=digest(path)
            for name,dt,friction,tolerance in variants:
                h=PadHand(checked(f"models/revo3/{cfg['model_id']}/scene.xml"),cfg);h.model.opt.timestep=dt;h.model.geom_friction[:,0]*=friction;h.model.opt.tolerance=tolerance
                values=rollout(h,goal,cfg,out/finger/name);results[finger][name]=values
                print(finger,name,json.dumps({k:values[k] for k in ['contact_fraction','longest_break_s','max_rollout_penetration_m','mean_hold_force_n','held_input_task_passed']}),flush=True)
        write_json(out/'checks.json',results)
        write_json('reports/latest_partner_dynamics.json',{'run_id':mf['run_id'],'checks':results})

if __name__=='__main__':main()
