"""Fixed retargeted goal under numerical setting perturbations, never a reachability search."""
import json
from pathlib import Path
from lab_common import configure,checked,run,write_json,digest
configure()
import numpy as np
from pinch_geometry import PadHand
from run_pinch import rollout


def main():
    source=checked('runs/20260913T090155Z_pinch_hold_f567ad65')
    cfg=json.loads((source/'config.json').read_text())
    goal=np.load(source/'pad_vector/trajectory.npz',allow_pickle=False)['goal']
    variants=[{'name':'dt_1ms','dt':.001,'friction_scale':1.,'tolerance':1e-8},
              {'name':'friction_80pct','dt':.002,'friction_scale':.8,'tolerance':1e-8},
              {'name':'friction_120pct_tight_solver','dt':.002,'friction_scale':1.2,'tolerance':1e-10}]
    with run('pinch_dynamics_check',{'source_run':source.name,'variants':variants}) as (out,mf):
        results={};mf['goal_sha256']=digest(source/'pad_vector/trajectory.npz')
        for variant in variants:
            h=PadHand(checked(f"models/revo3/{cfg['model_id']}/scene.xml"),cfg)
            h.model.opt.timestep=variant['dt'];h.model.geom_friction[:,0]*=variant['friction_scale'];h.model.opt.tolerance=variant['tolerance']
            results[variant['name']]=rollout(h,goal,cfg,out/variant['name'])
            print(variant['name'],json.dumps(results[variant['name']]),flush=True)
        write_json(out/'checks.json',results)

if __name__=='__main__':main()
