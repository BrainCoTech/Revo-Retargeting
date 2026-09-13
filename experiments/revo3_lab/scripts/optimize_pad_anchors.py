"""Bounded contact-anchor tuning inside fixed pads for one synthetic retargeting task."""
import copy
import json
from pathlib import Path
import shutil
from lab_common import checked,configure,run,write_json,digest
configure()
import numpy as np
from pinch_geometry import PadHand,solve_pads
from run_pinch import source_data,old_targets,rollout


def main():
    cfg=json.loads(checked('repos/Revo-Retargeting/experiments/revo3_lab/configs/pinch_little_synthetic_v1.json').read_text())
    cfg['solver']['collision_weight']=10000.
    with run('partner_anchor_tuning',{'template':cfg,'width_offsets_m':[-.004,0.,.004],
             'scope':'fixed human IK input, fixed pad regions; optimize retargeting contact anchors, no hardware workspace search'}) as (out,mf):
        data,points,path=source_data(cfg);old,old_path=old_targets(data,cfg)
        idx=int(np.argmin(abs(data['timestamps']-cfg['source_pose_s'])));p=points[idx]
        mf.update(source={'path':str(path),'sha256':digest(path),'frame':idx},baseline={'path':str(old_path),'sha256':digest(old_path)})
        candidates=[]
        for i,tw in enumerate([-.004,0.,.004]):
            for j,fw in enumerate([-.004,0.,.004]):
                c=copy.deepcopy(cfg);c['pads']['thumb']['width_center_m']=tw;c['pads']['little']['width_center_m']=fw
                h=PadHand(checked(f"models/revo3/{c['model_id']}/scene.xml"),c)
                prior=old['q_target'][idx].copy();prior[h.inactive_dofs]=0
                q,fit=solve_pads(h,p,prior,c);h.fk(q);h.collision_penalty()
                pen=max([0.]+[max(0.,-float(x.dist)) for x in h.data.contact if {int(x.geom1),int(x.geom2)}!=set(h.pad_geom_ids)])
                score=fit['pad_center_gap_m']*1000+pen*100000+max(0.,fit['opposed_angle_deg']-25)*.2
                item={'name':f'anchor_{i}_{j}','config':c,'fit':fit,'non_target_fk_penetration_m':pen,'score':score}
                folder=out/item['name'];folder.mkdir();write_json(folder/'config.json',c);write_json(folder/'fit.json',item)
                np.savez_compressed(folder/'goal.npz',q=q,prior=prior);candidates.append(item)
                print(item['name'],json.dumps({'fit':fit,'non_target_fk_penetration_m':pen,'score':score}),flush=True)
        tested=[]
        for item in sorted(candidates,key=lambda x:x['score'])[:3]:
            folder=out/item['name'];c=item['config'];h=PadHand(checked(f"models/revo3/{c['model_id']}/scene.xml"),c)
            arrays=np.load(folder/'goal.npz',allow_pickle=False)
            metrics={'old_vector':rollout(h,arrays['prior'],c,folder/'old_vector'),
                     'pad_vector':rollout(h,arrays['q'],c,folder/'pad_vector')}
            write_json(folder/'metrics.json',metrics);write_json(folder/'pads.json',h.pads)
            tested.append({'candidate':item['name'],'metrics':metrics,'score':item['score']})
            print('DYNAMIC',item['name'],json.dumps(metrics['pad_vector']),flush=True)
        selected=min(tested,key=lambda x:(not x['metrics']['pad_vector']['held_input_task_passed'],-x['metrics']['pad_vector']['contact_fraction'],x['score']))
        write_json(out/'selection.json',{'selected':selected['candidate'],'candidates':candidates,'tested':tested})
        root=out/selected['candidate']
        for name in ['config.json','metrics.json','pads.json','fit.json']:shutil.copyfile(root/name,out/name)
        for name in ['old_vector','pad_vector']:shutil.copytree(root/name,out/name)
        write_json('reports/latest_partner_anchor.json',{'run_id':mf['run_id'],'selected':selected})

if __name__=='__main__':main()
