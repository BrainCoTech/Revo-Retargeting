"""Three selected opposition states, two views each, using actual simulated poses."""
import json
from lab_common import checked,configure,run,write_json,digest
configure()
from render_pinch import Views,caption,png
import numpy as np
from pinch_geometry import PadHand
from run_pinch import source_data

CASES={'middle':'20260913T095245Z_pinch_clip_af7c6a28','ring':'20260913T094258Z_pinch_clip_277adcf4','little':'20260913T095459Z_pinch_clip_5f4a14aa'}

def main():
    with run('render_partners',{'cases':CASES}) as (out,mf):
        canvas=np.zeros((1160,1920,3),dtype=np.uint8);canvas[:]=[18,24,35];records=[]
        for col,(finger,run_id) in enumerate(CASES.items()):
            root=checked('runs/'+run_id);cfg=json.loads((root/'config.json').read_text());path=root/'pad_vector/trajectory.npz';tr=np.load(path,allow_pickle=False)
            i=int(np.argmin(abs(tr['source_times']-cfg['source_pose_s'])))
            if not tr['frame_valid_contact'][i]:raise RuntimeError('Selected source pose is not a valid-contact frame')
            h=PadHand(checked(f"models/revo3/{cfg['model_id']}/scene.xml"),cfg);data,_,_=source_data(cfg);view=Views(h,data)
            try:
                q=tr['q_actual'][i];view.camera.azimuth=150
                canvas[80:560,col*640:(col+1)*640]=view.robot(q)
                view.camera.lookat[:]=h.pad_fk(q)[0].mean(axis=0);view.camera.distance=.14;view.camera.azimuth=240
                canvas[620:1100,col*640:(col+1)*640]=view.robot(q)
            finally:view.close()
            source='SYNTHETIC IK' if cfg.get('source_kind')=='synthetic_ik' else 'RECORDED MANUS'
            m=json.loads((root/'metrics.json').read_text())['pad_vector']
            caption(canvas,finger.upper(),col*640+24,18,scale=3)
            caption(canvas,source,col*640+24,52)
            caption(canvas,'SAME ACTUAL STATE / SECOND VIEW',col*640+24,586)
            caption(canvas,f"VALID CONTACT {100*m['contact_fraction_during_source_close']:.1f}%",col*640+24,1120,(93,226,153))
            records.append({'finger':finger,'run_id':run_id,'frame':i,'source_time_s':float(tr['source_times'][i]),'trajectory_sha256':digest(path),'source_kind':source})
        png(out/'three_pairs.png',canvas);write_json(out/'frames.json',records)
        print(str(out),flush=True)

if __name__=='__main__':main()
