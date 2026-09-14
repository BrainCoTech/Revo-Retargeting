"""Compare a saved local shared replay to a saved DSW endpoint evaluation."""
import argparse
import json
from pathlib import Path
import numpy as np
from assets import ROOT, sha


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--local-run',type=Path,required=True)
    parser.add_argument('--reference-run',type=Path,required=True)
    args=parser.parse_args()
    manifest=json.loads((args.local_run/'manifest.json').read_text())
    reference_manifest=json.loads((args.reference_run/'manifest.json').read_text())
    if manifest['status'] != 'complete' or manifest['args']['solver'] != 'shared':
        raise ValueError('Expected a completed local shared-solver run')
    if manifest['source_sha256'] != reference_manifest['input']['sha256']:
        raise ValueError('Local and DSW runs must use the identical recording')
    rows=[json.loads(line) for line in (args.local_run/'frames.jsonl').read_text().splitlines()]
    output=[row['output'] for row in rows]
    fields={'target_tips_m':'target_tips_m','q_command_rad':'q_command','q_actual_rad':'q_actual',
            'actual_tips_m':'actual_tips_m','simulation_time_s':'simulation_times'}
    # Position inputs/timing should match exactly. Solver stopping on different
    # platforms may differ slightly: require <0.006 degrees and <1 micrometre.
    tolerances={'target_tips_m':1e-12,'q_command_rad':1e-4,'q_actual_rad':1e-4,
                'actual_tips_m':1e-6,'simulation_time_s':1e-10}
    report=dict(local_run=str(args.local_run),reference_run=str(args.reference_run),
                input_sha256=manifest['source_sha256'],frames=len(rows),differences={})
    with np.load(args.reference_run/'trajectory.npz',allow_pickle=False) as reference:
        np.testing.assert_array_equal(reference['joint_names'],manifest['joint_names'])
        np.testing.assert_array_equal(reference['times'],[row['timestamp_s'] for row in rows])
        np.testing.assert_array_equal(reference['target_valid'],[o['target_valid'] for o in output])
        for field,key in fields.items():
            local=np.asarray([o[field] for o in output])
            report['differences'][field]=dict(max_abs=float(np.max(abs(local-reference[key]))),
                                             absolute_tolerance=tolerances[field])
            np.testing.assert_allclose(local,reference[key],rtol=0,atol=tolerances[field])
        valid=reference['target_valid']
        errors=np.linalg.norm(np.asarray([o['actual_tips_m'] for o in output])-reference['target_tips_m'],axis=-1)[valid]*1000
        report['actual_tip_mean_mm']=float(errors.mean())
        report['actual_tip_p95_mm']=float(np.quantile(errors,.95))
    reference_details=json.loads((args.reference_run/'solver_details.json').read_text())
    assert [o['status'] for o in output] == [d['status'] for d in reference_details]
    assert not any(any(o['simulation_warning_counts']) for o in output)
    report['success']=True
    report['comparison_script_sha256']=sha(__file__)
    dest=ROOT/'validation'/f'shared_dsw_comparison_{args.local_run.name}.json'
    dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    print(f'Report: {dest}')


if __name__ == '__main__':
    main()
