"""Exercise both replay sources and verify causal saved dynamics; DSW only."""
import json
from pathlib import Path
import subprocess
import sys

from lab_common import checked, configure, run, write_json
configure()
import mujoco
import numpy as np
from check_endpoint_input import fixture, MP_INDICES
from revo3_model import Hand, prepare_model


def invoke(config):
    result = subprocess.run([sys.executable, str(Path(__file__).with_name('run_endpoint_evaluation.py')),
                             '--config', str(config)], text=True, capture_output=True)
    print(result.stdout, end='', flush=True)
    if result.returncode:
        raise RuntimeError(result.stderr + result.stdout)
    receipt = json.loads(result.stdout.strip().splitlines()[-1])
    if receipt['status'] != 'success':
        raise RuntimeError('Evaluation did not finish successfully')
    return checked('runs') / receipt['run_id']


def verify_causal_rollout(folder):
    """Reintegrate saved commands on their source intervals, independently of runner."""
    hand = Hand(prepare_model('right_official_pd_v2'))
    data = mujoco.MjData(hand.model)
    data.qpos[:] = hand.q0; data.ctrl[:] = hand.q0
    mujoco.mj_forward(hand.model, data)
    with np.load(folder/'trajectory.npz', allow_pickle=False) as trajectory:
        times = trajectory['times']; actual = trajectory['q_actual']
        np.testing.assert_array_equal(actual[0], hand.q0)
        assert trajectory['simulation_times'][0] == 0.
        for i, stamp in enumerate(times):
            while data.time < stamp-times[0]-1e-10:
                mujoco.mj_step(hand.model, data)
            np.testing.assert_allclose(actual[i], data.qpos, atol=1e-12)
            np.testing.assert_allclose(trajectory['simulation_times'][i], data.time, atol=1e-12)
            assert -1e-10 <= data.time-(stamp-times[0]) < hand.model.opt.timestep+1e-10
            data.ctrl[:] = trajectory['q_command'][i]
        assert np.isfinite(actual).all()
        return len(times)


def main():
    baseline = Path(__file__).resolve().parents[1]/'configs/endpoint_baseline.json'
    # Release the experiment lock before invoking each serial evaluation run.
    with run('endpoint_pipeline_fixture') as (fixture_dir, manifest):
        manifest['source_kind'] = 'synthetic_mediapipe_format'
        config = json.loads(baseline.read_text())
        path = fixture_dir/'frames.jsonl'
        detection = dict(side='Right', handedness_score=.95,
                         world_landmarks_m=fixture()[MP_INDICES].tolist())
        times = [0., 1/30, .15, .5, .5+1/30]
        rows = [dict(timestamp_s=t, frame_index=i, selected_side='Right',
                     detections=[] if i in (2,3) else [detection]) for i,t in enumerate(times)]
        path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        config['input'] = dict(source='mediapipe', source_kind='synthetic_mediapipe_format',
            file=str(path), input_field='raw', hand_side='Right', palm_x_sign=-1,
            scale=1., include_directions=False)
        config_path = fixture_dir/'config.json'
        write_json(config_path, config)
    mp_run = invoke(config_path)
    manus_run = invoke(baseline)
    mp_count = verify_causal_rollout(mp_run)
    manus_count = verify_causal_rollout(manus_run)
    assert mp_count == len(times)
    details = json.loads((mp_run/'solver_details.json').read_text())
    assert [d['status'] for d in details] == ['tracking','tracking','hold','return_neutral','tracking']
    source = [json.loads(line) for line in (mp_run/'source_frames.jsonl').read_text().splitlines()]
    assert all(row['source_kind'] == 'synthetic_mediapipe_format' for row in source)
    assert source[2]['native_points'] is None and source[3]['native_points'] is None
    with np.load(mp_run/'trajectory.npz', allow_pickle=False) as trajectory:
        np.testing.assert_array_equal(trajectory['times'], times)
        assert not trajectory['target_valid'][2:4].any()
    with run('check_endpoint_pipeline') as (out, manifest):
        report = dict(success=True, mediapipe_fixture_run=mp_run.name,
                      manus_run=manus_run.name, mediapipe_frames=mp_count,
                      manus_frames=manus_count, causal_command_reintegration=True,
                      real_mediapipe_validated=False)
        write_json(out/'checks.json', report)
        print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
