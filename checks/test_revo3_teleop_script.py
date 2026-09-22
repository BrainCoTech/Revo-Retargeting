"""Exercise launch orchestration with a fake ros2; never access ROS or hardware."""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def sandbox(tmp_path):
    shutil.copytree(ROOT / 'scripts', tmp_path / 'scripts')
    (tmp_path / 'install').mkdir()
    (tmp_path / 'install/setup.bash').write_text('# mock workspace\n')
    bindir = tmp_path / 'bin'
    bindir.mkdir()
    fake = bindir / 'ros2'
    fake.write_text('''#!/usr/bin/python3
import json, os, signal, sys, time
from pathlib import Path
root = Path(os.environ['FAKE_ROOT'])
args = sys.argv[1:]
if any(arg.endswith(':=') for arg in args):
    print('Invalid launch argument: empty value', file=sys.stderr)
    sys.exit(2)
if args[:3] == ['pkg', 'prefix', '--share']:
    print(root / 'share' / args[3])
    sys.exit(0)
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
(root / ('call-' + str(os.getpid()) + '.json')).write_text(json.dumps(args))
if os.environ.get('FAIL_ADAPTER') == '1' and args[:2] == ['launch', 'hand_input_adapters']:
    time.sleep(.25)
    sys.exit(7)
while True: time.sleep(.05)
''')
    fake.chmod(0o755)
    for rel in ['share/hand_input_adapters/launch/dv1_input.launch.py',
                'share/manus_revo3_retarget/config/input_humandex.yaml',
                'sdk/description/urdf/Revo_Human_DV1_URDF_Bimanual.urdf']:
        path = tmp_path / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    env = os.environ.copy()
    for key in ['START_MANUS_PUBLISHER', 'START_REVO3_DRIVER']:
        env.pop(key, None)
    env.update(PATH=str(bindir) + ':' + env['PATH'], FAKE_ROOT=str(tmp_path))
    return tmp_path, env


def calls(root):
    return [json.loads(p.read_text()) for p in root.glob('call-*.json')]


def launch(sandbox, args, count):
    root, env = sandbox
    proc = subprocess.Popen(['bash', str(root / 'scripts/teleop.sh'), *args],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)
    try:
        deadline = time.monotonic() + 8
        while len(calls(root)) < count and time.monotonic() < deadline:
            if proc.poll() is not None:
                break
            time.sleep(.05)
        assert len(calls(root)) == count, proc.stdout.read() if proc.poll() is not None else calls(root)
        proc.send_signal(signal.SIGTERM)
        output, _ = proc.communicate(timeout=10)
        assert proc.returncode == 130, output
        # Managed process groups must all be gone after stopping the entrypoint.
        for path in root.glob('call-*.json'):
            pid = int(path.stem.split('-')[1])
            with pytest.raises(ProcessLookupError):
                os.kill(pid, 0)
        return calls(root)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
        # Even a failed assertion may not leave fake launch processes behind.
        for path in root.glob('call-*.json'):
            try:
                os.kill(int(path.stem.split('-')[1]), signal.SIGKILL)
            except ProcessLookupError:
                pass


def pipeline(records):
    return next(row for row in records if row[:3] == ['launch', 'manus_revo3_retarget', 'pipeline_launch.py'])


@pytest.mark.parametrize('mode,sides', [('right', ['right']), ('both', ['left', 'right'])])
def test_dv1_starts_fk_per_hand_and_external_retarget(sandbox, mode, sides):
    root, _ = sandbox
    records = launch(sandbox, ['revo3', mode, 'input_source:=dv1', f'sdk_path:={root / "sdk"}'], len(sides) + 2)
    adapters = [row for row in records if row[1] == 'hand_input_adapters']
    assert len(adapters) == len(sides)
    assert {next(x for x in row if x.startswith('hand_mode:=')) for row in adapters} == {f'hand_mode:={side}' for side in sides}
    assert all(row[2] == 'dv1_input.launch.py' for row in adapters)
    target = pipeline(records)
    assert 'input_source:=external' in target
    assert 'launch_manus_publisher:=0' in target
    assert f'input_config:={root}/share/manus_revo3_retarget/config/input_humandex.yaml' in target
    assert {row[1] for row in records} == {'hand_input_adapters', 'manus_revo3_retarget', 'revo3_driver'}


@pytest.mark.parametrize('source,publisher', [('manus', '1'), ('humandex', '0'), ('external', '0')])
def test_existing_sources_remain_supported(sandbox, source, publisher):
    args = ['right'] if source == 'manus' else ['right', f'input_source:={source}']
    records = launch(sandbox, args, 2)
    assert f'input_source:={source}' in pipeline(records)
    assert f'launch_manus_publisher:={publisher}' in pipeline(records)
    assert not any(row[1] == 'hand_input_adapters' for row in records)


def test_custom_topics_reach_adapter_and_retarget(sandbox):
    root, _ = sandbox
    records = launch(sandbox, ['right', 'input_source:=dv1', f'sdk_path:={root / "sdk"}',
                               'right_joint_topic:=/custom/joints', 'right_input_topic:=/custom/kinematics'], 3)
    adapter = next(row for row in records if row[1] == 'hand_input_adapters')
    assert 'joint_topic:=/custom/joints' in adapter
    assert 'output_topic:=/custom/kinematics' in adapter
    assert 'right_input_topic:=/custom/kinematics' in pipeline(records)
    assert not any(x.startswith('right_joint_topic:=') for x in pipeline(records))


@pytest.mark.parametrize('args', [
    ['right', 'input_source:=dv1', 'urdf_path:=/definitely/missing/revo.urdf'],
    ['right', 'hand_mode:=left'],
    ['right', 'hand_type:=left'],
    ['right', 'input_source:=unknown'],
    ['right', 'launch_manus_publisher:=invalid'],
])
def test_invalid_input_is_rejected_before_driver(sandbox, args):
    root, env = sandbox
    result = subprocess.run(['bash', str(root / 'scripts/teleop.sh'), *args], env=env,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode != 0
    assert not calls(root), result.stdout + result.stderr


def test_adapter_failure_propagates_and_stops_siblings(sandbox):
    root, env = sandbox
    env['FAIL_ADAPTER'] = '1'
    try:
        result = subprocess.run(
            ['bash', str(root / 'scripts/teleop.sh'), 'right', 'input_source:=dv1',
             f'sdk_path:={root / "sdk"}'],
            env=env, capture_output=True, text=True, timeout=10)
        assert result.returncode == 7, result.stdout + result.stderr
        assert len(calls(root)) == 3
        for path in root.glob('call-*.json'):
            with pytest.raises(ProcessLookupError):
                os.kill(int(path.stem.split('-')[1]), 0)
    finally:
        for path in root.glob('call-*.json'):
            try:
                os.kill(int(path.stem.split('-')[1]), signal.SIGKILL)
            except ProcessLookupError:
                pass


def test_custom_adapter_configuration_is_forwarded(sandbox):
    root, _ = sandbox
    shared = root / 'adapter.yaml'
    right = root / 'right-adapter.yaml'
    shared.write_text('parameters: {}\n')
    right.write_text('parameters: {}\n')
    records = launch(sandbox, ['both', 'input_source:=dv1', f'sdk_path:={root / "sdk"}',
                               f'adapter_config:={shared}', f'right_adapter_config:={right}'], 4)
    adapters = [row for row in records if row[1] == 'hand_input_adapters']
    left_call = next(row for row in adapters if 'hand_mode:=left' in row)
    right_call = next(row for row in adapters if 'hand_mode:=right' in row)
    assert f'adapter_config:={shared}' in left_call
    assert f'adapter_config:={right}' in right_call
    assert not any(x.startswith('adapter_config:=') for x in pipeline(records))
