"""Verify installer opt-in without invoking apt, pip, Git mutations or SDK downloads."""
import os
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('option', [None, '--with-manus', 'MANUS_SDK_ARCHIVE', 'MANUS_SDK_DIR', 'MANUS_SDK_URL'])
def test_manus_install_and_check_are_explicit_opt_in(tmp_path, option):
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    shutil.copy(ROOT / 'scripts/install_revo3_deps.sh', scripts)
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    for name in ('sudo', 'git', 'python'):
        path = bin_dir / name
        path.write_text('#!/bin/bash\nexit 0\n')
        path.chmod(0o755)
    for name in ('install_manus_sdk.sh', 'check_system_deps.sh'):
        path = scripts / name
        path.write_text('#!/bin/bash\nprintf "%s\\n" "$0 $*" >> "$CALL_LOG"\n')
        path.chmod(0o755)
    env = os.environ.copy()
    for key in ('PYTHON', 'MANUS_SDK_ARCHIVE', 'MANUS_SDK_DIR', 'MANUS_SDK_URL'):
        env.pop(key, None)
    env.update(PATH=str(bin_dir) + ':' + env['PATH'], CALL_LOG=str(tmp_path / 'calls'))
    args = []
    if option == '--with-manus':
        args.append(option)
    elif option:
        env[option] = '/explicit/sdk/source'
    result = subprocess.run(['bash', str(scripts / 'install_revo3_deps.sh'), *args],
                            env=env, capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stdout + result.stderr
    calls = (tmp_path / 'calls').read_text()
    assert ('install_manus_sdk.sh' in calls) == bool(option)
    assert ('--with-manus' in calls) == bool(option)
    assert 'check_system_deps.sh revo3' in calls
