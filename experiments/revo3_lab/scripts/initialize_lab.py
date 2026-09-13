"""Inventory and register the single CPU DSW lab; uses only the standard library."""

import datetime as dt
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess


ROOT = Path('/mnt/workspace/wensheng/revo3-retargeting-lab')
INSTANCE = 'dsw-86hev0txafus51z1hp'


def output(command):
    result = subprocess.run(command, capture_output=True, text=True, check=True)
    return result.stdout.strip()


def main():
    for parent in [ROOT, *ROOT.parents]:
        if parent.is_symlink():
            raise RuntimeError(f'Symlink not allowed: {parent}')
    if not ROOT.parent.is_dir():
        raise RuntimeError('Expected project namespace is missing')
    mount = output(['findmnt', '-T', str(ROOT.parent), '-n', '-o', 'TARGET,SOURCE,FSTYPE'])
    if mount.split()[-1] not in {'nfs', 'nfs4', 'ext4', 'xfs'}:
        raise RuntimeError(f'Unverified filesystem: {mount}')
    stamp = dt.datetime.now(dt.timezone.utc).isoformat()
    cpu_quota = int(Path('/sys/fs/cgroup/cpu/cpu.cfs_quota_us').read_text())
    cpu_period = int(Path('/sys/fs/cgroup/cpu/cpu.cfs_period_us').read_text())
    memory = int(Path('/sys/fs/cgroup/memory/memory.limit_in_bytes').read_text())
    resources = {
        'checked_at': stamp, 'instance_id': INSTANCE, 'hostname': platform.node(),
        'uid': os.getuid(), 'os_release': Path('/etc/os-release').read_text(),
        'system_python': {'path': '/usr/bin/python3', 'version': platform.python_version()},
        'cpu_quota': cpu_quota / cpu_period if cpu_quota > 0 else None,
        'cpu_affinity': len(os.sched_getaffinity(0)), 'memory_limit_bytes': memory,
        'gpu_devices': [str(p) for p in Path('/dev').glob('nvidia*')],
        'mount': mount, 'mount_alias': '/mnt/data_nas',
        'storage_note': 'NFS df reports pool capacity, not project quota. Alias is not a backup. Provider quota/lifecycle not verified.',
        'disk_usage': dict(zip(('total', 'used', 'free'), shutil.disk_usage(ROOT.parent))),
        'tools': {name: shutil.which(name) for name in ['python3', 'uv', 'conda', 'git', 'blender']},
        'render_libraries': [line.strip() for line in output(['/sbin/ldconfig', '-p']).splitlines()
                             if any(name in line for name in ['OSMesa', 'libEGL', 'libGL.so'])],
        'status': 'cpu_preflight_passed',
    }
    registry = ROOT / 'registry'
    if (registry / 'lab.json').exists():
        old = json.loads((registry / 'lab.json').read_text())
        if old.get('root') != str(ROOT) or old.get('instance_id') != INSTANCE:
            raise RuntimeError('Existing lab identity mismatch')
    elif ROOT.exists() and any(ROOT.iterdir()):
        raise RuntimeError('Refusing to adopt an unregistered nonempty lab')
    for name in ['registry', 'repos/Revo-Retargeting/experiments/revo3_lab/scripts',
                 'envs', 'toolchains', 'locks', 'data/raw/manus_official/v1',
                 'data/quarantine', 'models', 'runs', 'reports', 'exports', 'cache', 'tmp']:
        path = ROOT / name
        if path.resolve() != path:
            raise RuntimeError(f'Noncanonical path: {path}')
        path.mkdir(parents=True, exist_ok=True)
    probe = ROOT / 'tmp/preflight_write_probe'
    with probe.open('x') as stream:
        stream.write('revo3-lab-preflight\n')
        stream.flush()
        os.fsync(stream.fileno())
    if probe.read_text() != 'revo3-lab-preflight\n':
        raise RuntimeError('NFS write verification failed')
    probe.unlink()
    lab = {
        'schema_version': 1, 'root': str(ROOT), 'instance_id': INSTANCE,
        'created_at': old['created_at'] if 'old' in locals() else stamp,
        'owner': 'wensheng', 'profile': 'cpu-2c-24g',
        'environment': str(ROOT / 'envs/core-py312'),
        'local_artifact_root': '/Users/woltim/code/Revo-Retargeting/artifacts/revo3_lab',
        'worker_count': 1, 'threads_per_worker': 1,
        'budgets_bytes': {'cache': 4 * 2**30, 'tmp': 2 * 2**30, 'runs': 4 * 2**30},
        'stop_new_jobs_below_free_bytes': 5 * 2**30,
        'source_code_status': 'file snapshot; not a full repository checkout',
        'outside_root_installs': [],
    }
    for name, content in [('lab.json', lab), ('resources.json', resources)]:
        path = registry / name
        stage = path.with_suffix('.json.pending')
        stage.write_text(json.dumps(content, indent=2, ensure_ascii=False) + '\n')
        stage.replace(path)
    for name in ['runs.jsonl', 'migrations.jsonl']:
        (registry / name).touch(exist_ok=True)
    print(json.dumps({'lab': lab, 'resources': resources}, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
