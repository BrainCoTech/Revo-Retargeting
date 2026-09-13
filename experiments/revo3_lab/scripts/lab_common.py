"""Registered, CPU-only experiment runtime and immutable run bookkeeping."""
import contextlib
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import traceback
import uuid

ROOT = Path('/mnt/workspace/wensheng/revo3-retargeting-lab')


def stamp():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def checked(path):
    path = Path(path)
    if not path.is_absolute():
        path = ROOT / path
    if ROOT.resolve() != ROOT or not path.resolve().is_relative_to(ROOT):
        raise ValueError(f'Path escapes registered root: {path}')
    return path


def digest(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def write_json(path, value):
    path = checked(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = checked(path.with_name(path.name + '.tmp-' + uuid.uuid4().hex[:8]))
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    os.replace(tmp, path)


def append_json(path, value):
    with checked(path).open('a') as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        f.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n')
        f.flush()
        os.fsync(f.fileno())


def configure():
    registration = json.loads(checked('registry/lab.json').read_text())
    if registration['root'] != str(ROOT) or registration['worker_count'] != 1:
        raise RuntimeError('Unrecognized runtime registration')
    os.environ['PYTHONNOUSERSITE'] = '1'
    for key in ['OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS']:
        os.environ[key] = '1'
    for key, rel in {'TMPDIR': 'tmp', 'XDG_CACHE_HOME': 'cache', 'MPLCONFIGDIR': 'cache/matplotlib',
                     'PYTHONPYCACHEPREFIX': 'cache/pycache', 'PIP_CACHE_DIR': 'cache/pip',
                     'HF_HOME': 'cache/huggingface', 'TORCH_HOME': 'cache/torch',
                     'UV_CACHE_DIR': 'cache/uv', 'UV_PYTHON_INSTALL_DIR': 'toolchains/python',
                     'TORCH_EXTENSIONS_DIR': 'cache/torch_extensions'}.items():
        path = checked(rel)
        path.mkdir(parents=True, exist_ok=True)
        os.environ[key] = str(path)
    return registration


def storage_check():
    reg = configure()
    usage = {}
    for folder, limit in reg['budgets_bytes'].items():
        usage[folder] = sum(checked(p).stat().st_size for p in checked(folder).rglob('*') if p.is_file())
        if usage[folder] >= limit:
            raise RuntimeError(f'Storage budget exhausted: {folder}')
    if shutil.disk_usage(ROOT).free < reg['stop_new_jobs_below_free_bytes']:
        raise RuntimeError('Insufficient filesystem free space')
    return usage


@contextlib.contextmanager
def run(kind, config=None):
    configure()
    guard = checked('registry/experiment.lock').open('a+')
    fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
    run_id = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + kind + '_' + uuid.uuid4().hex[:8]
    out = checked(Path('runs') / run_id)
    out.mkdir()
    source = Path(__file__).resolve().parent.parent
    manifest = {'schema_version': 1, 'run_id': run_id, 'kind': kind, 'status': 'running',
                'started_at': stamp(), 'command': sys.argv, 'root': str(ROOT), 'seed': 0,
                'config': config or {}, 'hardware_capability': 'confirmed_by_user',
                'hardware_capability_test': 'skipped_by_user',
                'environment': str(Path(sys.prefix)),
                'dependency_lock_sha256': digest(checked('locks/core-py312-linux-x86_64.txt')),
                'code_sha256': {str(p.relative_to(source)): digest(p) for p in source.rglob('*')
                                if p.is_file() and '__pycache__' not in p.parts},
                'source_snapshot': json.loads(checked('registry/source_snapshot.json').read_text())}
    write_json(out / 'manifest.json', manifest)
    append_json('registry/runs.jsonl', {k: manifest[k] for k in ['run_id', 'kind', 'status', 'started_at']})
    for rel in manifest['code_sha256']:
        destination = checked(out / 'code' / rel)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / rel, destination)

    class Tee:
        def __init__(self, console, log):
            self.console, self.log = console, log
        def write(self, text):
            self.console.write(text)
            self.log.write(text)
            self.log.flush()
        def flush(self):
            self.console.flush()
            self.log.flush()

    try:
        manifest['storage_at_start'] = storage_check()
        with (out / 'stdout.log').open('w') as log:
            with contextlib.redirect_stdout(Tee(sys.stdout, log)):
                yield out, manifest
        manifest['status'] = 'success'
    except BaseException:
        manifest['status'] = 'failed'
        manifest['error'] = traceback.format_exc()
        raise
    finally:
        manifest['finished_at'] = stamp()
        manifest['outputs_sha256'] = {str(p.relative_to(out)): digest(p) for p in out.rglob('*')
                                      if p.is_file() and p.name not in ['manifest.json', 'stdout.log']}
        write_json(out / 'manifest.json', manifest)
        append_json('registry/runs.jsonl', {k: manifest[k] for k in ['run_id', 'kind', 'status', 'finished_at']})
        print(json.dumps({'run_id': run_id, 'status': manifest['status']}), flush=True)
        guard.close()
