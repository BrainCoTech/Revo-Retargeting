#!/usr/bin/env python3
"""Install the registered CPU lab in place; never create a second environment.

Run with /usr/bin/python3 on the registered Ubuntu x86_64 host. The smoke test
checks generic MuJoCo physics, not Revo3 control, retargeting, or finger rubbing.
"""

import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import urllib.parse
import urllib.request
import uuid
import zipfile


ROOT = Path("/mnt/workspace/wensheng/revo3-retargeting-lab")
UV_VERSION = "0.8.22"
PACKAGE_INDEX = "https://mirrors.aliyun.com/pypi/simple"
SMOKE = r'''
import importlib.metadata, json, time
import mujoco
import numpy as np
import scipy.optimize
import yaml
import pyarrow as pa
import h5py
fit = scipy.optimize.least_squares(lambda x: x - 2.0, np.array([0.0]))
if not (fit.success and np.allclose(fit.x, [2.0])):
    raise RuntimeError("SciPy least_squares smoke failed")
if pa.table({"x": [1, 2]}).num_rows != 2 or yaml.safe_load("x: 1")["x"] != 1:
    raise RuntimeError("Arrow or YAML smoke failed")
model = mujoco.MjModel.from_xml_string("""<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 .1"/>
    <body pos="0 0 .4"><freejoint/>
      <geom name="ball" type="sphere" size=".05" density="1000"/>
    </body>
  </worldbody>
</mujoco>""")
data = mujoco.MjData(model)
start = time.perf_counter()
for _ in range(2000):
    mujoco.mj_step(model, data)
    if not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()):
        raise RuntimeError("Non-finite physics state")
elapsed = time.perf_counter() - start
force = np.zeros(6)
normal_force = 0.0
for contact in range(data.ncon):
    mujoco.mj_contactForce(model, data, contact, force)
    normal_force += max(0.0, float(force[0]))
height = float(data.qpos[2])
speed = float(np.linalg.norm(data.qvel))
if not (.045 <= height <= .055 and speed < .02 and normal_force > 0):
    raise RuntimeError(f"Ball did not settle: height={height}, speed={speed}, force={normal_force}")
print(json.dumps({
    "scope": "generic CPU physics only; not Revo3 control or rubbing validation",
    "steps": 2000, "simulation_seconds": float(data.time),
    "wall_seconds": elapsed, "steps_per_second": 2000 / elapsed,
    "final_height_m": height, "final_speed": speed,
    "final_normal_contact_force_n": normal_force, "final_contacts": int(data.ncon),
    "versions": {p: importlib.metadata.version(p) for p in
                 ["mujoco", "numpy", "scipy", "PyYAML", "pyarrow", "h5py"]}
}))
'''


def timestamp():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def locked_packages(path):
    """Accept only hashed name==version entries, continuations, and comments."""
    pattern = re.compile(r"([A-Za-z0-9][A-Za-z0-9_.-]*)==([A-Za-z0-9][A-Za-z0-9_.+!-]*)"
                         r"((?:\s+--hash=sha256:[0-9a-f]{64})+)")
    packages, continued, seen = [], [], set()
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        continued.append(line.removesuffix("\\").strip())
        if line.endswith("\\"):
            continue
        entry = " ".join(continued)
        continued.clear()
        match = pattern.fullmatch(entry)
        if not match:
            raise RuntimeError("Lock contains unsupported or unhashed package syntax")
        name, version, hash_text = match.groups()
        normalized = re.sub(r"[-_.]+", "-", name).lower()
        if normalized in seen:
            raise RuntimeError(f"Duplicate package in lock: {name}")
        seen.add(normalized)
        packages.append({"name": name, "version": version,
                         "hashes": sorted(set(re.findall(r"--hash=sha256:([0-9a-f]{64})", hash_text)))})
    if continued or not packages:
        raise RuntimeError("Lock is empty or has an unfinished continuation")
    return packages


def main():
    root = ROOT.resolve(strict=True)
    if root != ROOT or ROOT.is_symlink():
        raise RuntimeError("Fixed lab root must not be redirected by symbolic links")

    def checked(path):
        path = Path(path)
        if not path.is_absolute():
            path = root / path
        if not path.resolve().is_relative_to(root):
            raise RuntimeError(f"Path escapes registered lab root: {path}")
        return path

    # No writes, downloads, or subprocesses before this explicit registration gate.
    registration = json.loads(checked("registry/lab.json").read_text())
    if registration.get("root") != str(root):
        raise RuntimeError(f"registry/lab.json root must equal {root}")

    def directory(path):
        path = checked(path)
        path.mkdir(parents=True, exist_ok=True)
        return checked(path)

    def atomic_write(path, content):
        path = checked(path)
        directory(path.parent)
        temporary = checked(path.with_name(path.name + ".tmp-" + uuid.uuid4().hex[:8]))
        try:
            if isinstance(content, bytes):
                temporary.write_bytes(content)
            else:
                temporary.write_text(content)
            checked(path)
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)

    def json_write(path, value):
        atomic_write(path, json.dumps(value, ensure_ascii=False, indent=2) + "\n")

    lock_path = checked("registry/bootstrap-cpu.lock")
    with lock_path.open("a+") as guard:
        try:
            fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("Another CPU bootstrap is already running") from None

        run_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-bootstrap-cpu-" + uuid.uuid4().hex[:8]
        run_dir = directory(Path("runs") / run_id)
        manifest_path = checked(run_dir / "manifest.json")
        log_path = checked(run_dir / "bootstrap.log")
        environment_path = checked("envs/core-py312")
        requirements = Path(__file__).resolve().parents[1] / "requirements-core.in"
        manifest = {
            "schema_version": 1, "run_id": run_id, "kind": "bootstrap_cpu",
            "root": str(root), "started_at": timestamp(), "status": "running",
            "environment": str(environment_path), "commands": [],
            "script": {"path": str(Path(__file__).resolve()), "sha256": sha256(Path(__file__))},
            "requirements": {"path": str(requirements)},
            "host": {"hostname": platform.node(), "platform": platform.platform()},
            "scope": "CPU environment and generic physics smoke; not Revo3 validation",
            "log": str(log_path),
            "storage_checks": [],
            "storage_policy": "Logical-byte preflight and between-step checks; not a filesystem hard quota",
            "package_index": {"url": PACKAGE_INDEX,
                              "reason": "DSW files.pythonhosted.org wheel downloads stalled; Aliyun mirror is reachable. All lock hashes must match official PyPI release metadata before installation."},
        }
        source_snapshot = checked("registry/source_snapshot.json")
        if source_snapshot.exists():
            manifest["source_snapshot"] = {"path": str(source_snapshot), "sha256": sha256(source_snapshot)}

        def save():
            json_write(manifest_path, manifest)

        def event(status):
            with checked("registry/runs.jsonl").open("a") as stream:
                stream.write(json.dumps({"run_id": run_id, "kind": "bootstrap_cpu", "status": status,
                                         "timestamp": timestamp(), "manifest": str(manifest_path)}) + "\n")

        def storage_check(phase, enforce=True):
            usage = {}
            for name in ("cache", "tmp", "runs"):
                total = 0
                for current, directories, files in os.walk(checked(name)):
                    for child in directories:
                        checked(Path(current) / child)
                    for child in files:
                        total += checked(Path(current) / child).stat().st_size
                usage[name] = total
            snapshot = {"phase": phase, "timestamp": timestamp(), "logical_bytes": usage,
                        "free_bytes": shutil.disk_usage(root).free,
                        "budgets_bytes": registration.get("budgets_bytes"),
                        "minimum_free_bytes": registration.get("stop_new_jobs_below_free_bytes")}
            manifest["storage_checks"].append(snapshot)
            save()
            if enforce:
                budgets = registration["budgets_bytes"]
                minimum = registration["stop_new_jobs_below_free_bytes"]
                if not isinstance(minimum, int) or minimum <= 0:
                    raise RuntimeError("Invalid registered minimum free space")
                for name, used in usage.items():
                    if not isinstance(budgets[name], int) or budgets[name] <= 0 or used >= budgets[name]:
                        raise RuntimeError(f"Storage budget reached or invalid: {name}, used={used}, budget={budgets[name]}")
                if snapshot["free_bytes"] < minimum:
                    raise RuntimeError(f"Insufficient free space: {snapshot['free_bytes']} < {minimum}")

        save()
        event("running")
        process_env = dict(os.environ)
        for key in list(process_env):
            if key.startswith(("UV_", "PIP_", "PYTHON")):
                process_env.pop(key)
        env_record = {"name": "core-py312", "path": str(environment_path), "last_run_id": run_id}
        exit_code = 1
        try:
            storage_check("startup")
            if platform.system() != "Linux" or platform.machine() != "x86_64" or sys.version_info[:2] != (3, 12):
                raise RuntimeError("This bootstrap requires Linux x86_64 and Python 3.12")
            manifest["requirements"]["sha256"] = sha256(requirements)
            atomic_write(run_dir / "requirements-core.in", requirements.read_text())
            process_env.update({
                "UV_CACHE_DIR": str(directory("cache/uv")),
                "UV_PYTHON_INSTALL_DIR": str(directory("toolchains/python")),
                "UV_PYTHON_DOWNLOADS": "never", "UV_NO_PROGRESS": "1",
                "UV_HTTP_TIMEOUT": "20", "UV_HTTP_RETRIES": "1",
                "UV_LINK_MODE": "copy",
                "UV_CONCURRENT_BUILDS": "1", "UV_CONCURRENT_DOWNLOADS": "2", "UV_CONCURRENT_INSTALLS": "1",
                "XDG_CACHE_HOME": str(directory("cache/xdg")),
                "PIP_CACHE_DIR": str(directory("cache/pip")),
                "TMPDIR": str(directory(Path("tmp") / run_id)),
                "PYTHONNOUSERSITE": "1", "PYTHONDONTWRITEBYTECODE": "1",
                "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
                "NUMEXPR_NUM_THREADS": "1", "VECLIB_MAXIMUM_THREADS": "1", "BLIS_NUM_THREADS": "1",
            })
            process_env["TMP"] = process_env["TEMP"] = process_env["TMPDIR"]

            def command(argv, timeout=1200):
                argv = [str(item) for item in argv]
                storage_check("before_command")
                entry = {"argv": argv, "started_at": timestamp(), "status": "running"}
                manifest["commands"].append(entry)
                save()
                with checked(log_path).open("a") as log:
                    log.write("\n$ " + json.dumps(argv) + "\n")
                    log.flush()
                    try:
                        result = subprocess.run(argv, cwd=root, env=process_env, text=True,
                                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
                        output = result.stdout
                        entry.update(returncode=result.returncode, status="success" if result.returncode == 0 else "failed")
                        log.write(output)
                    except subprocess.TimeoutExpired as error:
                        entry.update(status="timeout", timeout_seconds=timeout)
                        output = error.stdout or b""
                        log.write(output.decode(errors="replace") if isinstance(output, bytes) else output)
                        raise RuntimeError(f"Command timed out after {timeout}s; see {log_path}") from error
                    except KeyboardInterrupt:
                        entry.update(status="interrupted", failure_reason="KeyboardInterrupt")
                        raise
                    except Exception as error:
                        entry.update(status="failed", failure_reason=f"{type(error).__name__}: {error}")
                        raise
                    finally:
                        entry["finished_at"] = timestamp()
                        save()
                if result.returncode:
                    raise RuntimeError(f"Command exited {result.returncode}; see {log_path}")
                storage_check("after_command")
                return output.strip()

            tool_dir = directory("toolchains/uv")
            uv = checked(tool_dir / "uv")
            receipt_path = checked(tool_dir / "receipt.json")
            metadata_url = f"https://pypi.org/pypi/uv/{UV_VERSION}/json"
            manifest["uv"] = {"version": UV_VERSION, "metadata_url": metadata_url}
            storage_check("before_uv_download")
            with urllib.request.urlopen(metadata_url, timeout=30) as response:
                metadata = json.load(response)
            wheels = sorted((item for item in metadata["urls"] if item["filename"].endswith(".whl")
                             and "manylinux" in item["filename"] and "x86_64" in item["filename"]),
                            key=lambda item: item["filename"])
            if not wheels:
                raise RuntimeError("PyPI has no uv manylinux x86_64 wheel")
            selected = wheels[0]
            source_url = selected["url"]
            parsed = urllib.parse.urlparse(source_url)
            if parsed.scheme != "https" or parsed.hostname != "files.pythonhosted.org":
                raise RuntimeError("Refusing uv wheel URL outside official PyPI file host")
            wheel = checked(tool_dir / Path(selected["filename"]).name)
            expected_hash = selected["digests"]["sha256"]
            manifest["uv"].update(url=source_url, wheel=str(wheel), sha256=expected_hash)
            save()
            if not wheel.exists() or sha256(wheel) != expected_hash:
                partial = checked(tool_dir / (wheel.name + ".partial"))
                command(["/usr/bin/curl", "--disable", "--fail", "--silent", "--show-error",
                         "--proto", "=https", "--connect-timeout", "15", "--max-time", "90",
                         "--max-filesize", str(100 * 1024 * 1024),
                         "--output", partial, "--url", source_url], timeout=100)
                if sha256(partial) != expected_hash:
                    raise RuntimeError("uv wheel SHA256 mismatch; partial retained for audit")
                os.replace(partial, checked(wheel))
            with zipfile.ZipFile(wheel) as archive:
                members = [name for name in archive.namelist() if name.endswith(".data/scripts/uv")]
                if len(members) != 1:
                    raise RuntimeError("Expected exactly one uv executable in wheel")
                binary = archive.read(members[0])
            binary_hash = hashlib.sha256(binary).hexdigest()
            if uv.exists() and sha256(uv) != binary_hash:
                raise RuntimeError("Existing uv binary differs from verified wheel; refusing overwrite")
            if not uv.exists():
                atomic_write(uv, binary)
            checked(uv).chmod(0o755)
            receipt = {"version": UV_VERSION, "metadata_url": metadata_url, "url": source_url,
                       "wheel": str(wheel), "sha256": expected_hash, "binary_sha256": binary_hash}
            json_write(receipt_path, receipt)
            manifest["uv"] = receipt
            version = command([uv, "--no-config", "--version"], timeout=30)
            if not version.startswith(f"uv {UV_VERSION}"):
                raise RuntimeError(f"Unexpected uv version: {version}")
            manifest["uv"]["reported_version"] = version

            directory(environment_path.parent)
            if not environment_path.exists():
                command([uv, "--no-config", "venv", "--python", "/usr/bin/python3", "--no-python-downloads", environment_path])
            if not checked(environment_path / "pyvenv.cfg").is_file():
                raise RuntimeError("Existing environment is incomplete; retained unchanged for repair")
            python = environment_path / "bin/python"
            identity = json.loads(command([python, "-c", "import json,sys,sysconfig; print(json.dumps({'prefix':sys.prefix,'base_prefix':sys.base_prefix,'version':list(sys.version_info[:3]),'paths':sysconfig.get_paths()}))"], timeout=30))
            if Path(identity["prefix"]).resolve() != environment_path.resolve() or identity["prefix"] == identity["base_prefix"] or identity["version"][:2] != [3, 12]:
                raise RuntimeError("Existing interpreter is not the registered Python 3.12 virtual environment")
            for name in ("purelib", "platlib", "scripts", "data"):
                if not checked(identity["paths"][name]).resolve().is_relative_to(environment_path.resolve()):
                    raise RuntimeError(f"Virtual environment {name} path escapes environment")
            for current, directories, _ in os.walk(environment_path):
                for name in directories:
                    checked(Path(current) / name)
            env_record["python"] = identity

            lock = checked("locks/core-py312-linux-x86_64.txt")
            directory(lock.parent)
            candidate = checked(run_dir / "core-py312-linux-x86_64.candidate.txt")
            command([uv, "--no-config", "pip", "compile", requirements, "--python", python, "--generate-hashes",
                     "--no-header", "--index-url", PACKAGE_INDEX, "--only-binary", ":all:", "-o", candidate])
            verification_path = checked(run_dir / "pypi_hash_verification.json")
            verification = {"started_at": timestamp(), "status": "running", "lock": str(candidate),
                            "lock_sha256": sha256(candidate), "packages": []}
            manifest["pypi_hash_verification"] = {"path": str(verification_path)}
            json_write(verification_path, verification)
            save()
            try:
                for package in locked_packages(candidate):
                    url = "https://pypi.org/pypi/{}/{}/json".format(
                        urllib.parse.quote(package["name"], safe=""),
                        urllib.parse.quote(package["version"], safe=""))
                    record = {**package, "url": url, "requested_at": timestamp(), "status": "running"}
                    verification["packages"].append(record)
                    json_write(verification_path, verification)
                    with urllib.request.urlopen(url, timeout=20) as response:
                        official_metadata = json.load(response)
                    official_hashes = {item["digests"]["sha256"] for item in official_metadata["urls"]}
                    if not official_hashes or not package["hashes"] or not set(package["hashes"]).issubset(official_hashes):
                        record["status"] = "failed"
                        raise RuntimeError(f"Lock hashes do not match official PyPI release: {package['name']}=={package['version']}")
                    record.update(status="verified", verified_at=timestamp(),
                                  official_distribution_count=len(official_metadata["urls"]))
                    json_write(verification_path, verification)
                verification["status"] = "verified"
            except BaseException as error:
                verification.update(status="interrupted" if isinstance(error, KeyboardInterrupt) else "failed",
                                    failure_reason=f"{type(error).__name__}: {error}")
                raise
            finally:
                verification["finished_at"] = timestamp()
                json_write(verification_path, verification)
                manifest["pypi_hash_verification"]["sha256"] = sha256(verification_path)
                save()
            if lock.exists() and sha256(lock) != sha256(candidate):
                raise RuntimeError(f"Resolved dependencies differ from existing lock; candidate retained at {candidate}")
            if not lock.exists():
                atomic_write(lock, candidate.read_text())
            manifest["dependency_lock"] = {"path": str(lock), "sha256": sha256(lock)}
            command([uv, "--no-config", "pip", "sync", lock, "--python", python, "--require-hashes",
                     "--index-url", PACKAGE_INDEX, "--only-binary", ":all:"])
            smoke_path = checked(run_dir / "smoke.py")
            atomic_write(smoke_path, SMOKE)
            smoke = json.loads(command([python, smoke_path], timeout=120))
            json_write(run_dir / "smoke.json", smoke)
            manifest["smoke"] = smoke
            env_record.update(versions=smoke["versions"], dependency_lock=manifest["dependency_lock"])
            manifest["status"] = "success"
            exit_code = 0
        except KeyboardInterrupt:
            manifest["status"] = "interrupted"
            manifest["failure_reason"] = "KeyboardInterrupt: bootstrap interrupted"
            exit_code = 130
            with checked(log_path).open("a") as log:
                log.write("\nINTERRUPTED: " + manifest["failure_reason"] + "\n")
        except Exception as error:
            manifest["status"] = "failed"
            manifest["failure_reason"] = f"{type(error).__name__}: {error}"
            with checked(log_path).open("a") as log:
                log.write("\nFAILED: " + manifest["failure_reason"] + "\n")
        finally:
            try:
                storage_check("finished", enforce=False)
            except Exception as error:
                manifest["storage_statistics_error"] = f"{type(error).__name__}: {error}"
                if exit_code != 130:
                    manifest["status"] = "failed"
                    exit_code = 1
            manifest["finished_at"] = timestamp()
            env_record.update(status=manifest["status"], updated_at=manifest["finished_at"], manifest=str(manifest_path))
            if "failure_reason" in manifest:
                env_record["failure_reason"] = manifest["failure_reason"]
            registry_path = checked("registry/environments.json")
            try:
                registry = json.loads(registry_path.read_text()) if registry_path.exists() else {"schema_version": 1, "environments": []}
                records = registry.setdefault("environments", [])
                if not isinstance(records, list):
                    raise RuntimeError("registry/environments.json environments must be a list")
                records[:] = [record for record in records if record.get("name") != "core-py312"] + [env_record]
                json_write(registry_path, registry)
            except Exception as error:
                manifest["registry_error"] = f"{type(error).__name__}: {error}"
                if exit_code != 130:
                    manifest["status"] = "failed"
                    exit_code = 1
            save()
            event(manifest["status"])
            print(json.dumps({"status": manifest["status"], "run_id": run_id, "manifest": str(manifest_path),
                              "failure_reason": manifest.get("failure_reason"), "registry_error": manifest.get("registry_error")}, ensure_ascii=False))
        return exit_code


if __name__ == "__main__":
    sys.exit(main())
