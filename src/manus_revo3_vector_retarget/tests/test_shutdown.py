"""Repeated process-group / ROS launch interrupts must not abort cleanup."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def test_second_sigint_during_cleanup_is_ignored_and_handler_is_restored(tmp_path):
    program = r'''
import signal, time
from pathlib import Path
from retarget_v2 import hardware
root = Path(__import__('sys').argv[1])
previous = signal.getsignal(signal.SIGINT)
def run(_args):
    try:
        (root / 'ready').touch()
        while True: time.sleep(.01)
    finally:
        (root / 'cleaning').touch()
        time.sleep(.5)
        (root / 'cleaned').touch()
hardware._run_hardware = run
hardware.hardware(None)
assert signal.getsignal(signal.SIGINT) is previous
'''
    environment = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[1]))
    process = subprocess.Popen([sys.executable, '-c', program, str(tmp_path)], env=environment,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    def wait_file(name):
        deadline = time.monotonic() + 5
        while not (tmp_path / name).exists() and time.monotonic() < deadline:
            assert process.poll() is None
            time.sleep(.01)
        assert (tmp_path / name).exists()
    try:
        wait_file('ready')
        process.send_signal(signal.SIGINT)
        wait_file('cleaning')
        process.send_signal(signal.SIGINT)
        output, error = process.communicate(timeout=5)
        assert process.returncode == 0, (output, error)
        assert (tmp_path / 'cleaned').exists()
        assert 'Traceback' not in error
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
