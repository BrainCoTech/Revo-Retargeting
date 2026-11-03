"""Opt-in ROS integration tests. All glove and MIT topics are remapped away from hardware.

After building/sourcing this workspace:
REVO3_CALIBRATION_INTEGRATION=1 python3 -m pytest test/test_staged_calibration_ros.py
The panel interaction test also needs a working DISPLAY (or xvfb-run).
"""
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from manus_ros2_msgs.msg import ManusGlove
from revo3_mit_controller_msgs.msg import Revo3MITCommand

from manus_revo3_retarget import retarget_tuning_panel as panel

pytestmark = pytest.mark.skipif(
    os.environ.get('REVO3_CALIBRATION_INTEGRATION') != '1',
    reason='requires an explicitly requested ROS integration run',
)


class Runtime:
    def __init__(self, tmp_path, start_enabled=False):
        self.namespace = '/calibration_test_' + uuid.uuid4().hex[:12]
        self.client = panel.ParameterClient()
        self.commands = {'left': [], 'right': []}
        self.targets = {'left': [], 'right': []}
        self.nodes = {}
        self.processes = []
        self.logs = []
        self.subscriptions = []
        self.gloves = self.client.create_publisher(ManusGlove, self.namespace + '/glove', 10)
        self.tmp_path = tmp_path
        self.start_enabled = start_enabled
        for side in ('left', 'right'):
            # Right is deliberately the single-hand node name to exercise the
            # panel's fallback from a requested name ending in _right.
            name = 'manus_revo3_retarget_left' if side == 'left' else 'manus_revo3_retarget'
            self.nodes[side] = self.namespace + '/' + name
            self.start(side, name)
            for field in ('commands', 'targets'):
                dest = getattr(self, field)[side]
                self.subscriptions.append(self.client.create_subscription(
                    Revo3MITCommand, f'{self.namespace}/{side}/{field}', dest.append, 100))
        self.select('right', fallback=True)

    def start(self, side, name):
        executable = Path(get_package_prefix('manus_revo3_retarget')) / 'lib/manus_revo3_retarget/retarget_node'
        log = (self.tmp_path / (side + '.log')).open('a')
        self.logs.append(log)
        args = [str(executable), '--ros-args', '-r', '__ns:=' + self.namespace, '-r', '__node:=' + name,
                '-p', 'hand_mode:=' + side, '-p', 'mit_command_publish_hz:=200.0',
                '-p', 'start_enabled:=' + str(self.start_enabled).lower(),
                '-r', '/manus_glove_0:=' + self.namespace + '/glove',
                '-r', '/manus_glove_1:=' + self.namespace + '/unused_glove']
        for source, dest in [('commands', 'commands'), ('retarget_targets', 'targets')]:
            args += ['-r', f'/revo3_{side}/joint_forward_mit_controller/{source}:={self.namespace}/{side}/{dest}']
        self.processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT))

    def select(self, side, fallback=False):
        name = self.nodes[side] + ('_right' if fallback else '')
        self.client.set_target(name)
        self.client.wait_ready(timeout_s=5)

    def pump(self, duration=0.2, gloves=False, allow_exit=False):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if gloves:
                for side in ('left', 'right'):
                    msg = ManusGlove()
                    msg.side = side
                    self.gloves.publish(msg)
            rclpy.spin_once(self.client, timeout_sec=0.01)
        if not allow_exit:
            assert all(p.poll() is None for p in self.processes), 'Retarget node exited; see test logs'

    def shutdown(self):
        for p in self.processes:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        for p in self.processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        for log in self.logs:
            log.close()
        self.client.destroy_node()


@pytest.fixture
def runtime(tmp_path, request):
    rclpy.init()
    instance = None
    try:
        instance = Runtime.__new__(Runtime)
        instance.__init__(tmp_path, start_enabled=getattr(request, 'param', False))
        yield instance
    finally:
        if instance is not None:
            instance.shutdown()
        rclpy.shutdown()


def assert_released(command):
    assert len(command.joint_names) == 21
    assert len(set(command.joint_names)) == 21
    for field in ('velocity', 'effort', 'kp', 'kd'):
        assert list(getattr(command, field)) == [0.0] * 21, field


def test_runtime_stages_disable_and_single_hand_fallback(runtime):
    r = runtime
    assert r.client.target_node == r.nodes['right']
    specs = panel.SPREAD_SPECS + panel.FOUR_FINGER_SPECS + panel.THUMB_SPECS
    names = [s.name('right') for s in specs] + [k.format(side='right') for k, _ in panel.BOOL_PARAMS]
    assert all(v is not None for v in r.client.get_values(names).values())
    r.pump(gloves=True)
    assert not r.commands['right'] and not r.targets['right']
    r.client.set_values({
        'physical_right_index_MPR_joint_offset_deg': 14.0,
        'physical_right_index_MCP_joint_offset_deg': 12.0,
        'physical_right_thumb_MCP_joint_offset_deg': 13.0,
        'retarget_calibration_mode': 'spread_only',
    })
    r.client.set_enabled(True)
    for mode in ('spread_only', 'four_finger_only', 'full'):
        r.client.set_values({'retarget_calibration_mode': mode})
        r.pump(gloves=True)
        target = r.targets['right'][-1]
        positions = dict(zip(target.joint_names, target.position))
        if mode == 'spread_only':
            assert positions['right_index_MPR_joint'] != 0.0
            assert all(v == 0.0 for k, v in positions.items() if '_MPR_joint' not in k)
        elif mode == 'four_finger_only':
            assert positions['right_index_MCP_joint'] != 0.0
            assert all(v == 0.0 for k, v in positions.items() if '_MPR_joint' in k or '_thumb_' in k)
        else:
            assert positions['right_thumb_MCP_joint'] != 0.0
        # Allow the existing linear interpolator to settle onto the target.
        r.pump()
        assert r.commands['right'][-1].position == pytest.approx(target.position)
        assert not r.commands['left']
    with pytest.raises(RuntimeError, match='retarget_calibration_mode'):
        r.client.set_values({'retarget_calibration_mode': 'typo'})
    r.client.set_enabled(False)
    r.pump()  # Drain DDS deliveries queued before the disable response.
    assert_released(r.commands['right'][-1])
    r.commands['right'].clear()
    r.targets['right'].clear()
    r.pump(gloves=True)
    assert not r.commands['right'] and not r.targets['right']
    r.client.set_enabled(True)
    r.pump()  # No fresh glove frame: the old interpolation target must stay cleared.
    assert not r.commands['right']
    r.pump(gloves=True)
    assert r.commands['right']


@pytest.mark.parametrize('runtime', [True], indirect=True)
def test_explicit_teleoperation_start_enabled(runtime):
    r = runtime
    r.pump(gloves=True)
    assert r.commands['left'] and r.commands['right']
    with pytest.raises(RuntimeError, match='startup-only'):
        r.client.set_values({'start_enabled': False})
    r.client.set_enabled(False)
    r.pump()
    r.commands['right'].clear()
    r.pump(gloves=True)
    assert not r.commands['right']


def test_panel_switch_export_and_close(runtime, tmp_path, monkeypatch):
    import tkinter as tk
    if not os.environ.get('DISPLAY'):
        pytest.skip('Tk interaction requires DISPLAY')
    root = tk.Tk()
    root.withdraw()
    def error(title, message):
        pytest.fail(f'{title}: {message}')
    monkeypatch.setattr(panel.messagebox, 'showerror', error)
    r = runtime
    ui = panel.RetargetTuningPanel(root, r.client, r.nodes, 'right', 'tester', tmp_path)
    try:
        r.pump()
        for commands in r.commands.values():
            assert_released(commands[-1])
            commands.clear()
        ui.toggle_output()
        r.pump(gloves=True)
        assert r.commands['right'] and any(r.commands['right'][-1].kp)
        for command in r.commands['left']:
            assert_released(command)
        ui.switch_hand()
        assert ui.current_side == 'left' and ui.output_enabled
        r.pump(gloves=True)
        r.commands['right'].clear()
        r.pump(gloves=True)
        assert not r.commands['right'] and r.commands['left']
        ui.stage_var.set('four_finger')
        ui._on_stage_changed()
        r.pump(gloves=True)
        assert ui.output_enabled
        assert all(v == 0.0 for k, v in zip(r.targets['left'][-1].joint_names, r.targets['left'][-1].position)
                   if '_MPR_joint' in k or '_thumb_' in k)
        ui.export_yaml()
        first = panel.read_latest_pointer(tmp_path, 'left')
        r.client.set_values({'physical_left_index_MCP_joint_offset_deg': 9.0})
        ui.refresh()
        ui.export_yaml()
        second = panel.read_latest_pointer(tmp_path, 'left')
        assert first and second and first != second
        saved = yaml.safe_load((panel.library_side_dir(tmp_path, 'left') / second).read_text())['manus_revo3_retarget']['ros__parameters']
        assert saved['physical_left_index_MCP_joint_offset_deg'] == 9.0
        assert 'retarget_calibration_mode' not in saved
        assert 'start_enabled' not in saved
        # Exported values must be accepted again and must not change stage/enable.
        r.client.set_values({'physical_left_index_MCP_joint_offset_deg': 0.0})
        r.client.set_values(saved)
        assert r.client.get_values(['physical_left_index_MCP_joint_offset_deg'])['physical_left_index_MCP_joint_offset_deg'] == 9.0
        ui._history_var.set(first)
        ui._set_history_latest()
        assert panel.read_latest_pointer(tmp_path, 'left') == first
        assert (tmp_path / 'tester_revo3_left.yaml').read_text() == (panel.library_side_dir(tmp_path, 'left') / first).read_text()
        ui._on_close()
        r.pump(gloves=True)
        for values in r.commands.values():
            assert_released(values[-1])
            values.clear()
        r.pump(gloves=True)
        assert not any(r.commands.values())
    finally:
        try:
            root.destroy()
        except tk.TclError:
            pass


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
def test_node_exit_releases_all_joints(runtime, stop_signal):
    r = runtime
    r.client.set_enabled(True)
    r.pump(gloves=True)
    assert any(r.commands['right'][-1].kp)
    proc = r.processes[1]
    proc.send_signal(stop_signal)
    r.pump(duration=0.8, gloves=True, allow_exit=True)
    assert proc.wait(timeout=5) == 0
    assert_released(r.commands['right'][-1])
    count = len(r.commands['right'])
    r.pump(allow_exit=True)
    assert len(r.commands['right']) == count


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
def test_standalone_panel_signal_releases(runtime, tmp_path, stop_signal):
    if not os.environ.get('DISPLAY'):
        pytest.skip('Tk interaction requires DISPLAY')
    r = runtime
    # Exercise the real entry point and its signal handlers with a hidden window.
    code = '''
import tkinter as tk
original = tk.Tk
def hidden_root():
    root = original()
    root.withdraw()
    return root
tk.Tk = hidden_root
from manus_revo3_retarget.retarget_tuning_panel import main
raise SystemExit(main())
'''
    log_path = tmp_path / 'panel.log'
    with log_path.open('w') as log:
        proc = subprocess.Popen([sys.executable, '-c', code,
                                 '--left-node', r.nodes['left'],
                                 '--right-node', r.nodes['right'],
                                 '--calibration-dir', str(tmp_path)],
                                stdout=log, stderr=subprocess.STDOUT)
    try:
        # The constructor sets the stage after connecting and disabling outputs.
        deadline = time.monotonic() + 8
        while r.client.get_values(['retarget_calibration_mode'])['retarget_calibration_mode'] != 'spread_only':
            assert time.monotonic() < deadline and proc.poll() is None, log_path.read_text()
            r.pump()
        r.client.set_enabled(True)
        r.pump(gloves=True)
        assert any(r.commands['right'][-1].kp)
        proc.send_signal(stop_signal)
        while proc.poll() is None and time.monotonic() < deadline:
            r.pump(gloves=True)
        assert proc.wait(timeout=3) == 0, log_path.read_text()
        r.pump(gloves=True)
        assert_released(r.commands['right'][-1])
        assert 'Traceback' not in log_path.read_text()
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


def test_panel_keeps_window_when_release_fails(runtime, tmp_path, monkeypatch):
    import tkinter as tk
    if not os.environ.get('DISPLAY'):
        pytest.skip('Tk interaction requires DISPLAY')
    root = tk.Tk()
    root.withdraw()
    r = runtime
    ui = panel.RetargetTuningPanel(root, r.client, r.nodes, 'right', 'tester', tmp_path)
    errors = []
    monkeypatch.setattr(panel.messagebox, 'showerror', lambda title, text: errors.append(text))
    def failed_release(*args, **kwargs):
        raise TimeoutError('unreachable')
    try:
        monkeypatch.setattr(r.client, 'set_enabled_for', failed_release)
        assert ui._on_close() is False
        assert root.winfo_exists()
        assert len(errors) == 1 and 'unreachable' in errors[0]
    finally:
        root.destroy()
