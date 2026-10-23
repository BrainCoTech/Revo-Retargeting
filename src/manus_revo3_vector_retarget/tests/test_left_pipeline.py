"""Selected-side entry points and left offline evaluation/replay, without ROS I/O."""
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from retarget_v2 import cli, viewer
from retarget_v2.input import Calibration
from retarget_v2.model import RobotModel
from retarget_v2.solver import Retargeter
from test_input import sdk_hand, transformed


@pytest.mark.parametrize("side", ["left", "right"])
def test_each_may3_hand_solves_glove_motion_in_its_own_model(side):
    def frame(bend):
        msg = sdk_hand(pip_bend_degrees=bend)
        if side == "left":
            msg = transformed(msg, np.diag([-1., 1., 1.]), np.zeros(3))
            msg["side"] = "Left"
        return msg

    with RobotModel(hand_side=side) as robot:
        solver = Retargeter(robot, Calibration.from_frames([frame(0.)] * 30, hand_side=side))
        poses = []
        for bend in (0., 30., 70.):
            q, status = solver.solve(frame(bend))
            assert status["accepted"] and np.isfinite(q).all()
            assert np.all(q >= robot.lower) and np.all(q <= robot.upper)
            poses.append(q)
        assert np.linalg.norm(poses[-1] - poses[0]) > .1


def test_live_cli_passes_selected_hand_to_runtime(monkeypatch):
    calls = []
    module = ModuleType("retarget_v2.live")
    module.live = calls.append
    monkeypatch.setitem(sys.modules, module.__name__, module)
    monkeypatch.setattr(sys, "argv", ["retarget", "live", "--hand-side", "left"])
    cli.main()
    assert calls[0].hand_side == "left"
    assert not hasattr(calls[0], "topic")
    assert not hasattr(calls[0], "output")
    assert calls[0].normal_sign == -1.
    assert calls[0].domain is None


def test_left_evaluate_and_simulate_preserve_side_and_reject_wrong_replay_model(tmp_path, monkeypatch):
    # Synthetic frames enter at the capture-reader boundary. No synthetic data
    # is presented as a real recording to the production capture validator.
    frame = transformed(sdk_hand(), np.diag([1., -1., 1.]), np.zeros(3))
    frame["side"] = "Left"
    rows = [dict(t=i / 60, msg=frame, phase="calibration" if i < 30 else "motion")
            for i in range(32)]
    monkeypatch.setattr(cli, "read_capture", lambda _path: ({"raw_sha256": "test-reader"}, rows))
    run = tmp_path / "left_run"
    monkeypatch.setattr(sys, "argv", ["retarget", "evaluate", str(tmp_path / "capture"),
                                    "--hand-side", "left", "--output", str(run)])
    cli.main()
    metadata = json.loads((run / "run.json").read_text())
    assert metadata["complete"]
    assert metadata["hand_side"] == metadata["calibration"]["hand_side"] == "left"
    assert all(name.startswith("left_") for name in metadata["joint_names"])
    with RobotModel(hand_side="left") as robot:
        _, trajectory = viewer._load_run(run, robot)
        assert trajectory["q_cmd"].shape == (32, 21)
        assert np.isfinite(trajectory["q_cmd"]).all()
    with RobotModel() as wrong_model, pytest.raises(ValueError, match="hand side"):
        viewer._load_run(run, wrong_model)
    viewer.simulate(SimpleNamespace(run=run))
    report = json.loads((run / "simulation_report.json").read_text())
    assert report["complete"] and report["finite"]
    assert report["hand_side"] == "left"
    assert all(name.startswith("left_") for name in report["per_joint_tracking_error_deg"])


@pytest.fixture
def pipeline_launch(monkeypatch):
    """Load the actual launch builder with local action objects, never launching ROS."""
    def action(kind):
        return lambda *args, **kwargs: SimpleNamespace(kind=kind, args=args, **kwargs)

    modules = {name: ModuleType(name) for name in (
        "launch", "launch.actions", "launch.substitutions", "launch_ros", "launch_ros.actions",
        "ament_index_python", "ament_index_python.packages")}
    modules["ament_index_python.packages"].get_package_prefix = lambda _name: "/installed"
    modules["launch"].LaunchDescription = lambda entities: SimpleNamespace(entities=entities)
    modules["launch.actions"].DeclareLaunchArgument = action("argument")
    modules["launch.actions"].ExecuteProcess = action("process")
    modules["launch.actions"].OpaqueFunction = action("function")
    modules["launch.substitutions"].LaunchConfiguration = (
        lambda name: SimpleNamespace(perform=lambda context: context[name]))
    modules["launch_ros.actions"].Node = action("node")
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1] / "launch/pipeline.launch.py"
    spec = importlib.util.spec_from_file_location("vector_pipeline_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = {a.args[0]: a.default_value for a in module.generate_launch_description().entities
               if a.kind == "argument"}
    return module, context


@pytest.mark.parametrize("side", ["left", "right"])
def test_launch_selects_hand_namespace_and_always_starts_manus(pipeline_launch, side):
    module, context = pipeline_launch
    context.update(hand_side=side)
    actions = module._launch(context)
    command = next(a.cmd for a in actions if a.kind == "process")
    assert command[command.index("--hand-side") + 1] == side
    assert command[command.index("--namespace") + 1] == f"/revo3_{side}"
    assert "--topic" not in command
    publishers = [a for a in actions if a.kind == "node"]
    assert len(publishers) == 1
    assert publishers[0].executable == "manus_data_publisher"
    assert "--allow-commands" not in command
    context.update(namespace="/custom", glove_id="-123", cache_dir="/tmp/vector-cache")
    command = next(a.cmd for a in module._launch(context) if a.kind == "process")
    assert command[command.index("--namespace") + 1] == "/custom"
    assert command[1] == "hardware"
    assert command[command.index("--glove-id") + 1] == "-123"
    assert command[command.index("--cache-dir") + 1] == "/tmp/vector-cache"


def test_launch_removes_viewer_modes_and_diagnostics(pipeline_launch):
    module, context = pipeline_launch
    assert not {'mode', 'duration', 'headless', 'allow_commands', 'require_bus_diagnostics',
                'topic', 'output', 'launch_manus_publisher'} & context.keys()
    assert next(a.cmd for a in module._launch(context) if a.kind == 'process')[1] == 'hardware'
    actions = module._launch(context)
    assert len([a for a in actions if a.kind == 'node']) == 1


@pytest.mark.parametrize("domain", [None, "188"])
def test_launch_inherits_ros_domain_without_overrides(pipeline_launch, monkeypatch, domain):
    module, context = pipeline_launch
    if domain is None:
        monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    else:
        monkeypatch.setenv("ROS_DOMAIN_ID", domain)
    assert "domain" not in context
    actions = module._launch(context)
    for action in actions:
        assert "ROS_DOMAIN_ID" not in getattr(action, "additional_env", {})
        assert "--domain" not in getattr(action, "cmd", [])


@pytest.mark.parametrize("side", ["left", "right"])
def test_launch_defaults_to_direct_output_and_can_enable_interpolation(pipeline_launch, side):
    module, context = pipeline_launch
    context.update(hand_side=side)
    assert context["interpolate"] == "false"
    command = next(a.cmd for a in module._launch(context) if a.kind == "process")
    assert "--interpolate" not in command
    context["interpolate"] = "true"
    command = next(a.cmd for a in module._launch(context) if a.kind == "process")
    assert command.count("--interpolate") == 1


@pytest.fixture
def compatible_pipeline(pipeline_launch):
    # Reuse the action doubles; importing the entry point never starts ROS.
    path = Path(__file__).resolve().parents[1] / "launch/pipeline_launch.py"
    spec = importlib.util.spec_from_file_location("compatible_pipeline_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = {a.args[0]: a.default_value for a in module.generate_launch_description().entities
               if a.kind == "argument"}
    return module, context


@pytest.mark.parametrize("mode,sides", [("left", ["left"]), ("right", ["right"]),
                                       ("both", ["left", "right"])])
def test_compatible_launch_has_legacy_services_and_one_manus(compatible_pipeline, mode, sides):
    module, context = compatible_pipeline
    context.update(hand_mode=mode, left_profile="/tmp/left profile.yaml",
                   right_profile="/tmp/right profile.yaml")
    actions = module._launch(context)
    assert len([a for a in actions if a.kind == "node"]) == 1
    commands = [a.cmd for a in actions if a.kind == "process"]
    assert len(commands) == len(sides)
    for side, command in zip(sides, commands):
        name = f"manus_revo3_retarget_{side}" if mode == "both" else "manus_revo3_retarget"
        assert command[command.index("--node-name") + 1] == name
        assert command[command.index("--hand-side") + 1] == side
        assert command[command.index("--namespace") + 1] == f"/revo3_{side}"
        assert command[command.index("--profile") + 1] == f"/tmp/{side} profile.yaml"
    context["launch_manus_publisher"] = "false"
    assert all(a.kind == "process" for a in module._launch(context))


def test_compatible_launch_preserves_domain_and_exposes_backend_options(compatible_pipeline):
    module, context = compatible_pipeline
    context.update(hand_type="left", numeric_threads="2", interpolate="true", left_glove_id="-123")
    process = next(a for a in module._launch(context) if a.kind == "process")
    assert process.additional_env["OPENBLAS_NUM_THREADS"] == "2"
    assert "ROS_DOMAIN_ID" not in process.additional_env
    assert "--domain" not in process.cmd
    assert "--interpolate" in process.cmd
    assert process.cmd[process.cmd.index("--glove-id") + 1] == "-123"
    context["numeric_threads"] = "-1"
    with pytest.raises(ValueError, match="numeric_threads"):
        module._launch(context)


def test_launch_uses_colcon_entrypoint_and_isolates_user_dependencies(pipeline_launch, monkeypatch):
    module, _ = pipeline_launch
    monkeypatch.setenv("RETARGET_PYTHON_PREFIX", "/missing/old-venv")
    context = {a.args[0]: a.default_value for a in module.generate_launch_description().entities
               if a.kind == "argument"}
    assert "python" not in context
    process = next(a for a in module._launch(context) if a.kind == "process")
    assert process.cmd[0] == "/installed/lib/manus_revo3_vector_retarget/retarget"
    assert process.additional_env["PYTHONNOUSERSITE"] == "1"
    # Old launch overrides must not select a different runtime either.
    context["python"] = "/missing/old-venv/bin/python"
    assert next(a for a in module._launch(context) if a.kind == "process").cmd[0] == "/installed/lib/manus_revo3_vector_retarget/retarget"
