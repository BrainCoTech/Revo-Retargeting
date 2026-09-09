"""Launch routing and controller switching without starting ROS or hardware."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.utilities import evaluate_parameters

os.environ.setdefault("ROS_LOG_DIR", tempfile.mkdtemp(prefix="revo2_position_launch."))
PACKAGE = Path(__file__).resolve().parents[1]


def module(name):
    spec = importlib.util.spec_from_file_location(name, PACKAGE / "launch" / f"{name}.py")
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def context_for(mod, **overrides):
    context = LaunchContext()
    for action in mod.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(overrides)
    return context


@pytest.mark.parametrize("side,count", [("left", 2), ("right", 2), ("both", 3)])
def test_position_routes_named_targets_even_when_split_flag_is_false(side, count):
    mod = module("pipeline_launch")
    context = context_for(mod, controller_backend="position", hand_mode=side,
                          use_split_controller="false")
    nodes = mod._create_nodes(context)
    assert len(nodes) == count
    for node in nodes[:-1]:
        args = node._Node__arguments
        assert "--target-only" in args
        assert "/revo2_left/retarget/target_joint_states" in args
        assert "/revo2_right/retarget/target_joint_states" in args
    params = evaluate_parameters(context, nodes[-1]._Node__parameters)[-1]
    assert params["output_mode"] == "position"
    assert params["left_target_joint_state_topic"] == "/revo2_left/retarget/target_joint_states"


def test_speed_pid_backend_still_uses_only_retarget_node():
    mod = module("pipeline_launch")
    nodes = mod._create_nodes(context_for(mod, controller_backend="ros2_control", hand_mode="left"))
    assert len(nodes) == 1
    assert "/revo2_left/revo2_pid_controller/target_joint_states" in nodes[0]._Node__arguments


def test_position_hardware_refuses_old_speed_yaml(tmp_path):
    mod = module("real_hand_pipeline_launch")
    old_config = tmp_path / "old.yaml"
    old_config.write_text("hardware:\n  finger_unit_mode: normalized\n")
    context = context_for(mod, controller_backend="position", hand_mode="left",
                          left_protocol_config_file=str(old_config))
    with pytest.raises(ValueError, match="normalized_position_control"):
        mod._create_actions(context)


@pytest.mark.parametrize("target,active", [
    ("joint_forward_pos_controller", ["joint_forward_pos_controller", "joint_forward_vel_controller"]),
    ("joint_forward_pos_controller", ["revo2_pid_controller"]),
    ("revo2_pid_controller", ["joint_forward_pos_controller", "joint_forward_vel_controller"]),
])
def test_switch_deactivates_competitors_even_if_target_is_active(tmp_path, target, active):
    state_file = tmp_path / "state.json"
    state_file.write_text(json.dumps({name: "active" if name in active else "inactive" for name in (
        "joint_forward_pos_controller", "joint_forward_vel_controller", "revo2_pid_controller")}))
    fake_ros2 = tmp_path / "ros2"
    fake_ros2.write_text('''#!/usr/bin/python3
import json, os, sys
from pathlib import Path
p = Path(os.environ["MOCK_CONTROLLERS"])
states = json.loads(p.read_text())
args = sys.argv[1:]
if args[1] == "list_controllers":
    for name, state in states.items(): print(name, "fake/Controller", state)
elif args[1] == "switch_controllers":
    assert args.count("--deactivate") <= 1
    mode = None
    for arg in args[2:]:
        if arg == "--activate": mode = "active"
        elif arg == "--deactivate": mode = "inactive"
        elif arg.startswith("-"): mode = None
        elif mode: states[arg] = mode
    p.write_text(json.dumps(states))
else: raise RuntimeError(args)
''')
    fake_ros2.chmod(0o755)
    script = module("real_hand_pipeline_launch")._switch_controller_command("/mock/controller_manager", target)[-1]
    env = dict(os.environ, PATH=f"{tmp_path}:{os.environ['PATH']}", MOCK_CONTROLLERS=str(state_file))
    result = subprocess.run(["bash", "-c", script], env=env, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    states = json.loads(state_file.read_text())
    assert states[target] == "active"
    assert all(state == "inactive" for name, state in states.items() if name != target)
