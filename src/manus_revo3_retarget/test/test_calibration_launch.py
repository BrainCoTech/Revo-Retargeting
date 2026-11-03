import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


def load_launch():
    path = Path(__file__).parents[1] / 'launch/pipeline_launch.py'
    spec = importlib.util.spec_from_file_location('calibration_pipeline', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_panel_launch_rejects_automatic_output():
    module = load_launch()
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(start_enabled='true', launch_tuning_panel='true')
    with pytest.raises(ValueError, match='start_enabled:=false'):
        module._create_runtime_nodes(context)


def test_exported_override_is_loadable_by_pipeline(tmp_path):
    module = load_launch()
    config = tmp_path / 'operator_revo3_right.yaml'
    params = {'physical_right_index_MPR_joint_scale': -1.25,
              'legacy_right_physical_thumb_pip_ik_scale': 0.75}
    config.write_text(yaml.safe_dump({'manus_revo3_retarget': {'ros__parameters': params}}))
    assert module._load_ros_parameters(str(config)) == params
