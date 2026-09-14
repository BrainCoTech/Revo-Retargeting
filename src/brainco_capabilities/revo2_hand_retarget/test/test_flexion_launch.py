"""Check explicit mapping selection across launch layers without starting ROS nodes."""
import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import yaml

ROOT=Path(__file__).resolve().parents[4]


def load(relative):
    spec=importlib.util.spec_from_file_location('launch_under_test',ROOT/relative)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_pipeline_passes_mapping_to_each_retarget_process(monkeypatch):
    module=load('src/brainco_capabilities/revo2_hand_retarget/launch/pipeline_launch.py')
    from types import SimpleNamespace
    monkeypatch.setattr(module, "Node", lambda **kwargs: SimpleNamespace(**kwargs))
    context=LaunchContext()
    context.launch_configurations.update(hand_mode='both',use_split_controller='true',controller_backend='ros2_control',control_config='retarget.yaml',retarget_config='',teleop_controller_config='',finger_flexion_config='flexion_manus.yaml')
    nodes=module._create_nodes(context)
    assert len(nodes)==2
    for node in nodes:
        command=node.arguments
        assert command[command.index('--finger-flexion-config')+1]=='flexion_manus.yaml'


def test_hardware_launch_declares_mapping_argument():
    module=load('src/brainco_capabilities/revo2_hand_retarget/launch/real_hand_pipeline_launch.py')
    declarations=[action.name for action in module.generate_launch_description().entities if isinstance(action,DeclareLaunchArgument)]
    assert 'finger_flexion_config' in declarations


def test_bringup_profiles_choose_robot_side_mapping():
    directory=ROOT/'src/brainco_bringup/revo2_teleop_bringup/profiles'
    for name,expected in [('manus_revo2','flexion_manus.yaml'),('humandex_revo2','flexion_humandex_pip.yaml')]:
        profile=yaml.safe_load((directory/(name+'.yaml')).read_text())
        assert profile['retarget']['finger_flexion_config']==expected
        assert not any('four_finger_' in key for key in profile['input']['parameters'])
