"""Compatibility Revo2 bringup; DV1 acquisition must run externally."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def create_actions(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    side = value('hand_mode')
    adapter_share = Path(get_package_share_directory('hand_input_adapters'))
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(adapter_share / 'launch' / 'dv1_input.launch.py')),
        launch_arguments={
            'hand_mode': side,
            'urdf_path': value('urdf_path'),
            'adapter_config': value('adapter_config'),
        }.items(),
    )]
    hardware = value('launch_revo2_driver') == 'true'
    retarget = value('launch_retarget') == 'true'
    if hardware and not retarget:
        raise ValueError('launch_revo2_driver requires launch_retarget:=true')
    if retarget:
        retarget_share = Path(get_package_share_directory('revo2_hand_retarget'))
        arguments = {'hand_mode': side, 'controller_backend': 'ros2_control',
                     'control_config': value('control_config'),
                     'finger_flexion_config': value('finger_flexion_config') or f'flexion_dv1_{side}.yaml'}
        name = 'pipeline_launch.py'
        if hardware:
            name = 'real_hand_pipeline_launch.py'
            arguments.update(launch_driver='true', launch_retarget='true',
                             switch_controllers='true', launch_plot='false', if_sim='false')
            protocol_config = value('revo2_protocol_config_file').strip()
            if protocol_config:
                path = Path(protocol_config).expanduser().resolve()
                if not path.is_file():
                    raise ValueError(f'Revo2 protocol config not found: {path}')
                arguments[f'{side}_protocol_config_file'] = str(path)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(retarget_share / 'launch' / name)),
            launch_arguments=arguments.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hand_mode', default_value='left', choices=['left', 'right']),
        DeclareLaunchArgument('urdf_path', description='Path to the external SDK DV1 URDF'),
        DeclareLaunchArgument('revo2_protocol_config_file', default_value='',
                             description='Optional Revo2 robot protocol YAML for the selected side'),
        DeclareLaunchArgument('adapter_config', default_value=''),
        DeclareLaunchArgument('finger_flexion_config', default_value=''),
        DeclareLaunchArgument('control_config', default_value='retarget.yaml'),
        *(DeclareLaunchArgument(k, default_value=v, choices=['true', 'false']) for k, v in (
            ('launch_retarget', 'false'), ('launch_revo2_driver', 'false'))),
        OpaqueFunction(function=create_actions),
    ])
