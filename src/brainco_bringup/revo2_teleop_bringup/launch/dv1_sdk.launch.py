"""SDK JointState -> adapter FK -> canonical hand, with optional Revo2 output."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_actions(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    side = value('hand_mode')
    sdk = Path(value('sdk_path')).expanduser().resolve()
    urdf = Path(value('urdf_path')).expanduser() if value('urdf_path') else (
        sdk / 'description/urdf/Revo_Human_DV1_URDF_Bimanual.urdf')
    if not urdf.is_file():
        raise ValueError(f'DV1 URDF not found: {urdf}; set sdk_path or urdf_path')
    adapter_share = Path(get_package_share_directory('hand_input_adapters'))
    config = Path(value('adapter_config')).expanduser() if value('adapter_config') else (
        adapter_share / 'config' / f'dv1_{side}.yaml')
    if not config.is_file():
        raise ValueError(f'Adapter config not found: {config}')
    actions = []
    if value('launch_sdk') == 'true':
        script = sdk / 'tools/ros2_joint_state_pub.py'
        if not script.is_file() or not value('port').strip():
            raise ValueError('Set sdk_path to sdk-encoder-fk-viewer and port to the glove device')
        actions.append(ExecuteProcess(cmd=[value('sdk_python'), str(script), '--hand', side,
                                           '--port', value('port')], output='screen'))
    actions.append(Node(
        package='hand_input_adapters', executable='humandex_hand_adapter',
        name='humandex_hand_adapter', output='screen',
        parameters=[str(config), {'input_mode': 'dv1_joint_states', 'hand_mode': side,
                                 'joint_topic': f'/humandex_{side}/joint_states',
                                 'urdf_path': str(urdf.resolve())}]))
    hardware = value('launch_revo2_driver') == 'true'
    retarget = value('launch_retarget') == 'true'
    if hardware and not retarget:
        raise ValueError('launch_revo2_driver requires launch_retarget:=true')
    if retarget:
        retarget_share = Path(get_package_share_directory('revo2_hand_retarget'))
        arguments = {'hand_mode': side, 'controller_backend': value('controller_backend'),
                     'teleop_controller_config': value('teleop_controller_config'),
                     'control_config': value('control_config')}
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
        DeclareLaunchArgument('sdk_path', default_value=''),
        DeclareLaunchArgument('urdf_path', default_value=''),
        DeclareLaunchArgument('port', default_value='', description='RevoHuman glove serial port'),
        DeclareLaunchArgument('revo2_protocol_config_file', default_value='',
                             description='Optional Revo2 robot protocol YAML for the selected side'),
        DeclareLaunchArgument('sdk_python', default_value='/usr/bin/python3'),
        DeclareLaunchArgument('adapter_config', default_value=''),
        DeclareLaunchArgument('control_config', default_value='retarget.yaml'),
        DeclareLaunchArgument('controller_backend', default_value='ros2_control',
                             choices=['ros2_control', 'position']),
        DeclareLaunchArgument('teleop_controller_config', default_value='teleop_controller.yaml'),
        *(DeclareLaunchArgument(k, default_value=v, choices=['true', 'false']) for k, v in (
            ('launch_sdk', 'true'), ('launch_retarget', 'false'), ('launch_revo2_driver', 'false'))),
        OpaqueFunction(function=create_actions),
    ])
