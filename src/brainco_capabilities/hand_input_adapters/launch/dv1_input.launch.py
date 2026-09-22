"""Listen to external DV1 JointState data and publish HandKinematics."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_actions(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    side = value('hand_mode')
    urdf = Path(value('urdf_path')).expanduser()
    if not urdf.is_file():
        raise ValueError(f'DV1 URDF not found: {urdf}; set urdf_path')
    share = Path(get_package_share_directory('hand_input_adapters'))
    config = Path(value('adapter_config')).expanduser() if value('adapter_config') else (
        share / 'config' / f'dv1_{side}.yaml')
    if not config.is_file():
        raise ValueError(f'Adapter config not found: {config}')
    overrides = {}
    if value('output_topic'):
        overrides[f'{side}_output_topic'] = value('output_topic')
    return [Node(
        package='hand_input_adapters', executable='humandex_hand_adapter',
        name='humandex_hand_adapter', output='screen',
        parameters=[str(config), {
            'input_mode': 'dv1_joint_states', 'hand_mode': side,
            'joint_topic': value('joint_topic') or f'/humandex_{side}/joint_states',
            'urdf_path': str(urdf.resolve()),
            **overrides,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hand_mode', default_value='right', choices=['left', 'right']),
        DeclareLaunchArgument('urdf_path', description='Path to the external SDK DV1 URDF'),
        DeclareLaunchArgument('adapter_config', default_value=''),
        DeclareLaunchArgument('joint_topic', default_value=''),
        DeclareLaunchArgument('output_topic', default_value=''),
        OpaqueFunction(function=create_actions),
    ])
