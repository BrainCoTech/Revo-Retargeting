import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_node(context: LaunchContext, *args, **kwargs):
    del args, kwargs
    side = LaunchConfiguration("side").perform(context).strip().lower()
    config_file = LaunchConfiguration("config_file").perform(context).strip()
    if not os.path.isabs(config_file):
        config_file = os.path.join(
            get_package_share_directory("glove_input_adapter"), "config", config_file
        )
    input_topic = LaunchConfiguration("input_topic").perform(context).strip()
    output_topic = LaunchConfiguration("output_topic").perform(context).strip()
    if not input_topic:
        input_topic = f"/humandex_{side}/joint_states"
    if not output_topic:
        output_topic = f"/revo2_{side}/revo2_pid_controller/target_joint_states"
    return [
        Node(
            package="glove_input_adapter",
            executable="humandex_absolute_revo2_node",
            name=f"humandex_absolute_revo2_{side}",
            parameters=[
                config_file,
                {"side": side, "input_topic": input_topic, "output_topic": output_topic},
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("side", default_value="left", choices=["left", "right"]),
        DeclareLaunchArgument("input_topic", default_value=""),
        DeclareLaunchArgument("output_topic", default_value=""),
        DeclareLaunchArgument("config_file", default_value="humandex_absolute_revo2.yaml"),
        OpaqueFunction(function=_create_node),
    ])
