import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_node(context: LaunchContext, *args, **kwargs):
    del args, kwargs
    side = LaunchConfiguration("side").perform(context).strip().lower()
    input_hand_mode = LaunchConfiguration("input_hand_mode").perform(context).strip().lower()
    if side not in ("left", "right"):
        raise RuntimeError("side must be left or right")
    if input_hand_mode not in ("left", "right", "both"):
        raise RuntimeError("input_hand_mode must be left, right, or both")
    if input_hand_mode != "both" and input_hand_mode != side:
        raise RuntimeError("side is not present in input_hand_mode")

    config_file = LaunchConfiguration("config_file").perform(context).strip()
    if not os.path.isabs(config_file):
        config_file = os.path.join(
            get_package_share_directory("glove_input_adapter"), "config", config_file
        )

    output_topic = LaunchConfiguration("output_topic").perform(context).strip()
    if not output_topic:
        output_topic = "/manus_glove_0" if side == "left" else "/manus_glove_1"
    joint_topic = LaunchConfiguration("joint_topic").perform(context).strip()
    if not joint_topic:
        joint_topic = f"/humandex_{side}/joint_states"

    return [
        Node(
            package="glove_input_adapter",
            executable="humandex_pose_adapter_node",
            name=f"humandex_pose_adapter_{side}",
            parameters=[
                config_file,
                {
                    "side": side,
                    "input_hand_mode": input_hand_mode,
                    "glove_id": 0 if side == "left" else 1,
                    "input_topic": LaunchConfiguration("input_topic").perform(context),
                    "joint_topic": joint_topic,
                    "output_topic": output_topic,
                    "expected_frame_id": LaunchConfiguration(
                        "expected_frame_id"
                    ).perform(context),
                    "ergonomics_enabled": LaunchConfiguration(
                        "ergonomics_enabled"
                    ).perform(context).lower() == "true",
                    "absolute_source_joint": LaunchConfiguration(
                        "absolute_source_joint"
                    ).perform(context),
                    "absolute_open_rad": float(
                        LaunchConfiguration("absolute_open_rad").perform(context)
                    ),
                    "absolute_closed_rad": float(
                        LaunchConfiguration("absolute_closed_rad").perform(context)
                    ),
                    "absolute_target_max_deg": float(
                        LaunchConfiguration("absolute_target_max_deg").perform(context)
                    ),
                },
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("side", default_value="right", choices=["left", "right"]),
        DeclareLaunchArgument(
            "input_hand_mode", default_value="right", choices=["left", "right", "both"]
        ),
        DeclareLaunchArgument("input_topic", default_value="/humandex_eef_pose"),
        DeclareLaunchArgument("joint_topic", default_value=""),
        DeclareLaunchArgument("output_topic", default_value=""),
        DeclareLaunchArgument("expected_frame_id", default_value="hand_base_link_local"),
        DeclareLaunchArgument("ergonomics_enabled", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("absolute_source_joint", default_value="PIP"),
        DeclareLaunchArgument("absolute_open_rad", default_value="0.20943951023931956"),
        DeclareLaunchArgument("absolute_closed_rad", default_value="-0.20943951023931956"),
        DeclareLaunchArgument("absolute_target_max_deg", default_value="84.00134234412998"),
        DeclareLaunchArgument("config_file", default_value="humandex_pose_adapter.yaml"),
        OpaqueFunction(function=_create_node),
    ])
