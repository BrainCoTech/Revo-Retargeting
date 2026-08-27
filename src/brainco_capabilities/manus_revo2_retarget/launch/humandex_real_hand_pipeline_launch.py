import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _selected_sides(hand_mode):
    if hand_mode == "both":
        return ("left", "right")
    if hand_mode in ("left", "right"):
        return (hand_mode,)
    raise RuntimeError("hand_mode must be left, right, or both")


def _create_actions(context: LaunchContext, *args, **kwargs):
    del args, kwargs
    hand_mode = LaunchConfiguration("hand_mode").perform(context).strip().lower()
    sides = _selected_sides(hand_mode)
    launch_glove_adapter = _as_bool(
        LaunchConfiguration("launch_glove_adapter").perform(context)
    )
    launch_revo2_pipeline = _as_bool(
        LaunchConfiguration("launch_revo2_pipeline").perform(context)
    )

    actions = [
        LogInfo(msg=f"Starting HumanDex Revo2 pipeline: hand_mode={hand_mode}")
    ]

    if launch_glove_adapter:
        adapter_share = get_package_share_directory("glove_input_adapter")
        adapter_launch = os.path.join(
            adapter_share, "launch", "humandex_pose_adapter.launch.py"
        )
        for side in sides:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(adapter_launch),
                    launch_arguments={
                        "side": side,
                        "input_hand_mode": hand_mode,
                        "input_topic": LaunchConfiguration("eef_pose_topic"),
                        "output_topic": f"/manus_glove_{0 if side == 'left' else 1}",
                        "expected_frame_id": LaunchConfiguration("eef_pose_frame_id"),
                        "config_file": LaunchConfiguration("adapter_config_file"),
                        "joint_topic": f"/humandex_{side}/joint_states",
                    }.items(),
                )
            )

    if launch_revo2_pipeline:
        retarget_share = get_package_share_directory("manus_revo2_retarget")
        real_hand_launch = os.path.join(
            retarget_share, "launch", "real_hand_pipeline_launch.py"
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(real_hand_launch),
                launch_arguments={
                    "hand_mode": hand_mode,
                    "update_rate": LaunchConfiguration("update_rate"),
                    "read_touch_status": LaunchConfiguration("read_touch_status"),
                    "touch_read_hz": LaunchConfiguration("touch_read_hz"),
                    "switch_delay": LaunchConfiguration("switch_delay"),
                    "retarget_delay": LaunchConfiguration("retarget_delay"),
                    "plot_delay": LaunchConfiguration("plot_delay"),
                    "launch_driver": LaunchConfiguration("launch_driver"),
                    "switch_controllers": LaunchConfiguration("switch_controllers"),
                    "launch_retarget": LaunchConfiguration("launch_retarget"),
                    "launch_plot": LaunchConfiguration("launch_plot"),
                    "launch_manus_publisher": "false",
                    "use_split_controller": "true",
                    "controller_backend": "ros2_control",
                    "control_config": LaunchConfiguration("control_config"),
                    "retarget_config": LaunchConfiguration("retarget_config"),
                    "protocol": LaunchConfiguration("protocol"),
                    "left_protocol_config_file": LaunchConfiguration(
                        "left_protocol_config_file"
                    ),
                    "right_protocol_config_file": LaunchConfiguration(
                        "right_protocol_config_file"
                    ),
                    "use_namespace": "true",
                    "if_sim": LaunchConfiguration("if_sim"),
                    "launch_rsp": LaunchConfiguration("launch_rsp"),
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "hand_mode", default_value="right", choices=["left", "right", "both"]
        ),
        DeclareLaunchArgument("eef_pose_topic", default_value="/humandex_eef_pose"),
        DeclareLaunchArgument("eef_pose_frame_id", default_value="hand_base_link_local"),
        DeclareLaunchArgument("adapter_config_file", default_value="humandex_pose_adapter.yaml"),
        DeclareLaunchArgument(
            "launch_glove_adapter", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "launch_revo2_pipeline", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument("update_rate", default_value="20"),
        DeclareLaunchArgument(
            "read_touch_status", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument("touch_read_hz", default_value="20.0"),
        DeclareLaunchArgument("switch_delay", default_value="16.0"),
        DeclareLaunchArgument("retarget_delay", default_value="18.0"),
        DeclareLaunchArgument("plot_delay", default_value="19.0"),
        DeclareLaunchArgument(
            "launch_driver", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "switch_controllers", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "launch_retarget", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "launch_plot", default_value="false", choices=["true", "false"]
        ),
        DeclareLaunchArgument("control_config", default_value="retarget.yaml"),
        DeclareLaunchArgument("retarget_config", default_value=""),
        DeclareLaunchArgument(
            "protocol", default_value="modbus", choices=["modbus", "canfd"]
        ),
        DeclareLaunchArgument("left_protocol_config_file", default_value=""),
        DeclareLaunchArgument("right_protocol_config_file", default_value=""),
        DeclareLaunchArgument("if_sim", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("launch_rsp", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=_create_actions),
    ])
