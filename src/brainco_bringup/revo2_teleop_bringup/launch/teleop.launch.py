"""Single profile-driven entry point for all Revo2 hand inputs."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


SUPPORTED_INPUTS = {"humandex", "manus", "hex"}


def _as_bool(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _profile_path(value: str) -> Path:
    requested = Path(value).expanduser()
    if requested.is_absolute() or requested.parent != Path("."):
        return requested
    share = Path(get_package_share_directory("revo2_teleop_bringup"))
    name = requested.name if requested.suffix else requested.name + ".yaml"
    return share / "profiles" / name


def _load_profile(value: str) -> tuple[Path, dict]:
    path = _profile_path(value)
    if not path.is_file():
        raise RuntimeError(f"teleoperation profile does not exist: {path}")
    with path.open("r", encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    if not isinstance(profile, dict):
        raise RuntimeError(f"teleoperation profile must be a YAML mapping: {path}")
    return path, profile


def _selected_hand_mode(context: LaunchContext, profile: dict) -> str:
    override = LaunchConfiguration("hand_mode").perform(context).strip().lower()
    value = override or str(profile.get("hand_mode", "right")).strip().lower()
    if value not in {"left", "right", "both"}:
        raise RuntimeError("hand_mode must be left, right, or both")
    return value


def _input_actions(source: str, hand_mode: str, config: dict, launch_driver: bool):
    actions = []
    params = dict(config.get("parameters") or {})
    params["hand_mode"] = hand_mode

    if source == "humandex":
        actions.append(Node(
            package="hand_input_adapters",
            executable="humandex_hand_adapter",
            name="humandex_hand_adapter",
            parameters=[params],
            output="screen",
        ))
    elif source == "manus":
        if launch_driver:
            actions.append(Node(
                package="manus_ros2",
                executable="manus_data_publisher",
                name="manus_data_publisher",
                output="screen",
            ))
        actions.append(Node(
            package="hand_input_adapters",
            executable="manus_hand_adapter",
            name="manus_hand_adapter",
            parameters=[params],
            output="screen",
        ))
    elif source == "hex":
        if launch_driver:
            driver_params = dict(config.get("driver_parameters") or {})
            actions.append(Node(
                package="hex_glove_driver",
                executable="hex_glove_udp_node",
                name="hex_glove_udp_node",
                parameters=[driver_params],
                output="screen",
            ))
        actions.append(Node(
            package="hand_input_adapters",
            executable="hex_hand_adapter",
            name="hex_hand_adapter",
            parameters=[params],
            output="screen",
        ))
    return actions


def _create_actions(context: LaunchContext, *args, **kwargs):
    del args, kwargs
    profile_path, profile = _load_profile(LaunchConfiguration("profile").perform(context))
    hand_mode = _selected_hand_mode(context, profile)
    input_config = dict(profile.get("input") or {})
    source = str(input_config.get("source", "")).strip().lower()
    if source not in SUPPORTED_INPUTS:
        raise RuntimeError(f"profile input.source must be one of {sorted(SUPPORTED_INPUTS)}")

    launch_input_override = LaunchConfiguration("launch_input_driver").perform(context).strip()
    launch_input_driver = (
        _as_bool(launch_input_override)
        if launch_input_override
        else bool(input_config.get("launch_driver", source != "humandex"))
    )

    if source == "humandex":
        parameters = dict(input_config.get("parameters") or {})
        joint_topic = LaunchConfiguration("humandex_joint_topic").perform(context).strip()
        pose_topic = LaunchConfiguration("humandex_pose_topic").perform(context).strip()
        if joint_topic:
            parameters["joint_topic"] = joint_topic
        if pose_topic:
            parameters["pose_topic"] = pose_topic
        input_config["parameters"] = parameters

    actions = [
        LogInfo(msg=(
            f"Revo2 teleop profile={profile_path} input={source} "
            f"hand_mode={hand_mode} launch_input_driver={launch_input_driver}"
        )),
        *_input_actions(source, hand_mode, input_config, launch_input_driver),
    ]

    retarget_share = Path(get_package_share_directory("revo2_hand_retarget"))
    common_launch = retarget_share / "launch" / "real_hand_pipeline_launch.py"
    retarget_config = dict(profile.get("retarget") or {})
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(common_launch)),
        launch_arguments={
            "hand_mode": hand_mode,
            "launch_driver": LaunchConfiguration("launch_revo2_driver"),
            "switch_controllers": LaunchConfiguration("switch_controllers"),
            "launch_retarget": "true",
            "launch_plot": LaunchConfiguration("launch_plot"),
            "use_split_controller": "true",
            "controller_backend": "ros2_control",
            "control_config": str(retarget_config.get("control_config", "retarget.yaml")),
            "retarget_config": str(retarget_config.get("algorithm_config", "")),
            "if_sim": LaunchConfiguration("if_sim"),
            "retarget_delay": str(retarget_config.get("start_delay_sec", 18.0)),
        }.items(),
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="humandex_revo2"),
        DeclareLaunchArgument("hand_mode", default_value=""),
        DeclareLaunchArgument("launch_input_driver", default_value=""),
        DeclareLaunchArgument("humandex_joint_topic", default_value=""),
        DeclareLaunchArgument("humandex_pose_topic", default_value=""),
        DeclareLaunchArgument("launch_revo2_driver", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("switch_controllers", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("launch_plot", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("if_sim", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=_create_actions),
    ])
