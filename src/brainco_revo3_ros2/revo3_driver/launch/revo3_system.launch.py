#!/usr/bin/env python3
# Copyright (c) 2025 BrainCo
# SPDX-License-Identifier: Apache-2.0
"""Launch file for a single Revo3 21-DoF dexterous hand."""

import os
import tempfile
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Default controller_manager update rate (Hz). Override via launch arg `update_rate`.
# Revo3 Modbus at 5 Mbps round-trip (read 21 motors + write MIT) typically caps
# the achievable rate to ~150-250 Hz on a single hand. Pushing too high will
# silently throttle JointTrajectoryController interpolation and produce
# visible stutter. Verify with: ros2 topic hz /<ns>/revo3_joint_state/joint_states
DEFAULT_HAND_UPDATE_RATE = "200"


def _resolve_config_path(config_root: str, override: str, default_filename: str) -> str:
    if override:
        return override if os.path.isabs(override) else os.path.join(config_root, override)
    return os.path.join(config_root, default_filename)


def _patch_rviz_config(template_path: str, robot_description_topic: str) -> str:
    """Write a temp RViz config with the namespaced robot_description topic."""
    with open(template_path) as f:
        content = f.read()

    # Support both legacy absolute and relative placeholders.
    for placeholder in ("/robot_description", "robot_description"):
        needle = f"Value: {placeholder}"
        if needle in content:
            content = content.replace(needle, f"Value: {robot_description_topic}", 1)
            break

    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".rviz", delete=False)
    tmp.write(content)
    tmp.close()
    return tmp.name


def _load_controllers(
    driver_share: str,
    hand_side: str,
    controllers_override: str,
    update_rate: str,
) -> str:
    """
    Read the controller YAML template, substitute HAND_PREFIX / UPDATE_RATE,
    write to a temp file and return the path.
    """
    config_root = os.path.join(driver_share, "config")
    template_path = _resolve_config_path(
        config_root, controllers_override, "revo3_controllers.yaml"
    )
    with open(template_path) as f:
        content = f.read()

    content = content.replace("HAND_PREFIX", hand_side)
    content = content.replace("UPDATE_RATE", update_rate)

    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    tmp.write(content)
    tmp.close()
    return tmp.name


def _generate_robot_description(
    description_package: str,
    hand_side: str,
    protocol_config_file: str,
    if_sim: str,
    initial_positions_file: str,
) -> str:
    """
    Expand the revo3 system xacro and return it as a string.
    The xacro must live at:
      <revo3_description>/urdf/revo3.single.system.xacro
    or the caller can supply a custom description_package.
    """
    share = get_package_share_directory(description_package)
    xacro_path = os.path.join(share, "urdf", "revo3.single.system.xacro")
    doc = xacro.process_file(
        xacro_path,
        mappings={
            "hand_side": hand_side,
            "protocol_config_file": protocol_config_file,
            "if_sim": if_sim,
            "initial_positions_file": initial_positions_file,
        },
    )
    return doc.toprettyxml(indent="  ")


def launch_setup(context: LaunchContext):
    description_package = context.perform_substitution(
        LaunchConfiguration("description_package")
    )
    hand_side = (
        context.perform_substitution(LaunchConfiguration("hand_side")).lower()
    )
    protocol_config_override = context.perform_substitution(
        LaunchConfiguration("protocol_config_file")
    )
    controllers_override = context.perform_substitution(
        LaunchConfiguration("controllers_file")
    )
    if_sim = context.perform_substitution(LaunchConfiguration("if_sim"))
    use_namespace = (
        context.perform_substitution(LaunchConfiguration("use_namespace")).lower() == "true"
    )
    rviz_config_override = context.perform_substitution(
        LaunchConfiguration("rviz_config_file")
    )
    initial_positions_override = context.perform_substitution(
        LaunchConfiguration("initial_positions_file")
    )
    update_rate = context.perform_substitution(LaunchConfiguration("update_rate"))

    driver_share = get_package_share_directory("revo3_driver")
    driver_config_root = os.path.join(driver_share, "config")

    # ── Protocol config ──────────────────────────────────────────────────────
    protocol_config_file = _resolve_config_path(
        driver_config_root,
        protocol_config_override,
        f"protocol_modbus_{hand_side}.yaml",
    )
    # ── Initial positions YAML ───────────────────────────────────────────────────────────
    initial_positions_file = _resolve_config_path(
        driver_config_root,
        initial_positions_override,
        f"initial_positions_{hand_side}.yaml",
    )
    # ── Controller YAML (template → resolved temp file) ──────────────────────
    controllers_file = _load_controllers(
        driver_share, hand_side, controllers_override, update_rate
    )

    # ── Robot description (URDF via xacro) ───────────────────────────────────
    robot_description = {
        "robot_description": _generate_robot_description(
            description_package,
            hand_side,
            protocol_config_file,
            if_sim,
            initial_positions_file,
        )
    }

    # ── Namespace / topic helpers ────────────────────────────────────────────
    namespace = f"revo3_{hand_side}" if use_namespace else ""
    cm_name = f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    joint_state_topic = (
        f"/{namespace}/revo3_joint_state/joint_states"
        if namespace
        else "/revo3_joint_state/joint_states"
    )

    # ── RViz config path ─────────────────────────────────────────────────────
    if rviz_config_override:
        rviz_config_path = (
            rviz_config_override
            if os.path.isabs(rviz_config_override)
            else os.path.join(
                get_package_share_directory(description_package),
                "rviz",
                rviz_config_override,
            )
        )
    else:
        rviz_config_path = os.path.join(
            get_package_share_directory(description_package), "rviz", "revo3_hand.rviz"
        )
    robot_description_topic = (
        f"/{namespace}/robot_description" if namespace else "/robot_description"
    )
    rviz_config_path = _patch_rviz_config(rviz_config_path, robot_description_topic)

    # ── Nodes ────────────────────────────────────────────────────────────────
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[robot_description, controllers_file],
        output="both",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[robot_description],
        remappings=[("joint_states", joint_state_topic)],
        condition=IfCondition(LaunchConfiguration("launch_rsp")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    def make_spawner(name: str, inactive: bool = False):
        args = [name]
        if inactive:
            args.append("--inactive")
        args.extend(["-c", cm_name])
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=args,
            output="both",
        )

    return [
        control_node,
        robot_state_publisher,
        rviz_node,
        TimerAction(period=1.0, actions=[make_spawner("revo3_joint_state")]),
        TimerAction(period=1.5, actions=[make_spawner("joint_forward_mit_controller")]),
        TimerAction(period=2.0, actions=[make_spawner("joint_forward_pos_controller", inactive=True)]),
        TimerAction(period=2.5, actions=[make_spawner("joint_traj_pos_controller", inactive=True)]),
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "description_package",
            default_value="revo3_description",
            description="Package that provides the revo3.single.system.xacro.",
        ),
        DeclareLaunchArgument(
            "hand_side",
            default_value="right",
            description="Hand side: left or right.",
            choices=["left", "right"],
        ),
        DeclareLaunchArgument(
            "protocol_config_file",
            default_value="",
            description=(
                "Absolute path or filename (relative to revo3_driver/config) of the "
                "Modbus protocol YAML. Empty → auto-selected from hand_side."
            ),
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="",
            description="Controller YAML template override. Empty → revo3_controllers.yaml.",
        ),
        DeclareLaunchArgument(
            "use_namespace",
            default_value="true",
            description="Namespace the nodes under revo3_{hand_side}.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "if_sim",
            default_value="false",
            description="Use mock_components/GenericSystem instead of the real hardware.",
        ),
        DeclareLaunchArgument(
            "launch_rsp",
            default_value="true",
            description="Launch robot_state_publisher.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz2 for visualization.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="",
            description=(
                "Absolute path or filename (relative to <description_package>/rviz) of the "
                "RViz config. Empty → revo3_hand.rviz."
            ),
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="",
            description=(
                "Absolute path or filename (relative to revo3_driver/config) of the initial "
                "joint positions YAML. Empty → auto-selected as initial_positions_{hand_side}.yaml."
            ),
        ),
        DeclareLaunchArgument(
            "update_rate",
            default_value=DEFAULT_HAND_UPDATE_RATE,
            description=(
                "controller_manager update_rate (Hz). Substituted into the controller YAML. "
                "Modbus 5 Mbps comfortably handles ~200 Hz; raising this without verifying "
                "joint_states hz will silently throttle JTC interpolation."
            ),
        ),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
