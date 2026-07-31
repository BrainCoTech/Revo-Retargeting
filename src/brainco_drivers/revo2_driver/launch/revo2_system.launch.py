#!/usr/bin/env python3

import os
import tempfile
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


HAND_UPDATE_RATES = {
    "left": "500",
    "right": "500",
}


def _resolve_config_path(config_root, override, default_filename):
    if override:
        return override if os.path.isabs(override) else os.path.join(config_root, override)
    return os.path.join(config_root, default_filename)


def _default_protocol_filename(hand_side, protocol):
    return f"protocol_{protocol}_{hand_side}.yaml"


def _default_initial_positions_filename(hand_side):
    return f"revo2_initial_positions.yaml"


def _generate_description(description_package, mappings):
    share = get_package_share_directory(description_package)
    xacro_path = os.path.join(share, "urdf", "revo2.single.system.xacro")
    return xacro.process_file(xacro_path, mappings=mappings).toprettyxml(indent="  ")


def _load_controllers(driver_share, hand_side, controllers_file_override, update_rate_override):
    config_root = os.path.join(driver_share, "config")
    template_path = _resolve_config_path(
        config_root,
        controllers_file_override,
        "revo2_controllers.yaml",
    )

    with open(template_path) as stream:
        content = stream.read()

    content = content.replace("HAND_PREFIX", hand_side)
    update_rate = update_rate_override or HAND_UPDATE_RATES[hand_side]
    content = content.replace("UPDATE_RATE", update_rate)

    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    tmp.write(content)
    tmp.close()
    return tmp.name


def launch_setup(context: LaunchContext):
    description_package = context.perform_substitution(LaunchConfiguration("description_package"))
    hand_side = context.perform_substitution(LaunchConfiguration("hand_side")).lower()
    protocol = context.perform_substitution(LaunchConfiguration("protocol")).lower()
    protocol_config_override = context.perform_substitution(LaunchConfiguration("protocol_config_file"))
    initial_positions_override = context.perform_substitution(LaunchConfiguration("initial_positions_file"))
    controllers_file_override = context.perform_substitution(LaunchConfiguration("controllers_file"))
    update_rate_override = context.perform_substitution(LaunchConfiguration("update_rate"))
    read_touch_status = context.perform_substitution(LaunchConfiguration("read_touch_status"))
    touch_read_hz = context.perform_substitution(LaunchConfiguration("touch_read_hz"))
    if_sim = context.perform_substitution(LaunchConfiguration("if_sim"))
    use_namespace = context.perform_substitution(LaunchConfiguration("use_namespace")).lower() == "true"

    driver_share = get_package_share_directory("revo2_driver")
    driver_config_root = os.path.join(driver_share, "config")
    desc_share = get_package_share_directory("revo2_description")
    desc_config_root = os.path.join(desc_share, "config")

    protocol_config_file = _resolve_config_path(
        driver_config_root,
        protocol_config_override,
        _default_protocol_filename(hand_side, protocol),
    )
    initial_positions_file = _resolve_config_path(
        desc_config_root,
        initial_positions_override,
        _default_initial_positions_filename(hand_side),
    )
    controllers_file = _load_controllers(
        driver_share,
        hand_side,
        controllers_file_override,
        update_rate_override,
    )

    robot_description = {
        "robot_description": _generate_description(
            description_package,
            {
                "hand_side": hand_side,
                "protocol_config_file": protocol_config_file,
                "initial_positions_file": initial_positions_file,
                "read_touch_status": read_touch_status,
                "touch_read_hz": touch_read_hz,
                "if_sim": if_sim,
            },
        )
    }

    namespace = f"revo2_{hand_side}" if use_namespace else ""
    controller_manager_name = f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    joint_state_topic = f"/{namespace}/revo2_joint_state/joint_states" if namespace else "/revo2_joint_state/joint_states"

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

    def make_spawner(names, inactive=False):
        """Spawn one or more controllers in a single spawner process.

        Passing several names to one spawner makes it load/configure them
        sequentially inside that process, so the (single-threaded) controller
        manager only ever services one spawn request at a time. The hand CM is
        especially prone to dropping spawn responses because its thread also
        does blocking Modbus I/O (write_multiple_registers timeouts), which
        stalls service handling; concurrent spawners overran it and left
        controllers stuck 'unconfigured' (FATAL 'already loaded' on retry).
        Combined with the OnProcessExit chaining below this removes the race.
        """
        if isinstance(names, str):
            names = [names]
        arguments = list(names)
        if inactive:
            arguments.append("--inactive")
        arguments.extend(["-c", controller_manager_name])
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=arguments,
            output="both",
        )

    # Active controllers first, in one sequential spawner; then the inactive
    # set after that spawner exits. Two processes total instead of four
    # concurrent ones, chained so the CM never faces overlapping spawn calls.
    active_spawner = make_spawner([
        "revo2_joint_state",
        "joint_forward_pos_controller",
    ])
    inactive_spawner = make_spawner([
        "joint_forward_vel_controller",
        "revo2_pid_controller",
    ], inactive=True)

    return [
        control_node,
        robot_state_publisher,
        TimerAction(period=1.0, actions=[active_spawner]),
        RegisterEventHandler(OnProcessExit(
            target_action=active_spawner,
            on_exit=[inactive_spawner],
        )),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="revo2_description",
            description="Description package with revo2 system xacro files.",
        ),
        DeclareLaunchArgument(
            "hand_side",
            default_value="left",
            description="Hand side: left or right.",
            choices=["left", "right"],
        ),
        DeclareLaunchArgument(
            "protocol",
            default_value="modbus",
            description="通信协议：modbus 或 canfd",
            choices=["modbus", "canfd"],
        ),
        DeclareLaunchArgument(
            "protocol_config_file",
            default_value="",
            description="协议配置文件（YAML），留空则使用默认配置。",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="",
            description="初始位置配置文件（YAML），留空则使用单手默认配置。",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="",
            description="控制器模板文件（YAML），留空则使用 revo2_controllers.yaml。",
        ),
        DeclareLaunchArgument(
            "update_rate",
            default_value="",
            description="Override controller_manager update_rate, e.g. 100 for Modbus speed tests.",
        ),
        DeclareLaunchArgument(
            "read_touch_status",
            default_value="true",
            description="Read Revo2 touch status synchronously from the SDK.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "touch_read_hz",
            default_value="20.0",
            description="Touch status SDK polling rate when read_touch_status is true.",
        ),
        DeclareLaunchArgument(
            "use_namespace",
            default_value="true",
            description="是否使用命名空间（true: 使用 revo2_{hand_side}，false: 不使用命名空间）",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "if_sim",
            default_value="false",
            description="使用 mock_components/GenericSystem 模拟硬件",
        ),
        DeclareLaunchArgument(
            "launch_rsp",
            default_value="true",
            description="是否启动单手 robot_state_publisher",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
