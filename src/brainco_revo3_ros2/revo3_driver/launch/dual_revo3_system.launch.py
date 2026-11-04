#!/usr/bin/env python3
# Copyright (c) 2025 BrainCo
# SPDX-License-Identifier: Apache-2.0
"""Launch file for dual (bimanual) Revo3 21-DoF dexterous hands."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="revo3_description",
            description="Description package with revo3 system xacro files.",
        ),
        DeclareLaunchArgument(
            "left_protocol_config_file",
            default_value="",
            description="左手 Modbus 协议配置文件（YAML），留空则使用默认配置。",
        ),
        DeclareLaunchArgument(
            "right_protocol_config_file",
            default_value="",
            description="右手 Modbus 协议配置文件（YAML），留空则使用默认配置。",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="",
            description="控制器模板文件（YAML）覆盖项，留空则使用 revo3_controllers.yaml。",
        ),
        DeclareLaunchArgument(
            "use_namespace",
            default_value="true",
            description="是否为左右手分别使用命名空间（revo3_left / revo3_right）。",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "if_sim",
            default_value="false",
            description="使用 mock_components/GenericSystem 模拟硬件。",
        ),
        DeclareLaunchArgument(
            "launch_rsp",
            default_value="true",
            description="是否启动左右手各自的 robot_state_publisher。",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="是否启动 RViz2 可视化（左右手各自一个窗口）。",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="",
            description=(
                "RViz 配置文件绝对路径或文件名（相对于 <description_package>/rviz）。"
                "留空则使用 revo3_hand.rviz。"
            ),
        ),
        DeclareLaunchArgument(
            "update_rate",
            default_value="200",
            description="controller_manager update_rate (Hz)，同时应用于左右手。",
        ),
    ]

    single_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare("revo3_driver"),
            "launch",
            "revo3_system.launch.py",
        ])
    )

    left_hand_launch = IncludeLaunchDescription(
        single_launch,
        launch_arguments={
            "description_package":   LaunchConfiguration("description_package"),
            "hand_side":             "left",
            "protocol_config_file":  LaunchConfiguration("left_protocol_config_file"),
            "controllers_file":      LaunchConfiguration("controllers_file"),
            "use_namespace":         LaunchConfiguration("use_namespace"),
            "if_sim":                LaunchConfiguration("if_sim"),
            "launch_rsp":            LaunchConfiguration("launch_rsp"),
            "launch_rviz":           LaunchConfiguration("launch_rviz"),
            "rviz_config_file":      LaunchConfiguration("rviz_config_file"),
            "update_rate":           LaunchConfiguration("update_rate"),
        }.items(),
    )

    right_hand_launch = IncludeLaunchDescription(
        single_launch,
        launch_arguments={
            "description_package":   LaunchConfiguration("description_package"),
            "hand_side":             "right",
            "protocol_config_file":  LaunchConfiguration("right_protocol_config_file"),
            "controllers_file":      LaunchConfiguration("controllers_file"),
            "use_namespace":         LaunchConfiguration("use_namespace"),
            "if_sim":                LaunchConfiguration("if_sim"),
            "launch_rsp":            LaunchConfiguration("launch_rsp"),
            "launch_rviz":           LaunchConfiguration("launch_rviz"),
            "rviz_config_file":      LaunchConfiguration("rviz_config_file"),
            "update_rate":           LaunchConfiguration("update_rate"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [left_hand_launch, right_hand_launch])
