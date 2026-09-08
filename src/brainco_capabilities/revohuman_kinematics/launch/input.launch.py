"""One or two explicitly assigned glove ports; no Revo2 hardware startup."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_nodes(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    sides = ("left", "right") if value("hand_mode") == "both" else (value("hand_mode"),)
    start_driver = value("launch_driver") == "true"
    if start_driver:
        ports = [value(side + "_port") for side in sides]
        if any(not port for port in ports) or len({str(Path(p).resolve()) for p in ports}) != len(ports):
            raise ValueError("Specify a distinct, nonempty serial port for each selected hand")
    actions = []
    for side in sides:
        if start_driver:
            actions.append(Node(package="revohuman_driver", executable="revohuman_node",
                                name=f"revohuman_driver_{side}", output="screen", parameters=[{
                                    "side": side, "port": value(side + "_port"),
                                    "sdk_path": value("sdk_path"),
                                    "target_rate_hz": float(value("target_rate_hz"))}]))
        actions.append(Node(package="revohuman_kinematics", executable="revohuman_kinematics_node",
                            name=f"revohuman_kinematics_{side}", output="screen", parameters=[{
                                "side": side, "config_file": value("config_file"),
                                "urdf_path": value("urdf_path"), "max_age_sec": float(value("max_age_sec"))}]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("revohuman_kinematics"))
    defaults = {"sdk_path": "", "left_port": "", "right_port": "", "target_rate_hz": "100.0",
                "max_age_sec": "0.5", "config_file": str(share / "config/kinematics.yaml"),
                "urdf_path": str(share / "urdf/revohuman_dv1_kinematics.urdf")}
    return LaunchDescription([
        DeclareLaunchArgument("hand_mode", default_value="left", choices=["left", "right", "both"]),
        DeclareLaunchArgument("launch_driver", default_value="true", choices=["true", "false"]),
        *(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
        OpaqueFunction(function=create_nodes),
    ])
