from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {"sdk_path": "", "port": "", "side": "left",
                "target_rate_hz": "100.0", "timeout_sec": "1.0",
                "reconnect_delay_sec": "2.0"}
    parameters = {
        key: ParameterValue(LaunchConfiguration(key), value_type=(
            float if key.endswith(("_hz", "_sec")) else str))
        for key in defaults
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()),
        Node(package="revohuman_driver", executable="revohuman_node",
             parameters=[parameters], output="screen"),
    ])
