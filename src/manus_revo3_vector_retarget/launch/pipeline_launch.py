"""Bringup entry point with the legacy hand mode and SetBool service names."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from retarget_v2.launch_support import backend_arguments, backend_process


def _launch(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    mode = value("hand_type") or value("hand_mode")
    if mode not in ("left", "right", "both"):
        raise ValueError("hand_mode must be left, right or both")
    actions = []
    if value("launch_manus_publisher") == "true":
        actions.append(Node(package="manus_ros2", executable="manus_data_publisher",
                            name="manus_data_publisher", parameters=[{"publish_rate_hz": 60.0}],
                            output="screen"))
    for side in (("left", "right") if mode == "both" else (mode,)):
        name = f"manus_revo3_retarget_{side}" if mode == "both" else "manus_revo3_retarget"
        actions.append(backend_process(value, side, node_name=name,
                                       profile=value(f"{side}_profile"),
                                       glove_id=value(f"{side}_glove_id")))
    return actions


def generate_launch_description():
    arguments = [
        *backend_arguments(),
        DeclareLaunchArgument("hand_mode", default_value="both", choices=["left", "right", "both"]),
        DeclareLaunchArgument("hand_type", default_value="", description="Legacy alias for hand_mode"),
        DeclareLaunchArgument("launch_manus_publisher", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("left_profile", default_value=""),
        DeclareLaunchArgument("right_profile", default_value=""),
        DeclareLaunchArgument("left_glove_id", default_value=""),
        DeclareLaunchArgument("right_glove_id", default_value=""),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=_launch)])
