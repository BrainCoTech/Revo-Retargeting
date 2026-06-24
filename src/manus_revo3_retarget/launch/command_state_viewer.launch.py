from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "hand_mode",
            default_value="both",
            description="Which side to watch: left, right, or both.",
        ),
        DeclareLaunchArgument(
            "use_revo3_namespace",
            default_value="true",
            description="Watch /revo3_<side>/... topics.",
        ),
        DeclareLaunchArgument(
            "command_topic_suffix",
            default_value="joint_forward_mit_controller/commands",
            description="Command topic suffix below each Revo3 namespace.",
        ),
        DeclareLaunchArgument(
            "update_ms",
            default_value="100",
            description="GUI refresh period in milliseconds.",
        ),
        DeclareLaunchArgument(
            "history_sec",
            default_value="10.0",
            description="Rolling time window for each joint plot.",
        ),
        DeclareLaunchArgument(
            "stale_sec",
            default_value="1.0",
            description="Mark rows stale when command or state data is older than this.",
        ),
        DeclareLaunchArgument(
            "warn_error_rad",
            default_value="0.10",
            description="Highlight rows when abs(state - command) reaches this value.",
        ),
        Node(
            package="manus_revo3_retarget",
            executable="command_state_viewer",
            name="revo3_command_state_viewer",
            arguments=[
                "--hand-mode", LaunchConfiguration("hand_mode"),
                "--command-topic-suffix", LaunchConfiguration("command_topic_suffix"),
                "--update-ms", LaunchConfiguration("update_ms"),
                "--history-sec", LaunchConfiguration("history_sec"),
                "--stale-sec", LaunchConfiguration("stale_sec"),
                "--warn-error-rad", LaunchConfiguration("warn_error_rad"),
                "--use-revo3-namespace", LaunchConfiguration("use_revo3_namespace"),
            ],
            output="screen",
        ),
    ])
