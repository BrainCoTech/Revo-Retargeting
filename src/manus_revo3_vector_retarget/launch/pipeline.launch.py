"""Launch the vector backend using the current workspace's model and messages."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from retarget_v2.launch_support import backend_arguments, backend_process


def _launch(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    hand_side = value("hand_side")
    if hand_side not in ("left", "right"):
        raise ValueError("hand_side must be left or right")
    actions = [Node(
            package="manus_ros2", executable="manus_data_publisher",
            name="manus_data_publisher", parameters=[{"publish_rate_hz": 60.0}],
            output="screen")]
    actions.append(backend_process(value, hand_side, namespace=value("namespace"),
                                   profile=value("profile"), glove_id=value("glove_id")))
    return actions


def generate_launch_description():
    arguments = [
        *backend_arguments(),
        DeclareLaunchArgument("hand_side", default_value="right", choices=["left", "right"]),
        DeclareLaunchArgument("glove_id", default_value=""),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("profile", default_value=""),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=_launch)])
