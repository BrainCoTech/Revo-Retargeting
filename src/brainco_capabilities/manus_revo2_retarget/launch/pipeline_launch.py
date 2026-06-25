import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _create_nodes(context, *args, **kwargs):
    del args, kwargs

    hand_mode = LaunchConfiguration("hand_mode").perform(context).strip().lower()
    if hand_mode not in ("left", "right", "both"):
        raise ValueError("hand_mode must be one of: left, right, both")
    launch_manus_publisher = _as_bool(LaunchConfiguration("launch_manus_publisher").perform(context))
    use_split_controller = _as_bool(LaunchConfiguration("use_split_controller").perform(context))
    controller_backend = LaunchConfiguration("controller_backend").perform(context).strip().lower()
    use_ros2_control_pid = controller_backend in ("ros2_control", "ros2_control_pid", "pid")
    control_config = LaunchConfiguration("control_config").perform(context)
    retarget_config = LaunchConfiguration("retarget_config").perform(context)
    teleop_controller_config = LaunchConfiguration("teleop_controller_config").perform(context)
    if teleop_controller_config and not os.path.isabs(teleop_controller_config):
        teleop_controller_config = os.path.join(
            get_package_share_directory("manus_revo2_retarget"),
            "config",
            teleop_controller_config,
        )

    nodes = []
    if launch_manus_publisher:
        nodes.append(
            Node(
                package="manus_ros2",
                executable="manus_data_publisher",
                name="manus_data_publisher",
                output="screen",
            )
        )

    target_only = use_split_controller or use_ros2_control_pid
    split_retarget_processes = target_only and hand_mode == "both"
    retarget_sides = ("left", "right") if split_retarget_processes else (hand_mode,)
    for side in retarget_sides:
        retarget_args = [
            "--hand-mode",
            side,
            "--control-config",
            control_config,
        ]
        if target_only:
            retarget_args.extend([
                "--target-only",
                "--left-target-joint-state-topic",
                "/revo2_left/revo2_pid_controller/target_joint_states",
                "--right-target-joint-state-topic",
                "/revo2_right/revo2_pid_controller/target_joint_states",
            ])
        if retarget_config:
            retarget_args.extend(["--config-file", retarget_config])
        nodes.append(
            Node(
                package="manus_revo2_retarget",
                executable="manus_revo2_retarget_node",
                name=(
                    f"manus_revo2_retarget_{side}"
                    if split_retarget_processes
                    else "manus_revo2_retarget"
                ),
                arguments=retarget_args,
                additional_env={"PYTHONNOUSERSITE": "1"},
                output="screen",
            )
        )

    if use_split_controller and not use_ros2_control_pid:
        nodes.append(
            Node(
                package="manus_revo2_retarget",
                executable="revo2_teleop_controller",
                name="revo2_teleop_controller",
                parameters=[
                    teleop_controller_config,
                    {"hand_mode": hand_mode},
                ],
                output="screen",
            )
        )
    return nodes


def generate_launch_description():
    package_share = get_package_share_directory("manus_revo2_retarget")

    return LaunchDescription([
        DeclareLaunchArgument(
            "hand_mode",
            default_value="right",
            description="Which side to retarget: left, right, or both.",
        ),
        DeclareLaunchArgument(
            "launch_manus_publisher",
            default_value="true",
            description="Start manus_ros2 manus_data_publisher.",
        ),
        DeclareLaunchArgument(
            "use_split_controller",
            default_value="true",
            description=(
                "Run split retarget + revo2_teleop_controller pipeline. "
                "Set false to use the legacy monolithic retarget node."
            ),
        ),
        DeclareLaunchArgument(
            "controller_backend",
            default_value="python_topic",
            description=(
                "Controller backend: python_topic uses revo2_teleop_controller + "
                "joint_forward_vel_controller; ros2_control uses revo2_pid_controller."
            ),
            choices=["python_topic", "topic_velocity", "ros2_control", "ros2_control_pid", "pid"],
        ),
        DeclareLaunchArgument(
            "control_config",
            default_value="retarget.yaml",
            description="MANUS Revo2 retarget runtime YAML.",
        ),
        DeclareLaunchArgument(
            "teleop_controller_config",
            default_value=PathJoinSubstitution([
                package_share,
                "config",
                "teleop_controller.yaml",
            ]),
            description="Revo2 split teleop controller parameter YAML.",
        ),
        DeclareLaunchArgument(
            "retarget_config",
            default_value="",
            description="Optional retargeting algorithm YAML. Empty uses the package default brainco.yml.",
        ),
        OpaqueFunction(function=_create_nodes),
    ])
