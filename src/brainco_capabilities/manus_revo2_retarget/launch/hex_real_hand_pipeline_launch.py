import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _create_actions(context: LaunchContext, *args, **kwargs):
    del args, kwargs

    launch_hex_bridge = _as_bool(LaunchConfiguration("launch_hex_bridge").perform(context))
    launch_revo2_pipeline = _as_bool(LaunchConfiguration("launch_revo2_pipeline").perform(context))
    hex_server_host = LaunchConfiguration("hex_server_host").perform(context).strip()
    angles_port = int(LaunchConfiguration("angles_port").perform(context))
    positions_port = int(LaunchConfiguration("positions_port").perform(context))
    connect_period_sec = float(LaunchConfiguration("connect_period_sec").perform(context))
    position_scale = float(LaunchConfiguration("position_scale").perform(context))
    angle_scale = float(LaunchConfiguration("angle_scale").perform(context))

    actions = [
        LogInfo(
            msg=(
                "Starting Hex glove Revo2 pipeline: "
                f"hex_server_host={hex_server_host}, "
                f"angles_port={angles_port}, positions_port={positions_port}"
            )
        )
    ]

    if launch_hex_bridge:
        actions.append(
            Node(
                package="hex_glove_driver",
                executable="hex_glove_udp_node",
                name="hex_glove_udp_node",
                parameters=[
                    {
                        "server_host": hex_server_host,
                        "angles_port": angles_port,
                        "positions_port": positions_port,
                        "connect_period_sec": connect_period_sec,
                        "raw_angles_topic": LaunchConfiguration("raw_angles_topic").perform(context),
                        "raw_positions_topic": LaunchConfiguration("raw_positions_topic").perform(context),
                        "left_glove_topic": LaunchConfiguration("left_glove_topic").perform(context),
                        "right_glove_topic": LaunchConfiguration("right_glove_topic").perform(context),
                        "publish_adapter_glove": True,
                        "publish_manus_glove": True,
                        "position_scale": position_scale,
                        "angle_scale": angle_scale,
                        "revo2_coordinate_transform": _as_bool(
                            LaunchConfiguration("revo2_coordinate_transform").perform(context)
                        ),
                        "zero_angles_on_first_frame": _as_bool(
                            LaunchConfiguration("zero_angles_on_first_frame").perform(context)
                        ),
                        "left_stretch_sign": float(LaunchConfiguration("left_stretch_sign").perform(context)),
                        "right_stretch_sign": float(LaunchConfiguration("right_stretch_sign").perform(context)),
                        "left_spread_sign": float(LaunchConfiguration("left_spread_sign").perform(context)),
                        "right_spread_sign": float(LaunchConfiguration("right_spread_sign").perform(context)),
                    }
                ],
                output="screen",
            )
        )

    if launch_revo2_pipeline:
        retarget_share = get_package_share_directory("manus_revo2_retarget")
        real_hand_launch = os.path.join(retarget_share, "launch", "real_hand_pipeline_launch.py")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(real_hand_launch),
                launch_arguments={
                    "hand_mode": LaunchConfiguration("hand_mode"),
                    "update_rate": LaunchConfiguration("update_rate"),
                    "switch_delay": LaunchConfiguration("switch_delay"),
                    "retarget_delay": LaunchConfiguration("retarget_delay"),
                    "plot_delay": LaunchConfiguration("plot_delay"),
                    "launch_driver": LaunchConfiguration("launch_driver"),
                    "switch_controllers": LaunchConfiguration("switch_controllers"),
                    "launch_retarget": LaunchConfiguration("launch_retarget"),
                    "launch_plot": LaunchConfiguration("launch_plot"),
                    "launch_manus_publisher": "false",
                    "use_split_controller": LaunchConfiguration("use_split_controller"),
                    "controller_backend": LaunchConfiguration("controller_backend"),
                    "plot_window": LaunchConfiguration("plot_window"),
                    "plot_joints": LaunchConfiguration("plot_joints"),
                    "control_config": LaunchConfiguration("control_config"),
                    "teleop_controller_config": LaunchConfiguration("teleop_controller_config"),
                    "retarget_config": LaunchConfiguration("retarget_config"),
                    "protocol": LaunchConfiguration("protocol"),
                    "left_protocol_config_file": LaunchConfiguration("left_protocol_config_file"),
                    "right_protocol_config_file": LaunchConfiguration("right_protocol_config_file"),
                    "initial_positions_file": LaunchConfiguration("initial_positions_file"),
                    "controllers_file": LaunchConfiguration("controllers_file"),
                    "use_namespace": LaunchConfiguration("use_namespace"),
                    "if_sim": LaunchConfiguration("if_sim"),
                    "launch_rsp": LaunchConfiguration("launch_rsp"),
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "hex_server_host",
            default_value="127.0.0.1",
            description="Hex hand-controller host. Use the Windows IPv4 address when the controller runs on Windows.",
        ),
        DeclareLaunchArgument(
            "angles_port",
            default_value="9011",
            description="Hex UDP angle JSON port.",
        ),
        DeclareLaunchArgument(
            "positions_port",
            default_value="9013",
            description="Hex UDP position JSON port.",
        ),
        DeclareLaunchArgument(
            "connect_period_sec",
            default_value="1.0",
            description="Seconds between CONNECT packets sent to the Hex hand controller.",
        ),
        DeclareLaunchArgument(
            "left_glove_topic",
            default_value="/manus_glove_0",
            description="Adapted ManusGlove topic for the left Hex glove.",
        ),
        DeclareLaunchArgument(
            "right_glove_topic",
            default_value="/manus_glove_1",
            description="Adapted ManusGlove topic for the right Hex glove.",
        ),
        DeclareLaunchArgument(
            "raw_angles_topic",
            default_value="/hex_glove/raw_angles",
            description="Raw Hex angle JSON debug topic.",
        ),
        DeclareLaunchArgument(
            "raw_positions_topic",
            default_value="/hex_glove/raw_positions",
            description="Raw Hex position JSON debug topic.",
        ),
        DeclareLaunchArgument(
            "position_scale",
            default_value="0.01",
            description="Scale from Hex position units to ROS meters.",
        ),
        DeclareLaunchArgument(
            "angle_scale",
            default_value="1.0",
            description="Scale applied to Hex ergonomics angle values.",
        ),
        DeclareLaunchArgument(
            "revo2_coordinate_transform",
            default_value="true",
            description="Apply the coordinate transform expected by the Revo2 retargeter.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "zero_angles_on_first_frame",
            default_value="false",
            description="Use the first Hex angle frame as ergonomics zero.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "left_stretch_sign",
            default_value="1.0",
            description="Left-hand stretch ergonomics sign.",
        ),
        DeclareLaunchArgument(
            "right_stretch_sign",
            default_value="1.0",
            description="Right-hand stretch ergonomics sign.",
        ),
        DeclareLaunchArgument(
            "left_spread_sign",
            default_value="1.0",
            description="Left-hand spread ergonomics sign.",
        ),
        DeclareLaunchArgument(
            "right_spread_sign",
            default_value="1.0",
            description="Right-hand spread ergonomics sign.",
        ),
        DeclareLaunchArgument(
            "launch_hex_bridge",
            default_value="true",
            description="Start hex_glove_udp_node.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "launch_revo2_pipeline",
            default_value="true",
            description="Include the Revo2 real-hand pipeline.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "hand_mode",
            default_value="right",
            description="Which Revo2 side to run: left, right, or both.",
            choices=["left", "right", "both"],
        ),
        DeclareLaunchArgument(
            "update_rate",
            default_value="20",
            description="Revo2 controller_manager hardware read/write rate.",
        ),
        DeclareLaunchArgument(
            "switch_delay",
            default_value="16.0",
            description="Seconds to wait before switching to revo2_pid_controller.",
        ),
        DeclareLaunchArgument(
            "retarget_delay",
            default_value="18.0",
            description="Seconds to wait before starting Hex-driven Revo2 retarget.",
        ),
        DeclareLaunchArgument(
            "plot_delay",
            default_value="19.0",
            description="Seconds to wait before starting retarget plot monitor.",
        ),
        DeclareLaunchArgument(
            "launch_driver",
            default_value="true",
            description="Start the Revo2 driver from this launch.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "switch_controllers",
            default_value="true",
            description="Switch Revo2 from position controller to revo2_pid_controller.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "launch_retarget",
            default_value="true",
            description="Start the target-only retarget pipeline.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "launch_plot",
            default_value="false",
            description="Start the target/actual/error plot monitor.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_split_controller",
            default_value="true",
            description="Keep retarget in target-only mode for ros2_control PID.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "controller_backend",
            default_value="ros2_control",
            description="Controller backend for Revo2. Keep ros2_control for revo2_pid_controller.",
            choices=["python_topic", "topic_velocity", "ros2_control", "ros2_control_pid", "pid"],
        ),
        DeclareLaunchArgument(
            "plot_window",
            default_value="10.0",
            description="Retarget plot visible time window in seconds.",
        ),
        DeclareLaunchArgument(
            "plot_joints",
            default_value="",
            description="Optional comma-separated joint list for the plot monitor.",
        ),
        DeclareLaunchArgument(
            "control_config",
            default_value="retarget.yaml",
            description="Hex/Revo2 retarget runtime YAML.",
        ),
        DeclareLaunchArgument(
            "teleop_controller_config",
            default_value="teleop_controller.yaml",
            description="Legacy Python topic controller YAML.",
        ),
        DeclareLaunchArgument(
            "retarget_config",
            default_value="",
            description="Optional retargeting algorithm YAML. Empty uses package default.",
        ),
        DeclareLaunchArgument(
            "protocol",
            default_value="modbus",
            description="Revo2 hardware protocol.",
            choices=["modbus", "canfd"],
        ),
        DeclareLaunchArgument(
            "left_protocol_config_file",
            default_value="",
            description="Optional left-hand protocol YAML override.",
        ),
        DeclareLaunchArgument(
            "right_protocol_config_file",
            default_value="",
            description="Optional right-hand protocol YAML override.",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="",
            description="Optional initial positions YAML override.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="",
            description="Optional controller YAML template override.",
        ),
        DeclareLaunchArgument(
            "use_namespace",
            default_value="true",
            description="Use revo2_left/revo2_right namespaces.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "if_sim",
            default_value="false",
            description="Use mock_components/GenericSystem instead of real hardware.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "launch_rsp",
            default_value="true",
            description="Start robot_state_publisher for the Revo2 hand.",
            choices=["true", "false"],
        ),
        OpaqueFunction(function=_create_actions),
    ])
