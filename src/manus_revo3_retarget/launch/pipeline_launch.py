from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import math
import yaml


NUMERIC_THREAD_ENV_KEYS = (
    "OPENBLAS_NUM_THREADS",
    "OMP_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "VECLIB_MAXIMUM_THREADS",
    "BLIS_NUM_THREADS",
)


def _numeric_thread_env(value):
    text = str(value).strip()
    if not text:
        return {}
    try:
        threads = int(text)
    except ValueError as exc:
        raise ValueError("numeric_threads must be an integer >= 0") from exc
    if threads < 0:
        raise ValueError("numeric_threads must be an integer >= 0")
    if threads == 0:
        return {}
    thread_value = str(threads)
    return {key: thread_value for key in NUMERIC_THREAD_ENV_KEYS}


def _load_ros_parameters(path):
    path = str(path).strip()
    if not path:
        return {}

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Parameter file must contain a mapping: {path}")

    merged = {}
    for key in ("/**", "manus_revo3_retarget", "/manus_revo3_retarget"):
        node_params = data.get(key)
        if not isinstance(node_params, dict):
            continue
        params = node_params.get("ros__parameters")
        if isinstance(params, dict):
            merged.update(params)

    direct_params = data.get("ros__parameters")
    if isinstance(direct_params, dict):
        merged.update(direct_params)

    if merged:
        return merged
    return dict(data)


def _create_runtime_nodes(context, *args, **kwargs):
    del args, kwargs

    hand_mode = LaunchConfiguration("hand_mode").perform(context).strip().lower()
    hand_type = LaunchConfiguration("hand_type").perform(context).strip().lower()
    if hand_type:
        hand_mode = hand_type
    if hand_mode not in ("left", "right", "both"):
        raise ValueError("hand_mode must be one of: left, right, both")
    manus_publish_rate_hz = float(LaunchConfiguration("manus_publish_rate_hz").perform(context))
    if not math.isfinite(manus_publish_rate_hz) or manus_publish_rate_hz <= 0.0:
        raise ValueError("manus_publish_rate_hz must be a finite positive value")
    launch_manus_publisher = LaunchConfiguration("launch_manus_publisher").perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    use_revo3_namespace = LaunchConfiguration("use_revo3_namespace").perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    command_topic_suffix = LaunchConfiguration("command_topic_suffix").perform(context)
    retarget_target_topic_suffix = LaunchConfiguration("retarget_target_topic_suffix").perform(context)
    mit_command_publish_hz = float(LaunchConfiguration("mit_command_publish_hz").perform(context))
    if not math.isfinite(mit_command_publish_hz) or mit_command_publish_hz <= 0.0:
        raise ValueError("mit_command_publish_hz must be a finite positive value")
    numeric_env = _numeric_thread_env(LaunchConfiguration("numeric_threads").perform(context))
    python_env = {"PYTHONNOUSERSITE": "1", **numeric_env}
    retarget_config = LaunchConfiguration("retarget_config").perform(context)
    calibration_config = LaunchConfiguration("calibration_config").perform(context)
    left_calibration_config = LaunchConfiguration("left_calibration_config").perform(context)
    right_calibration_config = LaunchConfiguration("right_calibration_config").perform(context)

    overrides = {
        "use_revo3_namespace": use_revo3_namespace,
        "command_topic_suffix": command_topic_suffix,
        "retarget_target_topic_suffix": retarget_target_topic_suffix,
        "mit_command_publish_hz": mit_command_publish_hz,
    }

    parameter_dicts = [
        _load_ros_parameters(LaunchConfiguration("control_config").perform(context)),
        _load_ros_parameters(LaunchConfiguration("thumb_retarget_config").perform(context)),
        _load_ros_parameters(LaunchConfiguration("four_finger_retarget_config").perform(context)),
        _load_ros_parameters(LaunchConfiguration("spread_retarget_config").perform(context)),
    ]
    if retarget_config:
        parameter_dicts.append(_load_ros_parameters(retarget_config))
    common_parameter_dicts = list(parameter_dicts)
    if calibration_config:
        common_parameter_dicts.append(_load_ros_parameters(calibration_config))

    nodes = []
    if launch_manus_publisher:
        nodes.append(
            Node(
                package="manus_ros2",
                executable="manus_data_publisher",
                name="manus_data_publisher",
                parameters=[{"publish_rate_hz": manus_publish_rate_hz}],
                output="screen",
            )
        )

    retarget_sides = ("left", "right") if hand_mode == "both" else (hand_mode,)
    for side in retarget_sides:
        side_parameter_dicts = list(common_parameter_dicts)
        if not calibration_config:
            side_calibration_config = left_calibration_config if side == "left" else right_calibration_config
            if side_calibration_config:
                side_parameter_dicts.append(_load_ros_parameters(side_calibration_config))
        nodes.append(
            Node(
                package="manus_revo3_retarget",
                executable="retarget_node",
                name=(
                    f"manus_revo3_retarget_{side}"
                    if hand_mode == "both"
                    else "manus_revo3_retarget"
                ),
                parameters=[
                    *side_parameter_dicts,
                    {
                        **overrides,
                        "hand_mode": side,
                    },
                ],
                additional_env=python_env,
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    package_share = get_package_share_directory("manus_revo3_retarget")

    return LaunchDescription([
        DeclareLaunchArgument(
            "hand_mode",
            default_value="both",
            description="Which side to launch: left, right, or both.",
        ),
        DeclareLaunchArgument(
            "hand_type",
            default_value="",
            description="Backward-compatible alias for hand_mode.",
        ),
        DeclareLaunchArgument(
            "launch_manus_publisher",
            default_value="true",
            description="Start manus_ros2 manus_data_publisher.",
        ),
        DeclareLaunchArgument(
            "manus_publish_rate_hz",
            default_value="60.0",
            description="MANUS glove ROS publish frequency in Hz.",
        ),
        DeclareLaunchArgument(
            "use_revo3_namespace",
            default_value="true",
            description="Publish commands to /revo3_<side>/... topics used by revo3_driver.",
        ),
        DeclareLaunchArgument(
            "command_topic_suffix",
            default_value="joint_forward_mit_controller/commands",
            description="Command topic suffix under each Revo3 namespace.",
        ),
        DeclareLaunchArgument(
            "retarget_target_topic_suffix",
            default_value="joint_forward_mit_controller/retarget_targets",
            description="Pre-interpolation retarget target topic suffix under each Revo3 namespace.",
        ),
        DeclareLaunchArgument(
            "mit_command_publish_hz",
            default_value="200.0",
            description="High-rate Revo3 MIT command publish frequency in Hz.",
        ),
        DeclareLaunchArgument(
            "numeric_threads",
            default_value="1",
            description="Set numeric library thread env vars for compatibility. Use 0 to leave existing env unchanged.",
        ),
        DeclareLaunchArgument(
            "control_config",
            default_value=PathJoinSubstitution([package_share, "config", "control.yaml"]),
            description="Control, topic, publish-rate, and MIT gain parameter YAML.",
        ),
        DeclareLaunchArgument(
            "thumb_retarget_config",
            default_value=PathJoinSubstitution([package_share, "config", "thumb_retarget.yaml"]),
            description="Thumb IK and thumb calibration parameter YAML.",
        ),
        DeclareLaunchArgument(
            "four_finger_retarget_config",
            default_value=PathJoinSubstitution([package_share, "config", "four_finger_retarget.yaml"]),
            description="Index/middle/ring/little flexion retarget parameter YAML.",
        ),
        DeclareLaunchArgument(
            "spread_retarget_config",
            default_value=PathJoinSubstitution([package_share, "config", "spread_retarget.yaml"]),
            description="Spread/MPR retarget parameter YAML.",
        ),
        DeclareLaunchArgument(
            "retarget_config",
            default_value="",
            description="Optional compatibility override YAML loaded after split retarget configs.",
        ),
        DeclareLaunchArgument(
            "calibration_config",
            default_value="",
            description="Optional final calibration override YAML loaded after all retarget configs for every launched side.",
        ),
        DeclareLaunchArgument(
            "left_calibration_config",
            default_value="",
            description="Optional left-hand final calibration override YAML.",
        ),
        DeclareLaunchArgument(
            "right_calibration_config",
            default_value="",
            description="Optional right-hand final calibration override YAML.",
        ),
        OpaqueFunction(function=_create_runtime_nodes),
    ])
