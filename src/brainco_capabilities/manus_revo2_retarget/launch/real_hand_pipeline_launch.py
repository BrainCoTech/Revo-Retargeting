import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _selected_sides(hand_mode):
    hand_mode = hand_mode.lower()
    if hand_mode == "both":
        return ["left", "right"]
    if hand_mode in ("left", "right"):
        return [hand_mode]
    raise RuntimeError(f"Unsupported hand_mode: {hand_mode}")


def _controller_manager_name(side, use_namespace):
    if use_namespace:
        return f"/revo2_{side}/controller_manager"
    return "/controller_manager"


def _switch_controller_command(controller_manager, target_controller):
    script = f"""
set -u
cm='{controller_manager}'
target='{target_controller}'
export NO_COLOR=1
export RCUTILS_COLORIZED_OUTPUT=0

controller_active() {{
  local name="$1"
  timeout 5 ros2 control list_controllers -c "$cm" 2>/dev/null | \\
    awk -v name="$name" 'index($0, name) && $0 ~ /active[[:space:]]/ {{found=1}} END {{exit found ? 0 : 1}}'
}}

controller_exists() {{
  local name="$1"
  timeout 5 ros2 control list_controllers -c "$cm" 2>/dev/null | \\
    awk -v name="$name" 'index($0, name) {{found=1}} END {{exit found ? 0 : 1}}'
}}

for attempt in $(seq 1 30); do
  if controller_active "$target"; then
    echo "[real_hand_pipeline] $cm $target is already active."
    exit 0
  fi

  if ! controller_exists "$target"; then
    echo "[real_hand_pipeline] Loading $target on $cm..."
    timeout 8 ros2 control load_controller -c "$cm" --set-state inactive "$target" || true
    sleep 1
  fi

  stop_args=()
  if controller_active joint_forward_pos_controller && [ "$target" != "joint_forward_pos_controller" ]; then
    stop_args+=(--deactivate joint_forward_pos_controller)
  fi
  if controller_active joint_forward_vel_controller && [ "$target" != "joint_forward_vel_controller" ]; then
    stop_args+=(--deactivate joint_forward_vel_controller)
  fi
  if controller_active revo2_pid_controller && [ "$target" != "revo2_pid_controller" ]; then
    stop_args+=(--deactivate revo2_pid_controller)
  fi

  echo "[real_hand_pipeline] Switching $cm to $target (attempt $attempt)..."
  if timeout 8 ros2 control switch_controllers \\
      -c "$cm" \\
      "${{stop_args[@]}}" \\
      --activate "$target" \\
      --activate-asap \\
      --strict; then
    if controller_active "$target"; then
      echo "[real_hand_pipeline] $cm $target is active."
      exit 0
    fi
  fi
  sleep 1
done

echo "[real_hand_pipeline] Failed to activate $target for $cm" >&2
timeout 2 ros2 control list_controllers -c "$cm" || true
exit 1
"""
    return ["bash", "-lc", script]


def _create_actions(context: LaunchContext, *args, **kwargs):
    del args, kwargs

    hand_mode = LaunchConfiguration("hand_mode").perform(context).lower()
    sides = _selected_sides(hand_mode)
    update_rate = LaunchConfiguration("update_rate").perform(context)
    protocol = LaunchConfiguration("protocol").perform(context)
    use_namespace = _as_bool(LaunchConfiguration("use_namespace").perform(context))
    launch_driver = _as_bool(LaunchConfiguration("launch_driver").perform(context))
    switch_controllers = _as_bool(LaunchConfiguration("switch_controllers").perform(context))
    controller_backend = LaunchConfiguration("controller_backend").perform(context).strip().lower()
    use_ros2_control_pid = controller_backend in ("ros2_control", "ros2_control_pid", "pid")
    launch_retarget = _as_bool(LaunchConfiguration("launch_retarget").perform(context))
    launch_plot = _as_bool(LaunchConfiguration("launch_plot").perform(context))
    switch_delay = float(LaunchConfiguration("switch_delay").perform(context))
    retarget_delay = float(LaunchConfiguration("retarget_delay").perform(context))
    plot_delay = float(LaunchConfiguration("plot_delay").perform(context))
    plot_window = LaunchConfiguration("plot_window").perform(context)
    plot_joints = LaunchConfiguration("plot_joints").perform(context).strip()

    if hand_mode == "both" and not use_namespace:
        raise RuntimeError("hand_mode:=both requires use_namespace:=true")

    driver_share = get_package_share_directory("revo2_driver")
    retarget_share = get_package_share_directory("manus_revo2_retarget")
    driver_launch = os.path.join(driver_share, "launch", "revo2_system.launch.py")
    retarget_launch = os.path.join(retarget_share, "launch", "pipeline_launch.py")
    plot_script = os.path.join(retarget_share, "tools", "revo2_retarget_plot.py")

    actions = [
        LogInfo(
            msg=(
                "Starting MANUS Revo2 real-hand pipeline: "
                f"hand_mode={hand_mode}, update_rate={update_rate} Hz"
            )
        )
    ]

    if launch_driver:
        for side in sides:
            protocol_config = LaunchConfiguration(f"{side}_protocol_config_file").perform(context)
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(driver_launch),
                    launch_arguments={
                        "hand_side": side,
                        "protocol": protocol,
                        "protocol_config_file": protocol_config,
                        "initial_positions_file": LaunchConfiguration("initial_positions_file"),
                        "controllers_file": LaunchConfiguration("controllers_file"),
                        "update_rate": update_rate,
                        "use_namespace": str(use_namespace).lower(),
                        "if_sim": LaunchConfiguration("if_sim"),
                        "launch_rsp": LaunchConfiguration("launch_rsp"),
                    }.items(),
                )
            )

    if switch_controllers:
        switch_actions = []
        target_controller = "revo2_pid_controller" if use_ros2_control_pid else "joint_forward_vel_controller"
        for side in sides:
            controller_manager = _controller_manager_name(side, use_namespace)
            switch_actions.append(
                ExecuteProcess(
                    cmd=_switch_controller_command(controller_manager, target_controller),
                    output="screen",
                )
            )
        actions.append(
            TimerAction(
                period=switch_delay,
                actions=[
                    LogInfo(msg=f"Switching Revo2 controller(s) to {target_controller}."),
                    *switch_actions,
                ],
            )
        )

    if launch_retarget:
        actions.append(
            TimerAction(
                period=retarget_delay,
                actions=[
                    LogInfo(msg="Starting MANUS publisher and Revo2 retarget node."),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(retarget_launch),
                        launch_arguments={
                            "hand_mode": hand_mode,
                            "launch_manus_publisher": LaunchConfiguration("launch_manus_publisher"),
                            "use_split_controller": LaunchConfiguration("use_split_controller"),
                            "controller_backend": LaunchConfiguration("controller_backend"),
                            "control_config": LaunchConfiguration("control_config"),
                            "teleop_controller_config": LaunchConfiguration("teleop_controller_config"),
                            "retarget_config": LaunchConfiguration("retarget_config"),
                        }.items(),
                    ),
                ],
            )
        )

    if launch_plot:
        plot_actions = []
        for side in sides:
            plot_cmd = [
                "python3",
                plot_script,
                "--side",
                side,
                "--window",
                plot_window,
            ]
            if plot_joints:
                plot_cmd.extend(["--joints", plot_joints])
            plot_actions.append(
                ExecuteProcess(
                    cmd=plot_cmd,
                    output="screen",
                )
            )
        actions.append(
            TimerAction(
                period=plot_delay,
                actions=[
                    LogInfo(msg="Starting Revo2 retarget plot monitor."),
                    *plot_actions,
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "hand_mode",
            default_value="right",
            description="Which side to run: left, right, or both.",
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
            description="Seconds to wait before switching to velocity controller.",
        ),
        DeclareLaunchArgument(
            "retarget_delay",
            default_value="18.0",
            description="Seconds to wait before starting MANUS retarget.",
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
        ),
        DeclareLaunchArgument(
            "switch_controllers",
            default_value="true",
            description="Switch Revo2 from position controller to velocity controller.",
        ),
        DeclareLaunchArgument(
            "launch_retarget",
            default_value="true",
            description="Start MANUS publisher and retarget pipeline.",
        ),
        DeclareLaunchArgument(
            "launch_plot",
            default_value="true",
            description="Start the target/actual/error plot monitor.",
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
            description="MANUS Revo2 retarget runtime YAML.",
        ),
        DeclareLaunchArgument(
            "teleop_controller_config",
            default_value="teleop_controller.yaml",
            description="Revo2 split teleop controller YAML.",
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
        ),
        DeclareLaunchArgument(
            "launch_rsp",
            default_value="true",
            description="Start robot_state_publisher for the Revo2 hand.",
        ),
        OpaqueFunction(function=_create_actions),
    ])
