"""Shared process construction for native and bringup-compatible launches."""
from pathlib import Path


def backend_arguments():
    from launch.actions import DeclareLaunchArgument

    return [
        DeclareLaunchArgument("cache_dir", default_value=""),
        DeclareLaunchArgument("calibration_frames", default_value="120"),
        DeclareLaunchArgument("normal_sign", default_value="-1"),
        DeclareLaunchArgument("numeric_threads", default_value="1"),
        DeclareLaunchArgument("interpolate", default_value="false", choices=["true", "false"],
                              description="Enable 200 Hz interpolation and startup ramp"),
        DeclareLaunchArgument("max_velocity_rad_s", default_value=""),
        DeclareLaunchArgument("max_acceleration_rad_s2", default_value=""),
    ]


def backend_process(value, side, *, namespace="", profile="", glove_id="", node_name=None):
    from ament_index_python.packages import get_package_prefix
    from launch.actions import ExecuteProcess

    threads = int(value("numeric_threads"))
    if threads < 0:
        raise ValueError("numeric_threads must be an integer >= 0")
    # The installed entry point keeps the Python interpreter selected by colcon
    # (revo_teleop), even when the ros2 launch CLI uses /usr/bin/python3.
    environment = {"PYTHONNOUSERSITE": "1"}
    if threads:
        environment.update({key: str(threads) for key in (
            "OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
            "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "BLIS_NUM_THREADS")})
    executable = Path(get_package_prefix("manus_revo3_vector_retarget")) / "lib/manus_revo3_vector_retarget/retarget"
    command = [str(executable), "hardware",
               "--hand-side", side, "--namespace", namespace or f"/revo3_{side}",
               "--calibration-frames", value("calibration_frames"),
               "--normal-sign", value("normal_sign")]
    if node_name:
        command.extend(["--node-name", node_name])
    if value("interpolate") == "true":
        command.append("--interpolate")
    options = {"profile": profile, "glove_id": glove_id,
               **{key: value(key) for key in ("cache_dir", "max_velocity_rad_s", "max_acceleration_rad_s2")}}
    for key, option in options.items():
        if option:
            command.extend(["--" + key.replace("_", "-"), option])
    return ExecuteProcess(cmd=command, output="screen", additional_env=environment)
