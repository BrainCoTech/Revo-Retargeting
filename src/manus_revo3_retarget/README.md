# Manus Revo3 Retarget

This package is the portable Manus-to-Revo3 retarget layer for the
`revoarm_hardware/Revoarm_ws` workspace.

## Runtime Assumptions

- Start the target repository Revo3 hardware launch first, for example
  `revo3_driver/launch/revo3_system.launch.py` or
  `revo3_driver/launch/dual_revo3_system.launch.py`.
- The default launch path starts the C++ `retarget_node`.
- This package publishes `revo3_mit_controller_msgs/msg/Revo3MITCommand`.
- Retargeting runs directly from the Manus subscription callback. MIT commands
  are published by a separate timer at `mit_command_publish_hz` (default 200 Hz)
  with linear interpolation between adjacent retarget targets.
- Default command topics:
  - `/revo3_left/joint_forward_mit_controller/commands`
  - `/revo3_right/joint_forward_mit_controller/commands`
- The retarget backend is the current physical legacy chain only.

## Build

```bash
cd revoarm_hardware/Revoarm_ws
conda activate retarget_revo3
source /opt/ros/humble/setup.bash
python -m colcon build --packages-select manus_ros2 manus_revo3_retarget
source install/setup.bash
```

## Launch

```bash
conda activate retarget_revo3
source install/setup.bash
ros2 launch manus_revo3_retarget pipeline_launch.py hand_mode:=both
```

`hand_mode:=both` starts two independent retarget processes:
`manus_revo3_retarget_left` and `manus_revo3_retarget_right`. Each process only
initializes and computes retargeting for its own side.

Useful overrides:

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:=right \
  launch_manus_publisher:=true \
  mit_command_publish_hz:=200
```

Default parameters are split by function:

- `config/control.yaml`: topics, profile output, MIT publish rate, MIT kp/kd.
- `config/thumb_retarget.yaml`: thumb IK, thumb calibration, pinch behavior.
- `config/four_finger_retarget.yaml`: index/middle/ring/little flexion mapping.
- `config/spread_retarget.yaml`: spread/MPR mapping and spread guard.

Use `calibration_config` only for a final one-off override loaded after those
split configs:

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:=right \
  calibration_config:=/path/to/physical_joint_calibration.yaml
```

## Online Retarget Tuning

Start the retarget pipeline first, then open the tuning panel in another shell:

```bash
source install/setup.bash
ros2 run manus_revo3_retarget retarget_tuning_panel \
  --node /manus_revo3_retarget_left \
  --side left
```

When `hand_mode:=both` is used, the retarget nodes are
`/manus_revo3_retarget_left` and `/manus_revo3_retarget_right`. When a single
side is launched, the node is `/manus_revo3_retarget`.

The panel adjusts retarget parameters plus the global MIT `kp/kd` defaults.
Per-joint MIT gains, command topics, and `mit_command_publish_hz` stay in
`control.yaml`.

If the Revo3 system is not using `/revo3_<side>` namespaces, override the command
topics directly or use `control_config`:

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:=right \
  use_revo3_namespace:=false
```

## Launch And Record MCAP

To start the pipeline and record the default Manus/Revo3 topics into
`manus_revo3_retarget/log`:

```bash
cd src/brainco_capabilities/manus_revo3_retarget
./scripts/run_pipeline_record_mcap.sh
```

The script records MCAP bags with `ros2 bag record -s mcap`. If the MCAP storage
plugin is missing, install it first:

```bash
sudo apt install ros-humble-rosbag2-storage-mcap
```

By default the script activates the `manusglove` conda environment before
sourcing ROS and the workspace. Override it with `CONDA_ENV_NAME=<env>` or set
`CONDA_ENV_NAME=none` to use the current shell environment.

Default recorded topics:

- `/manus_glove_0`
- `/manus_glove_1`
- `/revo3_left/joint_forward_mit_controller/commands`
- `/revo3_right/joint_forward_mit_controller/commands`
- `/revo3_left/joint_forward_mit_controller/retarget_targets`
- `/revo3_right/joint_forward_mit_controller/retarget_targets`
- `/revo3_left/revo3_joint_state/joint_states_aligned`
- `/revo3_right/revo3_joint_state/joint_states_aligned`
- `/manus_revo3_retarget/latency_events`

The `retarget_targets` topics contain the post-retarget, pre-linear-interpolation
MIT target for each side. The high-rate `commands` topics are still the values
after the linear interpolation timer.

The script also starts `joint_state_aligner`. It republishes each Revo3
`sensor_msgs/msg/JointState` with `name`, `position`, `velocity`, and `effort`
ordered by the latest command `joint_names`, so recorded state arrays line up
with the MIT command arrays. The raw joint state topics are left untouched.

Useful overrides:

```bash
# Pass launch arguments through to pipeline_launch.py.
./scripts/run_pipeline_record_mcap.sh hand_mode:=right

# Put logs somewhere else or use a custom bag folder name.
LOG_ROOT=/tmp/revo3_logs BAG_NAME=test_right \
  ./scripts/run_pipeline_record_mcap.sh hand_mode:=right

# Record all topics instead of the default list.
RECORD_ALL=1 ./scripts/run_pipeline_record_mcap.sh

# Also record the raw, hardware-published joint state topics.
RECORD_RAW_JOINT_STATES=1 ./scripts/run_pipeline_record_mcap.sh

# Replace the default topic list.
BAG_TOPICS="/manus_glove_0 /revo3_right/joint_forward_mit_controller/commands" \
  ./scripts/run_pipeline_record_mcap.sh hand_mode:=right

# Disable joint-state alignment if you only want raw topics.
ENABLE_JOINT_STATE_ALIGNER=0 ./scripts/run_pipeline_record_mcap.sh
```

## Quintic Joint Test

To run a direct MIT command test without Manus input, use:

```bash
cd src/brainco_capabilities/manus_revo3_retarget
./scripts/run_quintic_test_record_mcap.sh
```

This publishes a quintic trajectory for every Revo3 hand joint:
`0 deg -> 40 deg -> 0 deg -> 40 deg -> 0 deg`. It starts
`joint_state_aligner` and records MCAP with:

- `/revo3_left/joint_forward_mit_controller/commands`
- `/revo3_right/joint_forward_mit_controller/commands`
- `/revo3_left/revo3_joint_state/joint_states_aligned`
- `/revo3_right/revo3_joint_state/joint_states_aligned`

Useful overrides:

```bash
./scripts/run_quintic_test_record_mcap.sh --hand-mode right
./scripts/run_quintic_test_record_mcap.sh --target-deg 40 --move-duration-s 2.0 --hold-s 1.0 --rate-hz 100
```

## Command/State Time-Series Viewer

To inspect all Revo3 MIT command values next to live joint feedback as rolling
time-series curves:

```bash
ros2 launch manus_revo3_retarget command_state_viewer.launch.py hand_mode:=both
```

For a single side:

```bash
ros2 launch manus_revo3_retarget command_state_viewer.launch.py hand_mode:=right
```

The viewer subscribes to:

- `/revo3_left/joint_forward_mit_controller/commands`
- `/revo3_left/revo3_joint_state/joint_states`
- `/revo3_right/joint_forward_mit_controller/commands`
- `/revo3_right/revo3_joint_state/joint_states`

Each side gets a tab, and each joint gets a rolling plot. Blue is command
position, green is state position, and a red plot background means
`abs(state_position - command_position)` exceeds `warn_error_rad`.

Useful overrides:

```bash
ros2 launch manus_revo3_retarget command_state_viewer.launch.py \
  hand_mode:=right \
  history_sec:=20.0 \
  update_ms:=50 \
  warn_error_rad:=0.05
```
