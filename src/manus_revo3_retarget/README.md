# Manus Revo3 Retarget

This package is the Manus-to-Revo3 retarget layer for the
`Revo-Retargeting` workspace. The runtime retarget node is C++ and
uses Pinocchio with the URDFs from `revo3_description`.

## Runtime Assumptions

- Start the target repository Revo3 hardware launch first, for example
  `revo3_driver/launch/revo3_system.launch.py` or
  `revo3_driver/launch/dual_revo3_system.launch.py`.
- This package publishes `revo3_mit_controller_msgs/msg/Revo3MITCommand`.
- Retargeting runs directly from the Manus subscription callback. MIT commands
  are published by a separate timer at `mit_command_publish_hz` (default 200 Hz)
  using the latest retarget target.
- Default command topics:
  - `/revo3_left/joint_forward_mit_controller/commands`
  - `/revo3_right/joint_forward_mit_controller/commands`
- The thumb IK backend is Pinocchio and uses the shared `revo3_description` URDF.

## Build

```bash
cd ~/revo_retarget3/Revo-Retargeting
source /opt/ros/humble/setup.bash
colcon build --packages-select manus_ros2 revo3_driver manus_revo3_retarget
source install/setup.bash
```

## Launch

```bash
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

Default parameters are split by function. By default, no side-specific tuning
override is loaded, so these four YAML files are the effective startup config:

- `config/control.yaml`: topics, MIT publish rate, global MIT kp/kd.
- `config/thumb_retarget.yaml`: thumb IK and thumb calibration.
- `config/four_finger_retarget.yaml`: index/middle/ring/little flexion mapping.
- `config/spread_retarget.yaml`: spread/MPR mapping.

Use `calibration_config`, `left_calibration_config`, or `right_calibration_config`
only for a final one-off override loaded after those split configs:

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:=right \
  calibration_config:=/path/to/physical_joint_calibration.yaml
```

If the Revo3 system is not using `/revo3_<side>` namespaces, override the command
topics directly or use `control_config`:

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:=right \
  use_revo3_namespace:=false
```

## Staged Online Calibration

Start the hand driver once, then launch the retarget nodes and panel:

```bash
source install/setup.bash
ros2 launch manus_revo3_retarget pipeline_launch.py hand_mode:=both launch_tuning_panel:=true
```

Closing a panel started with `launch_tuning_panel:=true` also stops the pipeline
and its MANUS publisher. Separately launched hardware drivers remain running.
The panel first disables output and sends zero `kp`, `kd`, velocity and effort
for all 21 joints, releasing active motor holding torque. Closing the window,
Ctrl+C and SIGTERM use this same release path. If a previously connected node
does not confirm the release request, the panel shows an error and stays open
for retry. Retarget nodes also send a release before normal ROS shutdown.

**Rebuild `revo3_driver` as well as this package, and restart the hand driver.**
Older driver builds replace explicit zero gains with nonzero defaults and will
continue holding the fingers. Release does not command a return to zero pose;
passive mechanical friction remains. DDS acknowledgement confirms message
delivery, not motor feedback. Forced termination (`kill -9`), communication loss,
or shutting down the hardware driver at the same time cannot guarantee release.

This launch includes the MANUS publisher. If it is already running, add
`launch_manus_publisher:=false`. If the retarget nodes are already running,
open only the panel in a desktop terminal:

```bash
ros2 run manus_revo3_retarget retarget_tuning_panel --side right --operator alice
```

The panel inherits `ROS_DOMAIN_ID` from the shell; use the same domain as the
hand drivers and MANUS publisher. Do not launch duplicate publishers or drivers
against the same USB device. Tk requires a graphical display; an ordinary SSH
session without X forwarding cannot display the panel.

Direct node/pipeline startup now defaults to **output disabled**. In the panel,
choose the hand and enable its output, then work through these stages:

1. **Spread / MPR**: only the four non-thumb MPR targets are active.
2. **Four-finger flexion**: only the four fingers' MCP/PIP/DIP targets are active.
3. **Thumb / pinch**: all targets are active, retaining the previous adjustments.

Inactive targets in the first two stages are commanded to exactly zero, even
if they have nonzero output offsets. Hand/stage switches and re-enabling clear
old interpolation targets. Closing the panel disables both configured nodes.
Single-hand `hand_mode:=left/right` launches are supported: the panel falls back
to the unsuffixed `/manus_revo3_retarget` node when necessary.

The port keeps this repository's linear MIT interpolation, model files, and
legacy retarget configuration. Joint scale/offset controls use the existing
`physical_<side>_<joint>_scale/offset_deg` parameters. The middle-spread dynamic
switch and thumb geometry/IK/EMA controls use the existing `legacy_*` parameter
names. These output adjustments apply on top of the configured legacy mapping;
existing configuration and saved tuning files remain usable.

### Saving and loading calibration

Set an operator ID and optional calibration name, then select **Export Override
YAML** for each hand. The default location is:

```text
~/.brainco/retarget_calibration/
  library/left/<operator>_<name>_<timestamp>.yaml
  library/left/latest.json
  library/right/...
  <operator>_revo3_left.yaml
  <operator>_revo3_right.yaml
```

Each export creates a timestamped snapshot and updates the side's latest pointer
and compatibility copy. Replaced files retain a `.bak`. History **Set as latest**
changes the stored selection; it does not apply that snapshot to running nodes.
The export contains the panel's tuning parameters, not output enable, temporary
stage, MIT gains, or the other legacy mapping parameters in the base configs.

Use `--calibration-dir` to choose another directory. The panel also reads
`teleop_operator` and `retarget_calibration_dir` from `$ROBOT_CONFIG` or
`~/.brainco/robot_config/robot_config.json` when present.

The pipeline does **not** automatically resolve `latest.json`. Pass the exported
YAML paths (or the per-operator compatibility copies) explicitly on restart:

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py hand_mode:=both \
  launch_tuning_panel:=true \
  left_calibration_config:="$HOME/.brainco/retarget_calibration/alice_revo3_left.yaml" \
  right_calibration_config:="$HOME/.brainco/retarget_calibration/alice_revo3_right.yaml"
```

### Teleoperation without the panel

The repository's `scripts/teleop.sh` and `run_pipeline_record_mcap.sh` explicitly
pass `start_enabled:=true`, retaining their existing automatic output behavior.
For direct launch, opt in with `start_enabled:=true` (incompatible with
`launch_tuning_panel:=true`), or enable/disable a running node via ROS:

```bash
ros2 service call /manus_revo3_retarget_right/enable_retarget std_srvs/srv/SetBool '{data: true}'
ros2 service call /manus_revo3_retarget_right/enable_retarget std_srvs/srv/SetBool '{data: false}'
```

Use the `_left` service for the left node; single-hand launches use
`/manus_revo3_retarget/enable_retarget`. `start_enabled` is a read-only startup
parameter. Disable sends a zero-gain, zero-effort release command for every joint
before stopping publication. Enabling requires a fresh glove frame before any
new motion command is sent.

### Verification

After building and sourcing the workspace:

```bash
python3 -m pytest src/manus_revo3_retarget/test
REVO3_CALIBRATION_INTEGRATION=1 python3 -m pytest \
  src/manus_revo3_retarget/test/test_staged_calibration_ros.py
```

The opt-in integration tests start only retarget nodes and synthetic glove data.
All input/output topics are remapped into unique test namespaces, so no hardware
commands are published. The panel interaction test also requires `DISPLAY`.

## Launch And Record MCAP

To start the pipeline and record the default Manus/Revo3 topics into
`manus_revo3_retarget/log`:

```bash
cd src/manus_revo3_retarget
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

The `retarget_targets` topics contain the post-retarget, pre-linear-interpolation
MIT target for each side. The high-rate `commands` topics are published by the
timer.

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
cd src/manus_revo3_retarget
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
