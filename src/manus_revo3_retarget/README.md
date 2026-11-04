# Manus Revo3 Retarget

[中文指南](README_CN.md)

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
# Run from the Revo-Retargeting repository root.
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to manus_ros2 revo3_driver manus_revo3_retarget
source install/setup.bash
```

The driver and its model/SDK dependencies are checked out from the pinned
`src/brainco_revo3_ros2` submodule. After updating the main repository, run
`git submodule sync --recursive` and `git submodule update --init --recursive`
before rebuilding. See [driver submodule version management](../../README.md#driver-submodule-version)
for the maintainer-only `develop` update procedure and the driver fixes awaiting
review; routine use follows the main repository's verified SHA.

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

Both driver and pipeline startup default to both hands. Start the drivers once
with `./scripts/start_driver.sh`, then launch the retarget nodes and panel:

```bash
source install/setup.bash
ros2 launch manus_revo3_retarget pipeline_launch.py launch_tuning_panel:=true
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
delivery, not motor feedback. The node waits up to four seconds per hand for
reliable DDS readers; the panel allows ten seconds for an enable/release service call, including
a node configured for both hands. Forced termination (`kill -9`), communication loss,
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

### Spread direction and pinch calibration

Use the final output parameters for routine direction, neutral-pose, and travel
adjustments. They are stored in the split configs below and can be overridden
in an exported calibration YAML. Here, `<side>` is `left` or `right`, and
`<finger>` is `index`, `middle`, `ring`, or `little`.

| Observed behavior | Parameter to change | Base config |
| --- | --- | --- |
| One finger spreads in the opposite direction | Negate its current `physical_<side>_<finger>_MPR_joint_scale`, keeping its magnitude | `config/spread_retarget.yaml` |
| Fingers spread correctly but do not line up when held together | Its `physical_<side>_<finger>_MPR_joint_offset_deg` | `config/spread_retarget.yaml` |
| Spread travel is too large or too small | Magnitude of the same MPR scale, preserving its sign | `config/spread_retarget.yaml` |
| One finger bends too far or not far enough | `physical_<side>_<finger>_<joint>_joint_scale`, with `<joint>` = `MCP/PIP/DIP` | `config/four_finger_retarget.yaml` |
| A four-finger flexion joint has a persistent pose offset | Corresponding `physical_<side>_<finger>_<joint>_joint_offset_deg` | `config/four_finger_retarget.yaml` |
| Thumb palm-base posture misses the opposition pose | `physical_<side>_thumb_CMP_joint_offset_deg` / `physical_<side>_thumb_CMR_joint_offset_deg`, one at a time | `config/thumb_retarget.yaml` |
| Thumb flexion is insufficient or excessive | `physical_<side>_thumb_<joint>_joint_scale` / `..._offset_deg`, with `<joint>` = `MCP/PIP/DIP` | `config/thumb_retarget.yaml` |

The final compensation is:

```text
q_target_rad = scale * q_mapped_rad + offset_deg * pi / 180
```

`q_mapped` already includes the retarget algorithm and its legacy mapping.
Thumb IK also has internal legacy compensation and joint-limit clipping.
The final scale/offset operation above has no subsequent URDF-limit clamp in
this node. Keep edits small and check the commanded and measured joint angles.
The first two stages apply their zero-target masks after this operation.

The panel applies slider changes when released, entry changes on Enter, and
`+`/`−` clicks immediately. Joint-scale steps are `0.05`; joint-offset steps are
`0.5°`. MPR scale allows `-2.5..2.5` and offset `±40°`; flexion/thumb scales allow
`0..2.5` and offsets `±30°`. These UI ranges do not describe mechanical limits.

Calibrate in this order:

1. Confirm the MANUS hand and side are correct. Repeat straight, fingers-together,
   open, and bent poses; correct unstable glove input before tuning output.
2. In **Spread / MPR**, hold the fingers straight and test each finger's opening
   and closing direction. If reversed, change only the sign of that finger's
   current final MPR scale. Then tune fingers-together offsets and spread travel.
   For example, change `1.0` to `-1.0`; an existing `-1.2` becomes `1.2`.
3. In **Four-finger flexion**, check straight, half-closed, and closed poses.
   Adjust one MCP/PIP/DIP scale or offset at a time. Recheck the straight pose
   after changing travel.
4. In **Thumb / pinch**, test thumb contact with index, middle, ring, and little
   fingertips in sequence. Adjust thumb CMP/CMR offsets first, then thumb
   MCP/PIP/DIP flexion. If only one opposing finger misses, inspect that finger's
   MPR and flexion parameters. Recheck all four contacts and the open hand after
   every change; the sign needed for an offset depends on the actual hand pose.
5. Export the selected hand's YAML, restart with it explicitly, read back the
   parameters, and repeat the pose checks before using the result for capture.

The thumb stage enables all joints; it does not fit a contact constraint between
the thumb and each finger. The existing thumb geometry gains
`legacy_<side>_physical_thumb_reach_scale` and
`legacy_<side>_physical_thumb_ik_position_scale`, IK task weights
`legacy_<side>_physical_thumb_pip_ik_scale` / `..._dip_ik_scale`, and EMA weights
`legacy_<side>_physical_thumb_ema_prev` / `..._ema_cur` remain available for
advanced tuning. Change these only after the joint compensation checks. The
`legacy_<side>_physical_middle_spread_dynamic` switch changes middle-finger spread
behavior. Preserve the base legacy mapping while resolving an individual MPR
sign, so a second sign change does not cancel the first.

### Minimal override and configuration precedence

For a right index whose current final MPR scale is `1.0`, the following file
reverses direction while retaining a zero offset. Save only parameters you have
measured and intend to override:

```yaml
manus_revo3_retarget:
  ros__parameters:
    physical_right_index_MPR_joint_scale: -1.0
    physical_right_index_MPR_joint_offset_deg: 0.0
```

Launch it from the repository root:

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:=right launch_tuning_panel:=true \
  right_calibration_config:=/absolute/path/revo3_manus_right.yaml
```

Files are loaded in this order, with later values replacing earlier ones:
`control_config` → `thumb_retarget_config` → `four_finger_retarget_config` →
`spread_retarget_config` → optional `retarget_config` → calibration override.
A nonempty `calibration_config` is applied to every launched side and causes
`left_calibration_config` and `right_calibration_config` to be ignored. Otherwise,
each side receives only its own side-specific override. Finally, the launch's
runtime fields (`hand_mode`, `start_enabled`, `use_revo3_namespace`,
`command_topic_suffix`, `retarget_target_topic_suffix`, and
`mit_command_publish_hz`) take precedence over file values.

Use the actual node name for readback; a right-only launch uses the unsuffixed
node, while `hand_mode:=both` uses the `_left` and `_right` nodes:

```bash
ros2 node list
# hand_mode:=right
ros2 param get /manus_revo3_retarget physical_right_index_MPR_joint_scale
ros2 param get /manus_revo3_retarget physical_right_index_MPR_joint_offset_deg
# hand_mode:=both
ros2 param get /manus_revo3_retarget_right physical_right_index_MPR_joint_scale
```

To compare targets, timer-published commands, and hardware feedback with the
default namespaces:

```bash
ros2 topic echo /revo3_right/joint_forward_mit_controller/retarget_targets --once
ros2 topic echo /revo3_right/joint_forward_mit_controller/commands --once
ros2 topic echo /revo3_right/revo3_joint_state/joint_states --once
ros2 launch manus_revo3_retarget command_state_viewer.launch.py hand_mode:=right
```

Match `joint_names` in MIT messages to `name` in JointState before comparing
positions; angle values are radians. Targets are captured before linear
interpolation, commands show the interpolated output, and JointState reports
the measured hand pose. During release, zero gains and effort determine holding
torque; release does not move the hand to the stage's zero target.

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
changes the stored selection and updates the current operator's compatibility
copy; it does not apply that snapshot to running nodes.
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
