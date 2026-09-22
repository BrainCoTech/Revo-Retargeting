# Revo3 HandKinematics Retarget

[Chinese version](README_CN.md) · [Workspace setup](../../README.md)

## Start Revo3 with HumanDex / DV1

Run the build commands from the repository root.

### Step 1: Build

If this workspace is already built, skip step 1. On a new computer, prepare the system dependencies and initialize submodules as described below first.

```bash
bash scripts/setup_revo_conda.sh
conda activate revo_teleop
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 python -m colcon build --base-paths src --symlink-install \
  --packages-up-to manus_revo3_retarget revo3_driver revo2_teleop_bringup \
  --cmake-args -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
               -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python"
```

### Step 2: Start

**Terminal 1: start SDK acquisition and keep it running.** Adjust the SDK path and serial port to match your machine.

```bash
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
/usr/bin/python3 \
  "$HOME/code/tele-retarget/brainco_revohuman_sdk/tools/ros2_joint_state_pub.py" \
  --hand right \
  --port /dev/ttyACM0
```

**Terminal 2: from the Revo-Retargeting repository root, start DV1 adaptation with FK, Revo3 retargeting, and the hardware driver.**

```bash
source /opt/ros/humble/setup.bash
conda activate revo_teleop
source install/setup.bash
bash scripts/teleop.sh right input_source:=dv1
```

Use `left` for the other hand. For `both`, run SDK acquisition for each hand with its own serial port, then run `bash scripts/teleop.sh both input_source:=dv1`.
The adapter computes FK from the SDK joint states; no separate FK process is needed.
Override `sdk_path:=/path/to/brainco_revohuman_sdk` or `urdf_path:=/path/to/model.urdf` when needed.
The default SDK path is `$HOME/code/tele-retarget/brainco_revohuman_sdk`.

Without `input_source:=dv1`, `teleop.sh` keeps its MANUS default and starts the MANUS publisher.
The `humandex` source uses the older paired joint/pose topics; `external` consumes an existing `HandKinematics` publisher.

## Revo3 Hardware Connection and Device Naming

Power on the hand and connect it to the computer through USB serial. The default
configuration uses Modbus at 5 Mbps. With `auto_detect: true`, the driver scans
serial ports for slave ID 126 (left) or 127 (right). This mode ignores the configured
`port`, so no device alias is required.

Run the following commands from the repository root. This command lists local
serial ports and the USB topology without opening serial communication:

```bash
bash src/brainco_revo3_ros2/revo3_driver/setup/discover_revo3_serial.sh
```

The list includes other serial devices. Stop the driver and unplug/reconnect each
hand individually to identify its port. Your user account needs read/write access
to the serial port. If access is denied, run the following and log in again:

```bash
sudo usermod -aG dialout "$USER"
```

### Optional: Stable Device Aliases

Device numbers such as `/dev/ttyUSB*` and `/dev/ttyACM*` may change after reconnecting.
For a stable port name, identify the right-hand port and run the following,
replacing `/dev/ttyUSB0` with the verified port:

```bash
sudo bash src/brainco_revo3_ros2/revo3_driver/setup/setup_revo3_udev_rules.sh \
  /dev/ttyUSB0 right
ls -l /dev/revo3_hand_right
```

The script creates the `/dev/revo3_hand_right` symlink and retains the original
device name. Use `left` for `/dev/revo3_hand_left`. It overwrites
`/etc/udev/rules.d/99-revo3-hands.rules`, removes old aliases for any side omitted
from the invocation, and sets matching serial ports to mode `0666` (read/write
access for all local users). For both hands, provide both ports in a single call,
for example `/dev/ttyUSB0 right /dev/ttyUSB1 left`, rather than running the
single-hand command twice. Reconnect USB if the alias does not appear.

Rules prefer USB topology matching. The default `hub-relative` mode matches the
last two port levels only when the path is deep enough; shallower paths use the
full path. USB serial numbers and interface numbers are used only when `ID_PATH`
is unavailable. Recheck the bindings after changing hub ports or the USB topology;
an alias is not guaranteed to follow the same physical hand.

### Launch with a Fixed Port

Creating an alias does not update the driver configuration. First, copy the complete
right-hand protocol configuration:

```bash
cp src/brainco_revo3_ros2/revo3_driver/config/protocol_modbus_right.yaml \
  /tmp/revo3_protocol_right.yaml
```

Under `hardware` in the copy, set `auto_detect` to `false` and `port` to
`/dev/revo3_hand_right`, leaving all other fields unchanged. Source the ROS and
workspace environments from step 2, then launch:

```bash
REVO3_RIGHT_PROTOCOL_CONFIG=/tmp/revo3_protocol_right.yaml \
  bash scripts/teleop.sh right input_source:=dv1
```

For the left hand, use `protocol_modbus_left.yaml` and `REVO3_LEFT_PROTOCOL_CONFIG`.
For long-term use, save the copy in your own configuration directory and pass its
absolute path to the launch command.

## Architecture and Runtime Behavior

The package retains the name `manus_revo3_retarget` for compatibility with existing
scripts. The C++ retarget node subscribes only to `hand_teleop_msgs/HandKinematics`;
separate adapters connect MANUS and HumanDex. The Pinocchio solver, 21 output
joints, MIT interpolation, and controller topics retain their existing structure.

```text
MANUS → manus_hand_adapter ───────┐
                                 ├→ HandKinematics → Revo3 retarget → MIT controller
HumanDex mux → humandex_hand_adapter ┘
```

- `scripts/teleop.sh` includes the driver; do not start it again separately.
  The lower-level `pipeline_launch.py` includes only input and retargeting.
  When using it to control physical hardware, start the Revo3 driver separately.
- This package publishes `revo3_mit_controller_msgs/msg/Revo3MITCommand`.
- Retargeting runs directly from the HandKinematics subscription callback. MIT commands
  are published by a separate timer at `mit_command_publish_hz` (default 200 Hz)
  using the latest retarget target.
- Default command topics:
  - `/revo3_left/joint_forward_mit_controller/commands`
  - `/revo3_right/joint_forward_mit_controller/commands`
- The thumb IK backend is Pinocchio and uses the shared `revo3_description` URDF.

## Inputs and Parameters

`scripts/teleop.sh` accepts `left`, `right`, or `both` as its first argument.
For DV1, it starts one FK adapter per selected side and connects the resulting
HandKinematics topics to the retarget pipeline. SDK acquisition runs separately.
Use `START_REVO3_DRIVER=0` to reuse an existing driver.
For MANUS, prepare its official SDK and build `manus_ros2`; the default script
starts the MANUS publisher unless `START_MANUS_PUBLISHER=0` is set.

The older `input_source:=humandex` mode requires `/joint_states` and
`/humandex_eef_pose` with matching timestamps from external acquisition/FK.
Use `input_source:=dv1` for the SDK JointState input described above.
Driver protocol overrides use `REVO3_LEFT_PROTOCOL_CONFIG` and
`REVO3_RIGHT_PROTOCOL_CONFIG` environment variables.

If a HandKinematics publisher is already running, use `input_source:=external`
to skip the adapter and specify the field mapping with `input_config:=/path/to/input.yaml`.
`left_input_topic` and `right_input_topic` are passed to both the adapter and
retargeter, defaulting to `/hand_kinematics/left` and `/hand_kinematics/right`.

Input must contain a positive source timestamp, the correct side, and the
`hand_retarget_<side>` frame. Array lengths, name uniqueness, and finite numeric
values are validated. MCP/PIP/DIP values for all four fingers, the selected
spread fields, and thumb_tip are required; incomplete frames do not update the target.
The `thumb_pip` and `thumb_dip` positions and thumb angle references are optional.
`source` is message metadata only; the algorithm does not select behavior by device name.

| Configuration | MANUS | HumanDex |
|---|---|---|
| Four-finger flexion | `index_mcp/pip/dip`, etc., in rad | Same field names, in rad |
| `spread_joint_suffix` | `spread` | `mpr` |
| `spread_relative_to_middle` | `true` | `false` |
| `thumb_cmr_joint_name` | `thumb_mcp_spread` | `thumb_cmr` |
| Coordinate transformation | Adapter applies `(-y, -x, z)` | Adapter passes through palm-local coordinates |

Input configurations are in `config/input_manus.yaml` and `config/input_humandex.yaml`.
They load after the base retargeting configurations and before user calibration
overrides. Use them to change field selection and input signs. The retargeter
no longer converts MANUS angle units or uses MANUS node IDs.

A tip is the glove/model fingertip. The HumanDex_bimanual DIP-to-tip offsets are
included in the default BrainCo-HumanDex FK configuration. Robot IK continues
to use `thumb_tip_Link`. `config/revo3_pad_contacts.yaml` separately stores
finger-pad reference points and is not currently loaded by IK.
`config/humandex_fingertips.yaml` documents the model, offsets, and their scope;
they cannot be applied directly to different DV1 SDK models. MANUS/HumanDex adapters do not generate aggregate flexion; Revo2 performs this mapping independently.
See the [architecture migration guide](../../docs/revo3_architecture_migration.md) for compensation merging, normalized IK, and configuration migration. Existing periodic MIT publishing is retained: the last target
continues to be published when input stops. Input validation does not provide
an automatic stop on connection loss.

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

Default parameters are split by function. These four YAML files are loaded
before the selected `input_<source>.yaml` field mapping:

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
