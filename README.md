# Revo3 Retargeting

[简体中文](README_CN.md) · [Package guide](src/manus_revo3_retarget/README.md)

ROS 2 Humble workspace for teleoperating BrainCo Revo3 hands with MANUS gloves.

This branch is the runnable Revo3 workspace. The operator entrypoints are in `scripts/`.

The `revo3_retargeting` branch includes the Revo3 driver sources directly
under `src/brainco_revo3_ros2`, based on
[BrainCoTech/brainco_revo3_ros2](https://github.com/BrainCoTech/brainco_revo3_ros2)
with local zero-gain release and serial discovery fixes. No submodule setup is
needed for this branch.

## What This Branch Starts

`./scripts/teleop.sh` starts the full Revo3 teleoperation stack:

```text
Revo3 driver
MANUS publisher
MANUS -> Revo3 retarget pipeline
```

The retarget pipeline uses `manus_revo3_retarget/launch/pipeline_launch.py` and publishes `revo3_mit_controller_msgs/msg/Revo3MITCommand` to the Revo3 MIT controller topics.

## Fresh Computer Setup

Target environment:

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10

Clone the Revo3 retargeting branch, including its driver sources:

```bash
git clone --branch revo3_retargeting https://github.com/BrainCoTech/Revo-Retargeting.git
cd Revo-Retargeting
```

Create and activate a Python environment. Conda is recommended:

```bash
conda create -n revo_retargeting python=3.10 -y
conda activate revo_retargeting
```

Install system, ROS, Python, and Git LFS dependencies:

```bash
./scripts/install_revo3_deps.sh
```

The script installs the ROS control stack, Pinocchio, RViz support, MCAP bag support, Git LFS, and Python packages from `requirements.txt`.

MANUS SDK shared libraries are not stored in this repository. Download the official MANUS SDK from MANUS, then provide it to the installer with one of these options:

```bash
MANUS_SDK_ARCHIVE=/path/to/MANUS_SDK.zip ./scripts/install_revo3_deps.sh
MANUS_SDK_DIR=/path/to/unpacked/ManusSDK ./scripts/install_revo3_deps.sh
MANUS_SDK_URL=https://static.manus-meta.com/resources/manus_core_3/sdk/MANUS_Core_3.1.1_SDK.zip ./scripts/install_revo3_deps.sh
```

If all other dependencies are already installed, install only the MANUS SDK files:

```bash
MANUS_SDK_ARCHIVE=/path/to/MANUS_SDK.zip ./scripts/install_manus_sdk.sh
```

The SDK installer copies the official `libManusSDK*.so` files into `src/manus_ros2/ManusSDK/lib/`. After installing the SDK, verify the workspace with:

```bash
./scripts/check_system_deps.sh
```

To create a customer SDK package from a machine that already has the official
MANUS SDK files installed locally:

```bash
./scripts/package_manus_sdk.sh
```

This creates `dist/manus-sdk-linux-x86_64.tar.gz` plus a `.sha256` checksum.
Customers can install that archive with `MANUS_SDK_ARCHIVE=... ./scripts/install_manus_sdk.sh`.

Use the check script any time you move to a new computer or a new shell environment.

## Build

```bash
source /opt/ros/humble/setup.bash
python -m colcon build --symlink-install --packages-select \
  manus_ros2_msgs manus_ros2 \
  revo3_mit_controller_msgs revo3_description revo3_mit_controller revo3_driver \
  manus_revo3_retarget
source install/setup.bash
```

## Real Hardware Setup

On a new computer, configure Revo3 serial aliases and permissions once:

```bash
cd src/brainco_revo3_ros2/revo3_driver/setup
bash bootstrap_revo3.sh
bash check_revo3_setup.sh
cd -
```

Connect and calibrate MANUS before first use, or after changing users. These
commands overwrite the selected glove calibration (`.mcal`); Revo3 joint
calibration is a separate step below:

```bash
./scripts/calibrate_manus.sh right
./scripts/calibrate_manus.sh left
```

## Start Teleoperation

Start with one hand first:

```bash
./scripts/teleop.sh right
```

Left hand:

```bash
./scripts/teleop.sh left
```

Both hands:

```bash
./scripts/teleop.sh both
```

`Ctrl-C` stops all process groups started by the script.

## Online Calibration

Build with `colcon build --packages-up-to manus_ros2 revo3_driver manus_revo3_retarget`,
source `install/setup.bash`, and restart the hand driver to load its zero-gain
release fix. Use a desktop terminal with the same
`ROS_DOMAIN_ID` as the hand driver.

```bash
# Terminal 1: start both hand drivers (the default).
./scripts/start_driver.sh

# Terminal 2: start MANUS, both retarget nodes, and the staged calibration panel.
ros2 launch manus_revo3_retarget pipeline_launch.py launch_tuning_panel:=true
```

Both commands default to both hands. For a single hand, pass `right` or `left`
to `start_driver.sh` and the matching `hand_mode:=right` or `hand_mode:=left`
to the pipeline. The panel selects one hand at a time; use its hand switch to
calibrate the other hand.

If retargeting is already running, open only the panel:

```bash
ros2 run manus_revo3_retarget retarget_tuning_panel --side right --operator alice
```

The panel starts with output disabled. Enable the selected hand, tune spread,
four-finger flexion, and thumb/pinch in order, then export an override YAML.
History is stored under `~/.brainco/retarget_calibration/library/<side>/`.
Load a saved override with `right_calibration_config:=/absolute/path.yaml`
(or `left_calibration_config`). Avoid starting a second MANUS publisher or hand
driver against the same device.

Closing the panel or pressing Ctrl+C releases all 21 joints using zero MIT
gains and zero effort, allowing manual movement subject to mechanical friction.
This requires the updated driver; old builds replace zero gains with defaults.
If a connected retarget node does not confirm the release request, the panel
stays open and displays an error. Forced termination or loss of communication
cannot guarantee release.

Direct `pipeline_launch.py` now defaults to output disabled; use
`start_enabled:=true` for automatic teleoperation without the panel.
`./scripts/teleop.sh` keeps its automatic output behavior by passing that
argument explicitly. See the [package calibration guide](src/manus_revo3_retarget/README.md#staged-online-calibration)
for stages, service calls, saved configurations, and tests.

### Spread direction and pinch calibration

Use the panel's final output parameters for routine adjustments:

| Observation | Parameter to adjust | Base configuration |
|---|---|---|
| One finger spreads in the opposite direction | Negate its current `physical_<side>_<finger>_MPR_joint_scale` | `spread_retarget.yaml` |
| Neutral spread is shifted, or its range is wrong | MPR `offset_deg`, or the magnitude of its scale | `spread_retarget.yaml` |
| Thumb opposition or bending misses contact | Thumb CMP/CMR first, then MCP/PIP/DIP output offsets/scales | `thumb_retarget.yaml` |
| Only one opposing finger bends incorrectly | That finger's MCP/PIP/DIP output offsets/scales | `four_finger_retarget.yaml` |

The files are under `src/manus_revo3_retarget/config/`. Preserve the legacy
mapping while tuning these final output parameters. Work through straight-hand
spread, four-finger flexion, then thumb-to-index/middle/ring/little contact;
change one parameter at a time and recheck opening after each change. Save the
results as an override instead of overwriting shared base configurations.

See the [step-by-step parameter guide](src/manus_revo3_retarget/README.md#spread-direction-and-pinch-calibration)
([中文](src/manus_revo3_retarget/README_CN.md#侧摆方向与对指校准)) for exact names,
examples, node selection, loading order and acceptance checks. The full stage
uses the existing retargeting algorithm; contact still needs operator checking.

| Entrypoint | Default hands | Initial retarget output |
|---|---|---|
| `./scripts/start_driver.sh` | Both | Starts only the drivers |
| Direct `pipeline_launch.py` | Both | Disabled |
| `./scripts/teleop.sh` | Right | Enabled explicitly |

## Useful Script Options

Start only the Revo3 driver:

```bash
./scripts/start_driver.sh right
./scripts/start_driver.sh both
```

Run teleoperation while reusing an already running Revo3 driver:

```bash
START_REVO3_DRIVER=0 ./scripts/teleop.sh right
```

Run teleoperation while reusing an already running MANUS publisher:

```bash
START_MANUS_PUBLISHER=0 ./scripts/teleop.sh right
```

Pass extra launch arguments through to `pipeline_launch.py`:

```bash
./scripts/teleop.sh right numeric_threads:=1 mit_command_publish_hz:=200
```

## Package Layout

```text
src/manus_ros2_msgs             MANUS ROS 2 messages
src/manus_ros2                  MANUS SDK bridge, without redistributing SDK .so files
src/brainco_revo3_ros2          Included Revo3 driver sources with online calibration fixes
src/brainco_revo3_ros2/revo3_mit_controller_msgs
src/brainco_revo3_ros2/revo3_mit_controller
src/brainco_revo3_ros2/revo3_description
src/brainco_revo3_ros2/revo3_driver
src/manus_revo3_retarget        MANUS to Revo3 retarget pipeline
```

## Troubleshooting

If `teleop.sh` says `Missing install/setup.bash`, build the workspace first.

If `check_system_deps.sh` reports missing workspace paths, check that you cloned
the complete `revo3_retargeting` branch. Its driver sources are included
in the repository.

If ROS packages are missing, rerun:

```bash
./scripts/install_revo3_deps.sh
```

If MANUS SDK files are missing, download the official MANUS SDK and run:

```bash
MANUS_SDK_ARCHIVE=/path/to/MANUS_SDK.zip ./scripts/install_manus_sdk.sh
```

If a new clone does not contain `scripts/` or `src/`, the branch was not published with the runnable workspace files. A fresh computer needs this branch to track `scripts/`, `requirements.txt`, and the ROS packages, including the driver sources, under `src/`.
