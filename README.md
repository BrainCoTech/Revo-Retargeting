# Revo Retargeting

ROS 2 Humble workspace for BrainCo Revo3 and Revo2 teleoperation with shared hand input adapters.

[Chinese version](README_CN.md) · [Revo2 guide](README_REVO2.md) · [DV1 adapter](src/brainco_capabilities/hand_input_adapters/README.md) · [Revo3 package guide](src/manus_revo3_retarget/README.md)

## Start Revo3 with HumanDex / DV1

Run the build commands from the repository root.

### Step 1: Build

Run step 1 for the first build and after updating the adapter. On a new computer, prepare the system dependencies and initialize submodules as described below first.

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

Use the SDK branch `feat/tracker-world-frame-and-bringup`; this interface follows commit `2683152` documented in [Issue #13](https://github.com/HAOTianGa03/brainco_revohuman_sdk/issues/13). Complete the [SDK preparation steps](src/brainco_bringup/revo2_teleop_bringup/README_DV1.md#sdk-准备独立工作区) first: install the native Python SDK, build its three ROS packages, and register each hand by USB topology. Keep the SDK and this repository in separate workspaces; source only the corresponding workspace in each terminal.

**Terminal 1: start official SDK acquisition and keep it running.** This example selects the right hand; replace the registry set name as needed.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_revohuman_ws/install/setup.bash
ros2 launch revohuman_bringup revohuman.launch.py mode:=right set_name:=bench \
  tracker_mode:=off publish_tactile:=off camera_mode:=off
```

**Terminal 2: start DV1 adaptation with FK, Revo3 retargeting, and the hardware driver.**

```bash
cd ~/code/tele-retarget/Revo-Retargeting
source /opt/ros/humble/setup.bash
conda activate revo_teleop
source install/setup.bash
bash scripts/teleop.sh right input_source:=dv1
```

Use `left` in both commands for the other hand. For both hands, set SDK `mode:=pair`, then run `bash scripts/teleop.sh both input_source:=dv1`. A single hand defaults to `joint_state_layout:=sdk_single`, consuming `/revohuman/{side}/joint_states`. Both hands default to `sdk_pair`, selecting 21 joints per side from `/revohuman/pair/joint_states`.

When running two independent SDK single-hand processes (`mode:=left` and `mode:=right`), use `bash scripts/teleop.sh both input_source:=dv1 joint_state_layout:=sdk_single`. Do not run pair and single-hand acquisition together: each glove allows one owning process.

The adapter subscribes with BEST_EFFORT QoS, preserves the SDK sample timestamp, and computes FK directly from the supplied radians. Debug topics remain `/humandex_{side}/fk_joint_states` and `/humandex_{side}/eef_pose`. The source `frame_id` is separate from the internal palm frame; if you customize it in the SDK, pass the corresponding `left_source_frame_id:=...` / `right_source_frame_id:=...`.
Override `sdk_path:=/path/to/brainco_revohuman_sdk` or `urdf_path:=/path/to/model.urdf` when needed.
Place the SDK checkout beside this repository, with the directory name `brainco_revohuman_sdk`:

```text
<parent>/
├── Revo-Retargeting/
└── brainco_revohuman_sdk/
```

The default SDK path is resolved relative to this repository, independently of the terminal's current directory.

For an older `/humandex_{side}/joint_states` publisher, explicitly select `joint_state_layout:=legacy` and set `left_joint_topic:=...` / `right_joint_topic:=...` as needed. Its joint names and frame must follow the legacy contract. See the [adapter layout table](src/brainco_capabilities/hand_input_adapters/README.md#dv1-sdk-direct-input). The old `revohuman_revo2` profile uses this repository's raw serial driver and must not run alongside SDK acquisition.

Without `input_source:=dv1`, `teleop.sh` keeps its MANUS default and starts the MANUS publisher.
The `humandex` source uses the older paired joint/pose topics; `external` consumes an existing `HandKinematics` publisher.

## Fresh Computer Setup

Target environment:

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10

Clone the repository and initialize the Revo3 driver submodule:

```bash
git clone https://github.com/BrainCoTech/Revo-Retargeting.git
cd Revo-Retargeting
git switch feat/revo3-hand-kinematics
git submodule update --init --recursive
```

Install system, ROS, Python, Git LFS, and submodule dependencies:

```bash
./scripts/install_revo3_deps.sh
```

The script installs the ROS control stack, Pinocchio, RViz support, MCAP bag support, Git LFS, and Python packages from `requirements.txt`. The shared `revo_teleop` environment is created by `setup_revo_conda.sh` in step 1.

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

The SDK installer copies the official `libManusSDK*.so` files into `src/brainco_drivers/manus_ros2/ManusSDK/lib/`. After installing the SDK, verify the workspace with:

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

## Hardware Connection

See the [Revo3 hardware connection and device naming guide](src/manus_revo3_retarget/README.md#revo3-hardware-connection-and-device-naming). Device aliases are optional when automatic detection is enabled.

For MANUS, connect and calibrate before first use:

```bash
bash scripts/calibrate_manus.sh right
bash scripts/calibrate_manus.sh left
```

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
src/brainco_capabilities/manus_ros2_msgs     MANUS ROS 2 messages
src/brainco_drivers/manus_ros2               MANUS SDK bridge, without redistributing SDK .so files
src/brainco_revo3_ros2                      Revo3 upstream driver submodule
src/brainco_revo3_ros2/revo3_mit_controller_msgs
src/brainco_revo3_ros2/revo3_mit_controller
src/brainco_revo3_ros2/revo3_description
src/brainco_revo3_ros2/revo3_driver
src/brainco_capabilities/hand_teleop_msgs     Device-neutral HandKinematics message
src/brainco_capabilities/hand_input_adapters  MANUS / HumanDex / Hex input adapters
src/manus_revo3_retarget                     HandKinematics to Revo3 retarget pipeline
```

## Troubleshooting

If `teleop.sh` says `Missing install/setup.bash`, build the workspace first.

If `check_system_deps.sh` reports missing workspace paths, initialize submodules:

```bash
git submodule update --init --recursive
```

If ROS packages are missing, rerun:

```bash
./scripts/install_revo3_deps.sh
```

If MANUS SDK files are missing, download the official MANUS SDK and run:

```bash
MANUS_SDK_ARCHIVE=/path/to/MANUS_SDK.zip ./scripts/install_manus_sdk.sh
```

If a new clone does not contain `scripts/` or `src/`, the branch was not published with the runnable workspace files. A fresh computer needs this branch to track `scripts/`, `requirements.txt`, `.gitmodules`, and the ROS packages under `src/`.
