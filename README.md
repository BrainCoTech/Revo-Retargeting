# Revo3 Retargeting

ROS 2 Humble workspace for teleoperating BrainCo Revo3 hands with MANUS gloves.

This branch is the runnable Revo3 workspace. The operator entrypoints are in `scripts/`.

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

Clone the repository and initialize the Revo3 driver submodule:

```bash
git clone https://github.com/BrainCoTech/Revo-Retargeting.git
cd Revo-Retargeting
git checkout revo3_retargeting
git submodule update --init --recursive
```

Create and activate a Python environment. Conda is recommended:

```bash
conda create -n revo_retargeting python=3.10 -y
conda activate revo_retargeting
```

Install system, ROS, Python, Git LFS, and submodule dependencies:

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

Connect and calibrate MANUS before first use, or after changing users:

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
src/brainco_revo3_ros2          Revo3 upstream driver submodule
src/brainco_revo3_ros2/revo3_mit_controller_msgs
src/brainco_revo3_ros2/revo3_mit_controller
src/brainco_revo3_ros2/revo3_description
src/brainco_revo3_ros2/revo3_driver
src/manus_revo3_retarget        MANUS to Revo3 retarget pipeline
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
