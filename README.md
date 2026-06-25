# Revo2 Retargeting

ROS 2 Humble workspace for teleoperating BrainCo Revo2 hands with MANUS gloves.

This branch contains the runnable Revo2 workspace. The recommended hardware path is:

```text
MANUS SDK / manus_ros2
  -> manus_ros2_msgs/ManusGlove
  -> manus_revo2_retarget
  -> sensor_msgs/JointState target
  -> revo2_driver revo2_pid_controller
  -> Revo2 hardware velocity command interface
```

## Package Layout

```text
src/brainco_capabilities/manus_ros2          MANUS SDK bridge
src/brainco_capabilities/manus_ros2_msgs     MANUS ROS 2 messages
src/brainco_capabilities/manus_revo2_retarget
src/brainco_drivers/revo2_driver             Revo2 ros2_control driver
src/brainco_description/revo2_description    Revo2 hand description
```

Detailed retargeting, tuning, and troubleshooting notes live in:

```text
src/brainco_capabilities/manus_revo2_retarget/README.md
```

## Setup

Target environment:

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10

Create and activate a Python environment:

```bash
conda create -n manusglove python=3.10 -y
conda activate manusglove
python -m pip install --upgrade pip
python -m pip install rospkg catkin_pkg colcon-common-extensions
python -m pip install -r requirements.txt
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
```

Install the BrainCo Stark SDK used by `revo2_driver`:

```bash
bash src/brainco_drivers/revo2_driver/scripts/download_sdk.sh
```

MANUS SDK shared libraries are not stored in this repository. Put the official MANUS SDK files under:

```text
src/brainco_capabilities/manus_ros2/ManusSDK/include/
src/brainco_capabilities/manus_ros2/ManusSDK/lib/
```

The expected runtime libraries are:

```text
src/brainco_capabilities/manus_ros2/ManusSDK/lib/libManusSDK.so
src/brainco_capabilities/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so
```

## Build

```bash
source /opt/ros/humble/setup.bash
python -m colcon build --symlink-install --packages-select \
  manus_ros2_msgs manus_ros2 \
  revo2_description revo2_driver \
  manus_revo2_retarget
source install/setup.bash
```

## Start Teleoperation

Recommended right-hand real-hardware launch:

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control
```

Disable the plot window:

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control \
  launch_plot:=false
```

Left hand or both hands:

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py hand_mode:=left
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py hand_mode:=both
```

## Hardware Setup

Configure Revo2 serial aliases and permissions once on a new computer:

```bash
cd src/brainco_drivers/revo2_driver/setup
bash bootstrap_revo2.sh
bash check_revo2_setup.sh
cd -
```

For MANUS official glove calibration, see:

```text
src/brainco_capabilities/manus_revo2_retarget/README.md
```

## Troubleshooting

If build fails with missing BrainCo Stark SDK files, run:

```bash
bash src/brainco_drivers/revo2_driver/scripts/download_sdk.sh
```

If build fails with missing MANUS libraries, install the official MANUS SDK files into `src/brainco_capabilities/manus_ros2/ManusSDK/lib/`.

If runtime starts but the hand does not move, confirm the controller is active:

```bash
ros2 control list_controllers -c /revo2_right/controller_manager
ros2 topic echo /revo2_right/revo2_pid_controller/target_joint_states --once
```
