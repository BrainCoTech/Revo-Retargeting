# Revo2 Retargeting

ROS 2 Humble workspace for teleoperating BrainCo Revo2 hands with MANUS gloves.

中文版: [README_CN.md](README_CN.md)

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
src/brainco_drivers/hex_glove_driver         Hex glove UDP bridge
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
  manus_ros2_msgs manus_ros2 hex_glove_driver \
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

## Hex Glove Teleoperation

The Hex glove path uses `hex_glove_driver` to convert UDP data from the Windows Hex controller into MANUS-compatible glove topics. The rest of the path still uses the Revo2 ros2_control PID controller:

```text
Windows Hex controller
  -> hex_glove_udp_node
  -> /manus_glove_0 or /manus_glove_1
  -> manus_revo2_retarget
  -> /revo2_<side>/revo2_pid_controller/target_joint_states
  -> revo2_pid_controller
  -> Revo2
```

Start the Windows Hex controller first, connect and calibrate the glove, then find the Windows IPv4 address with `ipconfig`.

To verify Hex glove data without starting Revo2 hardware:

```bash
ros2 launch manus_revo2_retarget hex_real_hand_pipeline_launch.py \
  hand_mode:=right \
  hex_server_host:=<Windows_Hex_IP> \
  launch_revo2_pipeline:=false
```

Check the converted and raw topics:

```bash
ros2 topic hz /manus_glove_1
ros2 topic echo /manus_glove_1 --once
ros2 topic echo /hex_glove/raw_angles --once
ros2 topic echo /hex_glove/raw_positions --once
```

To run the full right-hand Hex -> Revo2 PID pipeline:

```bash
ros2 launch manus_revo2_retarget hex_real_hand_pipeline_launch.py \
  hand_mode:=right \
  hex_server_host:=<Windows_Hex_IP>
```

## Controller Behavior

`revo2_driver` intentionally starts with `joint_forward_pos_controller` active by default. This is the safer default for standalone hardware bring-up because the hand can be tested with direct position commands and does not depend on a live MANUS retargeting stream.

The recommended MANUS teleoperation path uses `revo2_pid_controller` instead:

```text
MANUS -> retarget target JointState -> revo2_pid_controller -> velocity command interface
```

The real-hand pipeline includes a helper that tries to switch from the default position controller to `revo2_pid_controller`. If that automatic switch fails, check the current controller state:

```bash
ROS2CLI_DISABLE_DAEMON=1 ros2 control list_controllers \
  -c /revo2_right/controller_manager
```

Expected teleoperation state:

```text
revo2_joint_state            active
joint_forward_pos_controller inactive
revo2_pid_controller         active
joint_forward_vel_controller inactive
```

If `joint_forward_pos_controller` is still active and `revo2_pid_controller` is inactive, switch only the active position controller:

```bash
ROS2CLI_DISABLE_DAEMON=1 ros2 control switch_controllers \
  -c /revo2_right/controller_manager \
  --deactivate joint_forward_pos_controller \
  --activate revo2_pid_controller \
  --activate-asap \
  --strict
```

After a successful manual switch, relaunching with `switch_controllers:=false` avoids repeating the automatic switch helper:

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control \
  switch_controllers:=false \
  launch_plot:=false
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
ROS2CLI_DISABLE_DAEMON=1 ros2 control list_controllers -c /revo2_right/controller_manager
ros2 topic echo /revo2_right/revo2_pid_controller/target_joint_states --once
```

If `ros2 control` reports `!rclpy.ok()` or `xmlrpc.client.Fault`, restart the ROS 2 CLI daemon or disable it for that command:

```bash
ros2 daemon stop
ros2 daemon start
ROS2CLI_DISABLE_DAEMON=1 ros2 control list_controllers -c /revo2_right/controller_manager
```
