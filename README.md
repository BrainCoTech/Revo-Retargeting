# Revo2 Retargeting

ROS 2 Humble workspace for teleoperating BrainCo Revo2 hands from HumanDex,
MANUS, or Hex gloves.

中文版: [README_CN.md](README_CN.md)

## Architecture

Every input backend terminates at the same device-neutral boundary:

```text
device driver / upstream acquisition
  -> hand_input_adapters
  -> hand_teleop_msgs/HandKinematics
  -> revo2_hand_retarget
  -> revo2_pid_controller target JointState
  -> revo2_driver
```

`HandKinematics` contains one side and one source timestamp per message. Joint
angles are radians, landmarks are meters, and missing measurements are omitted
instead of fabricated. Device adapters own coordinate/unit conversion; the
retargeter has no dependency on MANUS messages or HumanDex/Hex transport.

Main packages:

```text
src/brainco_capabilities/hand_teleop_msgs       neutral input contract
src/brainco_capabilities/hand_input_adapters    HumanDex, MANUS, Hex adapters
src/brainco_capabilities/revo2_hand_retarget    device-neutral retarget core
src/brainco_bringup/revo2_teleop_bringup        profile-driven composition
src/brainco_drivers/hex_glove_driver             raw UDP transport only
src/brainco_drivers/manus_ros2                   native MANUS SDK bridge
src/brainco_drivers/revo2_driver                 Revo2 ros2_control driver
```

## Build

Target environment: Ubuntu 22.04, ROS 2 Humble, Python 3.10.

```bash
source /opt/ros/humble/setup.bash
python -m colcon build
source install/setup.bash
```

Install the BrainCo Stark SDK when building `revo2_driver`:

```bash
bash src/brainco_drivers/revo2_driver/scripts/download_sdk.sh
```

The MANUS profile additionally requires the official SDK libraries under
`src/brainco_drivers/manus_ros2/ManusSDK/`.

## Launch

All inputs use one entry point and differ only by profile:

```bash
ros2 launch revo2_teleop_bringup teleop.launch.py \
  profile:=humandex_revo2 hand_mode:=right

ros2 launch revo2_teleop_bringup teleop.launch.py \
  profile:=manus_revo2 hand_mode:=right

ros2 launch revo2_teleop_bringup teleop.launch.py \
  profile:=hex_revo2 hand_mode:=right
```

HumanDex acquisition remains external. The default MANUS and Hex profiles start
their own input drivers; override with `launch_input_driver:=false` when an
existing process owns the device.

Start with a read-only offline pipeline before enabling Revo2 hardware:

```bash
ros2 launch revo2_teleop_bringup teleop.launch.py \
  profile:=humandex_revo2 hand_mode:=right \
  launch_revo2_driver:=false switch_controllers:=false
```

Confirm `/hand_kinematics/right` and the retarget target topic contain sane data
before launching real hardware. The Revo2 driver intentionally starts from its
safer position-controller configuration; the full teleoperation bringup switches
to `revo2_pid_controller` only when requested.

See package-level READMEs for contract fields, adapter parameters, and profiles.
