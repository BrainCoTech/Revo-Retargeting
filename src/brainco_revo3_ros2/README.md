# BrainCo Revo3 ROS 2

[English](README.md) | [简体中文](README_CN.md)

ROS 2 packages for BrainCo Revo3 21-DoF dexterous hands. This repository contains the hardware driver, URDF/xacro description, and MIT command controller needed for Modbus-based control.

## Packages

```text
brainco_revo3_ros2/
├── revo3_driver/                 # ros2_control hardware interface and launch files
├── revo3_description/            # Revo3 URDF/xacro, meshes, and RViz config
├── revo3_mit_controller/         # MIT command controller plugin
└── revo3_mit_controller_msgs/    # MIT command message definition
```

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- `ros2_control`, `ros2_controllers`, `xacro`, `robot_state_publisher`
- A Revo3 hand connected through Modbus serial for hardware mode

## SDK

The tested BC Revo3 SDK `v1.0.4` is included in `revo3_driver/vendor/dist`, so a fresh clone can build without downloading extra SDK files.

To refresh the SDK manually:

```bash
cd <workspace>/src/brainco_revo3_ros2/revo3_driver
bash scripts/download_sdk.sh
```

## Build

```bash
mkdir -p ~/revo3_ws/src
cd ~/revo3_ws/src
git clone https://github.com/BrainCoTech/brainco_revo3_ros2.git

cd ~/revo3_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-up-to revo3_driver --symlink-install
source install/setup.bash
```

## Launch

Simulation mode, no hardware required:

```bash
ros2 launch revo3_driver revo3_system.launch.py hand_side:=right if_sim:=true
```

Hardware mode:

```bash
ros2 launch revo3_driver revo3_system.launch.py hand_side:=right
```

Dual-hand mode:

```bash
ros2 launch revo3_driver dual_revo3_system.launch.py
```

The default protocol files use SDK auto detection. Stable serial aliases are optional and can be configured with:

```bash
cd ~/revo3_ws/src/brainco_revo3_ros2/revo3_driver/setup
bash bootstrap_revo3.sh
bash check_revo3_setup.sh
```

## Interfaces

The hardware exposes per-joint command interfaces:

- `position` in rad
- `velocity` in rad/s
- `effort` as torque feedforward in mA
- `kp`
- `kd`

State interfaces:

- `position` in rad
- `velocity` in rad/s
- `current` in A
- `motor_state` as raw SDK status bitmask

See `revo3_driver/README.md` and `revo3_driver/README_INTERFACES_CN.md` for details.

## License

Apache License 2.0. See [LICENSE](LICENSE).
