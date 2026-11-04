# Revo3 Driver

[English](README.md) | [简体中文](README_CN.md)

## Overview

`revo3_driver` provides a ROS 2 `ros2_control` hardware interface for BrainCo Revo3 21-DoF dexterous hands.

Key features:

- Modbus serial communication at 5 Mbps
- Single-hand and dual-hand launch files
- MIT command interfaces: `position`, `velocity`, `effort`, `kp`, `kd`
- State feedback: `position`, `velocity`, `current`, `motor_state`
- `mock_components/GenericSystem` simulation mode
- udev helper scripts for stable `/dev/revo3_hand_left` and `/dev/revo3_hand_right` aliases

## Build

The tested BC Revo3 SDK `v1.0.4` is included under `vendor/dist`.

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-up-to revo3_driver --symlink-install
source install/setup.bash
```

To refresh the SDK:

```bash
cd <workspace>/src/brainco_revo3_ros2/revo3_driver
bash scripts/download_sdk.sh
```

## Launch

Simulation:

```bash
ros2 launch revo3_driver revo3_system.launch.py hand_side:=right if_sim:=true
```

Right hand hardware:

```bash
ros2 launch revo3_driver revo3_system.launch.py hand_side:=right
```

Left hand hardware:

```bash
ros2 launch revo3_driver revo3_system.launch.py hand_side:=left
```

Dual hand:

```bash
ros2 launch revo3_driver dual_revo3_system.launch.py
```

## Serial Setup

```bash
cd <workspace>/src/brainco_revo3_ros2/revo3_driver/setup
bash bootstrap_revo3.sh
bash check_revo3_setup.sh
```

Default Modbus slave IDs:

- Left hand: `126`
- Right hand: `127`

Optional aliases:

- Left hand: `/dev/revo3_hand_left`
- Right hand: `/dev/revo3_hand_right`

## Controllers

Launch activates:

- `revo3_joint_state`
- `joint_forward_mit_controller`

It also loads these controllers inactive:

- `joint_forward_pos_controller`
- `joint_traj_pos_controller`

Switch from MIT control to trajectory control:

```bash
ros2 control switch_controllers \
  --controller-manager /revo3_right/controller_manager \
  --deactivate joint_forward_mit_controller \
  --activate joint_traj_pos_controller
```

If `allow_partial_joints_goal` is enabled in `config/revo3_controllers.yaml`, a partial goal can be sent like this:

```bash
ros2 action send_goal \
  /revo3_right/joint_traj_pos_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names:
    - right_index_MCP_joint
    - right_index_PIP_joint
    - right_index_DIP_joint
  points:
    - positions: [0.6, 0.6, 0.4]
      time_from_start: {sec: 2, nanosec: 0}
"
```

The default configuration requires full joint goals for `joint_traj_pos_controller`. Keep `allow_partial_joints_goal: false` for stricter application-level checks, or enable it before sending partial joint goals.

## State Topics

With the default namespace:

```bash
ros2 topic echo /revo3_right/revo3_joint_state/joint_states
ros2 topic echo /revo3_right/revo3_joint_state/dynamic_joint_states
```

`dynamic_joint_states` includes `current` and `motor_state`. See `README_INTERFACES_CN.md` for the full interface map.

## License

Apache License 2.0. See [LICENSE](LICENSE).
