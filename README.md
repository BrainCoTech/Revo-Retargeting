# Revo Retargeting

Control BrainCo Revo3 hands with MANUS gloves. This branch is `feat/manus-revo3-vector`; the quick start below uses the Vector backend and a right hand.

[中文使用说明](README_CN.md) · [First-time setup (中文)](docs/vector_setup_CN.md) · [Vector reference (中文)](src/manus_revo3_vector_retarget/README.md) · [HumanDex / DV1, legacy and Revo2](docs/other_inputs_EN.md)

## Daily use

Complete the first-time setup once. Power the hand, connect its USB adapter and the MANUS receiver, and turn on and wear the glove. Run commands from this repository root in two terminals.

### Terminal 1: start the right hand

```bash
conda activate revo_teleop
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=11
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/teleop.sh right retarget_backend:=vector
```

This starts the hardware driver, MANUS publisher and Vector backend. Hold a natural, steady hand pose for bone-length calibration: 120 frames initially, or 30 verification frames with a matching cache.

**Calibration does not enable motion. Enable following in terminal 2.**

### Terminal 2: check readiness and enable

```bash
conda activate revo_teleop
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=11
source /opt/ros/humble/setup.bash
source install/setup.bash

# Continue when phase is READY and ready_to_arm is true.
ros2 topic echo /manus_revo3_retarget/status std_msgs/msg/String --once

# Enable physical hand following.
ros2 service call /manus_revo3_retarget/enable_retarget std_srvs/srv/SetBool '{data: true}'
```

Domain 11 is an example; use the same `ROS_DOMAIN_ID` in both terminals. Run one teleoperation pipeline at a time. Omitting `retarget_backend:=vector` selects legacy.

### Pause, exit and recalibrate

```bash
# Pause; call with data: true to resume.
ros2 service call /manus_revo3_retarget/enable_retarget std_srvs/srv/SetBool '{data: false}'

# Repeat bone-length calibration, then explicitly enable again.
ros2 service call /manus_revo3_retarget/recalibrate std_srvs/srv/Trigger '{}'
```

Press `Ctrl+C` in terminal 1 to stop its processes. Stale input stops command publication; recovering input requires explicit re-enabling. Pausing stops MIT publication and leaves the hand powered.

## Left or both hands

Replace `right` with `left` or `both` in terminal 1. Single-hand services keep the same names. For both hands, enable each side:

```bash
ros2 service call /manus_revo3_retarget_left/enable_retarget std_srvs/srv/SetBool '{data: true}'
ros2 service call /manus_revo3_retarget_right/enable_retarget std_srvs/srv/SetBool '{data: true}'
```

## Started but not moving

| Status or symptom | Action |
| --- | --- |
| `WAITING_INPUT` | Check glove power, receiver and MANUS logs. |
| `CALIBRATING` | Hold a natural, steady pose until calibration completes. |
| `READY`, `enabled=false` | Call `enable_retarget` with `data: true`. |
| `BLOCKED` or `fault` | Read `reason` and `controller.reason`; restore the driver/input, then re-enable when ready. |
| Missing status topic or service | Check workspace setup and matching ROS domains. |
| Missing `install/setup.bash` or Vector package | Follow the [first-time setup](docs/vector_setup_CN.md). |

Four-finger hardware MPR positive points toward the thumb on both hands. Both Vector profiles use `scale: -1` for these joints. Restart Vector after profile changes; see the [coordinate reference](src/manus_revo3_vector_retarget/README.md#模型和硬件坐标).

Both thumbs use a 30° direction offset by default so MCP can come down during opposition. Append `thumb_offset_deg:=<deg>` to tune it (larger lowers MCP); see [thumb direction offset](src/manus_revo3_vector_retarget/README.md#拇指方向补偿).

Direct output is the default. Append `interpolate:=true` to enable 200 Hz interpolation and a two-second startup ramp.
