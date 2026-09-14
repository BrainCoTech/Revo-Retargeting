# Revo2 Hand Retarget

This package maps `hand_teleop_msgs/HandKinematics` to the six-joint Revo2
command contract. It contains retarget algorithms, target filtering, optional
calibration, target-only publishing, and the Revo2 controller-facing launch.

The package deliberately does not start or parse glove drivers. Input selection
belongs to `revo2_teleop_bringup`; all device-specific conversion belongs to
`hand_input_adapters`.

Input topics:

- `/hand_kinematics/left`
- `/hand_kinematics/right`

Default target topics:

- `/revo2_left/revo2_pid_controller/target_joint_states`
- `/revo2_right/revo2_pid_controller/target_joint_states`

Run only the retarget layer:

```bash
ros2 launch revo2_hand_retarget pipeline_launch.py \
  hand_mode:=right controller_backend:=ros2_control
```

Use `revo2_teleop_bringup/teleop.launch.py` for normal operation so the selected
input adapter, retargeter, controller, and hardware lifecycle are composed from
one profile.

## Explicit aggregate flexion

MANUS/HumanDex input requires `finger_flexion_config:=flexion_manus.yaml` or `flexion_humandex_pip.yaml` (or a calibrated profile). Profile-driven bringup selects this explicitly. Use `ros2 run revo2_hand_retarget calibrate_dv1_fingers` for DV1 endpoints. See [migration](../../../docs/revo3_architecture_migration.md).
