# Revo2 Teleoperation Bringup

All glove backends use the same entry point:

```bash
ros2 launch revo2_teleop_bringup teleop.launch.py \
  profile:=humandex_revo2 \
  hand_mode:=right
```

Available profiles are `humandex_revo2`, `manus_revo2`, and `hex_revo2`.
HumanDex acquisition remains external by default.  The MANUS and Hex profiles
start their input drivers unless `launch_input_driver:=false` is supplied.

For offline, read-only validation without Revo2 hardware:

```bash
ros2 launch revo2_teleop_bringup teleop.launch.py \
  profile:=humandex_revo2 \
  hand_mode:=right \
  launch_revo2_driver:=false \
  switch_controllers:=false
```

RevoHuman SDK / DV1 input is available as `profile:=revohuman_revo2`.
Pass `revohuman_sdk_path`, `revohuman_left_port` / `revohuman_right_port`,
and optionally `revohuman_config_file`. This profile starts the SDK driver and
FK adapter; `launch_input_driver:=false` leaves the FK adapter running for an
already active raw publisher. See the `revohuman_kinematics` README for calibration.
