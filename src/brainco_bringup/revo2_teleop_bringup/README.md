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
