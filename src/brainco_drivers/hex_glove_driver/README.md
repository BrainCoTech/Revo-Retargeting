# Hex Glove Driver

`hex_glove_driver` owns UDP transport only. It validates that incoming payloads
contain a Hex hand object and republishes the original JSON on:

- `/hex_glove/raw_angles`
- `/hex_glove/raw_positions`

It deliberately does not know about MANUS messages, Revo2 coordinates, joint
semantics, unit conversion, angle zeroing, or calibration. Those conversions
belong to `hand_input_adapters/hex_hand_adapter`.

Run the driver by itself:

```bash
ros2 run hex_glove_driver hex_glove_udp_node --ros-args \
  -p server_host:=<windows_ipv4>
```

The supported parameters are `server_host`, `angles_port`, `positions_port`,
`connect_message`, `connect_period_sec`, `raw_angles_topic`, and
`raw_positions_topic`.

For the complete Hex-to-Revo2 pipeline use the unified profile:

```bash
ros2 launch revo2_teleop_bringup teleop.launch.py profile:=hex_revo2
```
