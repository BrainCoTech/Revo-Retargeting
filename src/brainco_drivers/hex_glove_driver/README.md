# Hex Glove Driver

`hex_glove_driver` is the ROS 2 bridge for Hex glove UDP data. It talks to the Windows Hex controller, publishes raw JSON debug topics, and converts Hex hand data into `manus_ros2_msgs/ManusGlove` messages that can be consumed by the Revo2 retargeting pipeline.

Full Hex -> Revo2 launch instructions live in:

```text
../../brainco_capabilities/manus_revo2_retarget/README_HEX.md
```

## Data Flow

```text
Windows Hex controller
  -> UDP JSON angles / positions
  -> hex_glove_udp_node
  -> /hex_glove/raw_angles
  -> /hex_glove/raw_positions
  -> /manus_glove_0 or /manus_glove_1
```

`/manus_glove_0` is the left-hand adapted topic, and `/manus_glove_1` is the right-hand adapted topic. These topic names match the retarget node's existing MANUS-style inputs.

## Build

From the workspace root:

```bash
cd /home/jiimmy/Brainco/Code/Revo-Retargeting
conda activate manusglove
source /opt/ros/humble/setup.bash
python -m colcon build --symlink-install --packages-select \
  manus_ros2_msgs hex_glove_driver
source install/setup.bash
```

## Run The Driver Only

Start the Windows Hex controller first, connect and calibrate the glove, enable UDP broadcast, then find the Windows IPv4 address with `ipconfig`.

Run the bridge without Revo2 hardware:

```bash
ros2 run hex_glove_driver hex_glove_udp_node \
  --ros-args \
  -p server_host:=<Windows_Hex_IP>
```

Check raw and adapted topics:

```bash
ros2 topic hz /hex_glove/raw_angles
ros2 topic hz /hex_glove/raw_positions
ros2 topic hz /manus_glove_1
ros2 topic echo /manus_glove_1 --once
```

## Parameters

Network parameters:

```text
server_host          Windows Hex controller IPv4. Default: 127.0.0.1
angles_port          UDP angle JSON port. Default: 9011
positions_port       UDP position JSON port. Default: 9013
connect_message      Packet sent to the controller to request data. Default: CONNECT
connect_period_sec   Seconds between CONNECT packets. Default: 1.0
```

Topic parameters:

```text
raw_angles_topic      Raw angle JSON topic. Default: /hex_glove/raw_angles
raw_positions_topic   Raw position JSON topic. Default: /hex_glove/raw_positions
left_glove_topic      Adapted left ManusGlove topic. Default: /hex_glove_0
right_glove_topic     Adapted right ManusGlove topic. Default: /hex_glove_1
left_manus_topic      Legacy alias for left_glove_topic
right_manus_topic     Legacy alias for right_glove_topic
topic                 Legacy alias for raw_positions_topic
```

The full Hex Revo2 launch overrides the adapted glove topics to:

```text
left_glove_topic=/manus_glove_0
right_glove_topic=/manus_glove_1
```

Adapter parameters:

```text
publish_adapter_glove           Publish adapted ManusGlove messages. Default: true
publish_manus_glove             Compatibility gate for adapted publishing. Default: true
position_scale                  Hex position units to meters. Default: 0.01
angle_scale                     Scale applied to Hex ergonomic angles. Default: 1.0
revo2_coordinate_transform      Apply Revo2 coordinate transform. Default: true
zero_angles_on_first_frame      Use the first angle frame as zero. Default: false
left_stretch_sign               Left stretch sign. Default: 1.0
right_stretch_sign              Right stretch sign. Default: 1.0
left_spread_sign                Left spread sign. Default: 1.0
right_spread_sign               Right spread sign. Default: 1.0
```

Thumb position calibration parameters:

```text
thumb_position_calibration_enabled   Enable thumb position affine calibration. Default: false
left_thumb_position_matrix           3x3 row-major matrix
right_thumb_position_matrix          3x3 row-major matrix
left_thumb_position_scale_xyz        Per-axis scale
right_thumb_position_scale_xyz       Per-axis scale
left_thumb_position_offset_xyz       Per-axis offset in meters
right_thumb_position_offset_xyz      Per-axis offset in meters
```

## Common Examples

Run against a Windows host:

```bash
ros2 run hex_glove_driver hex_glove_udp_node \
  --ros-args \
  -p server_host:=192.168.1.20
```

Publish directly to the retarget node's expected right-hand topic:

```bash
ros2 run hex_glove_driver hex_glove_udp_node \
  --ros-args \
  -p server_host:=192.168.1.20 \
  -p right_glove_topic:=/manus_glove_1
```

Use the first received open-hand frame as the angle zero:

```bash
ros2 run hex_glove_driver hex_glove_udp_node \
  --ros-args \
  -p server_host:=192.168.1.20 \
  -p zero_angles_on_first_frame:=true
```

Disable the Revo2 coordinate transform for raw geometry debugging:

```bash
ros2 run hex_glove_driver hex_glove_udp_node \
  --ros-args \
  -p server_host:=192.168.1.20 \
  -p revo2_coordinate_transform:=false
```

## Troubleshooting

If no raw topics publish:

```bash
ping <Windows_Hex_IP>
ros2 topic list | grep hex_glove
```

Then check:

- The Windows Hex controller is running.
- The glove is connected and calibrated.
- UDP broadcast is enabled in the Hex controller.
- Windows firewall is not blocking UDP.
- `server_host` is the Windows IPv4 address, not the Ubuntu machine address.

If raw topics publish but `/manus_glove_1` or `/manus_glove_0` does not publish, check the node log for JSON parse warnings and confirm the incoming JSON contains `rightHand` or `leftHand`.

If adapted topics publish but Revo2 does not move, continue with the full pipeline checks in:

```text
../../brainco_capabilities/manus_revo2_retarget/README_HEX.md
```
