# Revo3 Description

`revo3_description` contains the ROS 2 description assets used by `revo3_driver`.

## Contents

- `urdf/revo3.single.system.xacro`: standalone Revo3 hand entry point with `ros2_control`
- `urdf/revo3_hand.xacro`: hand macro
- `urdf/revo3_left_may3.urdf`: left hand model
- `urdf/revo3_right_may3.urdf`: right hand model
- `meshes/`: visual and collision meshes
- `rviz/revo3_hand.rviz`: RViz configuration

## Build

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
colcon build --packages-select revo3_description --symlink-install
source install/setup.bash
```

## License

Apache License 2.0. See [LICENSE](LICENSE).
