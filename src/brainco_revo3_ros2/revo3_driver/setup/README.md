# Revo3 串口 setup（Modbus）

与 `revo2_driver/setup/` 对齐：发现 → bootstrap → udev → 检查。

本目录包含 Revo3 Modbus 串口发现、udev 绑定和检查脚本。

---

## 快速开始

```bash
cd <workspace>/src/brainco_revo3_ros2/revo3_driver

bash scripts/download_sdk.sh          # 可选：刷新随仓 SDK
cd setup
bash bootstrap_revo3.sh               # 默认 auto（缺二进制会自动 build_detect）
bash check_revo3_setup.sh
```

日常 launch 跑手仍需 `colcon build revo3_driver`；**串口 bootstrap 与 colcon 无关**（探测工具由 `setup/CMakeLists.txt` 独立编译）。

---

## 脚本一览

| 脚本 | 用途 |
|------|------|
| `discover_revo3_serial.sh` | 列 tty + USB 拓扑 |
| `bootstrap_revo3.sh` | 一键 udev 绑定（默认 auto；`--manual` 交互） |
| `detect_revo3_ports_auto.sh` | SDK 扫 126/127（bootstrap 默认内部调用） |
| `build_detect_revo3_ports.sh` | 独立编译探测工具（不经过 colcon） |
| `setup/CMakeLists.txt` | 探测工具专用 CMake |
| `setup/src/detect_revo3_ports.cpp` | 探测工具源码（使用 `auto_detect_modbus_revo3`） |
| `setup/bin/detect_revo3_ports` | 编译产物（gitignore） |
| `setup_revo3_udev_rules.sh` | 写 udev（sudo） |
| `check_revo3_setup.sh` | 检查 `/dev/revo3_hand_*` 和 protocol config |

---

## 与 Revo2 setup 的区别

| 项 | Revo2 | Revo3 |
|----|-------|-------|
| SDK 探测函数 | `auto_detect_modbus_revo2` | `auto_detect_modbus_revo3` |
| Symlink | `/dev/revo2_hand_left/right` | `/dev/revo3_hand_left/right` |
| Udev 规则文件 | `99-revo2-hands.rules` | `99-revo3-hands.rules` |
| Slave ID | 126=left, 127=right | 126=left, 127=right（相同） |
| Protocol 版本 | V2 | V3 New |

---

## 注意

- 默认 auto 会先编译/调用 `detect_revo3_ports`（需先 `bash ../scripts/download_sdk.sh`）；失败可用 `--manual`
- bootstrap 只写 udev，**不会**改 yaml；默认 yaml 使用 `auto_detect: true`，无需 udev 也可启动。若改成固定串口模式，请确认 yaml 中：
  - `port: /dev/revo3_hand_left` / `/dev/revo3_hand_right`
  - `auto_detect: false`
- 若 `ls -l /dev/revo3_hand_*` 两个别名指向同一 tty，重跑 `bash bootstrap_revo3.sh`（udev 脚本会自动 fallback 到 exact 模式）
- 仅 Modbus
- `bootstrap` **不会**启动 ROS；启动双手请用 `dual_revo3_system.launch.py`
