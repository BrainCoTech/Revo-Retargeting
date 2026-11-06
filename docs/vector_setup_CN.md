# Vector 首次安装

[返回日常使用](../README_CN.md)。本页只在新电脑或首次构建时执行；命令均在仓库根目录运行。
需要 Ubuntu 22.04、已安装的 ROS 2 Humble 和可用的 Conda，Python 使用 3.10。

## 1. 获取代码和驱动

```bash
git clone --branch feat/manus-revo3-vector https://github.com/BrainCoTech/Revo-Retargeting.git
cd Revo-Retargeting
git submodule update --init --recursive
```

已有此分支的工作区可跳过克隆。切换分支前先保存自己的改动。

本分支将驱动固定到可获取的 `2fa9db4b4bf2c46fe78a2b3b8f1224b3a002baf5`，
已用该版本完成构建和 ROS 合成测试，左右手 May3 URDF 哈希均与 Vector profile 一致。
使用上面的 `git submodule update` 即可对齐版本。

<details>
<summary>旧版本初始化时报 not our ref 34528e75</summary>

旧提交曾引用无法从远端获取的 `34528e75`。先更新主仓库到当前分支的最新提交，
再执行 `git submodule update --init --recursive`。仅需在旧工作区临时恢复驱动时，可执行：

```bash
git -C src/brainco_revo3_ros2 fetch origin 2fa9db4b4bf2c46fe78a2b3b8f1224b3a002baf5
git -C src/brainco_revo3_ros2 checkout --detach 2fa9db4b4bf2c46fe78a2b3b8f1224b3a002baf5
```

子模块有本地改动时，先保存再切换版本。

</details>

## 2. 安装系统依赖和 SDK

以下命令以已配置 ROS 2 Humble 软件源为前提：

```bash
sudo apt-get update
sudo apt-get install -y \
  git git-lfs curl unzip build-essential cmake libeigen3-dev libncurses-dev \
  libgl1 libglx-mesa0 libusb-1.0-0 python3-pip python3-venv python3-yaml \
  ros-humble-ros-base ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-robot-state-publisher
git lfs install --local
bash src/brainco_revo3_ros2/revo3_driver/scripts/download_sdk.sh
```

MANUS 的官方 SDK 动态库需另外安装。将下一条中的路径换成你下载的 SDK 压缩包：

```bash
MANUS_SDK_ARCHIVE=/实际路径/MANUS_SDK.zip bash scripts/install_manus_sdk.sh
```

也可使用解压目录：

```bash
MANUS_SDK_DIR=/实际路径/ManusSDK bash scripts/install_manus_sdk.sh
```

两种方式任选一种。安装后，`src/brainco_drivers/manus_ros2/ManusSDK/lib/` 应包含真实的
`libManusSDK*.so`。缺少这些文件时包仍可能构建成功，但不会生成 MANUS 采集程序。
SDK 打包与分发方法见 [MANUS bridge 文档](../src/brainco_drivers/manus_ros2/README.md)。

## 3. 准备 Python 环境并构建

新建 `revo_teleop`；已有该环境时跳过创建这一条，直接激活并安装依赖：

```bash
conda create -y -n revo_teleop python=3.10 pip
conda activate revo_teleop
export PYTHONNOUSERSITE=1
python -m pip install -r requirements.txt 'setuptools<80' pytest
source /opt/ros/humble/setup.bash
python -m colcon build --base-paths src --symlink-install \
  --packages-up-to manus_revo3_vector_retarget revo3_driver \
  --cmake-args -DBUILD_TESTING=OFF \
               -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
               -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python"
source install/setup.bash
```

`PYTHONNOUSERSITE=1` 避免用户目录中的 Python 包覆盖环境依赖。Vector 使用
NumPy 1.24–1.x、SciPy 1.10–1.15.x、MuJoCo 3.3.7–3.x；MuJoCo 用于正运动学，无窗口也需要安装。
旧工作区更新后可在已加载上述环境的终端增量构建：

```bash
python -m colcon build --base-paths src --symlink-install --packages-select manus_revo3_vector_retarget
source install/setup.bash
```

检查安装结果，以下操作不会启动节点或控制真机：

```bash
python -m pip check
python -c 'import numpy, scipy, mujoco, rclpy; print(numpy.__version__, scipy.__version__, mujoco.__version__)'
ros2 pkg executables manus_ros2
ros2 run manus_revo3_vector_retarget retarget --help
```

应能看到 `manus_data_publisher`、`manus_calibration_tool` 和 Vector 的命令帮助。

## 4. 连接和手套标定

机器手上电并连接 USB，MANUS 接收器插入电脑、手套打开。串口权限不足时执行：

```bash
sudo usermod -aG dialout "$USER"
```

修改用户组后重新登录。默认自动检测从站地址：右手 `127`、左手 `126`，波特率 5 Mbps。
固定串口别名等细节见 [连接说明](../src/manus_revo3_retarget/README_CN.md#revo3-真机连接与设备命名)。

在已加载环境的终端执行右手的 MANUS 手套标定，按程序提示完成：

```bash
bash scripts/calibrate_manus.sh right
```

左手换成 `left`，双手分别标定。此时不要同时运行 MANUS 采集程序。
MANUS 手套标定完成后，按[主 README](../README_CN.md#日常使用启动标定启用)启动。
Vector 会另外采集骨长，完成后还需显式启用输出。
