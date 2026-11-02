# 环境配置

本页介绍 WSL2 Ubuntu 22.04、ROS 2 Humble 的依赖安装、模拟器连接、YOLO 权重与功能包构建。Windows 侧运行 OpenHUTB AIR 模拟器。

## 环境与依赖

推荐环境为：

- Ubuntu 22.04；
- ROS 2 Humble；
- Python 3.10；
- OpenHUTB AIR 模式；
- WSL2 场景下由 Windows 主机运行 OpenHUTB，WSL2 Ubuntu 运行 ROS 2。

建议优先在 Linux/WSL2 自身文件系统中进行 ROS 2 构建，以减少 `/mnt/c`、`/mnt/d` 等跨文件系统目录带来的构建和文件访问开销。

先加载 ROS 2 环境：

```bash
source /opt/ros/humble/setup.bash
```

可使用带系统包访问能力的 Python 虚拟环境安装非 ROS Python 依赖：

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r src/air/openhutb_yolo_mlp_control/requirements.txt
```

`rclpy`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`std_msgs`、`vision_msgs`、`launch` 和 `launch_ros` 等 ROS 2 依赖由 ROS 环境提供。

## OpenHUTB/AirSim RPC 连接

OpenHUTB AIR 模式使用 AirSim RPC 接口，本示例默认端口为 `41451`。

### 原生 Linux 环境

如果 OpenHUTB 与 ROS 2 运行在同一 Linux 主机，可使用：

```bash
export AIRSIM_IP=127.0.0.1
```

### WSL2 连接 Windows 主机上的 OpenHUTB

如果 OpenHUTB 在 Windows 主机运行，而 ROS 2 在 WSL2 Ubuntu 中运行，WSL2 默认 NAT 网络模式下不应直接假定 `127.0.0.1` 一定能够访问 Windows 侧 RPC 服务。

可先在 WSL2 中获取默认网关地址：

```bash
export AIRSIM_IP=$(ip route | awk '/default/ {print $3; exit}')
echo "$AIRSIM_IP"
```

然后检查 RPC 端口是否可访问：

```bash
nc -vz "$AIRSIM_IP" 41451
```

如果系统尚未安装 `nc`，可以执行：

```bash
sudo apt update
sudo apt install -y netcat-openbsd
```

端口连通时会看到类似：`Connection to 172.x.x.x 41451 port [tcp/*] succeeded!`。

如果出现 `Connection refused` 或超时，应先确认 OpenHUTB 已经进入 AIR 模式，并检查 Windows 防火墙、WSL2 网络配置以及实际监听地址。

> WSL2 网络模式可能因系统配置不同而有所差异。如果已经使用支持 localhost 转发的网络模式，也可以测试 `127.0.0.1:41451`；最终以 `nc` 的实际连通结果为准。

## 相机分辨率

本示例的 YOLO 窗口按 **1280×720** 的相机图像配置。使用默认的 **256×144** 图像时，固定像素大小的性能信息面板会遮挡大部分画面，文字也可能显示不全。调整 OpenCV 窗口大小或 YOLO 推理尺寸不会提高相机采集分辨率。

在运行模拟器的 Windows 主机上，编辑“文档”目录中的 `AirSim/settings.json`（通常为 `%USERPROFILE%\Documents\AirSim\settings.json`）。将以下字段合并到现有配置的顶层，保留其他配置：

```json
{
  "CameraDefaults": {
    "CaptureSettings": [
      {
        "ImageType": 0,
        "Width": 1280,
        "Height": 720
      }
    ]
  }
}
```

`ImageType: 0` 对应 RGB 场景图像。若现有 `Vehicles.<车辆名>.Cameras.<相机名>.CaptureSettings` 已单独配置分辨率，应直接修改该相机中 `ImageType: 0` 项的 `Width` 和 `Height`；相机级配置会覆盖 `CameraDefaults`。本示例默认使用相机 `0`。

保存后重启 OpenHUTB，进入 AIR 模式并重新启动 ROS 感知节点。在已加载 ROS 环境的 WSL 终端确认实际图像尺寸，预期依次输出 `1280` 和 `720`：

```bash
ros2 topic echo /openhutb/camera/rgb --once --field width
ros2 topic echo /openhutb/camera/rgb --once --field height
```

## YOLO11n 模型

YOLO11n 权重需要单独准备，并通过 launch 参数传入本地模型路径。

- [YOLO11n 模型下载地址](https://pan.baidu.com/s/1nHSTqk4aYdC5HOpih6jPrg?pwd=fffa)
- 提取码：`fffa`

建议将权重放在 Linux/WSL2 文件系统中，例如：

```bash
mkdir -p ~/models
```

如果权重当前位于 Windows 的 `D:\models\yolo11n.pt`，在 WSL2 中对应路径通常为：`/mnt/d/models/yolo11n.pt`。

可以直接使用该路径，也可以复制到 WSL2 文件系统：

```bash
cp /mnt/d/models/yolo11n.pt ~/models/yolo11n.pt
```

启动前建议先确认文件存在：

```bash
test -f ~/models/yolo11n.pt && echo "YOLO model found"
```

运行本示例时建议显式传入本地绝对路径，不依赖 Ultralytics 自动联网下载。这样可以避免网络受限时出现 GitHub Release `403 Forbidden` 或下载失败。

## 构建

在仓库或 ROS 2 工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate

colcon build \
  --packages-select openhutb_yolo_mlp_control \
  --symlink-install
```

构建完成后加载工作空间：

```bash
source install/setup.bash
```

每次打开新的终端，建议按以下顺序恢复环境：

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
```

修改 launch 文件、`setup.py`、资源文件或模型安装配置后，应重新执行 `colcon build`。

配置完成后，参阅[YOLO 感知与八字轨迹控制](yolo_trajectory_control.md)或返回[文档目录](README.md)。

## 说明

本文档整理、代码重构和问题排查过程中使用了大模型辅助；提交者已检查全部内容并对最终结果负责。
