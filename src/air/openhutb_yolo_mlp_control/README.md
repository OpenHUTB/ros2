# OpenHUTB YOLO + MLP ROS 2 功能包

该功能包使用 ROS 2 消息解耦 OpenHUTB/AirSim、YOLO 目标检测和 MLP 轨迹控制。  
其中仅 `airsim_bridge` 直接调用 OpenHUTB/AirSim Python API，其余节点通过 ROS 2 话题交换数据。

## 节点关系

```text
OpenHUTB / AirSim
      ^   |
      |   +--> /openhutb/camera/rgb (sensor_msgs/Image) --> yolo_detector
      |   +--> /openhutb/odom       (nav_msgs/Odometry) --> mlp_controller
      |
      +------ /openhutb/cmd_vel     (geometry_msgs/Twist) <-- mlp_controller

trajectory_target
      +------ /openhutb/target      (geometry_msgs/PointStamped) --> mlp_controller

yolo_detector
      +------ /openhutb/detections  (vision_msgs/Detection2DArray)
      +------ /openhutb/yolo/image  (sensor_msgs/Image)
```

## 环境与构建

推荐使用 Ubuntu 22.04 / ROS 2 Humble。WSL2 环境下也可运行 ROS 2，OpenHUTB 可运行在 Windows 主机上。

请使用与当前 ROS 2 发行版兼容的 Python 环境。不要直接使用无法导入 `rclpy` 的独立 Conda 环境；Python 版本和 ABI 需要与 ROS 2 环境匹配。

在 ROS 2 工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash

python3 -m venv --system-site-packages .venv
source .venv/bin/activate

python -m pip install -r src/air/openhutb_yolo_mlp_control/requirements.txt

rosdep install --from-paths src --ignore-src -r -y

colcon build   --packages-select openhutb_yolo_mlp_control   --symlink-install

source install/setup.bash
```

如果已经配置好兼容的 Python 环境，可以直接复用，无需重复创建 `.venv`。

## OpenHUTB / AirSim 连接

运行 ROS 2 节点前，请先启动 OpenHUTB，并进入包含无人机的 AIR 场景。

原生 Linux 环境中，如果 OpenHUTB/AirSim 与 ROS 2 运行在同一系统，可使用：

```bash
export AIRSIM_IP=127.0.0.1
```

在 WSL2 中运行 ROS 2、Windows 主机运行 OpenHUTB 时，可以先尝试获取 Windows 主机地址：

```bash
export AIRSIM_IP=$(ip route | awk '/default/ {print $3; exit}')
```

检查 RPC 端口是否可访问：

```bash
nc -vz "$AIRSIM_IP" 41451
```

如果连接失败，请确认 OpenHUTB 已完全启动、防火墙允许访问 TCP 41451，并根据实际网络配置设置 Windows 主机地址。

## 运行 YOLO 感知

YOLO 权重 `yolo11n.pt` 不包含在本仓库中，请提前下载并保存到本地，例如：

```text
~/models/yolo11n.pt
```

启动感知节点：

```bash
ros2 launch openhutb_yolo_mlp_control perception.launch.py   airsim_ip:="$AIRSIM_IP"   model_path:="$HOME/models/yolo11n.pt"   show_window:=true
```

在支持 WSLg 的 WSL2 环境中，`show_window:=true` 可直接显示实时 YOLO 检测窗口。  
在无图形界面的 Linux、SSH 或服务器环境中，可改为：

```bash
show_window:=false
```

查看相关话题：

```bash
ros2 topic list
ros2 topic echo /openhutb/detections
```

## 运行八字轨迹控制

功能包中的 `models/mlp_controller.pth` 可直接用于轨迹控制，无需在每次运行前重新训练模型。

创建结果目录并启动八字轨迹控制：

```bash
mkdir -p "$HOME/openhutb_results"

ros2 launch openhutb_yolo_mlp_control trajectory_control.launch.py   airsim_ip:="$AIRSIM_IP"   auto_takeoff:=true   figure8_points:=160   result_dir:="$HOME/openhutb_results"
```

如果无人机已经处于飞行状态，可以使用：

```bash
auto_takeoff:=false
```

控制器订阅 `nav_msgs/Odometry` 和 `geometry_msgs/PointStamped`，并发布 `geometry_msgs/Twist`。  
`mlp_controller` 不直接调用 AirSim API，飞行控制命令统一由 `airsim_bridge` 转发。

如需重新生成训练数据并训练 MLP，可执行：

```bash
ros2 run openhutb_yolo_mlp_control generate_training_data
ros2 run openhutb_yolo_mlp_control train_mlp
```

轨迹完成后，可以根据保存的 CSV 生成轨迹分析图：

```bash
python -m openhutb_yolo_mlp_control.analyze_eight_trajectory   --csv "$HOME/openhutb_results/eight_trajectory.csv"   --outdir "$HOME/openhutb_results/analysis"
```

分析目录中会生成：

```text
trajectory_xy.png
tracking_error.png
path_tracking_error.png
```

## 运行完整系统

同时运行 OpenHUTB/AirSim 桥接、YOLO 感知、八字轨迹目标和 MLP 控制：

```bash
ros2 launch openhutb_yolo_mlp_control main.launch.py   airsim_ip:="$AIRSIM_IP"   yolo_model_path:="$HOME/models/yolo11n.pt"   show_window:=true
```

可通过以下命令查看启动文件提供的参数：

```bash
ros2 launch openhutb_yolo_mlp_control main.launch.py --show-args
```

## 坐标系约定

为保持与已训练控制器的一致性，`/openhutb/odom`、`/openhutb/target` 和 `/openhutb/cmd_vel` 使用 AirSim NED 坐标系。  
里程计消息的坐标系名称为：

```text
airsim_ned
```

## OpenHUTB AirSim PythonClient 兼容性

请不要额外安装旧版 PyPI `airsim` / `msgpack-rpc-python` 依赖栈。该功能包已经在 `vendor/` 目录中提供与 OpenHUTB 配套的 AirSim PythonClient，并通过本地 `msgpackrpc_compat.py` 处理 RPC 兼容性。

ROS 2 环境只需安装 `requirements.txt` 中列出的非 ROS Python 依赖；`rclpy`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`std_msgs`、`vision_msgs`、`launch` 和 `launch_ros` 等依赖由 ROS 2 环境提供。

## 常见问题

### `ConnectionRefusedError: [Errno 111] Connection refused`

通常表示 ROS 2 节点无法连接 OpenHUTB/AirSim RPC 服务。请检查：

```bash
nc -vz "$AIRSIM_IP" 41451
```

并确认 `airsim_ip` 指向实际运行 OpenHUTB 的主机。

### YOLO 模型下载出现 `403`

如果本地没有指定的 YOLO 权重，Ultralytics 可能尝试联网下载模型。在网络受限环境下可能出现 GitHub 下载失败。建议提前下载 `yolo11n.pt`，并使用本地绝对路径启动：

```bash
model_path:="$HOME/models/yolo11n.pt"
```
