# uav_nn_perception_navigation

基于神经网络的无人机感知、导航与端到端控制模块，运行于 **AirSim 1.8.1 + ROS Noetic**。

## 功能一览

| 编号 | 功能 | 入口 | 说明 |
| --- | --- | --- | --- |
| 1 | 键盘遥控 | `main.py --mode teleop` | 终端按键控制无人机六个自由度 |
| 2 | 激光雷达感知 + 神经网络控制 | `main.py --mode perception` | 点云障碍检测，神经网络输出速度指令 |
| 3 | SLAM 建图 + 导航 | `main.py --mode slam` / `--mode nav` | 边飞边建图，基于地图规划路径 |
| 4 | 端到端视觉控制 | `main.py --mode end2end` | 图像直接输入网络，输出控制指令 |

## 运行环境

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 20.04（或 WSL2 中的 Ubuntu 20.04） |
| ROS | Noetic |
| 仿真器 | AirSim 1.8.1（Windows 或 Linux 均可） |
| Python | 3.8 |

## 依赖安装

```bash
pip3 install -r requirements.txt

# AirSim 官方客户端（PyPI 上的 airsim 包在部分 Python 版本下无法构建，
# 推荐直接使用 microsoft/AirSim 仓库中的 PythonClient 源码）
git clone --depth 1 https://github.com/microsoft/AirSim.git
export AIRSIM_PYTHON_CLIENT=$PWD/AirSim/PythonClient
```

把 `config/settings.json` 复制到 AirSim 的配置位置：

- **Windows**：放在 `Blocks.exe` **同目录**。
  注意：如果系统用户名是中文，AirSim 无法正确解析 `C:\Users\用户名\Documents\AirSim\`，
  必须使用可执行文件同目录的方案。
- **Linux**：`~/Documents/AirSim/settings.json`

## 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

如果 AirSim 运行在 Windows 主机上，虚拟机里的 ROS 需要通过主机 IP 访问：

```bash
# WSL2 中的 Windows 主机地址
python3 -c "import struct; \
[print('.'.join(str(b) for b in bytes.fromhex(l.split()[2])[::-1])) \
 for l in open('/proc/net/route').readlines()[1:] if l.split()[1]=='00000000']"
```

## 运行

```bash
# 键盘遥控（先启动 AirSim 仿真器）
roslaunch uav_nn_perception_navigation teleop.launch airsim_ip:=<主机IP>
```

| 按键 | 动作 |
| --- | --- |
| w / s | 前进 / 后退 |
| a / d | 左移 / 右移 |
| q / e | 左转 / 右转 |
| r / f | 上升 / 下降 |
| 空格 | 急停 |

## 话题与服务

| 名称 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/uav/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 速度指令，m/s 与 rad/s |
| `/uav/odom` | `nav_msgs/Odometry` | 发布 | 位姿与速度（已由 NED 转到 ENU） |
| `/uav/takeoff` | `std_srvs/Trigger` | 服务 | 起飞 |
| `/uav/land` | `std_srvs/Trigger` | 服务 | 降落 |

## 目录结构

```
uav_nn_perception_navigation/
├── launch/          # 各功能的启动文件
├── scripts/         # main.py 统一入口与各节点
├── config/          # AirSim 配置与参数
├── models/          # 训练好的网络权重
└── docs/            # 说明文档（对应仓库 docs/air/ 下的页面）
```

## 常见问题

**连接失败 `Retry connection over the limit`**
仿真器没启动，或者 IP 不对。先确认 Blocks 窗口已加载完成，再确认 `airsim_ip`。

**改了 settings.json 不生效**
Windows 下中文用户名会导致 `Documents\AirSim\settings.json` 被覆盖回默认值，
把配置放到可执行文件同目录即可。

**键盘没反应**
节点必须运行在真实终端里；通过管道或后台运行时读不到标准输入。

## 感知节点

`lidar_perception.py` 从 AirSim 读取激光雷达点云，发布两路数据：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/uav/lidar/scan` | `sensor_msgs/LaserScan` | 标准扫描，供 RViz 显示与 gmapping 建图 |
| `/uav/lidar/features` | `std_msgs/Float32MultiArray` | 12 维扇区距离，供控制节点推理 |

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `~scan_bins` | 180 | 扫描格数 |
| `~scan_fov` | 360.0 | 扫描视场（度） |
| `~scan_frame` | laser | 扫描坐标系，需与 base_link 区分 |
| `~scan_source` | lidar | 扫描来源，可选 `depth`（深度相机反投影） |
