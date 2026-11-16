# uav_openhutb_lidar_nn_nav

基于 **OpenHUTB** 模拟器的无人机感知、建图导航与神经网络控制模块。

- 课程任务：任务（1）键盘遥控 / 任务（2）传感器感知与控制 / 任务（3）建图与导航 / 任务（4）端到端模型
- 运行环境：Ubuntu 20.04 + ROS Noetic（VirtualBox 虚拟机）；OpenHUTB 运行在 Windows 宿主机
- 入口：`python3 main.py --list`，或 `roslaunch uav_openhutb_lidar_nn_nav main.launch mode:=<模式>`

## 1. 设计思路

**只有 `airsim_bridge` 直接调用模拟器接口**，其余节点全部通过 ROS 话题解耦：

```
         OpenHUTB（Windows 宿主机，RPC 41451）
                    │  airsim Python 客户端
                    ▼
             airsim_bridge.py            ← 唯一接触模拟器的节点
             ├── /uav/odom          (ENU 位姿)
             ├── /uav/lidar/scan    (前向 180° 扫描)
             ├── /uav/camera/image  (机载 RGB)
             ├── /uav/cmd_vel       (订阅)
             └── /uav/takeoff, /uav/land (服务)
                    │
     ┌──────────────┼───────────────┬──────────────┐
     ▼              ▼               ▼              ▼
keyboard_control  lidar_perception  grid_nav    nn_policy
   (任务1)          (任务2)         (任务3)      (任务4)
```

这样换模拟器只需改桥接层，算法层代码完全不动。

## 2. 依赖

```bash
# ROS 相关依赖
sudo apt install -y python3-numpy python3-opencv

# OpenHUTB 自带的 Python 客户端（不是 PyPI 上的 airsim）
# 把 OpenHUTB 安装目录下的 PythonClient 加入环境变量
export PYTHONPATH=$PYTHONPATH:/path/to/OpenHUTB/PythonClient
```

> 若 `import airsim` 失败，说明 Python 客户端路径没配好。PyPI 上的 `airsim` 包与 OpenHUTB
> 的 RPC 版本不一致，**不要**直接 `pip install airsim`。

## 3. 编译

```bash
cd ~/catkin_ws
cp -r /path/to/src/air/uav_openhutb_lidar_nn_nav src/
catkin_make
source devel/setup.bash
```

## 4. 运行

先启动 Windows 上的 OpenHUTB，进入**包含无人机的 AIR 场景**，确认端口连通：

```bash
nc -vz <模拟器IP> 41451
```

`<模拟器IP>` 的取值：VirtualBox NAT 用 `10.0.2.2`，桥接网卡用宿主机实际 IP，同机运行用 `127.0.0.1`。

### 没有安装 OpenHUTB 时：用本地仿真后端

模块内置 `backend:=mock`，用几何求交模拟雷达与运动，**话题、服务与 OpenHUTB 后端完全一致**，
可在评审环境中直接复现整条链路：

```bash
python3 main.py --mode perception --backend mock     # 任务2
python3 main.py --mode nav --backend mock            # 任务3
```

| 模式 | 命令 | 说明 |
| --- | --- | --- |
| 任务1 | `python3 main.py --mode keyboard` | 键盘遥控（需交互终端） |
| 任务2 | `python3 main.py --mode perception` | 激光感知 + 反应式避障 |
| 任务3 | `python3 main.py --mode nav` | 栅格建图 + A* + 航点跟踪 |
| 任务4 采集 | `python3 main.py --mode record` | 采集 (扫描, 动作) 数据 |
| 任务4 训练 | `python3 main.py --mode train` | 离线训练，不需要模拟器 |
| 任务4 飞行 | `python3 main.py --mode nn` | 用网络接管飞行 |

指定 IP 或后端：

```bash
python3 main.py --mode perception --airsim-ip 10.0.2.2
python3 main.py --mode perception --backend mock
roslaunch uav_openhutb_lidar_nn_nav main.launch mode:=nav backend:=mock goal_x:=3.5 goal_y:=3.5
```

> 两个后端都支持 `auto_takeoff`（默认开启），启动即起飞；键盘模式下按 `t`/`g` 手动起降。

## 5. 话题与服务

| 名称 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/uav/cmd_vel` | `geometry_msgs/Twist` | 订阅 | ROS 惯例：x 前、y 左、z 上（m/s），`angular.z` rad/s |
| `/uav/odom` | `nav_msgs/Odometry` | 发布 | 20 Hz，已由 NED 转到 ENU，`frame_id=odom` |
| `/uav/lidar/scan` | `sensor_msgs/LaserScan` | 发布 | 前向 180°，`range_max=40 m` |
| `/uav/camera/image` | `sensor_msgs/Image` | 发布 | `bgr8`，尺寸取决于 OpenHUTB 相机配置 |
| `/uav/takeoff` | `std_srvs/Trigger` | 服务 | 起飞并悬停 |
| `/uav/land` | `std_srvs/Trigger` | 服务 | 降落 |

## 6. 常见问题

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `连接 OpenHUTB 失败` | 模拟器没启动 / IP 错 / 防火墙 | 先进入 AIR 场景，再 `nc -vz <IP> 41451` |
| `ModuleNotFoundError: airsim` | Python 客户端路径未配置 | `export PYTHONPATH=...OpenHUTB/PythonClient` |
| 键盘没反应 | 非交互终端 | 用 `python3 main.py --mode keyboard` 在真实终端运行 |
| 雷达全是最大量程 | 场景里没有 LidarSensor / 高度带不匹配 | 检查 OpenHUTB 的 `settings.json`，或调 `~lidar_height` |
| 飞机不动 | 未起飞 | 先调 `/uav/takeoff` 服务，键盘模式按 `t` |
| 一直往上飞 | 坐标系符号反了 | 桥接层已做 `z_ned = -z_enu`，不要在参数里再取反 |

## 7. 文档

完整原理、公式、算法流程与源码解析见 `docs/air/uav_openhutb_lidar_nn_nav.md`。
