# 基于深度强化学习的无人机雷达建图与端到端飞行控制

本模块在 AirSim + ROS Noetic 上实现四类无人机能力：键盘遥控、激光雷达感知与
DQN 导航、SLAM 建图与导航同步、图像端到端 DQN 控制。

与监督学习路线不同，本模块的控制器由**深度强化学习**训练得到：智能体在仿真场景中
自行探索，以“接近目标、朝向对齐、避开障碍”为奖励信号优化策略，不需要人工标注
专家轨迹。

本页说明模块总体设计、键盘遥控（PR #892）与雷达感知（PR #893）；DQN 训练与部署、
建图、端到端控制按依赖顺序在后续 PR 中补充。

## 1 运行环境

| 项目 | 版本 |
| --- | --- |
| 仿真器 | AirSim 1.8.1（Blocks 预编译场景，Windows） |
| 操作系统 | Ubuntu 20.04（WSL2） |
| ROS | Noetic |
| Python | 3.8 |
| 依赖 | numpy、msgpack-rpc-python |

AirSim 与 ROS 之间通过 msgpack-rpc（默认 41451 端口）通信。WSL2 下需要把
`airsim_ip` 设为 Windows 主机在 WSL 网段里的地址，即 `/proc/net/route` 中的网关地址。

编译与运行：

```bash
cd ~/catkin_ws
cp -r <模块目录> src/
catkin_make
source devel/setup.bash

# 任务 1：键盘遥控
roslaunch uav_lidar_dqn_navigation main.launch mode:=teleop
```

## 2 系统架构

```mermaid
flowchart LR
    A[AirSim Blocks] -- msgpack-rpc --> B[drone_bridge]
    B -- /drone/odom, /drone/state --> C[teleop_keyboard]
    C -- /drone/cmd_vel --> B
    A -- lidar_1 点云 --> D[lidar_scan_node]
    D -- /drone/lidar/sectors --> E[DQN 控制（后续 PR）]
    D -- /drone/lidar/scan --> F[gmapping（后续 PR）]
```

桥接节点是模块唯一的 AirSim 客户端：它把 ROS 的机体系速度指令翻译成 AirSim 的
`moveByVelocityZAsync` 调用，并把位姿、速度与原始状态发布回 ROS。控制节点只依赖
ROS 话题，因此可以脱离 AirSim 单独调试。

### 2.1 坐标系

AirSim 使用 NED（北-东-地），ROS 使用 ENU（东-北-天）。桥接节点在发布里程计时
做如下换算：

$$
x_{\text{ENU}} = y_{\text{NED}}, \qquad
y_{\text{ENU}} = x_{\text{NED}}, \qquad
z_{\text{ENU}} = -z_{\text{NED}}
$$

速度指令从机体系旋到世界系，偏航角速度还需反号：ROS 逆时针为正，AirSim 顺时针为正。

## 3 键盘遥控

`teleop_keyboard.py` 在终端读取单键输入，维护速度档位与里程统计，并把
`geometry_msgs/Twist` 发到 `/drone/cmd_vel`。

| 按键 | 功能 |
| --- | --- |
| W / S | 前进 / 后退 |
| A / D | 左偏航 / 右偏航 |
| Q / E | 上升 / 下降 |
| 1 / 2 / 3 | 低速 / 中速 / 高速档 |
| T / L | 起飞 / 降落 |

起飞与降落调用桥接节点提供的 `/drone/takeoff`、`/drone/land` 服务；高度由
`moveByVelocityZAsync` 保持，避免松键后掉高。

## 4 雷达感知

AirSim 的 `lidar_1` 水平视场 ±90°、16 线、量程 40 m。原始点云无法直接作为
控制输入，也不可能直接扔给 SLAM —— 旋转式雷达单帧只覆盖一小段水平角。因此
`lidar_features.py` 提供两种压缩方式：

- **24 扇区特征**：把 ±90° 等分 24 份，每份取最近命中距离并除以量程，
  得到 1 表示通畅、趋近 0 表示贴脸的归一化向量。位移与旋转不变性由机体系
  表达保证，DQN 用它做局部避障。
- **180° 扫描**：`extract_scan()` 把累积 1 秒的点云按角度装进 180 个 bin，
  无命中的角度填 `max_range`，生成标准 `sensor_msgs/LaserScan` 供建图使用。

扫描的角度参数按采样点计算：`angle_min = -90°`、`angle_increment = 1°`，
最后一个采样点为 `angle_min + (bins - 1) * angle_increment = +89°`，
即 180 个点覆盖 `[-90°, +89°]`，而不是 `[-90°, +90°]`。

`scan.launch` 同时发布 `odom->base_link`（桥接节点，`publish_tf:=true`）和
`base_link->laser`（静态 TF，雷达位于机体中心下方 0.5 m），因此单独检出本
PR 即可在 RViz 中查看扫描；`map->odom` 要等 #922 的 gmapping 提供。

```bash
roslaunch uav_lidar_dqn_navigation main.launch mode:=scan
rosrun rviz rviz   # Fixed Frame 选 odom，添加 LaserScan 订阅 /drone/lidar/scan
```

## 5 话题与服务

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/drone/cmd_vel` | geometry_msgs/Twist | 机体系速度指令 |
| `/drone/odom` | nav_msgs/Odometry | ENU 位姿与速度 |
| `/drone/state` | std_msgs/Float32MultiArray | NED 原始状态 |
| `/drone/lidar/scan` | sensor_msgs/LaserScan | 180° 累积扫描 |
| `/drone/lidar/sectors` | std_msgs/Float32MultiArray | 24 维扇区距离 |
| `/drone/takeoff` | std_srvs/Trigger | 起飞 |
| `/drone/land` | std_srvs/Trigger | 降落 |

## 6 后续 PR

| PR | 内容 |
| --- | --- |
| #893 | 雷达扇区特征与扫描话题发布 |
| #918 | DQN 策略训练流程与模型权重 |
| #922 | DQN 导航节点与 gmapping 建图链路 |
| #932 | 图像端到端 DQN 与演示动图 |

## 大模型使用声明

本模块在开发过程中使用大模型辅助编写代码与文档，提交者对全部内容负责。
