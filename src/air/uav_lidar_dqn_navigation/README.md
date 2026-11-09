# uav_lidar_dqn_navigation

基于激光雷达与深度强化学习（DQN）的无人机感知、建图与导航模块，运行于
AirSim + ROS Noetic。模块最终包含四项能力：

1. **键盘遥控**：终端按键控制速度与偏航，带档位切换、里程统计
2. **雷达感知 + DQN 导航**：24 扇区雷达特征输入 Dueling Double DQN，输出离散速度指令
3. **建图与导航同步**：gmapping 粒子滤波边飞边建图，DQN 策略同时导航
4. **图像端到端 DQN**：前置相机 84×84 灰度图直接输入卷积网络，输出控制指令

与监督式回归方案不同，本模块的控制器全部由**强化学习**训练得到：智能体在
AirSim 场景里自行探索，以“接近目标、朝向对齐、避开障碍”为奖励信号优化策略。

本 PR 在模块骨架之上加入雷达感知层：24 扇区特征提取、180° 累积扫描发布，
以及对应的启动文件。DQN 训练与部署、建图、端到端控制按依赖顺序在后续 PR 中
逐层加入，README 与模块文档随之下沉。

## 运行环境

| 项目 | 版本 |
| --- | --- |
| 仿真器 | AirSim 1.8.1（Blocks 场景，Windows） |
| 系统 | Ubuntu 20.04（WSL2 亦可） |
| ROS | Noetic |
| Python | 3.8 |
| 依赖 | numpy、msgpack-rpc-python |

AirSim 与 ROS 的通信走 msgpack-rpc（默认 41451 端口）。WSL2 下需要把
`airsim_ip` 设为 Windows 主机在 WSL 网段里的地址（`/proc/net/route` 的网关）。
AirSim 的 `settings.json` 放在 `Blocks.exe` 同目录，传感器配置见 `config/settings.json`。

## 编译

```bash
cd ~/catkin_ws
cp -r <本模块目录> src/
catkin_make
source devel/setup.bash
```

若使用 AirSim 仓库的 PythonClient 源码，需要设置：

```bash
export AIRSIM_PYTHON_CLIENT=/path/to/AirSim/PythonClient
```

## 运行

```bash
# 任务 1 键盘遥控（在启动终端里按键：W/S 前后、A/D 转向、Q/E 升降、1/2/3 档位、T/L 起降）
roslaunch uav_lidar_dqn_navigation main.launch mode:=teleop

# 雷达感知：180° 累积扫描 + 24 扇区特征（另开 RViz 可看 /drone/lidar/scan）
roslaunch uav_lidar_dqn_navigation main.launch mode:=scan
```

也可以直接用统一入口 `main.py`：

```bash
rosrun uav_lidar_dqn_navigation main.py --mode teleop   # 桥接 + 遥控
rosrun uav_lidar_dqn_navigation main.py --mode bridge   # 只启动桥接
rosrun uav_lidar_dqn_navigation main.py --mode scan     # 雷达扫描发布
```

### 雷达感知

- `lidar_features.py` 把一帧点云压成 24 维扇区距离（每扇区取最近距离 / 40 m），
  与目标、速度、上一步动作拼成 32 维状态，供后续 DQN 使用；纯 NumPy 实现，
  `catkin_make run_tests` 或 `python3 -m unittest discover -s test` 均可离线验证。
- `lidar_scan_node.py` 累积 1 秒点云拼成 180° LaserScan（`/drone/lidar/scan`）。
  AirSim 旋转雷达单帧只覆盖一小段水平角，不累积的话建图会严重缺帧。
- `scan.launch` 同时发布 `odom->base_link`（桥接节点）与 `base_link->laser`
  静态 TF，单独检出本 PR 也能在 RViz 里正常显示。完整建图链路（`map->odom`
  由 gmapping 提供）见 PR #922。

## 话题与服务

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/drone/cmd_vel` | geometry_msgs/Twist | 机体系速度指令 |
| `/drone/odom` | nav_msgs/Odometry | ENU 位姿与速度 |
| `/drone/state` | std_msgs/Float32MultiArray | NED 原始状态 |
| `/drone/lidar/scan` | sensor_msgs/LaserScan | 180° 累积扫描（建图） |
| `/drone/lidar/sectors` | std_msgs/Float32MultiArray | 24 维扇区距离（控制） |
| `/drone/takeoff` | std_srvs/Trigger | 起飞到 `takeoff_height` |
| `/drone/land` | std_srvs/Trigger | 降落 |

## 目录

```
scripts/drone_bridge.py      AirSim <-> ROS 桥接
scripts/teleop_keyboard.py   任务 1 键盘遥控
scripts/lidar_features.py    雷达特征工程（24 扇区 / 扫描）
scripts/lidar_scan_node.py   雷达扫描话题发布
scripts/main.py              统一入口
test/test_lidar_features.py  特征工程单元测试（不依赖 AirSim）
launch/teleop.launch         键盘遥控启动文件
launch/scan.launch           雷达感知启动文件
launch/main.launch           统一启动入口（本阶段支持 teleop / scan）
```

## 后续 PR

| PR | 内容 |
| --- | --- |
| #893 | 雷达扇区特征与扫描话题发布 |
| #918 | DQN 策略训练流程与模型权重 |
| #922 | DQN 导航节点与 gmapping 建图链路 |
| #932 | 图像端到端 DQN 与模块文档、演示动图 |

## 实测结果

见模块文档 `docs/air/uav_lidar_dqn_navigation/index.md` 与后续 PR 的实测记录。
