# uav_lidar_dqn_navigation

基于激光雷达与深度强化学习（DQN）的无人机感知、建图与导航模块，运行于
AirSim + ROS Noetic。模块最终包含四项能力：

1. **键盘遥控**：终端按键控制速度与偏航，带档位切换、里程统计
2. **雷达感知 + DQN 导航**：24 扇区雷达特征输入 Dueling Double DQN，输出离散速度指令
3. **建图与导航同步**：gmapping 粒子滤波边飞边建图，DQN 策略同时导航
4. **图像端到端 DQN**：前置相机 84×84 灰度图直接输入卷积网络，输出控制指令

与监督式回归方案不同，本模块的控制器全部由**强化学习**训练得到：智能体在
AirSim 场景里自行探索，以“接近目标、朝向对齐、避开障碍”为奖励信号优化策略。

本 PR 在感知层之上加入 DQN 策略训练：Dueling Double DQN 网络、经验回放、
目标网络与 ε-greedy 探索，训练好的权重落在 `models/`。DQN 部署、建图、
端到端控制按依赖顺序在后续 PR 中逐层加入，README 与模块文档随之下沉。

## 运行环境

| 项目 | 版本 |
| --- | --- |
| 仿真器 | AirSim 1.8.1（Blocks 场景，Windows） |
| 系统 | Ubuntu 20.04（WSL2 亦可） |
| ROS | Noetic |
| Python | 3.8 |
| 依赖 | numpy、torch>=1.13、msgpack-rpc-python |

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

## 训练

任务 2 的 DQN 在 AirSim 里自行探索，状态为 32 维（24 扇区 + 目标 4 + 速度 3 +
上一步动作 1），动作为 6 个离散速度指令（前进/左前/右前/左转/右转/悬停）。
网络为 Dueling 结构（128-128），训练用 Double DQN 目标、经验回放与 Huber 损失。

奖励由四部分组成：距离进度、朝向对齐、时间惩罚、近障惩罚。1.2 m 内障碍
按距离线性扣分；0.5 m 内按事故终止（-20），避免策略学到“贴墙继续飞”。

```bash
# 一键训练（200 回合）
roslaunch uav_lidar_dqn_navigation train.launch episodes:=200

# 等价的手动方式
rosrun uav_lidar_dqn_navigation train_dqn_nav.py --episodes 200
rosrun uav_lidar_dqn_navigation main.py --mode train --episodes 200
```

训练日志写入 `models/train_nav_log.csv`（每回合一行，含 success / collision /
near_miss），随仓库一起提交。权重约定：

| 文件 | 含义 |
| --- | --- |
| `dqn_nav.pt` | 当前最佳（近 20 回合成功率最高），部署默认加载 |
| `dqn_nav_best.pt` | 同上，显式保留一份 |
| `dqn_nav_last.pt` | 最后一回合的权重，仅供对比 |

`--resume` 会同时恢复 Adam 优化器与步数，断点续训不会重置动量。

### 本次训练结果（seed 0，交付权重）

| 指标 | 数值 |
| --- | --- |
| 回合数 | 200 |
| 整体成功率 | 62.5% |
| 末 50 回合成功率 | 94% |
| 末 20 回合成功率 | 100% |
| 碰撞次数（全程） | 28 |
| 首次成功 | 第 21 回合 |
| 末 20 回合平均步数 / 奖励 | 78.8 / 144.7 |

逐回合数据见 `models/train_nav_log.csv`。`dqn_nav.pt` 与 `dqn_nav_best.pt`
的 SHA-256 相同，确认交付的是最佳权重而不是最后一回合覆盖的结果。

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
scripts/dqn_network.py       Dueling 网络与动作空间
scripts/dqn_agent.py         经验回放 / 目标网络 / Double DQN
scripts/train_dqn_nav.py     任务 2 训练
scripts/main.py              统一入口
test/test_lidar_features.py  特征工程单元测试（不依赖 AirSim）
launch/teleop.launch         键盘遥控启动文件
launch/scan.launch           雷达感知启动文件
launch/train.launch          DQN 训练启动文件
launch/main.launch           统一启动入口（本阶段支持 teleop / scan）
```

## 后续 PR

| PR | 内容 |
| --- | --- |
| #922 | DQN 导航节点与 gmapping 建图链路 |
| #932 | 图像端到端 DQN 与模块文档、演示动图 |

## 实测结果

见模块文档 `docs/air/uav_lidar_dqn_navigation/index.md` 与后续 PR 的实测记录。
