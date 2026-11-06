# uav_lidar_dqn_navigation

基于激光雷达与深度强化学习（DQN）的无人机感知、建图与导航模块，运行于
AirSim + ROS Noetic。四项能力：

1. **键盘遥控**：终端按键控制速度与偏航，带档位切换、里程统计
2. **雷达感知 + DQN 导航**：24 扇区雷达特征输入 Dueling Double DQN，输出离散速度指令
3. **建图与导航同步**：hector_slam 用扫描匹配边飞边建图，DQN 策略同时导航
4. **图像端到端 DQN**：前置相机 84×84 灰度图直接输入卷积网络，输出控制指令

与监督式回归方案不同，本模块的控制器全部由**强化学习**训练得到：智能体在
AirSim 场景里自行探索，以“接近目标、朝向对齐、避开障碍”为奖励信号优化策略。

## 运行环境

| 项目 | 版本 |
| --- | --- |
| 仿真器 | AirSim 1.8.1（Blocks 场景，Windows） |
| 系统 | Ubuntu 20.04（WSL2 亦可） |
| ROS | Noetic |
| Python | 3.8 |
| 依赖 | numpy、torch>=1.13、opencv-python、msgpack-rpc-python |

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

所有功能都能通过 launch 启动：

```bash
# 任务 1 键盘遥控（在启动终端里按键：W/S 前后、A/D 转向、Q/E 升降、1/2/3 档位、T/L 起降）
roslaunch uav_lidar_dqn_navigation main.launch mode:=teleop

# 任务 2 雷达 + DQN 导航
roslaunch uav_lidar_dqn_navigation main.launch mode:=dqn

# 任务 3 hector_slam 建图 + DQN 导航同步
roslaunch uav_lidar_dqn_navigation main.launch mode:=slam

# 任务 4 图像端到端 DQN
roslaunch uav_lidar_dqn_navigation main.launch mode:=e2e
```

也可以直接用统一入口 `main.py`：

```bash
rosrun uav_lidar_dqn_navigation main.py --mode dqn
```

### 训练

```bash
# 任务 2：雷达特征 DQN
rosrun uav_lidar_dqn_navigation train_dqn_nav.py --episodes 200

# 任务 4：图像端到端 DQN
rosrun uav_lidar_dqn_navigation train_dqn_e2e.py --episodes 200
```

训练日志写入 `models/train_nav_log.csv`、`models/train_e2e_log.csv`，
权重保存为 `models/dqn_nav.pt`、`models/dqn_e2e.pt`。

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/drone/cmd_vel` | geometry_msgs/Twist | 机体系速度指令 |
| `/drone/odom` | nav_msgs/Odometry | ENU 位姿与速度 |
| `/drone/state` | std_msgs/Float32MultiArray | NED 原始状态 |
| `/drone/lidar/scan` | sensor_msgs/LaserScan | 180° 累积扫描（建图） |
| `/drone/lidar/sectors` | std_msgs/Float32MultiArray | 24 维扇区距离（控制） |
| `/drone/image_raw` | sensor_msgs/Image | 前置相机图像 |
| `/drone/goal` | geometry_msgs/PointStamped | NED 目标点 |

## 目录

```
scripts/drone_bridge.py      AirSim <-> ROS 桥接
scripts/teleop_keyboard.py   任务 1 键盘遥控
scripts/lidar_scan_node.py   雷达扫描发布
scripts/lidar_features.py    特征工程（训练/部署共用）
scripts/dqn_network.py       Dueling 网络与动作空间
scripts/dqn_agent.py         经验回放 / 目标网络 / Double DQN
scripts/train_dqn_nav.py     任务 2 训练
scripts/dqn_nav_node.py      任务 2 部署
scripts/camera_node.py       相机话题
scripts/train_dqn_e2e.py     任务 4 训练
scripts/dqn_e2e_node.py      任务 4 部署
launch/                      teleop / dqn / slam / e2e / main
```

## 实测结果

见本模块文档与 `models/train_*_log.csv`。
