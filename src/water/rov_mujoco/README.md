# 水下机器人 MuJoCo 仿真 — ROS2 功能包

本功能包把 MuJoCo 中的水下机器人（ROV + UR5e 机械臂）封装为 ROS2 节点，
并给出四个课程任务的完整实现：物理/键盘控制、传感器与轨迹跟踪、SLAM 与导航、
图像端到端控制。**所有感知、规划、控制与端到端策略都是神经网络**，训练与推理均为
纯 NumPy 实现（无需 PyTorch），训练脚本、测试脚本与 ROS2 封装一并提供。

## 环境要求

- Ubuntu 22.04 + ROS2 Humble（本机实测）；Ubuntu 20.04 + Noetic 理论上兼容但未实测
- Python 3.10（NumPy 1.21.5、SciPy 1.8.0）
- MuJoCo 3.12.0
- 图形：无头测试必须 `MUJOCO_GL=osmesa`；3D 视窗需 `LIBGL_ALWAYS_SOFTWARE=1 MUJOCO_GL=glfw`

## 安装

```bash
cd ~/ros2_ws/src
# 拷贝本功能包到工作空间后：
cd ~/ros2_ws
colcon build --packages-select rov_mujoco --symlink-install
source install/setup.bash
cd src/rov_mujoco          # 或 src/water/rov_mujoco
```

`main.sh` 会自动检测 WSL 并设置 `MUJOCO_GL=osmesa`（无窗口模式）。

## 一键运行

| 命令 | 说明 |
|------|------|
| `./main.sh` | 无窗口基础仿真 |
| `./main.sh --gui` | 3D 视窗 + 6-DOF 键盘遥控（任务 1） |
| `./main.sh --test` | 任务 1 自动化测试 |
| `./main.sh --track` | 3D 视窗下的多传感器感知 + 神经网络轨迹跟踪（任务 2） |
| `./main.sh --eval-trajectory` | 任务 2 两工况性能对比测试 |
| `./main.sh --task3` | 3D 视窗下的声呐 SLAM + 神经网络导航（任务 3） |
| `./main.sh --test3` | 任务 3 建图/规划/导航闭环测试 |
| `./main.sh --task4` | 3D 视窗下的图像端到端视觉伺服（任务 4） |
| `./main.sh --launch4` | `ros2 launch` 启动任务 4 两个节点 |
| `./main.sh --test4` | 任务 4 三架构、延迟、闭环与零权值消融测试 |
| `./main.sh --test-nn` | 任务 2 神经网络训练有效性/零权值消融 |
| `./main.sh --test-planning` | 任务 3 Neural A* 与局部避障策略训练有效性 |
| `./main.sh --launch` | `ros2 launch` 启动仿真 + 键盘节点 |
| `./main.sh --launch3` | `ros2 launch` 启动任务 3 |

## 任务与模型

### 任务 1：物理仿真 + 键盘控制

`main.sh --gui` 打开 MuJoCo Viewer；`--test` 验证 W/S/A/D/Q/E/J/L/I/K/U/O/空格/±。
实测：按 W 后位移 0.0364 m、速度 0.3442 m/s；`--test` EXIT=0。

### 任务 2：多传感器 + 给定轨迹跟踪

- 传感器：72 束声呐 `/rov/sonar/scan`、前视相机 `/rov/camera/image_raw`、IMU、深度。
- 控制网络：`10 → 64 → 64 → 4`（tanh），输出 `[fx, fy, fz, tau_z]`，
  权值 `rov_mujoco/nn_weights.npz`，由 `train/train_trajectory_policy.py` 行为克隆 LOS/PID 专家。
- 训练：60k 样本，80 epochs；验证 MSE 0.000428，与专家相关系数 0.9991。
- 两工况实测（`pr05_task2_tracking.log`）：

| 工况 | 控制 | 3D RMSE | 水平 RMSE | 垂向 RMSE |
|---|---|---|---|---|
| 静水 | LOS+PID | 0.216 m | 0.214 m | 0.034 m |
| 静水 | NN | 0.221 m | 0.219 m | 0.032 m |
| 强剪切洋流 | LOS+PID | 0.223 m | 0.220 m | 0.034 m |
| 强剪切洋流 | NN | 0.227 m | 0.225 m | 0.031 m |

NN 略逊于专家基准（0.005–0.007 m），但完全由网络输出驱动；零权值消融下定点悬停
RMSE 从 0.504 m 退化到 1.098 m。

### 任务 3：声呐 SLAM + 神经网络导航

- 建图：`sonar_slam_node` 贝叶斯占据栅格，发布 `/map` 与 TF。
- 全局规划：Neural A*，启发式 `f = g + h*(1 + λ(1-Φ))`，λ=0.35；
  引导网络 `13 → 64 → 32 → 1`（sigmoid），由 `train/train_neural_astar.py` 训练，
  复现解析引导场 corr 0.99999、MSE 3e-6。
- 局部避障：`76 → 128 → 64 → 3`（tanh），`train/train_local_policy.py` 从声呐势场专家蒸馏；
  验证 MSE 0.0839，vx/wz 相关系数 0.88/0.91。
- 闭环实测（两场景各 4 航点）：

| 场景 | 到达 | 终点误差 | 里程 | 仿真时间 |
|---|---|---|---|---|
| 静水 | 4/4 | 0.100 m | 6.25 m | 20.0 s |
| 强剪切洋流 | 4/4 | 0.100 m | 6.36 m | 20.3 s |

### 任务 4：图像端到端控制

- 输入：前视相机 84×84 灰度图；输出 `[vx, vy, wz]`。
- 三种架构（纯 NumPy）：`mlp`、`small_cnn`、`nature_cnn`；
  权值 `models/task4_{arch}_bc.npz`，由 `train/train_task4.py` 行为克隆几何专家。
- 训练：1200 帧，30 epochs；验证集：

| 架构 | 参数量 | MSE | corr vx/vy/wz |
|---|---|---|---|
| mlp | 903,683 | 0.0552 | 0.736 / 0.936 / 0.903 |
| small_cnn | 84,579 | 0.0357 | 0.842 / 0.957 / 0.944 |
| nature_cnn | 337,347 | 0.0269 | 0.882 / 0.965 / 0.967 |

- `--test4` 闭环（初始横偏 0.6 m）：Nature-CNN `RMSE_y=0.080 m`、终点偏差 0.028 m、
  19.6 s 到达；零权值 `RMSE_y=1.112 m`、终点偏差 1.422 m。
- `--launch4`：相机 → `/cmd_vel`，约 70 s 控制时间从 `x=-3.5` 到达三通 `x=2.85, y=3.23`。

详细文档：`docs/rov_end_to_end.md`。

## 训练

```bash
python3 train/train_trajectory_policy.py     # 任务 2 轨迹控制网络
python3 train/train_neural_astar.py          # 任务 3 Neural A* 引导网络
python3 train/train_local_policy.py          # 任务 3 局部避障策略
python3 train/train_task4.py --samples 1200 --epochs 30   # 任务 4 视觉策略
```

首次运行任务 4 训练会渲染并缓存 `models/task4_bc_dataset.npz`（约 4.5 分钟），
之后复用缓存；`--refresh-data` 可强制重新采集。

## 键盘操作

| 按键 | 功能 |
|------|------|
| W/S | 前进/后退 |
| A/D | 左移/右移 |
| Q/E | 上浮/下潜 |
| J/L | 左偏航/右偏航 |
| I/K | 前俯仰/后俯仰 |
| U/O | 左翻滚/右翻滚 |
| 空格 | 急停 |
| +/- | 调节速度比例 |

## ROS2 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 速度指令 |
| `/rov/odom` | `nav_msgs/Odometry` | 发布 | 里程计 |
| `/rov/depth` | `std_msgs/Float64` | 发布 | 深度 |
| `/rov/imu` | `sensor_msgs/Imu` | 发布 | 姿态与角速度 |
| `/rov/sonar/scan` | `sensor_msgs/LaserScan` | 发布 | 72 束声呐 |
| `/rov/camera/image_raw` | `sensor_msgs/Image` | 发布 | 前视相机 |
| `/rov/global_path` | `nav_msgs/Path` | 发布 | 全局规划路径 |
| `/rov/local_path` | `nav_msgs/Path` | 发布 | 局部规划路径 |
| `/rov/avoidance_count` | `std_msgs/Int32` | 发布 | 避障事件计数 |
| `/map` | `nav_msgs/OccupancyGrid` | 发布 | 占据栅格地图 |
| `/joint_states` | `sensor_msgs/JointState` | 发布 | 机械臂关节状态 |

## 测试汇总（2026-10-18 本机实测）

| 命令 | 结果 |
|------|------|
| `--test` | EXIT=0，位移/速度/键位断言通过 |
| `--eval-trajectory` | EXIT=0，两工况 NN vs LOS 对比完成 |
| `--test-nn` | EXIT=0，专家一致性 corr>0.999；零权值闭环退化 2.2× |
| `--test3` | EXIT=0，两场景 4/4 航点，终点误差 0.100 m |
| `--test-planning` | EXIT=0，Neural A* 节点削减 57.8%，引导场 corr 0.99999 |
| `--test4` | EXIT=0，三架构加载、延迟 0.7–3.9 ms、闭环与零权值消融通过 |
| `--launch4` | EXIT=0，两节点启动，相机/`/cmd_vel` 正常，抵达三通 |

一键回归（依次跑上面 6 个测试入口，约 5–8 分钟）：

```bash
./tools/verify_all.sh
cat verification_logs/summary.txt
```

完整日志见作业材料 `00_总览/实测输出/`。

## 目录结构

```text
rov_mujoco/
├── main.py / main.sh / main.launch.py     # 统一入口
├── rov_mujoco/
│   ├── mujoco_sim_node.py                 # 物理仿真、传感器、相机发布
│   ├── keyboard_teleop_node.py            # 键盘遥控
│   ├── sonar_slam_node.py                 # 声呐 SLAM
│   ├── autonomous_navigation_node.py      # 自主导航
│   ├── end_to_end_node.py                 # 任务 4 视觉伺服节点
│   ├── nn_core.py                         # 纯 NumPy MLP 与 Adam
│   ├── nn_trajectory_controller.py        # 任务 2 控制网络
│   ├── neural_path_planner.py             # Neural A* + 局部避障策略
│   ├── end_to_end_networks.py             # 任务 4 三种视觉网络
│   ├── nn_weights.npz                     # 任务 2 权值
│   ├── planner_weights.npz                # Neural A* 权值
│   └── local_policy_weights.npz           # 局部避障权值
├── train/                                 # 4 个训练脚本
├── test/                                  # 各任务自动化测试
├── launch/                                # main/task3/task4/轨迹跟踪
├── models/                                # MuJoCo 模型 + task4 权值
└── docs/                                  # 任务 4 详细文档
```

## 文档

- [任务 4 端到端视觉伺服](docs/rov_end_to_end.md)

## 已知限制

- 训练与推理为纯 NumPy/CPU；任务 4 单帧推理 0.7–3.9 ms，训练约 8 分钟。
- 任务 4 的行为克隆数据只覆盖管道走廊（`|dy|≤0.6 m`、`|yaw|≤0.28 rad`），
  超出范围属分布外；运行时保留 `|yaw|>0.25 rad` 的航向安全兜底。
- WSL 下 osmesa 渲染较慢，launch 的相机频率约 3.6 Hz（单帧渲染约 0.28 s）。
- `--task4` 3D 视窗已在 WSLg 软件渲染下短时验证（视窗打开、控制正常，推理约 374 ms）；
  受帧率限制未跑完整巡检。
- Ubuntu 20.04 / Noetic 未实测。
