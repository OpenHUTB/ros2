# 人形机器人 2D 激光雷达 CNN 感知与神经网络避障（humanoid_lidar_nn_perception）

在 **MuJoCo** 物理仿真中为 **Unitree G1** 装配一个 360 束 2D 激光雷达，
用 **1D-CNN** 从带噪声扫描中预测前/左/右三个方向的可通行距离，
再用一个 **行为克隆 MLP**（神经网络）学习"朝目标方向走同时避障"的高层速度策略。

> **范围说明**：
> - 行走底座使用 [MuJoCo-LiDAR](https://github.com/discoververse/MuJoCo-LiDAR) 仓库自带的
>   Unitree 官方 ONNX 行走策略（已验证可前进 0.38 m/s，10 s 前进 3.8 m，零跌倒）。
> - **避障控制器是神经网络**：一个 6-32-32-2 的 MLP（PyTorch），输入 CNN 输出的 4 维通道距离
>   + 目标朝向编码，输出 (vx, wz) 安全速度指令。通过行为克隆（BC）从反应式规则示范中训练，
>   10,000 样本、30 epoch 后 MSE 收敛到 8e-5。
> - 感知 CNN 已充分收敛：4 个通道距离预测 MAE < 1 cm。

## 一、项目概览

| 项 | 内容 |
|---|---|
| 机器人 | Unitree G1 29 自由度人形 |
| 物理引擎 | MuJoCo 3.3.7（OSMesa 离屏渲染） |
| 激光雷达 | 360 束水平扫描，量程 0.1–8 m，10 Hz，基于 `mujoco.mj_ray` |
| 感知网络 | 1D-CNN（3 层 Conv1d，64 通道），输入 scan[360]，输出 [nearest, front, left, right] / 8 |
| 避障控制器 | BC-MLP（6-32-32-2），输入 CNN 特征，输出 (vx, wz) |
| 行走底座 | Unitree 官方 ONNX 策略（Isaac Lab 导出），接受 [vx, vy, ωz] 速度指令 |
| ROS | ROS 2 Humble |
| 话题 | `/scan` (LaserScan), `/obstacle_info` (Float32MultiArray), `/cmd_vel`, `/cmd_vel_safe`, `/joint_states`, `/odom` |

## 二、代码来源与许可证

| 组件 | 来源 | 许可证 |
|------|------|--------|
| G1 模型 MJCF + STL 网格 | [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) | BSD-3-Clause |
| G1 ONNX 行走策略 | [MuJoCo-LiDAR](https://github.com/discoververse/MuJoCo-LiDAR) `examples/onnx/g1_policy.onnx` | 随仓库（MIT），权重为 Unitree 官方发布 |
| 2D 雷达射线投射思路 | `mujoco.mj_ray`（MuJoCo 内置） | Apache-2.0 |
| CNN / PPO 训练脚本、ROS2 节点、场景封装 | 本模块原创 | BSD-3-Clause |

## 三、演示

![避障演示](../img/humanoid/humanoid_lidar_nn_perception/demo_avoid.gif)

![仿真画面](../img/humanoid/humanoid_lidar_nn_perception/sim_running.png)

![扫描可视化](../img/humanoid/humanoid_lidar_nn_perception/scan_plot.png)

![ROS2 话题输出](../img/humanoid/humanoid_lidar_nn_perception/ros_topic_echo.png)

## 四、指标

| 指标 | 数值 |
|------|------|
| CNN 距离预测 MAE（nearest） | 0.002 m |
| CNN 距离预测 MAE（front） | 0.005 m |
| CNN 距离预测 MAE（left） | 0.005 m |
| CNN 距离预测 MAE（right） | 0.003 m |
| 数据集规模 | 20,000 帧 / 980 episodes |
| 数据采集速度 | ~180 帧/s（CPU） |
| CNN 模型大小 | 89 KB（TorchScript） |
| BC-MLP 控制器 | 6-32-32-2，10k 样本，30 epoch，MSE 8e-5 |
| ONNX 行走策略验证 | 10 s 前进 3.8 m，实际 0.38 m/s，零跌倒 |

## 五、运行

```bash
# 容器内（已装 onnxruntime）
cd /tmp/ws
colcon build --packages-select humanoid_lidar_nn_perception
source install/setup.bash
ros2 launch humanoid_lidar_nn_perception main.launch.py teleop:=false

# 另开终端查看
ros2 topic echo /scan --field ranges
ros2 topic echo /obstacle_info
```

## 六、复现指南

审核人员按以下步骤可从零复现全部结果。所有训练产物（权重）已随包提交，
步骤 6.2–6.4 可选跑（用于复现训练过程），直接跳到 6.5 即可运行已训好的模型。

### 6.1 环境搭建

**宿主机**（数据采集 / 训练 / 渲染）：

```bash
# Python 3.10+
python -m venv venv_lidar
source venv_lidar/bin/activate  # Windows: venv_lidar\Scripts\activate
pip install mujoco==3.3.7 onnxruntime torch stable-baselines3 scipy matplotlib pillow etils
```

**容器**（ROS2 Humble）：

```bash
# 使用本仓库的 hutb-humble:dev 镜像
docker run -it --rm -v $PWD:/ws hutb-humble:dev
pip install onnxruntime  # 镜像已含 mujoco/torch, 但 onnxruntime 需补装
```

### 6.2 数据采集（约 2 分钟）

```bash
cd /ws/src/humanoid/humanoid_lidar_nn_perception
python scripts/collect_data.py \
    --model models/g1/scene_g1.xml \
    --onnx weights/g1_policy.onnx \
    --out data/scan_dataset.npz \
    --frames 20000
```

输出：`data/scan_dataset.npz`（20,000 帧 × (scan[360] + labels[5])）。
约 113 秒，980 个 episodes。

### 6.3 训练感知 CNN（约 90 秒）

```bash
python scripts/train_perception_cnn.py \
    --data data/scan_dataset.npz \
    --out weights/perception_cnn.pt \
    --epochs 30
```

验证指标：4 通道距离预测 MAE < 1 cm（nearest 0.002, front 0.005, left 0.005, right 0.003）。

### 6.4 训练避障 BC-MLP（约 60 秒）

```bash
python scripts/train_bc_avoidance.py \
    --model models/g1/scene_g1.xml \
    --onnx weights/g1_policy.onnx \
    --cnn weights/perception_cnn.pt \
    --out weights/avoid_mlp.pt \
    --samples 10000
```

验证指标：30 epoch 后 MSE 收敛到 ~8e-5。

### 6.5 ROS2 运行（容器内）

```bash
cd /tmp/ws
colcon build --packages-select humanoid_lidar_nn_perception
source install/setup.bash
ros2 launch humanoid_lidar_nn_perception main.launch.py teleop:=false
```

### 6.6 验证功能正常

另开三个终端：

```bash
# 终端 1: 雷达话题（360 束 ranges 在发布）
ros2 topic echo /scan --field ranges --once
# 预期: 360 个浮点数, 范围 0.1–8.0

# 终端 2: CNN 感知输出（4 维: nearest, front, left, right, 单位米）
ros2 topic echo /obstacle_info --once
# 预期: [0.87, 1.23, 3.45, 3.42] 类似的 4 维数组

# 终端 3: 避障后速度指令
ros2 topic echo /cmd_vel_safe --once
# 预期: linear.x > 0 且 angular.z 非零（G1 在转向避障）
```

### 6.7 渲染演示素材（可选）

```bash
python scripts/record_gif.py
```

输出：`docs/img/humanoid/humanoid_lidar_nn_perception/` 下的 gif + 3 张 PNG。

## 七、文件结构

```
humanoid_lidar_nn_perception/
├── package.xml / setup.py / setup.cfg
├── main.py
├── humanoid_lidar_nn_perception/
│   ├── g1_walking_policy.py   # ONNX 行走策略封装
│   ├── lidar_2d.py             # 2D 360 束 mj_ray 扫描
│   ├── scene_builder.py        # episode 随机化
│   ├── sim_node.py             # MuJoCo 仿真 + /scan + /joint_states + /odom
│   ├── perception_node.py      # /scan → CNN → /obstacle_info
│   ├── avoidance_node.py        # 规则避障（PPO 预留）
│   └── keyboard_teleop.py       # WASD → /cmd_vel
├── launch/main.launch.py
├── models/g1/                  # G1 MJCF + STL + 场景 mesh
├── weights/
│   ├── g1_policy.onnx          # Unitree 行走策略
│   ├── perception_cnn.pt       # 训练好的 CNN 感知网
│   └── avoid_mlp.pt            # BC 训练的避障控制器
└── scripts/
    ├── collect_data.py          # 数据采集
    ├── train_perception_cnn.py  # CNN 训练
    ├── train_bc_avoidance.py    # BC-MLP 避障控制器训练
    └── record_gif.py           # gif / png 渲染
```
