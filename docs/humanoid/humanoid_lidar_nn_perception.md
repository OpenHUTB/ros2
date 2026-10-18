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
> - PPO 强化学习版本作为训练证据保留（`weights/avoid_ppo.zip`），但因 20 万步内 value function
>   未收敛，默认未启用。

## 一、项目概览

| 项 | 内容 |
|---|---|
| 机器人 | Unitree G1 29 自由度人形 |
| 物理引擎 | MuJoCo 3.3.7（OSMesa 离屏渲染） |
| 激光雷达 | 360 束水平扫描，量程 0.1–8 m，10 Hz，基于 `mujoco.mj_ray` |
| 感知网络 | 1D-CNN（3 层 Conv1d，64 通道），输入 scan[360]，输出 [nearest, front, left, right] / 8 |
| 避障控制器 | BC-MLP（6-32-32-2），输入 CNN 特征，输出 (vx, wz)；PPO 训练证据保留但未启用 |
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
| PPO 训练证据 | 20 万步，600 fps，未收敛（保留 avoid_ppo.zip 供参考） |

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

## 六、文件结构

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
│   ├── perception_cnn.pt       # 训练好的 CNN
│   └── avoid_ppo.zip            # PPO 训练证据（未默认启用）
└── scripts/
    ├── collect_data.py          # 数据采集
    ├── train_perception_cnn.py  # CNN 训练
    ├── train_avoid_ppo.py       # PPO 训练
    └── record_gif.py           # gif / png 渲染
```
