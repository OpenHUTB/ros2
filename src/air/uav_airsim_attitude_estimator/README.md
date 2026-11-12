# uav_airsim_attitude_estimator

用神经网络从机载 IMU 时序直接回归无人机的 **roll / pitch 姿态角**，在 AirSim（Blocks 场景）中完成数据采集、训练、实飞演示与 ROS2 节点封装。

- 输入：IMU 最近 5 帧读数，每帧 6 轴（ax, ay, az, gx, gy, gz），共 30 维
- 网络：纯 NumPy 三层 MLP `30 → 32 → 2`
- 输出：`roll`、`pitch`（弧度）
- 监督标签：AirSim 姿态四元数解算出的欧拉角，只用于训练，推理时网络只接触 IMU

## 效果

| 阶段 | 指标 | 数值 |
|---|---|---|
| 采集 | 样本数 | 660 条（6 回合 × 110 步，飞行中采样） |
| 采集 | 标签范围 | roll −0.053 ~ 0.046 rad，pitch −0.179 ~ 0.118 rad |
| 训练 | 验证 MSE | 5.85e-4 rad² |
| 训练 | 验证 MAE | 0.0175 rad（1.0°） |
| 训练 | 验证 R² | **0.631** |
| 实飞 | 演示 MAE | **0.9°**（roll 0.018 rad / pitch 0.015 rad，150 步螺旋航线） |
| 实飞 | 演示 gif | 3.9 MB，320×240，160 帧 |

离线与在线误差基本一致（1.0° vs 0.9°），说明采集与演示用的是同一套飞行状态，没有分布偏移。

## 与仓库内其它无人机模块的区别

| 模块 | 传感器 | 输出 |
|---|---|---|
| `uav_airsim_depth_obstacle_avoidance` | 前视深度 64×64 | 偏航角速率 + 目标高度（控制量） |
| `uav_airsim_imu_velocity_estimator` | IMU 5 帧 × 6 轴 | 速度标量（1 维） |
| `uav_airsim_wind_rejection` | IMU + 位置误差 | 补偿速度（控制量） |
| `uav_airsim_ttc_predictor` | 前视深度 64×64 | 碰撞剩余时间（安全指标） |
| **本模块** | IMU 5 帧 × 6 轴 | **roll / pitch 姿态角（2 维）** |

与 `imu_velocity_estimator` 虽然共用 IMU，但输出量不同：那个回归速度标量，本模块回归姿态角度，是姿态解算而非速度估计。二者互补，可拼成完整的 IMU 状态估计。

## 目录结构

```
uav_airsim_attitude_estimator/
├── main.py                                   统一入口（--collect / --train / --demo）
├── uav_airsim_attitude_estimator/
│   ├── __init__.py
│   ├── env.py        AirSim 客户端封装、IMU 读取、四元数转欧拉角、飞行控制
│   ├── mlp.py        纯 NumPy MLP + Adam 优化器
│   ├── dataset.py    飞行中采集 IMU 窗与真值姿态
│   ├── train.py      输入标准化 + 标签 z-score + 逐样本训练
│   ├── demo.py       螺旋航线实飞演示并录 gif
│   └── node.py       ROS2 节点（订阅 /uav/imu，发布 /uav/attitude）
├── launch/main.launch.py
├── test/test_mlp.py                          不依赖 AirSim/ROS 的自检与复算
├── models_trained/
│   ├── attitude.npz          权重 + 标准化参数
│   └── attitude_data.npz     660 条训练样本
├── package.xml / setup.py / setup.cfg / requirements.txt / resource/
└── README.md
```

## 用法

```bash
# 依赖（numpy / Pillow / airsim==1.8.1）
pip install -r requirements.txt

# 自检：不需要 AirSim，约 2 秒，5/5 通过
python3 test/test_mlp.py

# 采集（需要先启动 AirSim Blocks）
python3 main.py --collect --episodes 6 --steps 110

# 训练
python3 main.py --train --epochs 120 --lr 0.001

# 实飞演示并生成 demo.gif
python3 main.py --demo --steps 150 --gif demo.gif

# ROS2 节点（weights 参数可给绝对路径，相对路径会按包根目录解析）
ros2 launch uav_airsim_attitude_estimator main.launch.py
```

节点订阅 `sensor_msgs/Imu` 类型的 `/uav/imu`，发布 `std_msgs/Float32MultiArray` 类型的 `/uav/attitude`，`data[0]` 是 roll、`data[1]` 是 pitch（弧度）。

## 已知局限

1. **roll 幅度小**。Blocks 场景下 `moveByVelocityAsync` 的侧倾只有 ±3°，roll 标签方差远小于 pitch，因此 roll 的估计误差占总误差的比重更高，R² 0.631 主要被 roll 拉低。
2. **过拟合明显**。训练 MSE 与验证 MSE 差一个量级，660 条样本对 30 维输入偏少；继续加样本或加 dropout 能改善，但当前按验证 MSE 留最优已保证了不使用过拟合权重。
3. **只在低机动飞行下验证**。采集和演示的速度都在 0.4~2.6 m/s、偏航速率 ±0.8 rad/s，大机动（急停、快速横滚）下的姿态未覆盖。
4. **IMU 是理想传感器**。AirSim 的 `getImuData` 不含真实 IMU 的温漂、量化噪声和标定误差；若要迁移到实机需要额外加噪训练。

## 环境

- AirSim v1.8.1（Blocks，WindowsNoEditor）
- 依赖见 `requirements.txt`：numpy / Pillow / airsim==1.8.1（`pip install -r requirements.txt`）
- 实测 Python 3.13 + numpy 2.5.4 + pillow 12.3.0 + airsim 1.8.1
- 提交目标环境：Ubuntu 20.04, ROS 2 Humble

## 图与复现产物

- `docs/img/uav/attitude/demo.gif` —— 实飞演示动图（160 帧）
- `docs/img/uav/attitude/holdout_compare.png` —— 留出集 99 条的预测-真值对照（时间序列 + 散点）
- `docs/img/uav/attitude/terminal_selfcheck.png` —— `python3 test/test_mlp.py` 实跑的终端输出（5/5 通过）

以上三张图都由 `docs/air/uav_airsim_attitude_estimator.md` 引用。
