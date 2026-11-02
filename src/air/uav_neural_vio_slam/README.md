# uav_neural_vio_slam — 神经视觉惯性里程计与三后端 SLAM 对比

对应作业任务（2）传感器感知与任务（3）建图导航。对比实验①：
**同一模型、不同 SLAM 算法**。

前端是 CNN + GRU 的神经 VIO（纯 numpy 手写前向与反向，不依赖深度学习框架），
输入深度图与 IMU 序列，输出相对位姿增量；三个后端共用同一前端、同一数据集，
只换后端：**EKF-SLAM**、**学习式观测噪声的 EKF-SLAM**、**FastSLAM 1.0**。

依赖 [uav_mujoco_sim](../uav_mujoco_sim) 提供 `/camera/depth`、`/lidar/points`、
IMU 与真值位姿。

---

## 1. 与课程已有模块的区别

| 已有模块 | 区别 |
|---|---|
| `octree_uav_3d_pathfinding` | 直接用仿真器给的真值位姿建图，没有状态估计、没有 SLAM 后端 |
| `carla_slam_gmapping` | 是车；gmapping 为黑箱，做不了"同前端换后端"的受控对比 |

课程内尚无"状态估计与建图分开评测"的工作。

## 2. 架构

```
深度图 ─┐
        ├─→ CNN(2 层) ─┐
IMU ────┘              ├─→ GRU ─→ ┬─ head_p  → Δp_b
                       │          ├─ head_r  → Δrpy_b
                       │          └─ head_s  → log σ²(6)
                       │
                       └─→ ┬ EKF-SLAM         （常数 R）
                           ├ EKF-SLAM + 神经观测模型（R 由网络按观测几何给出）
                           └ FastSLAM 1.0     （粒子滤波 + 逐粒子路标）
                                    │
                                    └─→ 位姿 + 路标 → 占用栅格
```

前端与后端之间只有 `(Δp_b, Δrpy_b, σ)` 一条接口，换后端不改前端一行。

## 3. 安装

运行环境：ROS2 Humble（Ubuntu 20.04，Python 3.8）+ MuJoCo 3.12.0（仿真侧 Python ≥3.10）。

```bash
cd ~/uav_ws/ros2_ws/src
cp -r <本目录> .

# 仿真侧（需要 Python ≥3.10）
uv venv ~/uav_ws/.venv --python 3.12
uv pip install --python ~/uav_ws/.venv/bin/python 'mujoco==3.12.0' numpy

# ROS 侧（Python 3.8）
cd ~/uav_ws/ros2_ws
colcon build --packages-select uav_mujoco_sim uav_neural_vio_slam
source install/setup.bash
```

`core/` 与 `plotting.py` 只依赖 numpy 与 matplotlib，没有 mujoco 的机器也能跑
`--evaluate` 与 `--figures` 复现已有数据集的全部结论。

## 4. 运行步骤

### 4.1 先跑自检

不需要 ROS、不需要显示器：

```bash
cd ~/uav_ws/ros2_ws/src/uav_neural_vio_slam
python main.py --selftest
```

### 4.2 采集与训练（需要 mujoco）

```bash
python main.py --collect  --out data/vio_dataset.npz
python main.py --train    --data data/vio_dataset.npz --out data/
```

采集 5 集八字闭环航线（每集 800 帧 @10 Hz），训练前端与观测噪声模型，
权重写入 `data/vio_net.npz` 与 `data/obs_model.npz`。

### 4.3 后端对比

```bash
# 只需要 numpy，可在任何机器上复现
python main.py --evaluate --data data/vio_dataset.npz --model data/ \
               --out results.json --outage 5,15,30
```

### 4.4 出图

```bash
python main.py --figures --data data/vio_dataset.npz --model data/ \
               --outdir docs/img/air/uav_neural_vio_slam --outage 5,15,30
```

### 4.5 ROS2 在线运行

```bash
# 终端 1：仿真 + 桥接 + VIO 节点（一条命令起全套）
ros2 launch uav_neural_vio_slam vio.launch.py with_sim:=true scene:=urban_block

# 只起 VIO 节点（假定 /camera/depth 与仿真端已在别处运行）
ros2 launch uav_neural_vio_slam vio.launch.py backend:=ekf_learned
```

`backend` 取 `odom` / `ekf` / `ekf_learned` / `fastslam`。

## 5. 与底座的接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/camera/depth` | sensor_msgs/Image（32FC1，米） | 前端输入 |
| 发布 | `/vio/odom` | nav_msgs/Odometry | 估计位姿（局部系） |
| 发布 | `/vio/landmarks` | visualization_msgs/MarkerArray | 估计路标 |
| 发布 | `/vio/map` | nav_msgs/OccupancyGrid | 占用栅格 |
| 发布 | `/vio/status` | std_msgs/String | 后端、帧数、速率、路标数 |
| 发布 | `/imu/data` | sensor_msgs/Imu | 见下 |
| 发布 | `/tf` | tf2_msgs/TFMessage | `map → base_link` |

**IMU 来源**：底座的桥接节点只在内部取用 IMU，不发布话题；本节点另开一条到
仿真端的 TCP 控制通道，以 100 Hz 轮询并转发为 `/imu/data`，其他模块可直接复用。

## 6. 文件结构

```
uav_neural_vio_slam/
├── main.py                      入口（作业要求以 main 开头）
├── uav_neural_vio_slam/
│   ├── cli.py                   采集 / 训练 / 评估 / 出图 / 自检
│   ├── selftest.py              37 项自检，无 ROS
│   ├── plotting.py              配图与动图
│   ├── core/
│   │   ├── nn.py                手写 Conv2d / GRUCell / Linear / Adam
│   │   ├── network.py           VioNet（CNN + IMU 分支 + GRU + 三个头）
│   │   ├── world.py             SLAM 基准世界（30 立柱）+ 八字闭环航线
│   │   ├── landmarks.py         激光雷达 → (距离, 方位) 观测
│   │   ├── dataset.py           数据采集与存取
│   │   ├── training.py          训练与在线推理
│   │   ├── ekf_slam.py          EKF-SLAM（全图联合协方差）
│   │   ├── fastslam.py          FastSLAM 1.0
│   │   ├── obs_model.py         学习式观测噪声模型
│   │   ├── mapping.py           占用栅格（向量化射线投射）
│   │   ├── metrics.py           ATE / RPE / 路标误差 / 地图 IoU
│   │   └── experiment.py        四后端对比与 GPS 中断
│   └── ros/vio_node.py          ROS2 在线节点
├── config/vio.yaml              参数
├── launch/vio.launch.py         启动入口
├── data/                        数据集与预训练权重（离线复现）
└── test/                        pytest（41 项）
```

## 7. 实测结果

仿真侧 Windows 11 + Intel Core 5 220H（核显），ROS 侧 Ubuntu 20.04 虚拟机，
5 集八字闭环航线（每集 80 s @10 Hz），GPS 每 1 s 一次、σ=0.5 m。

（数据见 `docs/air/uav_neural_vio_slam.md`，由 `--evaluate` 直接生成。）

## 8. 已知限制

- 观测模型针对立柱（圆柱面），不是通用特征；换到真实点云需重训检测器。
- `--collect` 需要 mujoco，只能在 Python ≥3.10 下跑；ROS 节点只做推理，Python 3.8 即可。
- 深度图在 ROS 侧按面积平均下采样到 48×64 以匹配训练分辨率；`launch` 里已把
  仿真相机直接设成 64×48，避免训练与部署分辨率不一致。
- 前端网络在 CPU 上单帧推理约 3 ms（48×64 深度 + 10×6 IMU），满足 10 Hz 关键帧。
