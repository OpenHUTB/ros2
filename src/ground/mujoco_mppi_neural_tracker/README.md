# mujoco_mppi_neural_tracker — MPPI 与蒸馏神经网络：8 字赛道轨迹跟踪

同一台 MuJoCo 阿克曼小车、同一条 8 字赛道、同一份带噪感知，比较三种控制器：

| 控制器 | 是什么 | 计算量 |
| --- | --- | --- |
| `pp` | 纯追踪（几何基线） | ≈0.03 ms/步 |
| `mppi` | **MPPI 采样 MPC**：512 条轨迹 × 25 步滚动优化 | ≈13 ms/步 |
| `nn` | **蒸馏网络**：把 MPPI 的策略压进 2 978 参数的 MLP | ≈0.15 ms/步 |

回答的问题很具体：**把 MPC 算力换成网络推理，精度掉多少、鲁棒性掉多少？**

## 目录

```
mujoco_mppi_neural_tracker/
├── main.py                     # --demo / --bench / --test / --ros
├── main.sh / main.bat
├── models/track.xml            # 8 字场地 + 阿克曼小车 + 两个相机
├── models/policy.npz           # 蒸馏网络权重（13 KB，随包发布）
├── mppi_nn/
│   ├── track.py                # Gerono 双纽线参考线、锥桶、横向/航向误差
│   ├── env.py                  # MuJoCo 环境：速度伺服、解析雷达、带噪里程计
│   ├── mppi.py                 # MPPI 控制器 + 纯追踪基线
│   ├── net.py                  # 12→64→32→2 MLP（手写反向传播 + Adam）
│   ├── nav.py                  # 位姿估计（里程计+雷达修正）、闭环、专家采样
│   ├── train.py                # 模仿学习、DAgger、出图出 GIF
│   ├── sweep.py                # 控制器 × 感知噪声 的对照实验
│   └── node.py                 # ROS 2 节点
├── launch/main.launch.py
├── config/params.yaml
└── test/test_mppi_nn.py        # 26 个单元测试
```

## 运行环境

| 项 | 版本 |
| --- | --- |
| 系统 | Ubuntu 20.04（ROS 2 Humble）/ Windows 10、11 |
| Python | 3.8 / 3.10 |
| 依赖 | `numpy`、`mujoco>=3.1`、`matplotlib`、`pillow`、`pytest` |

```bash
cd src/ground/mujoco_mppi_neural_tracker
pip install -r requirements.txt
python main.py --demo        # 采样 + 蒸馏 + 三控制器对照 + 噪声扫描，约 2 min
python main.py --test        # 26 passed
```

> 课程虚拟机里没有 mujoco，且 rosdep 无此键：先 `python3 -m pip install mujoco>=3.1`（无外网时用宿主机
> `pip download --python-version 38 --platform manylinux2014_x86_64 --abi cp38` 带进去装）。

## ROS 2

```bash
colcon build --packages-select mujoco_mppi_neural_tracker
source install/setup.bash
ros2 launch mujoco_mppi_neural_tracker main.launch.py                    # 默认 nn
ros2 launch mujoco_mppi_neural_tracker main.launch.py controller:=mppi  # 切 MPPI
```

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/car/scan` | `sensor_msgs/LaserScan` | 32 线锥桶雷达，`frame_id=laser` |
| `/car/odom` | `nav_msgs/Odometry` | 编码器 + 陀螺估计，`frame_id=map` |
| `/car/cmd` | `geometry_msgs/Twist` | `linear.x`=速度，`angular.z`=偏航率 |
| `/car/steering` | `std_msgs/Float32` | 前轮转角 δ（rad） |
| `/car/status` | `std_msgs/String` | `ctrl/step/progress/e/cmd` |
| `/tf` | `tf2_msgs/TFMessage` | `map→base_link`、`base_link→laser` |

## 实测（`python main.py --demo`，CPU）

| 控制器 | 横向 RMS | 最大误差 | 完成 | 步数 | 耗时/步 |
| --- | --- | --- | --- | --- | --- |
| 纯追踪 | 0.046 m | 0.096 m | 是 | 807 | 0.05 ms |
| **MPPI** | **0.029 m** | **0.066 m** | 是 | 828 | 11.42 ms |
| **蒸馏网络** | **0.028 m** | 0.075 m | 是 | 834 | 0.12 ms |

感知噪声扫描（横向测量噪声 0 / 1 / 2 档）：

| 控制器 | 噪声 0 | 噪声 1 | 噪声 2 |
| --- | --- | --- | --- |
| 纯追踪 RMS | 0.052（0.069） | 0.057（0.074） | 0.064（0.084） |
| MPPI RMS | 0.038（0.047） | 0.050（0.058） | **0.146（0.429，1/5 未完成）** |
| 网络 RMS | 0.037（0.047） | 0.046（0.053） | **0.064（0.067，5/5 完成）** |

结论：**MPPI 精度最高但算力是网络的约 95 倍；一旦感知噪声变大，MPPI 每步重规划会把噪声直接吃进
代价函数，5 个种子里出现 1 次跑出赛道（最差 0.429 m），蒸馏网络因为输出平滑 5/5 完成（最差 0.067 m）。**

## 设计要点

1. **感知真的参与控制**：控制器吃的是"里程计 + 雷达横向修正"的估计位姿，不是真值；噪声可以加大。
2. **雷达是解析的**：锥桶是已知半径的圆柱，射线-圆柱求交用 numpy 向量化，比 32 次 `mj_ray` 快约 20 倍（有单测与暴力解对照）。
3. **确定性**：MPPI 与回合随机源都由 seed 固定，同一配置重复跑结果完全一致（有单测守护）。
4. 原理公式、算法流程、源码解析和"为什么网络比 MPC 更抗噪"的分析见 `docs/ground/mujoco_mppi_neural_tracker.md`。
