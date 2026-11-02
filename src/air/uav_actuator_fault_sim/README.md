# uav_actuator_fault_sim —— 无人机执行器与传感器故障注入仿真层

本包在 [uav_mujoco_sim](../uav_mujoco_sim/README.md) 的 MuJoCo 四旋翼底座上增加**可编程故障注入**
能力：让某一片桨在指定时刻完全失效、推力衰减、卡死在固定推力或渐进退化，也可以给 IMU 加零偏、
噪声与丢包。所有故障都作用在"控制器下发推力 → 实际推力"这一层，因此对上层控制器完全透明，
可以拿任意控制器来接（几何控制器、重分配控制器、神经网络策略）。

一句话定位：**它是后续所有"神经容错飞行"模块的数据发生器与评测台**，单独完成课程任务①
（物理仿真 + 键盘运动控制）。

## 为什么需要它

单桨完全失效是四旋翼最难的故障之一。本包实测（`open_ground`，悬停 2 m 时注入，见
`results/comparison.csv`）：

| 情形 | 最低高度 | 故障→坠地 | 触地速度 |
| --- | --- | --- | --- |
| 健康 | 1.500 m | 不坠地 | — |
| 单桨推力衰减 60% | 1.208 m | 不坠地 | — |
| 单桨完全失效，无补偿 | **−109.8 m** | **0.64 s** | 5.13 m/s |
| 单桨完全失效，损失推力均摊给其余三桨 | −67.9 m | 0.68 s | — |

两条结论驱动了整个选题：①完全失效必须坠机，现象稳定可复现，适合做"坠机 vs 存活"的对照演示；
②**朴素的均摊补偿救不回来**，说明这个坑需要真正的方法（检测 + 加权重分配 + 残差学习）去填。

## 目录结构

```
uav_actuator_fault_sim/
├── main.py                    # 入口：进程内定位包并转发到 cli
├── main.sh / main.bat         # 双击即跑（无参数默认 --demo）
├── setup.py / setup.cfg / package.xml
├── config/faults.yaml         # ROS 节点参数
├── launch/fault_sim.launch.py # ros2 launch 入口
├── resource/uav_actuator_fault_sim
├── test/                      # pytest：故障算子 / 场景矩阵 / 指标 / 仿真闭环
└── uav_actuator_fault_sim/
    ├── fault.py        # 故障算子：RotorFault / SensorFault / FaultSpec / FaultInjector
    ├── scenario.py     # 场景与用例矩阵：5 物理场景 × 7 故障 × seeds
    ├── runner.py       # 闭环仿真：控制器 → 注入 → MuJoCo → 记录
    ├── metrics.py      # FlightResult 与指标聚合
    ├── evaluate.py     # 对比表 / 完整矩阵 / CSV 输出
    ├── render.py       # 并排对比动图（健康 vs 故障）
    ├── keyboard_teleop.py  # 终端键盘直飞（任务①）
    ├── ros_node.py     # ROS2 节点：发布 odom/status/health，订阅 cmd_vel/goal
    ├── selftest.py     # 内置自检
    ├── cli.py          # argparse 分发
    └── _compat.py      # 定位并导入底座（不改底座一行代码）
```

## 运行环境

| 项 | 要求 |
| --- | --- |
| Python | ≥ 3.10（仿真侧）；ROS2 节点需 **Humble**（自带 Python 3.10） |
| 依赖 | `mujoco >= 3.0`、`numpy >= 1.21`、`pillow`（录 gif）、`pytest`（跑测试） |
| 底座 | 同仓库 `src/air/uav_mujoco_sim`（不需要安装，源码树里直接可见即可） |
| 图形 | **不需要独显、不需要 Gazebo/Unreal**，MuJoCo 自带的离屏渲染即可 |

```bash
pip install "mujoco>=3.0" numpy pillow pytest
```

## 快速开始

在 `src/air/uav_actuator_fault_sim/` 目录下：

```bash
# 1. 自检：故障算子 + 用例矩阵 + 闭环飞行，14 项全过
python main.py --selftest

# 2. 对比表：同一场景下 健康 vs 6 种故障
python main.py --demo

# 3. 完整矩阵（5 场景 × 7 故障 = 35 条）写 CSV
python main.py --eval results/comparison.csv

# 4. 录一张并排对比动图（左健康、右单桨失效）
python main.py --gif results/fault_comparison.gif --seconds 6

# 5. 终端键盘直飞（w/s 前后、a/d 左右、r/f 升降、q/e 偏航、空格清零）
python main.py --keyboard --fault healthy
python main.py --keyboard --fault single_loss --onset 5
```

Windows 直接双击 `main.bat`，Linux/Mac 用 `./main.sh`（无参数默认 `--demo`）。

## ROS2 启动

```bash
colcon build --packages-select uav_mujoco_sim uav_actuator_fault_sim --parallel-workers 4
source install/setup.bash
ros2 launch uav_actuator_fault_sim fault_sim.launch.py
# 或直接
ros2 run uav_actuator_fault_sim fault_node --ros-args -p fault:=single_loss -p onset:=2.0
```

launch 参数可覆盖 `scene / seed / fault / rotor / onset / severity / controller / seconds`。

### 话题

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 发布 | `/uav/odom` | `nav_msgs/Odometry` | 世界系 ENU 位姿与速度 |
| 发布 | `/uav/status` | `std_msgs/String` | `READY / HOVER / VELOCITY / GOAL / FAULT / CRASHED / DONE` |
| 发布 | `/uav/fault` | `std_msgs/String` | 当前故障描述（如 `完全失效@桨1`） |
| 发布 | `/uav/health` | `std_msgs/Float32MultiArray` | 四片桨的故障增益真值，供检测器监督学习 |
| 发布 | `/tf` | `tf2_msgs/TFMessage` | `odom → base_link`（可选） |
| 订阅 | `/uav/cmd_vel` | `geometry_msgs/Twist` | 机体系速度指令，0.5 s 无指令自动悬停 |
| 订阅 | `/uav/goal` | `geometry_msgs/Point` | 目标点 |

## 故障模型

统一写成"实际推力 = 增益 × 指令 + 偏置"：

$$T_i^{\text{act}}(t) = g_i(t)\,u_i^{\text{cmd}}(t) + b_i(t), \qquad T_i^{\text{act}} \in [0, T_{\max}]$$

| 类型 | $g_i(t)$ | $b_i(t)$ | 用途 |
| --- | --- | --- | --- |
| `loss` 完全失效 | $1-\alpha(t)$ | 0 | 桨停转，最致命 |
| `degradation` 推力衰减 | $1+(s-1)\alpha(t)$ | 0 | 桨叶损伤、电机效率下降 |
| `stuck` 卡死 | $1-\alpha(t)$ | $s\,T_{\max}\alpha(t)$ | 电调卡在固定占空比 |
| `progressive` 渐进退化 | $\max(0.05,\ 1-\dot\lambda (t-t_0))$ | 0 | 缓慢恶化，检验预测能力 |

其中 $\alpha(t)=\min\left(1, \frac{t-t_0}{r}\right)$ 是注入过渡（`ramp`，$r=0$ 时直接生效）。

传感器故障 `SensorFault(gyro_bias, noise_scale, dropout)` 作用于 IMU 读数，用于后续做
"传感器 + 执行器复合故障"的鲁棒性实验。

## 复现实验的命令与产物

- `results/comparison.csv` —— 35 条用例的指标表（`python main.py --eval`）
- `results/fault_comparison.gif` —— 并排演示动图（`python main.py --gif`）
- `python -m pytest test -q` —— 45 个单元/闭环测试

## 设计约束

- **不修改底座**：只调用 `uav_mujoco_sim` 的公开 API（`Quadrotor`、`GeometricController`、
  `mixer`、`make_scene`、`VisionSensors`…），底座作者正在并行提同一系列的 PR，改动必然冲撞。
- **不 import MuJoCo 也能 `import uav_actuator_fault_sim`**：故障算子与指标只依赖 numpy，
  便于在没有仿真环境的机器上做单元测试。
- **确定性**：同一 `seed` 的用例可复现（`test_runner.py::test_determinism_same_seed_same_result`）。
