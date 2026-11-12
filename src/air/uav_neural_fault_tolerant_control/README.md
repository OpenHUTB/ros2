# 无人机桨故障下的神经容错控制

`uav_neural_fault_tolerant_control` —— 在 [MuJoCo 四旋翼仿真与 ROS2 桥接底座](../uav_mujoco_sim/README.md)
上做"桨坏了还能不能飞"的闭环控制：底座控制器算出期望合推力与力矩，本模块负责把它**按故障后的
可用推力重新分配**到四个桨上，再用一个很小的网络修正分配误差。

对应课程任务②的"给定轨迹控制"部分。本模块是"神经容错飞行"系列的第 3 个：

| 模块 | 角色 |
| --- | --- |
| `uav_actuator_fault_sim` | 故障注入层，产出带故障的飞行与评测指标 |
| `uav_rotor_fault_detector_nn` | 从 IMU 与指令估计四桨健康度 |
| **`uav_neural_fault_tolerant_control`** | **拿到健康度后怎么分配推力、怎么控制** |

## 为什么需要一个"分配层"

几何控制器输出四个期望量（合推力 `T` 与三轴力矩 `τ`），底座用固定的逆混控把它们摊到四个桨上。
桨一旦退化，这个摊法就错了：

| 工况（悬停 1.5 m，t=2.0 s 注入故障） | 几何控制器 | 朴素补偿（丢的推力均摊） | 本模块（加权分配 + 神经残差） |
| --- | --- | --- | --- |
| 单桨推力衰减 50% | 最低 1.377 m，回不到 1.5 m | **3.17 s 坠机** | 1.500 m，RMSE 0.003 m |
| 单桨卡死（固定输出） | 勉强保持 1.5 m | **6.13 s 坠机** | 1.500 m，RMSE 0.003 m |
| 单桨完全失效 | 2.84 s 坠机 | 2.85 s 坠机 | 3.27 s 坠机（跟踪误差 0.605，比加权分配小一半） |

## 方法

1. **控制效率矩阵**：从底座 `mixer` 反推出 `u → [T, τx, τy, τz]` 的 4×4 矩阵 `B`，
   于是"期望量 → 桨推力"变成一个带箱约束的最小二乘问题：

   $$
   \min_{u}\ \big\| \mathbf W^{1/2}(\mathbf B\,\Lambda u - d)\big\|_2^2 + \lambda \| \Lambda u \|_2^2
   \quad \text{s.t. } 0 \le u_i \le T_{\max}
   $$

   其中 `Λ = diag(g)`，`g` 是四桨健康度。**上界必须是 `g_i·T_max`**（有效推力的物理上限），
   写成 `T_max` 会让退化的桨在解里"假装"还能出满推力（自检抓到过这个 bug）。

2. **桨彻底失效时放弃偏航**：少一个桨后四个期望量之间出现约束，硬保偏航会互相抢预算，
   姿态先崩。按可控桨数切换优先级权重 `[20,1,1,1]` → `[20,10,10,0]`
   （Mueller & D'Andrea, ICRA 2014 的结论）。

3. **神经残差**：加权最小二乘忽略了箱约束，精确解（枚举有效集）才是最优。用
   `(12) → (64) → (64) → (4)` 的 MLP 学 `(u* − u_ls)/T_max`，**输出层零初始化**，
   所以未训练时就等价于近似分配、不会比基线差；部署时残差限幅 `±0.25·T_max`，
   避免网络在分布外给出过大修正、把箱约束的反馈余量吃光。

## 实测（12 次飞行 = 6 工况 × hover/zigzag，seed 0，8 s）

| 模式 | 生存率 | 平均最低高度 m | 平均位置 RMSE m | 平均控制能量 |
| --- | --- | --- | --- | --- |
| `geometric`（底座原样） | 0.583 | 0.873 | 0.191 | 21.7 |
| `naive`（丢的推力均摊） | 0.333 | 0.565 | 0.422 | 22.2 |
| `ls`（加权分配） | 0.583 | 0.894 | 0.263 | 25.5 |
| **`neural`（加权分配 + 残差）** | **0.583** | **0.906** | **0.188** | **23.7** |

- 权重 5252 个参数、训练 1.2 s（纯 CPU）；数据集 80 次飞行 11979 个样本。
- 分配精度：相对精确 QP 解，单桨指令平均误差 **0.113 N**，网络单步推理 **约 1 μs**。
- 单桨/双桨完全失效四种模式全部坠机 —— 这是物理上不可恢复的（三桨要同时满足合推力与两个
  力矩，解唯一、没有反馈余量；双桨连姿态自由度都不够），文档里作为"超出容错边界"诚实写出。

## 目录结构

```
uav_neural_fault_tolerant_control/
├── uav_neural_fault_tolerant_control/
│   ├── _compat.py       # 从 src/air 找到并导入仿真底座
│   ├── allocation.py    # 效率矩阵、加权最小二乘、精确箱约束 QP、朴素补偿
│   ├── controller.py    # 四种模式的容错分配控制器（与底座同签名）
│   ├── faults.py        # 六种故障工况的脚本
│   ├── features.py      # 12 维特征、残差标签、残差叠加
│   ├── model.py         # MLP 权重的构造/存取
│   ├── data.py          # 生成带精确标签的数据集
│   ├── train.py         # 训练
│   ├── evaluate.py      # 四种模式 × 六工况的对比评测
│   ├── render.py        # 动图与统计图
│   ├── ros_node.py      # ROS2 节点（发布 /uav/health、/uav/rotor_cmd 等）
│   ├── selftest.py      # 27 项自检
│   └── cli.py           # 命令行入口
├── config/control.yaml  # 参数默认值真源
├── launch/control.launch.py
├── results/allocator.pt # 入库的权重（26 KB）
├── test/                # 45 个 pytest 用例
└── tools/gen_figures.py # 生成文档配图
```

## 运行环境

| 项目 | 要求 |
| --- | --- |
| Python | ≥ 3.10 |
| PyTorch | ≥ 2.0（CPU 版即可，**不需要显卡**） |
| MuJoCo | ≥ 3.0（`pip install`，不需要 Gazebo / Unreal） |
| ROS2 | Humble（跑 `ros2 launch` 时才需要；纯命令行评测不需要 ROS） |

## 快速开始

```bash
# 只有权重、没有数据集时也能直接跑评测与动图
python main.py --selftest          # 27 项自检，约 3 s
python -m pytest test -q           # 45 个用例

python main.py --eval             # 四种模式 × 六工况对比
python main.py --demo results/fault_response.gif

# 想从头复现训练（约 30 s 生成数据 + 2 s 训练）
python main.py --gen-data results/allocator_dataset.npz --episodes 80
python main.py --train --threads 8
python tools/gen_figures.py        # 文档配图
```

`results/` 里入库的只有权重；数据集 12 MB 由 `--gen-data` 现场生成（`results/.gitignore` 已忽略）。

## ROS2 启动

```bash
colcon build --packages-select uav_mujoco_sim uav_neural_fault_tolerant_control
ros2 launch uav_neural_fault_tolerant_control control.launch.py
ros2 launch uav_neural_fault_tolerant_control control.launch.py \
    fault:=loss rotor:=1 onset:=3.0 duration:=15.0
```

参数分两层：`config/control.yaml` 是默认值真源（顶部键用通配 `/**`），launch 只覆盖
常改的场景/故障/时长。节点到 `duration` 自动退出。

| 话题 | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `/uav/odom` | 发布 | `nav_msgs/Odometry` | 世界系 ENU 位姿与速度 |
| `/uav/status` | 发布 | `std_msgs/String` | `HOVER` / `FAULT` / `CRASHED` / `DONE` |
| `/uav/health` | 发布 | `std_msgs/Float32MultiArray` | 四桨健康度 |
| `/uav/rotor_cmd` | 发布 | `std_msgs/Float32MultiArray` | 四桨推力指令（N） |
| `/uav/fault` | 发布 | `std_msgs/String` | 当前注入的故障描述 |
| `/uav/goal` | 订阅 | `geometry_msgs/Point` | 临时目标点 |
| `/uav/cmd_vel` | 订阅 | `geometry_msgs/Twist` | 速度偏置 |

## 设计约束

- **不修改仿真底座**：只用它的公开接口（`Quadrotor` / `mixer` / `GeometricController`），
  别人换控制器或换场景时本模块不用动。
- 控制器与底座同签名 `act(quad, p_d, v_d, a_d, yaw_d) -> ndarray(4)`，可以整体替换。
- 顶层 `import` 不拉 MuJoCo / torch，没装仿真环境也能 `import` 包查看分配数学。
- 相同 seed 结果可复现。
