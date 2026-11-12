# 桨故障下的端到端推力策略（`uav_fault_e2e_thrust_policy`）

对应课程任务④「每种机器人端到端模型（输入图像、输出控制指令）」，也是「神经容错飞行」系列
的最后一个模块。

## 这个模块和已有的端到端模块有什么不同

仓库里 `uav_depth_gate_racing_e2e`、`uav_depth_track_nn`、`uav_e2e_rl` 等都是**深度图 → 速度
或位姿指令**。本模块有三点不同：

| 对比项 | 已有端到端模块 | 本模块 |
| --- | --- | --- |
| 观测 | 深度图 + 机体速度 | 深度图 + **四桨健康度** + 三轴角速率 + 上一步指令 |
| 动作 | 速度 / 位置指令 | **四个桨的归一化推力**（直接到执行器层） |
| 训练数据 | 正常飞行 | **故障注入**下的飞行（含推力衰减、卡死） |
| 评价 | 是否通过门 / 跟踪误差 | 故障工况下的**到达率、坠机率、末端进度** |

换句话说：不给它"桨坏了"这个结论，让它自己从健康度那一路上看到坏桨，然后自己决定四个桨各出多少力。

## 任务与观测

- 任务：`corridor` 场景里沿 **+x 飞 10 m**、保持 **1.5 m** 高度，14 s 内完成；`z < 0.3` 判坠机，`|y| > 3` 判失败。
- 深度图：底座自带前视相机 `front_cam`，64×64 深度 → 2×2 均值池化到 **32×32**，按 10 m 归一化到 [0,1]。
- 本体量 16 维：机体系速度 3、`sin(roll)`、`sin(pitch)`、三轴角速率 `/4.0`、**四桨健康度 4**、上一步推力指令 4。
- 观测里**没有世界系位置、没有目标点坐标**——否则等于偷看 GPS，端到端就没意义了。
- 输出：4 个归一化推力 ∈ [0,1]，乘单桨满推力 6 N 后按健康度缩放得到实际推力。

## 网络结构（三种，对应任务⑤的"同算法不同网络结构"对比）

| 结构 | 说明 | 参数量 | 测试集 MAE |
| --- | --- | --- | --- |
| `mlp` | 深度图直接展平 + 本体量 → 128 隐层 | 145 924 | 0.0109 |
| `cnn` | 3 层卷积 + 全局平均池化（单帧） | 27 812 | **0.0095** |
| `cnn_stack` | 最近 3 帧深度图当 3 通道（能看到墙逼近速度） | 28 100 | 0.0110 |

## 数据：DAgger 3 轮

`--gen-data` 用几何控制器（PR3 的加权分配 + 梯形速度剖面参考）当专家：

- 5 个训练工况：healthy、衰减 0.5@桨2、衰减 0.3@桨1、卡死@桨0、渐进退化@桨3
- 3 轮 DAgger，β 从 0.9 线性降到 0.5（β 是"这一步听专家"的概率），专家演示加 σ=0.02 噪声
- 只记录**高度 > 1.0 m**的样本：策略失控后掉到地上的状态，专家标签没有意义，会把网络训坏
- 结果：45 段飞行、**14 794 个样本**、数据集约 6 MB（不入库，`--gen-data` 约 1 分钟可重建）

## 实测结果（7 个工况 × seed 0）

| 模式 | 到达率 | 坠机率 | 末端进度 m | 最低高度 m | 最大倾角 ° | 指令偏离 |
| --- | --- | --- | --- | --- | --- | --- |
| expert | **0.57** | 0.43 | 8.06 | 0.98 | 30.4 | 0.000 |
| mlp | 0.00 | 1.00 | 3.90 | 0.28 | 132.8 | 0.338 |
| **cnn** | **0.29** | 0.71 | 6.95 | 0.61 | 44.0 | 0.247 |
| cnn_stack | 0.00 | 1.00 | 4.23 | 0.29 | 55.1 | 0.224 |

结论要点（诚实版）：

1. **健康与温和衰减下，端到端策略能自己飞到 x≈10 m**；卡死与单桨完全失效都会坠机——后者在 PR3 里已经证明物理上不可恢复。
2. 三种结构里 **cnn 最好**（27.8 K 参数、MAE 0.0095、44° 最大倾角）；`mlp` 直接把 32×32 图展平，
   参数最多但也最容易过拟合到像素位置，闭环倾角 133° 直接翻掉。
3. **端到端推力回归的瓶颈是精度**：测试 MAE 0.0095 对应约 0.06 N 的单桨误差，而桨间差动本身只有
   0.5% 量级。同样的误差放在"速度指令"上无关紧要，放在"推力"上就会慢慢把姿态带歪。
4. **DAgger 收益有限**：纯 BC 与 3 轮 DAgger 的 cnn 到达率都是 0.29。单步随机接管修正了状态分布，
   但没解决回归精度——这条负面结论也写进了文档。

## 目录结构

```
src/air/uav_fault_e2e_thrust_policy/
├── main.py / main.sh / main.bat        # 入口（默认 --selftest）
├── setup.py / setup.cfg / package.xml  # ament_python 安装
├── config/policy.yaml                  # 节点参数默认值（顶层键 /**）
├── launch/policy.launch.py             # ros2 launch 入口
├── requirements.txt
├── results/                            # 三个权重入库；数据集与 gif 不入库
├── tools/gen_figures.py                # 文档配图
├── test/                               # 47 个 pytest 用例
└── uav_fault_e2e_thrust_policy/
    ├── obs.py       # 深度裁剪/池化、本体量、观测打包
    ├── env.py       # ThrustEnv：直接把四个桨推力写进仿真
    ├── expert.py    # 梯形速度剖面 + 加权分配专家
    ├── policy.py    # mlp / cnn / cnn_stack 三种结构
    ├── data.py      # DAgger 采集、按 episode 切分
    ├── train.py     # 训练循环
    ├── evaluate.py  # 闭环评测表
    ├── cli.py       # --selftest/--gen-data/--train/--eval/--demo/--node
    ├── selftest.py  # 40 项自检
    └── ros_node.py  # ROS2 节点
```

## 运行环境

| 项 | 要求 |
| --- | --- |
| 系统 | Ubuntu 22.04（ROS2 Humble）或 Windows 10/11 |
| Python | ≥ 3.10 |
| 依赖 | `mujoco>=3.0.0`、`numpy>=1.21`、`torch>=2.0`（CPU 版即可）、`matplotlib`、`pillow` |
| 硬件 | 不需要独显、不需要 Gazebo/Unreal/CUDA |
| 底座 | 复用同仓库的 `uav_mujoco_sim`（不改它一行） |

## 快速开始

```bash
cd src/air/uav_fault_e2e_thrust_policy

python main.py --selftest           # 40 项自检，不需要 ROS
python main.py --gen-data           # DAgger 3 轮，约 2 分钟
python main.py --train --arch all   # 三种结构各训一遍，约 5 分钟（CPU）
python main.py --eval               # 7 工况闭环对比表
python main.py --demo               # 录一段演示 gif
python main.sh                      # 等价于 --selftest
```

仓库里已带 `results/policy_{mlp,cnn,cnn_stack}.pt`，所以 `--eval` / `--demo` 可以直接跑。

## ROS2 启动

```bash
# 先建工作空间（在仓库根）
colcon build --packages-select uav_mujoco_sim uav_fault_e2e_thrust_policy
source install/setup.bash

ros2 launch uav_fault_e2e_thrust_policy policy.launch.py \
    fault:=degradation rotor:=2 onset:=2.5 severity:=0.5
```

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/uav/odom` | `nav_msgs/Odometry` | 位姿与速度（ENU） |
| `/uav/status` | `std_msgs/String` | `RUN/CRASHED/SUCCESS/DONE` |
| `/uav/rotor_cmd` | `std_msgs/Float32MultiArray` | 策略实际下发的四桨归一化推力 |
| `/uav/health` | `std_msgs/Float32MultiArray` | 四桨健康度（只有故障脚本知道） |
| `/uav/depth_sample` | `std_msgs/Float32MultiArray` | 深度图 16 倍抽样，方便 rqt 看 |
| `/uav/goal` | `geometry_msgs/Point` | 订阅但不使用（端到端策略不依赖目标点） |

## 设计约束

- **不改 `uav_mujoco_sim`**：只用它的公开 API（`Quadrotor` / `make_scene` / `VisionSensors` / `GeometricController`）。
- **顶层不 import torch**：没有 torch 的机器也能 `import uav_fault_e2e_thrust_policy` 并使用纯 numpy 部分。
- **可确定性复现**：所有飞行都按 seed 播种，`--eval` 同一 seed 结果完全一致。
- 权重入库、数据集不入库（见 `results/.gitignore` 与根 `.gitignore` 的例外规则）。

## 已知限制

1. 只覆盖 MuJoCo 单机、单一走廊场景；没有风扰、没有动态障碍。
2. 单桨完全失效与双桨失效不可恢复（PR3 已论证），本模块只把它们当边界案例。
3. 端到端策略的到达率明显低于"专家 + 分配"的经典结构——这正是本模块想说明的问题：
   端到端到执行器层对回归精度的要求比到速度层严苛得多。
4. 健康度由故障脚本直接给出，尚未接 PR2 的 `uav_rotor_fault_detector_nn` 在线估计器；
   接上后需要重新训一轮（观测分布会变）。
