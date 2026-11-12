# MSK-Arm：Hill 肌肉驱动的人体手臂仿真、工作空间建图规划与纯 NumPy 神经网络肌肉控制

本模块是 `OpenHUTB/ros2` 仓库的**肌肉骨骼人（肌骨驱动手臂）**示例，覆盖课程作业「肌肉骨骼人」选题从肌肉动力学到神经网络肌肉控制的完整实现（按“模板稍简单”实现：两连杆平面手臂 + 4 块 Hill 型肌肉）：

1. **物理仿真 + 键盘运动控制**：两连杆刚体动力学（重力/阻尼/关节限位）+ 4 块 Hill 型肌肉（肩屈/肩伸/肘屈/肘伸，激活动力学、力-长度-速度关系），`Q/E/A/D/W/S/Space` 键盘控制肌肉激活；
2. **传感器感知与运动控制**：关节编码器、肌肉激活/肌力、末端位置、前视 32×32 深度相机；解析逆运动学 + 计算力矩（PD + 重力/科氏补偿）+ 肌肉激活映射；
3. **工作空间建图与规划**：深度相机反投影为工作空间占用栅格（对数几率），A* 规划末端路径（绕隔板），多目标序列（draw 5 点）；
4. **端到端模型**：深度图（32×32）→ 2 层 CNN → 4 个肌肉激活；
5. **神经网络训练 / 测试 / ROS 封装**：纯 NumPy 实现 MLP/CNN/Adam/DQN/REINFORCE，7 个模型、8 个 ROS 节点、6 个 launch；
6. **多场景性能评价**：`reach`（单目标）、`shelf`（隔板绕行）、`draw`（5 点轨迹）三场景闭环 benchmark，含碰撞、到达目标数、末端误差、峰值力矩指标与 RL 对比。

完整原理、算法与实验数据见文档页：[`docs/musculoskeletal/msk_arm_nn.md`](../../../docs/musculoskeletal/msk_arm_nn.md)。

## 目录结构

```
msk_arm_nn/
├── main.sh                    # 统一入口（teleop / reach / perception / e2e / benchmark / test / train / setup）
├── msk_arm_nn/                # 核心 Python 包（纯 Python + NumPy）
│   ├── config.py              # 连杆/肌肉参数、传感器、任务场景
│   ├── scenes.py              # reach / shelf / draw 工作空间目标与隔板
│   ├── sim.py                 # 两连杆动力学 + Hill 肌肉 + 深度相机 + 碰撞/到达判定
│   ├── render.py              # 侧视渲染（骨骼/肌肉激活着色/目标/轨迹/HUD）
│   ├── slam.py                # 工作空间占用图（深度相机射线映射）
│   ├── planning.py            # A* 工作空间路径规划
│   ├── control.py             # IK + 重力补偿 + 肌肉协同控制
│   ├── datasets.py            # 专家数据采集与特征构造
│   ├── runtime.py             # roslaunch 系统 Python -> 项目 venv 自动切换
│   └── nn/                    # MLP / CNN / Adam / DQN / REINFORCE
├── scripts/                   # ROS 节点（sim/nav/perception/e2e/metrics/viewer/teleop）
├── launch/                    # teleop / reach / perception / e2e / benchmark / bringup
├── tools/                     # 采集、训练、评测、GIF 录制
├── models/                    # 训练好的 .npz 模型
├── results/                   # 训练指标、三场景 benchmark、ROS 运行记录（JSON）
└── tests/                     # 物理/建图/规划/控制自测
```

## 环境要求

- Ubuntu 20.04.6 LTS + ROS Noetic（`/opt/ros/noetic`）
- Python ≥ 3.8（项目私有 venv；仅需 NumPy + Pillow，无 GPU、无 MuJoCo 依赖）
- 依赖：`numpy`、`Pillow`、`PyYAML`、`rospkg`（见 `requirements.txt`）

## 快速开始

```bash
cd src/musculoskeletal/msk_arm_nn
bash main.sh setup
bash scripts/build_ros.sh          # 独立工作空间 ~/msk_ws
source ~/msk_ws/devel/setup.bash
roslaunch msk_arm_nn teleop.launch scene:=reach
```

键盘（先点击 “MSK ARM” 窗口获得焦点）：

| 按键 | 作用 |
| --- | --- |
| `Q` / `E` | 肩屈肌 / 肩伸肌激活 + |
| `A` / `D` | 肘屈肌 / 肘伸肌激活 + |
| `W` | 共激活（所有肌肉 +0.05） |
| `S` | 放松（所有肌肉 −0.05） |
| `Space` | 全部放松 |

## 运行模式

| 命令 | 说明 |
| --- | --- |
| `bash main.sh teleop reach` | 肌肉驱动物理仿真 + 键盘激活（HUD：q1/q2/末端误差/峰值力矩/到达/碰撞） |
| `bash main.sh reach shelf` | 深度建图 + A* 规划 + IK + 重力补偿肌肉协同够取 |
| `bash main.sh perception reach` | 深度图感知网络 |
| `bash main.sh e2e shelf` | 端到端 CNN：深度图 → 4 肌肉激活 |
| `bash main.sh benchmark reach` | 指标评测（写 `results/ros_*.json`） |
| `bash main.sh test` | 物理 / 建图 / 规划 / 控制自测 |
| `bash main.sh train` | 重新采集数据并训练全部网络 |

## 实测结果（Ubuntu 20.04 虚拟机，4 核 CPU，无 GPU）

| 指标 | 结果 |
| --- | --- |
| 经典（建图 + A* + 肌肉协同）三场景 | **成功率 100%**（reach 1.7 s / shelf 3.1 s / draw 23.1 s，0 碰撞，到达 7/7 目标） |
| ROS 闭环（reach/shelf/draw） | reach 1.00 s/0.018 m、shelf 0.76 s/0.048 m、draw 26.6 s/5 目标/0.006 m（0/0/1 碰撞） |
| 工作空间建图 | 覆盖率 99.9–100%，障碍栅格 F1 0.955 / 0.983 |
| A* 路径（三场景） | reach 0.197 m、shelf 0.191 m、draw 5 段 0.063–0.144 m，最小离障 ≥0.040 m |
| 神经激活 MLP | 激活 MAE **0.0289**（目标 ≤0.10） |
| IK 网络 | 关节角 MAE **0.68°**（目标 ≤2°） |
| 端到端 CNN | 激活 MAE 0.162（均值基线 0.179，见限制） |
| 强化学习对比 | DQN 2×64 / DQN 3×64 / REINFORCE（见 `results/benchmark_rl.json`） |

## 已知限制

- 两连杆 + 4 块肌肉是简化模型，不含完整肌腱/骨骼几何与接触动力学；
- 端到端 CNN 的激活 MAE（约 0.16）只略优于均值基线：单帧 32×32 深度图不足以推断精确肌力分配；
- ROS 异步闭环下 draw 偶发 1 次碰撞（纯离线闭环为 0），与 50 Hz 位姿/目标时序有关；
- 采样与训练为 CPU 版本，未做真实肌电（EMG）或人体参数辨识。

## 参考与说明

- 本仓库已有肌肉骨骼示例（肌腱、MuJoCo 肌骨手臂等）作为组织与文档参考；
- 本模块所有代码、模型与数据均由本项目自行实现/生成，未使用他人代码；
- 神经网络为纯 NumPy（无 PyTorch/TensorFlow），可在无 GPU 的普通虚拟机中完整训练与评测。
