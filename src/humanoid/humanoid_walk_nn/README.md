# HNN：简化双足人形机器人步行仿真、激光 SLAM 与纯 NumPy 神经网络导航

本模块是 `OpenHUTB/ros2` 仓库的**人形机器人（双足步行）**示例，覆盖课程作业「人形机器人」选题从物理仿真到神经网络闭环导航的完整实现：

1. **物理仿真 + 键盘运动控制**：简化双足步行动力学——步行速度/转向速率限幅、线性倒立摆式横向平衡、步态相位与摔倒判定；`W/A/S/D/Space` 键盘遥控；
2. **传感器感知与运动控制**：72 束 360° 激光雷达（噪声/野值/丢点）、IMU（roll/pitch/yaw rate）、6 关节编码器、GNSS、前视 32×32 深度相机；感知 MLP 对激光去噪后再送入控制器；
3. **建图与导航**：对数几率占据栅格 + 相关性扫描匹配 SLAM；A* 全局规划、纯跟踪步行控制、平衡感知限速、障碍减速/停车与摔倒恢复；
4. **端到端模型**：前视深度图（32×32）→ 2 层 CNN → `(walk_speed, turn_rate)`；
5. **神经网络训练 / 测试 / ROS 封装**：纯 NumPy 实现 MLP/CNN/Adam/DQN/REINFORCE，7 个模型（`models/*.json` 元信息 + `models/*.npz` 权重，总共约 1.3 MB）、8 个 ROS 节点、6 个 launch；
6. **多场景性能评价**：`hall`（走廊）、`warehouse`（货架 + 移动搬运车）、`obstacle`（木箱训练场）三场景闭环 benchmark，含碰撞、摔倒、终点误差指标与 RL 算法/网络结构对比。

完整原理、算法与实验数据见文档页：[`docs/humanoid/humanoid_walk_nn.md`](../../../docs/humanoid/humanoid_walk_nn.md)。

## 目录结构

```
humanoid_walk_nn/
├── main.sh                       # 统一入口（teleop / slam_nav / perception / e2e / benchmark / test / train / setup）
├── humanoid_walk_nn/             # 核心 Python 包（纯 Python + NumPy）
│   ├── config.py                 # 步行参数、传感器、场景、平衡限制
│   ├── scenes.py                 # hall / warehouse / obstacle 平面图、立柱、搬运车
│   ├── sim.py                    # 简化双足动力学 + 激光/GNSS/IMU/关节/深度相机
│   ├── render.py                 # 俯视图渲染（平面图/步态动画/点云/HUD）
│   ├── slam.py                   # 占据栅格 + 扫描匹配 SLAM
│   ├── planning.py               # A*（墙/木箱/立柱） + DWA 局部规划
│   ├── control.py                # 纯跟踪 + 平衡感知步行控制 + 摔倒恢复
│   ├── datasets.py               # 专家数据采集与特征构造
│   ├── runtime.py                # roslaunch 系统 Python -> 项目 venv 自动切换
│   └── nn/                       # MLP / CNN / Adam / DQN / REINFORCE
├── scripts/                      # ROS 节点（sim/slam/nav/perception/e2e/metrics/viewer/teleop）
├── launch/                       # teleop / slam_nav / perception / e2e / benchmark / bringup
├── tools/                        # 采集、训练、评测、GIF 录制
├── models/                       # 模型的 meta 与指标（*.json）+ 权重（*.npz，约 1.34 MB，已入库）
├── results/                      # 训练指标、三场景 benchmark 汇总（JSON/CSV）
└── tests/                        # 物理 / 建图 / 规划 / 控制自测
```

## 环境要求

- Ubuntu 20.04.6 LTS + ROS Noetic（`/opt/ros/noetic`）
- Python ≥ 3.8（项目私有 venv；仅需 NumPy，无 GPU、无 MuJoCo/CARLA 依赖）
- 依赖：`numpy`、`Pillow`、`opencv-python`、`PyYAML`（见 `requirements.txt`；`rospkg` 由 ROS Noetic 提供）

## 快速开始

```bash
cd src/humanoid/humanoid_walk_nn
bash main.sh setup
bash main.sh train                 # 可选：重新训练并覆盖 models/*.npz 与 results/*（权重已入库，clone 后无需先训练）
bash scripts/build_ros.sh          # 独立工作空间 ~/humanoid_ws
source ~/humanoid_ws/devel/setup.bash
roslaunch humanoid_walk_nn teleop.launch scene:=hall
```

键盘（先点击 “HUMANOID” 窗口获得焦点）：

| 按键 | 作用 |
| --- | --- |
| `W` | 向前步行 |
| `S` | 后退小步 |
| `A` / `D` | 左转 / 右转 |
| `Space` | 原地站立 |

## 运行模式

| 命令 | 说明 |
| --- | --- |
| `bash main.sh teleop hall` | 双足步行物理仿真 + 键盘遥控（HUD：速度/平衡/摔倒/碰撞） |
| `bash main.sh slam_nav warehouse` | 激光建图 + 扫描匹配 + A* + 纯跟踪自主步行 |
| `bash main.sh perception hall` | 感知 MLP 激光去噪，发布 `/humanoid/scan_filtered` |
| `bash main.sh e2e warehouse` | 端到端 CNN：深度图 → 步行速度/转向 |
| `bash main.sh benchmark` | 四控制器 × 三场景闭环评测（每格 3 条 episode、同 seed、60 s 上限，写 `results/benchmark.json` / `.csv`） |
| `bash main.sh test` | 物理 / 建图 / 规划 / 控制自测 + 手写反传梯度校验 |
| `bash main.sh train` | 重新采集数据并训练全部网络 |

## 实测结果（Ubuntu 20.04 虚拟机，4 核 CPU，无 GPU）

| 指标 | 结果 |
| --- | --- |
| 经典 A* + 纯跟踪步行（三场景，3 episode/格、同 seed） | **成功率 100%**（hall 30.5 s / warehouse 50.8 s / obstacle 30.6 s，0 碰撞、0 摔倒，终点误差 0.60 / 0.59 / 0.59 m） |
| SLAM 绝对轨迹误差 ATE RMSE | **0.168 m**（地图 150×375 @0.08 m，覆盖率 48.3%） |
| A* 路径长度（三场景） | 28.2 / 30.0 / 25.6 m，绕行比 1.08 / 1.17 / 1.13，最小离障 0.70 / 0.67 / 0.70 m |
| 激光去噪（感知 MLP） | RMSE **0.729 → 0.432 m（+40.7%）**，有效波束 +35.6% |
| 神经控制（行为克隆） | v MAE **0.066 m/s**（≤0.15 ✓）、ω MAE **0.110 rad/s**（≤0.20 ✓） |
| 强化学习对比 | DQN 2×64 **76.7%**、DQN 3×64 **83.3%**、REINFORCE **6.7%** |
| ROS 闭环（slam_nav） | hall 32.45 s / 0.581 m；warehouse 32.8 s / 0.518 m（0 碰撞、0 摔倒） |

## 评测协议与安全层

- **同一随机种子**：benchmark 中同一 scene+episode 的所有控制器共用同一个 seed，每格 3 条
  episode，报告 mean±std；逐条记录（含 seed）见 `results/benchmark.csv`。
- **成功判据**：终点到目标 ≤0.60 m（与 `tests/test_control.py` 一致）；控制器在
  `config.GOAL_TOLERANCE=0.45 m` 内停止，`final_error` 为 episode 结束时的剩余距离。
- **安全层**：所有学习型控制器（neural / neural_two_level / e2e）输出后统一执行 `walking_guard`，
  任一方向激光 <0.40 m 时硬性禁止平移（含倒车），0.40–0.75 m 限速；经典栈沿用控制模块自带的
  0.35 m 硬安全距离。行为克隆因此不再顶墙，但也可能卡住（成功率不虚高）。
- **蒸馏评估**：行为克隆/端到端与“经典”共享几何专家，比较的是蒸馏质量而非方法优劣。
- **模型元信息**：`models/*.json` 含 arch/lr/epochs/dataset_hash/config_hash/code_hash 等 meta 与评测指标；
  权重 `models/*.npz` 已随仓库提供（约 1.3 MB），干净 clone 可直接复现 `results/benchmark.json`；如需重新训练执行 `bash main.sh train` 覆盖。
- **梯度校验**：`tests/test_gradients.py` 用中心差分校验 Dense/Conv2D/激活层反传
  （相对误差 <1e-5，实测 ~1e-9），`bash main.sh test` 会执行。
- **pytest 覆盖范围**：`python -m pytest tests` 只收集 9 项（PID / 纯跟踪 / 安全层 / 平衡限幅 / 摔倒恢复等）；`tests/test_gradients.py` 与 `test_control.py` 的 mission 闭环流程不在 pytest 收集范围内，完整回归请用 `bash main.sh test`。
- **日志不入库**：逐帧 `results/ros_*_slam_nav.json` 已移除并加入 `.gitignore`，
  由 `bash main.sh slam_nav <scene>` 重新生成。

## 已知限制

- neural / neural_two_level 在室内墙角与立柱处会陷入局部极小（持续顶墙），三场景闭环 0%，文档中给出原因；
- 端到端深度 CNN 的 ω MAE 0.186、v MAE 0.106（“预测均值”基线 0.147 / 0.053）：32×32 深度图对转向意图信息不足，闭环仅 warehouse 偶发成功（1/3），hall 与 obstacle 均失败；
- 简化双足模型没有完整关节级接触动力学，以步速/转向限幅 + 平衡约束 + 摔倒判定近似；
- 摔倒计数在 benchmark 中全为 0：专家与学习策略都被平衡预算 `|v·ω| ≤ 0.75` 约束；RL 中保留高风险动作，DQN 学到规避。

## 参考与说明

- 本仓库已有双足/人形示例（ZMP 步态、H1 行走、摔倒保护等）作为文档与组织参考；
- 本模块所有代码、模型与数据均由本项目自行实现/生成，未使用他人代码；
- 神经网络为纯 NumPy（无 PyTorch/TensorFlow），可在无 GPU 的普通虚拟机中完整训练与评测。
