# 基于神经网络的无人机感知—规划—控制避障

<div class="grid cards" markdown>

-   :material-quadcopter:{ .lg .middle } **分层神经避障**

    ---

    深度图 → CNN 占用栅格 → 神经启发式 A\* → 模仿学习控制器，
    四个模块全部由神经网络实现。

-   :material-cube-outline:{ .lg .middle } **CA-Sim 仿真器**

    ---

    纯 NumPy + Pillow 的轻量 3D 仿真：射线投射 RGB-D、双积分动力学、
    动障碍、四个评测场景，无需 GPU 与游戏引擎。

-   :material-robot-outline:{ .lg .middle } **ROS1 / ROS2 双栈**

    ---

    同一套 `ca_drone` 核心库，同时提供 Noetic 与 Humble 的
    launch、节点、Gazebo world，并已在 Ubuntu 20.04 虚拟机实测。

-   :material-chart-line:{ .lg .middle } **可复现实验**

    ---

    数据集、权重、训练曲线、GIF 与指标全部由脚本生成，
    每张图都能用一条命令重新画出来。

</div>

## 这个项目做了什么

无人机在未知的三维环境中，仅依靠机载深度相机从起点飞到目标点，途中避开静态箱体、
圆柱与来回运动的障碍物。与常见的“规则控制器 + 传统 A\*”方案不同，本项目的
**感知、规划、控制、端到端** 四条链路全部使用神经网络：

| 模块 | 输入 | 输出 | 网络 | 参数量 |
| --- | --- | --- | --- | ---: |
| 感知 `PerceptionNet` | 48×64 深度图 | 同尺寸占用概率图 | 小型 U-Net | 63,961 |
| 规划 `NeuralHeuristic` | 11×11 局部栅格 + 目标偏移 | cost-to-go（米） | MLP | 24,321 |
| 控制 `ControlNet` | 9×9 局部栅格 + 目标/速度/间隙 | 机体系速度 \((v_x,v_y)\) | MLP | 27,906 |
| 端到端 `E2ENet` | 48×64 深度图 + 目标极坐标 | 机体系速度 \((v_x,v_y)\) | 4 层 CNN + MLP | 110,611 |

!!! note "关于深度学习框架"
    实验环境没有 GPU，也无法安装 PyTorch。为了满足“算法必须是神经网络”的要求，
    本项目用 NumPy 手写了反向模式自动微分、卷积、池化、Adam 等组件
    （`ca_drone/nn/`），四个网络都是**真正训练**出来的，权重以 `.npz` 保存。
    推理不依赖任何深度学习框架，单帧 0.1–2 ms。

## 快速跳转

<div class="grid cards" markdown>

-   [:material-play-circle: **看动图演示**](gifs.md)

    场景的闭环飞行 GIF（均小于 10 MB），以及感知/规划模块的可视化。

-   [:material-chart-bar: **看性能评价**](results.md)

    成功率、碰撞率、路径长度、最小间隙、规划与推理耗时，含对比表格与柱状图。

-   [:material-rocket-launch: **三分钟跑起来**](quickstart.md)

    `python main.py --module full --scene sparse --gif` 一条命令出结果。

-   [:material-server-network: **ROS 启动**](ros1.md)

    `roslaunch neural_ca_drone_ros main.launch module:=full scene:=sparse`。

</div>

## 关键结果一览

| 指标 | 分层神经方案（full） | 几何专家（expert） | 端到端（e2e） |
| --- | ---: | ---: | ---: |
| 五场景成功率 | **5 / 5** | 5 / 5 | 2 / 5（40%） |
| 最小间隙（最差场景） | **1.30 m** | 1.02 m | 0.54 m |
| 单场景墙钟时间 | ≈ 5 s（Windows）/ 43.5 s（虚拟机） | ≈ 4 s | ≈ 10 s |
| 单帧推理 | 0.22 ms（控制）+ 0.9 ms（感知） | — | 0.82 ms |
| 规划（含神经启发式） | 15.3 ms（均值） | — | — |

> 表中的“虚拟机”指 Ubuntu 20.04 + Python 3.8 + Pillow 7 的 VMware 环境，
> 用于验证代码在 ROS Noetic/Humble 目标系统上的可运行性；Windows 数字来自本机复现。

## 环境与依赖

- **仿真/训练/评测**：Python ≥ 3.8，仅需 `numpy`、`Pillow`、`matplotlib`。
- **ROS1**：Ubuntu 20.04 + ROS Noetic（`catkin_make`）。
- **ROS2**：Ubuntu 22.04 / 20.04 + ROS Humble（`colcon build`）。
- 不依赖 CUDA、PyTorch、Gazebo 即可完整复现全部实验；
  Gazebo 只用于演示 ROS 部署路径（见 [Gazebo 仿真步骤](gazebo.md)）。

!!! warning "阅读顺序建议"
    第一次阅读建议按「任务与方案 → 快速开始 → 仿真环境 → 四个模块 → 实验与结果」的顺序；
    只关心部署可直接看 [ROS1](ros1.md) / [ROS2](ros2.md)。
