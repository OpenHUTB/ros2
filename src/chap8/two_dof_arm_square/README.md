# two_dof_arm_square

MuJoCo 平面 2R 双自由度机械臂画正方形仿真：大臂 0.40 m、小臂 0.25 m，末端黑色小球沿红色正方形边框运动。

> **这是运动学回放，不是动力学仿真。** 每帧先用解析 IK 算出关节角直接写入 `qpos`，
> 再调用 `mj_forward` 重算正运动学与渲染几何；全程不调用 `mj_step`、不积分动力学，
> 也不经过 position 执行器的力矩闭环。XML 里的 `kp=100`、阻尼 `1.0` 仅作模型定义。

## 运行环境（实际验证）

| 项 | 版本 |
|---|---|
| 操作系统 | Windows 11（本机）；Ubuntu 22.04（云主机，无独立显卡） |
| Python | 3.10+（实测 Ubuntu 3.12.11 / Windows 3.13） |
| 模拟器 | MuJoCo `>=3.2,<4.0`（本机实测 3.17.0） |
| 关键依赖 | numpy（实测 Ubuntu 1.26.4 / Windows 2.5.4）；Pillow 仅录帧导出 GIF 时需要（Ubuntu 10.4.0）；MuJoCo 不在 apt 仓库，需 pip 安装 |
| ROS | ROS 1 Noetic（catkin 包结构）。本包纯 Python + MuJoCo，不依赖任何 ROS 话题，`launch` 仅用于一键启动脚本；ROS 侧 `roslaunch` 未在本机端到端复跑，独立 Python 运行已实测通过 |
| 渲染后端 | `MUJOCO_GL=glfw`；无显卡的虚拟机可配 Xvfb |

## 安装

依赖两条路径，按运行方式选一条：

| 运行方式 | 装什么 | 命令 |
|---|---|---|
| 独立运行（不装 ROS） | 全部走 pip | `pip install -r requirements.txt` |
| 作为 ROS 1 包运行 | apt/rosdep 装 `python3-numpy`；`mujoco` 不在 apt 仓库，仍要 pip 装 | `rosdep install --from-paths src --ignore-src -y`，再 `pip install -r requirements.txt` |

## 运行

在**包目录**下执行。先进入本包目录（按你的摆放二选一，进入后下面命令相同）：

```bash
# 仓库内 checkout：  cd src/chap8/two_dof_arm_square
# catkin 工作空间：  cd ~/ws/src/two_dof_arm_square
pip install -r requirements.txt
python3 main.py              # 打开 MuJoCo 窗口，关闭窗口即退出
python3 main.py --headless   # 无头模式，跑 1 圈后打印误差并退出
```

## 作为 ROS 1 包运行（catkin_make + roslaunch）

1. 把本包目录整个放进 catkin 工作空间的 `src/` 下，例如 `~/ws/src/two_dof_arm_square`；
2. 在**工作空间根目录**（含 `src/` 的那一层）执行编译：

```bash
cd ~/ws
catkin_make
source devel/setup.bash
```

3. 启动：

```bash
roslaunch two_dof_arm_square two_dof_arm_square.launch
```

## 预期现象

1. 机械臂从目标点 (0.40,-0.05) 出发，沿四条直线边平滑循环运动；
2. 末端黑色小球紧贴红色正方形边框，无弧线偏离；
3. `--headless` 模式下终端打印四个顶点实测坐标与偏差。

## 验证结果与口径

`--headless` 输出的四顶点偏差为 **0.000 mm**。需要说明：本实现是解析 IK 解直接写入 `qpos` 后 `mj_forward`，
MuJoCo 正运动学与手写 IK 共用同一套杆长几何（`L1=0.40` / `L2=0.25` 与 XML capsule fromto 一致），
因此该 0.000 mm 是 **IK ↔ FK 互逆的自洽恒等式**，在浮点精度内为 0；
它**只证明 IK 与正运动学自洽，不代表带 PD 控制器 / 执行器闭环的真实动态跟踪误差**。

**本演示证明了什么**：2R 臂解析 IK 解算正确、正方形四顶点与边轨迹几何位置与模型一致。
**未证明什么**：动力学跟踪精度、带摩擦/负载/重力补偿下的表现、控制器收敛性能，均未验证。

## 演示 GIF 复现

文档站的 `docs/img/two_dof_arm_square_demo.gif` 由 `test/record_demo_gif.py` 离屏渲染生成，不是另外录屏；在包目录执行
`python3 test/record_demo_gif.py` 即可复现（录帧需 Pillow，离屏后端 `MUJOCO_GL=egl`）。一圈 204 个插值点隔帧抽稀（`STEP=2`）为 102 帧，实测 1244×702、每帧 30 ms。

## 已知限制

- 本实现是运动学回放，不调用 `mj_step`，不做动力学积分；XML 里的 `kp=100` / 阻尼 `1.0` 仅作模型定义，不经过执行器力矩闭环；
- 仅验证肘向上构型；肘向下（`q2<0`）构型未测试；
- 仅在 mujoco 3.17.0（`>=3.2,<4.0` 区间内）上验证，区间内其他版本与更早版本未做回归；
- ROS 侧 `roslaunch` 未在本机端到端复跑，独立 Python 路径已实测通过。

## 文档

详细说明见 [文档站页面](../../../docs/chapter/mujoco_two_dof_arm_square.md)。

## 大模型使用声明

本作业的代码与文档在编写过程中使用了**豆包（Doubao）** AI 大模型辅助，本人已核对并确认提交内容，对提交内容负责。
