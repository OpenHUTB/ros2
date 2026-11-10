# humanoid_embodied_control — 具身人（人形机器人）物理仿真与平衡行走控制

## 概述

本功能包在 **MuJoCo** 中对一台双足人形机器人做物理仿真，通过键盘对它做运动控制，
并用传感器数据（激光测距、深度相机）做感知，进而对**给定轨迹**做闭环运动控制。

对应课程任务（1）（2）中"人形机器人 / 肌肉骨骼人"这一对象：

| 任务 | 要求 | 本模块实现 |
|---|---|---|
| （1） | 对机器人进行物理仿真，通过键盘对机器人进行运动控制 | MuJoCo 刚体人形 + `--mode teleop` 键盘遥操作 |
| （2） | 使用传感器获取到的数据进行感知，对机器人进行运动控制（感知、给定轨迹的控制） | 射线扫描 → 扇区可通行性 + 深度 ROI → 状态机决策 → 参数化步态 + 平衡控制器 |

详细文档：[docs/humanoid/embodied_humanoid_control.md](../../../docs/humanoid/embodied_humanoid_control.md)

## 三种运行模式

| 模式 | 需要 MuJoCo | 用途 |
|---|---|---|
| `sim`（默认） | 是 | 完整闭环：物理仿真 + 感知 + 平衡步态，走完整"走向目标"任务 |
| `offline` | **否** | 纯运动学回放：同一套感知/规划/步态/状态机，用于无 GPU 环境自证控制链路 |
| `teleop` | 是 | 只做物理仿真 + 键盘控制（任务 1） |

## 环境要求

| 组件 | 版本 |
|---|---|
| Python | 3.9+（MuJoCo 3.x 要求） |
| MuJoCo | ≥ 3.0（开发时为 3.3.7） |
| NumPy | ≥ 1.20 |
| ROS | ROS 1 Noetic（launch/节点）或 ROS 2 Humble |
| pytest | 单测 |

> **Noetic 的 Python 版本坑**：Noetic 自带 python3.8，装不上 MuJoCo 3.x。
> `main.sh` 会自动挑选 3.9+ 的解释器；也可用 `HUMANOID_PYTHON=/usr/bin/python3.10` 显式指定。

## 安装

```bash
pip3 install -r src/humanoid/humanoid_embodied_control/requirements.txt
```

## 运行

```bash
cd src/humanoid/humanoid_embodied_control

# 1) 完整闭环（物理仿真 + 感知 + 平衡行走）
python3 main.py --mode sim --seconds 40

# 2) 离线自证（不需要 MuJoCo）
python3 main.py --mode offline --seconds 40

# 3) 键盘遥操作
python3 main.py --mode teleop
```

一键脚本：`bash main.sh --mode sim`、`main.bat --mode sim`。

### 键盘映射（`--mode teleop`）

| 按键 | 动作 |
|---|---|
| `W` / `S` | 加速 / 减速（含倒退） |
| `A` / `D` | 左转 / 右转（小角度） |
| `Q` / `E` | 左转 / 右转（大角度） |
| 空格 | 停止并站立 |
| `ESC` | 退出 |

### ROS 1 Noetic

```bash
roslaunch humanoid_embodied_control main.launch
roslaunch humanoid_embodied_control main.launch goal_x:=3.0 goal_y:=0.5 period:=1.3
```

> `ctrl_hz` 不要调到 100 以下：步态更新率不足会让机器人"单腿支撑太久"而摔倒，
> 这是实测结论（见文档"调参记录"）。

## 控制链路

```text
MuJoCo ray-cast ──► 扇区可通行性 ─┐
                                   ├─► 状态机（WALK/AVOID/REACHED）─► 速度标量
深度 ROI ────────► 可通行比例 ─────┘                                      │
                                                                          ▼
IMU（四元数/陀螺）──► 平衡 PD（根节点 6 个广义力）◄── 参数化步态（12 个关节目标角）
                                                                          │
                                          ctrl[0:6]=力矩 / ctrl[6:18]=目标角
                                                                          ▼
                                                                    MuJoCo 物理步进
```

## 文件结构

```text
humanoid_embodied_control/
├── main.py                  # 课程约定主入口（转发到 scripts/main.py）
├── main.sh / main.bat       # 一键脚本（自动挑 Python 版本）
├── requirements.txt
├── package.xml / setup.py / setup.cfg
├── models/humanoid.xml      # MuJoCo 人形模型（19 qpos / 18 执行器）
├── launch/main.launch       # ROS 1 Noetic 启动文件
├── config/                  # 预留参数目录
├── resource/                # ament 索引占位
├── humanoid_embodied_control/
│   ├── geometry.py          # 角度归一化、四元数→欧拉、变换、钳位
│   ├── perception.py        # 激光→扇区；深度→可通行比例；融合
│   ├── planning.py          # 代价地图 + A* + 前视点
│   ├── gait.py              # 参数化步态（相位→12 关节角）
│   ├── balance.py           # 姿态 PD + 关节角度指令 + 踝策略
│   ├── state_machine.py     # WALK / AVOID / REACHED / FALLEN
│   ├── metrics.py           # RMSE / AoS / 能量
│   └── nodes/walk_node.py   # 与 ROS 无关的节点内核
└── scripts/
    ├── main.py              # 三模式实现（sim / offline / teleop）
    └── walk_node_ros1.py    # ROS 1 适配层
```

## 单测

```bash
cd hutb-ros2
python3 -m pytest tests/test_humanoid_embodied/ -q
```

109 条用例，覆盖：纯函数层（角度/感知/规划/指标/状态机）、平衡层参数校验、
MuJoCo 模型断言（编译、IMU 配置、质量、站立 20 s、行走 25 s）、CLI 端到端。

## 实现要点（踩过的坑，都在代码注释里留了证据）

1. **IMU site 必须写 `zaxis`**：否则 `framequat` 永远输出恒等四元数，
   所有基于姿态的反馈都在读一个"不动"的假值。这一条曾让调试绕了很久。
2. **`<position>` 执行器的 `ctrl` 是目标角，不是力矩**：把 Python 算的力矩
   （数百 N·m）填进去，等效于要求关节转到几百弧度，机器人第一帧就被弹飞。
   现在下行契约是"根节点收力矩、关节收角度"。
3. **站立构型不能完全伸直**：大腿/小腿各 0.34 m，与躯干到脚的落差相等，
   站立处在运动学奇异点，0.05 rad 屈膝就掉半米。站姿必须屈膝 ~0.30 rad。
4. **步频不能太低**：`0.77 cycle/s` 会因单腿支撑过久而摔倒，
   需要 ~3.85 cycle/s 的踏步频率；因此 `phase_rate` 不设 `1/period` 上限。
5. **场景物体要静态**：把障碍柱/目标球包成 `<body>` 会让它们计入机器人质量
   （实测背着 155 kg），改为 world 下的静态 `<geom>`。
6. **踝策略符号必须实测标定**：`ankle=+0.20 → pitch=-9.04°`，
   与"背屈=脚尖上翘"的直觉相反。

## 已知限制

* 行走速度约 **0.10 m/s**，转向能力有限（无偏航舵，靠左右腿摆幅差异实现），
  长距离会累积航向漂移；精确的落脚点规划（LIPM / MPC）留给后续工作。
* 平衡依赖模型层的 6 个 `balance_*` 通用执行器通道（等价于"外部扶正力"),
  不是纯关节驱动的 zero-moment-point 行走。这一点在文档里明确说明，
  避免把"能走"误读成"完全仿生"。
