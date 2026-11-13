# Dubins 曲线轨迹生成与可视化

带运动学约束的最短可行路径生成模块：给定起末位姿与最小转弯半径，枚举 6 种基本组合（LSL/LSR/RSL/RSR/RLR/LRL）的最短 Dubins 路径，纯 pygame 2D 可视化。

## 概述

Dubins 曲线由 Lester E. Dubins 于 1957 年提出，是带曲率约束（最小转弯半径 R）的车辆在起末位姿已知时**最短可行路径**的标准解。Dubins 证明：最短路径必为以下 6 种基本组合之一：

- **CSC 类型**（圆-直线-圆）：LSL / LSR / RSL / RSR
- **CCC 类型**（圆-圆-圆）：LRL / RLR

其中 L = 左转圆弧（逆时针），R = 右转圆弧（顺时针），S = 直线段。本项目采用**几何构造法**计算 6 种组合长度（圆心 + 公切线 + 弧长），相比闭式参数公式更直观、数值稳定。

## 算法原理

### 问题定义

给定起点 (x0, y0, yaw0) 和终点 (x1, y1, yaw1) 及最小转弯半径 R，求带曲率约束 |κ| ≤ 1/R 的最短可行路径。

### 6 种基本组合

| 类型 | 组合 | 适用 | 几何构造 |
|---|---|---|---|
| CSC | LSL | 两端同向左转 | 左左圆外公切线 |
| CSC | RSR | 两端同向右转 | 右右圆外公切线 |
| CSC | LSR | 第一段左转、第三段右转 | 左右圆内公切线 |
| CSC | RSL | 第一段右转、第三段左转 | 右左圆内公切线 |
| CCC | LRL | 左转-右转-左转 | 中间反向相切圆 |
| CCC | RLR | 右转-左转-右转 | 中间反向相切圆 |

### 圆心计算

对于位姿 pose=(x, y, yaw)：

- 左转圆心 = (x - R·sin yaw, y + R·cos yaw)
- 右转圆心 = (x + R·sin yaw, y - R·cos yaw)

### 公切线

- **外公切线**（LSL / RSR）：两个等径同侧转圆心连线方向即为切线方向
- **内公切线**（LSR / RSL）：切线方向相对圆心连线偏角 φ=asin(2R/dist)，要求 dist ≥ 2R

### 弧长

沿圆心 c1 的 turn1 方向从起点角度走到切点 1 角度，再沿 c2 的 turn2 方向从切点 2 走到终点。单段弧长限定在 [0, 2πR)。

### 与 Bezier 曲线的区别

| 维度 | Bezier 曲线 | Dubins 曲线 |
|---|---|---|
| 输入 | 控制点序列 | 起末位姿 + R |
| 约束 | 自由（光滑插值） | 带曲率上界（车辆运动学） |
| 目标 | 形状插值 | 最短可行路径 |
| 用途 | 轨迹生成 | 路径规划 |

## 实现细节

### 源码结构

| 文件 | 内容 | 行数 |
|---|---|---|
| `dubins.py` | 核心算法：圆心、公切线、6 种组合、最短路径、采样 | 433 |
| `main.py` | pygame 可视化：起末箭头、候选路径、最短高亮、HUD、录制 | 288 |
| `config.py` | 画布尺寸、颜色、起末位姿、R、采样参数、params.yaml 加载 | 121 |
| `test_dubins.py` | 36 项离线单测（含独立闭式对照） | 475 |


### 算法正确性（两个易错点）

**1. 内公切线的方向符号**

设切线方向为 dir、n 为 dir 的左法向。切点按转向落在圆心两侧
（L 圆取 -n 侧、R 圆取 +n 侧），因此

```
p2 - p1 = (c2 - c1) + s·2R·n,   s = +1 (L,R) / -1 (R,L)
```

要求 `p2 - p1 ∥ dir` 即 `(p2-p1)·n = 0`，得到

```
sin(dir - base) = s · 2R / dist
```

`sin` 给两支解 `base + a` 与 `base + π - a`，但只有 `cos(dir-base) > 0` 的那支
让直线段沿 `+dir` 前进，所以应取 `dir = base + asin(s·2R/dist)`。
**s 必须随 turn1 翻转**：固定用同一支会让 RSL（s = -1）无解，表现为
「6 种候选里 RSL 从不出现」，`shortest_path` 于是退而选择更长的类型。

**2. CCC 有两个中间圆，必须取较短者**

中间圆与两个主圆外切，圆心落在 c1-c2 连线中垂线上，有 ± 两个候选。
两者都是几何上闭合的合法 Dubins 曲线（端点位置与朝向都能对上），但总长通常不同。
只取第一个满足距离条件的候选会返回较长解，并让 `shortest_path` 选错类型
（例如 `start=(0,0,0), end=(2,2,π), R=2` 时合法 RLR 仅 11.5556，
只取单支会得到 26.1435，从而误选 LRL 15.9586）。

**3. 零长度路径**

起末位姿完全相同时，六种构造都会因圆心重合而失败；现在直接返回 `(0, 0, 0)`，
`shortest_path` 返回长度 0 而不是抛 `ValueError`。

**校验方式**：测试里内置一份独立的闭式实现（Shkel/Lumelsky 归一化公式）做交叉对照——
CSC 四种逐类型长度一致（1e-6 内）、`shortest_path` 恒等于六种候选的最小值、
不劣于闭式最优、不短于直线距离，且路径端点（位置与朝向）都闭合。
随机位姿 4000 组的离线核对结果：CSC 逐类型不一致 0 处、CCC 端点不闭合 0 处、
`shortest_path` 非最优 0 处。

> 演示 gif 已按修正后的实现重录：候选项从 3 条恢复到 4 条（RSL 回归），
> 最短路径仍是 RSR 874.9 px。

### 参数配置

`config/params.yaml` 会真正被读取（`config.load_params()`），查找顺序：

1. 环境变量 `DUBINS_PARAMS` 指向的文件
2. ROS 2 安装空间 `<share>/dubins_path/config/params.yaml`（`ros2 run` 走的路径）
3. 源码树 `<包根>/config/params.yaml`

可配置键：`canvas.{width,height,fps}`、`dubins.{start,end,radius}`、`sampling.{curve_samples,anim_steps}`。
未安装 PyYAML 时自动退回内置默认值，不会报错。

### 可视化效果

演示 gif（`dubins_demo.gif`）展示：

1. **6 条候选路径**（淡灰色细线）：所有可行组合，无解的不画
2. **最短路径高亮**（青色粗线）：6 种中长度最短者
3. **起末位姿箭头**（绿/红箭头）：标识起点朝向与终点朝向
4. **HUD**：起末位姿、半径 R、最短类型与长度、各候选长度对比

![Dubins 曲线演示](../img/ground/dubins_path/dubins_demo.gif)

## 运行步骤

### 直接运行

```bash
pip install pygame numpy pillow
python main.py                  # 交互模式
python main.py --record         # 录制演示 gif
```

### ROS 2 运行

```bash
cd ~/ros2_ws
colcon build --packages-select dubins_path --symlink-install
source install/setup.bash
ros2 run dubins_path main
```

无显示设备的机器（VM / CI）用 launch 参数切到离屏录制，不会因为没有窗口而崩：

```bash
ros2 launch dubins_path main.launch.py mode:=record output:=/tmp/dubins_demo.gif
ros2 launch dubins_path main.launch.py start:="150,480,0" end:="750,150,180" radius:=80
```

## 单元测试

```bash
cd dubins_path
python -m pytest test/ -v          # 或 python test/test_dubins.py
```

36 项单测覆盖：mod2pi 边界、6 种组合非负、最短路径类型与长度、最短 ≤ 候选、缩放相似性、
起末位姿对齐、双方向端点对齐、采样点数量、_advance 各段运动学，以及

- **RSL 回归**：该用例下 RSL 必须存在且为最短（7.7156），并随机扫描确认六种类型都出现过；
- **CCC 取短支**：`RLR` 在该用例下应取 11.5556（而非 26.1435），`shortest_path` 选 RLR；
- **零长度路径**：同位姿返回 0、朝向差 2π 视为同位姿、零长度采样只返回起点；
- **独立闭式对照**：CSC 逐类型与闭式一致、`shortest_path` 不劣于闭式最优且端点闭合；
- **打包自检**：setup.py 引用的文件真实存在、share 目录只注册一次；params.yaml 键生效。

CI：`.github/workflows/dubins_path_tests.yml` 在无 ROS / 无显示设备的环境里跑
`pytest test/` 与 `main.py --mode record` 冒烟。

## 大模型使用声明

本项目在开发过程中使用了大语言模型辅助代码编写与文档生成，具体用于：

- Dubins 曲线几何构造算法的 Python 实现
- pygame 可视化代码框架
- 单元测试用例设计
- 文档撰写

核心算法逻辑（圆心构造、公切线方向、弧长积分）均基于经典教材与公开文献实现，AI 辅助部分经过人工审核与虚拟机实测验证。