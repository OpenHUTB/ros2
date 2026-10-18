# 地面载具横向控制算法对比（纯跟踪 / Stanley / LQR）

本文档介绍 `src/ground/lateral_control_benchmark` 模块：在统一车辆模型、统一参考轨迹与统一指标下，
对三种最常用的横向控制算法做**定量对比**。

与仓库中其他地面载具模块的区别：本模块**不依赖 CARLA / Gazebo / 任何仿真器**，
核心算法与闭环仿真只用 NumPy，因此在没有 GPU、没有仿真环境的机器上也能完整复现；
同时提供 ROS 1 (Noetic) 节点与 `roslaunch` 入口。

* 源代码目录：[`src/ground/lateral_control_benchmark`](https://github.com/OpenHUTB/ros2/tree/master/src/ground/lateral_control_benchmark)
* 一键复现：`python3 main.py --mode bench`（详见模块 `README.md`）

## 1. 三种算法原理

### 1.1 车辆运动学模型（三者共用）

后轴参考点的运动学自行车模型，状态 $[x, y, \theta, v]$，控制量 $[\delta, a]$：

$$
\dot{x} = v\cos\theta,\qquad \dot{y} = v\sin\theta,\qquad
\dot{\theta} = \frac{v\tan\delta}{L},\qquad \dot{v} = a
$$

其中 $L$ 为轴距（默认 2.7 m），$\delta$ 为前轮转角（左转为正）。
模块用 RK4 积分，步长 0.02 s。另提供线性二自由度动力学模型（含轮胎侧偏刚度、
质心侧偏角），用 `--model dynamic` 切换，用于检验控制器在中高速下的表现。

### 1.2 纯跟踪（pure pursuit）

取前视距离 $L_d$（随车速自适应 $L_d=\mathrm{clamp}(k_{ld}v, L_{d,min}, L_{d,max})$）处的路径点为目标:

$$
\alpha = \mathrm{atan2}(y_{t}-y,\ x_{t}-x) - \theta,\qquad
\delta = \arctan\!\left(\frac{2L\sin\alpha}{L_d}\right)
$$

特点：几何法、无曲率先验；前视点自带阻尼，收敛快；但在连续变曲率轨迹上残差较大。

### 1.3 Stanley

$$
\delta = \psi_e - \arctan\!\left(\frac{k\,e}{k_{soft}+v}\right) + \arctan(L\kappa)
$$

* $\psi_e = \mathrm{wrap}(\theta_{path}-\theta)$ 为**路径切向相对车头**的偏角；
* $e$ 为控制点横向偏差（左偏为正）；两项都带上述符号，若写反成本车相对路径的定义
  却不翻号，闭环会变成正反馈（这是这类实现最常见的错误）。
* 第三项为**曲率前馈**，直接补偿路径弯曲造成的稳态偏差。

**取点位置的影响**：经典写法取**前轴中心**，而前轴比车辆实际位置超前一个轴距，
转弯时前馈取到的是车辆前方的曲率，会留下稳态偏差。本模块默认取**后轴**
（与运动学模型参考点一致），可用 `use_rear_axle=False` 复现经典写法：

| 场景 | 前轴（经典）稳态偏差 / m | 后轴（默认）稳态偏差 / m |
|---|---|---|
| 直线 | 0.0000 | 0.0000 |
| 圆弧 R=20 m | **0.4809** | **0.0011** |
| 正弦（幅值 3 m） | 0.4047 | 0.0029 |
| 双移线 | 0.0270 | 0.0002 |

### 1.4 LQR（本实现为 LQRI，带横向偏差积分）

误差状态 $e=[e,\ \psi_e,\ r]$，控制量为期望横摆角速度：

$$
\dot{e} = v\psi_e,\qquad \dot{\psi}_e = r - \kappa v,\qquad \dot{r} = (r_{des}-r)/T
$$

线性化后 $A=\begin{bmatrix}0&v&0\\0&0&1\\0&0&-1/T\end{bmatrix}$，
$B=[0,0,1/T]^\top$。解连续 Riccati 方程得到 $K$，控制律：

$$
r_{des} = \underbrace{\kappa v}_{\text{曲率前馈}} - K\,[e,\psi_e,r-r_{ref}]^\top - K_i z,\qquad \dot{z}=e
$$

再折算为前轮转角 $\delta=\arctan(L r/v)$。

两个关键实现点：

1. **必须有曲率前馈**：否则反馈项要"持续存在"才能维持转向，表现为恒定的稳态偏差。
2. **必须有横向偏差积分**：只有比例状态反馈时，低速起步瞬态造成的偏差会被锁死
   （实测直道残留约 0.05 m、圆弧上会累积到十几米）。加入积分后稳态误差趋于零，
   积分项做了限幅与"只在中等误差时累积"的抗饱和处理。
3. Riccati 方程用**纯 NumPy 的 Hamiltonian 特征分解**求解（`lqr_solver.py`），
   不依赖 `scipy`——少一个依赖就少一个"装不上"的失败点。

## 2. 算法流程

```mermaid
graph TD
    A[参考轨迹 ReferencePath] --> B[按弧长参数化 切向/法向/曲率]
    C[车辆状态 x,y,yaw,v] --> D[投影到参考轨迹]
    B --> D
    D --> E{选择控制器}
    E -->|纯跟踪| F[前视点 -> 几何转角]
    E -->|Stanley| G[横向偏差+航向偏差+曲率前馈]
    E -->|LQR| H[误差状态 -> Riccati 增益 -> 期望横摆角速度]
    F --> I[转角限幅/限速率]
    G --> I
    H --> I
    I --> J[运动学模型 RK4 积分]
    J --> C
    D --> K[记录横向偏差/航向偏差]
    K --> L[指标: 均值/最大/RMS/稳态/收敛时间/离道占比]
```

## 3. 运行步骤

### 3.1 离线（推荐）

```bash
cd src/ground/lateral_control_benchmark

python3 test/test_controllers.py       # 66 项单元测试
python3 main.py --mode bench           # 四场景 × 三控制器
python3 main.py --mode bench --model dynamic --speed 15
python3 main.py --mode sweep --param stanley.k --values 1.0,2.0,4.0
```

### 3.2 ROS 1 (Noetic)

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
cp -r ~/ros2/src/ground/lateral_control_benchmark .
cd ~/catkin_ws && source /opt/ros/noetic/setup.bash
catkin_make && source devel/setup.bash
roslaunch lateral_control_benchmark main.launch controller:=stanley scenario:=sine
```

话题：`/lateral_control/odom`、`/lateral_control/cmd`、`/lateral_control/styles`、
`/lateral_control/metrics`、`/lateral_control/path`（latched，可在 RViz 中看参考路径）。

## 4. 评测方法（公平性设计）

| 维度 | 做法 |
|---|---|
| 车辆模型 | 三者共用同一模型与同一积分器 |
| 纵向速度 | 共用同一速度策略（巡航 + 曲率限速 + 一阶响应），差异只来自横向算法 |
| 初始条件 | 同一初始位姿：前方 2 m、横偏 1.2 m、航向偏 0.10 rad、初速 3 m/s |
| 指标口径 | 同一 `evaluate()` 计算 13 项指标，定义见下 |
| 参数调优 | 三者都用各自较优参数（见 README），避免"给对手用坏参数" |

指标定义：

* **平均绝对横向偏差** $\overline{|e|}$：全时段平均；
* **RMS** $\sqrt{\overline{e^2}}$：对大偏差更敏感；
* **稳态偏差**：轨迹末段 25% 的平均绝对偏差，反映稳态精度；
* **收敛时间**：$|e|$ **首次进入并此后一直保持**在 0.2 m 以内的时刻；从未满足则取总时长；
* **离道占比**：$|e|>1\,\mathrm{m}$ 的采样占比，反映安全性；
* **转角变化率 RMS**：转角差分得到，反映转向平顺性；
* **综合排名**：在同一场景内部对各指标排名，再汇总平均名次（越小越好），
  避免"场景难度"直接决定名次。
* **测试速度**：8 m/s（弯道自动限速）；`--speed` 可改，`--model dynamic` 可切动力学模型。

## 5. 结果与性能评价

### 5.1 指标对比（巡航 8 m/s，运动学模型）

| 控制器 | 场景 | 平均\|e\| / m | RMS / m | 最大\|e\| / m | 稳态\|e\| / m | 收敛 / s |
|---|---|---|---|---|---|---|
| pure_pursuit | 直线 | 0.1252 | 0.3436 | 1.2491 | 0.0000 | 1.38 |
| stanley | 直线 | 0.1293 | 0.3302 | 1.2490 | 0.0000 | 2.06 |
| lqr | 直线 | 0.1686 | 0.3301 | 1.2490 | 0.0337 | 1.90 |
| pure_pursuit | 圆弧 R=20 m | 0.0172 | 0.1164 | 1.1122 | 0.0002 | 1.22 |
| stanley | 圆弧 R=20 m | 0.0187 | 0.1115 | 1.1122 | **0.0011** | 1.88 |
| lqr | 圆弧 R=20 m | 0.0232 | 0.1120 | 1.1122 | 0.0012 | 1.76 |
| pure_pursuit | 正弦 | 0.1519 | 0.2915 | 1.2772 | 0.0826 | 1.40 |
| stanley | 正弦 | 0.0866 | 0.2706 | 1.2772 | **0.0029** | 2.14 |
| lqr | 正弦 | 0.1187 | 0.2712 | 1.2772 | 0.0166 | 1.98 |
| pure_pursuit | 双移线 | 0.0882 | 0.2806 | 1.2422 | 0.0057 | 1.38 |
| stanley | 双移线 | 0.0870 | 0.2695 | 1.2422 | **0.0002** | 2.06 |
| lqr | 双移线 | 0.1180 | 0.2703 | 1.2422 | 0.0137 | 1.90 |

综合排名（各场景内部按 8 项指标排名后汇总，越小越好）：

| 控制器 | 平均名次 |
|---|---|
| pure_pursuit | **1.81** |
| stanley | 1.91 |
| lqr | 2.28 |

### 5.2 结论

1. **稳态精度**：Stanley 最优（毫米级），因为它同时使用横向偏差、航向偏差与曲率前馈；
   纯跟踪在连续变曲率（正弦）上残差最大（0.083 m），因为没有曲率先验。
2. **收敛速度**：纯跟踪最快（1.35 s），前视点提供了天然阻尼；LQR 与方法相当（1.89 s）；
   Stanley 最慢（2.04 s），高增益让它"稳"但略显迟钝。
3. **平顺性**：纯跟踪与 Stanley 的转角变化率 RMS 更低（20～29 °/s），
   LQR 因增益较大达到 33～41 °/s。
4. **实现细节 > 算法选择**：Stanley 换成前轴取点后，圆弧稳态偏差从 0.001 m 升到 0.48 m。
   这说明在同一算法下，**参考点/前馈取值点**这类细节对精度的影响，可能比换算法还大。
5. **速度适应性**：在 5 / 8 / 15 m/s 下三个控制器均保持稳定（`--speed` 可复核），
   Stanley 与 LQR 的稳态偏差随速度基本不变，纯跟踪的稳态偏差随速度略有上升
   （8 m/s 时 0.022 m，15 m/s 时 0.041 m）。

## 6. 与仓库已有模块的区别

仓库里已经出现过"纯跟踪 / 横向控制"这些词，为避免与既有模块混淆，这里把定位说清楚：

| 已有模块 | 它做了什么 | 与本模块的关系 |
|---|---|---|
| `carla_perception_control` | 在 CARLA 中做**感知 + 给定轨迹跟踪**，横向用纯跟踪作为其中一个构件 | 纯跟踪只是它的一环；本模块把三种算法并列做基准对比 |
| `carla_nn_car_following` | 在 CARLA 中做**自适应跟车（ACC）**，横向用纯追踪 / 面包屑跟踪 | 同上；本模块不做 ACC，只比横向算法 |
| `car_2d_target_tracker` | 2D 平面内的**目标追踪** | 任务是"追目标"，不是"跟轨迹" |
| `carla_keyboard_control` / `mujoco_car_project` | 仿真器内的**键盘遥控演示** | 本模块无遥控、无仿真器 |

本模块的定位是**"横向控制算法之间的对比基准"**：不新增一个纯跟踪实现，
也不做一个新的跟车任务，而是回答"在相同车辆模型、相同轨迹、相同指标下，
这三种横向算法究竟差多少、差在哪里"。它同时是仓库里目前唯一一个
**不依赖 CARLA / Gazebo 等外部仿真器**即可完整复现的地面载具模块。

## 7. 问题记录（开发中的实际缺陷）

| # | 现象 | 根因 | 修复 |
|---|---|---|---|
| 1 | LQR 在 1 m 偏差下算出 −256° 转角、闭环发散 | 线性化时横摆角速度方程缺少转向增益，且 B 矩阵符号与误差方向相反，闭环成了正反馈 | 按 $r'=(r_{des}-r)/T$ 重写线性化，明确 $B=[0,0,1/T]^\top$ 与反馈符号 |
| 2 | LQR 直道残留 0.05 m、圆弧累积到 14 m | 只有比例状态反馈，低速起步瞬态偏差被锁死 | 增加横向偏差积分（LQRI），并做抗饱和限幅 |
| 3 | LQR 加 5 m 预瞄后圆弧误差从 0.03 m 升到 14 m | 误差状态已含航向偏差，再对预瞄点取误差会重复计入航向信息 | 默认 `lookahead=0`，并把这个现象写进文档 |
| 4 | Stanley 前轴取点在圆弧上偏差 0.48 m | 前轴比车辆实际位置超前一个轴距，前馈取值偏外 | 默认改为后轴取点，保留 `use_rear_axle=False` 复现经典写法 |
| 5 | 轨迹图被 `axis("equal")` 压成一条直线 | 横向变化远小于纵向长度，等比例不可读 | 去掉等比例约束 |
| 6 | 曲率前馈在低速出现 0/0 | $L\kappa v/v$ 在 $v\to0$ 时未定义 | 控制量改为期望横摆角速度，转角换算处对 $v$ 取下限 |

## 8. 常见问题排查

| 现象 | 解决 |
|---|---|
| `ModuleNotFoundError: numpy` | `sudo apt-get install -y python3-numpy` |
| 出图报缺少 matplotlib | `sudo apt-get install -y python3-matplotlib`，或加 `--no-figures` |
| `roslaunch` 找不到 `lateral_control_node.py` | 确认已 `catkin_make` 并 `source devel/setup.bash`；脚本需要可执行权限 |
| 节点启动但话题没有数据 | 检查 `rostopic list` 是否有 `/lateral_control/styles`；确认 `rate_hz` 参数有效 |
| 图中中文显示为方块 | 安装中文字体（Noto Sans CJK / 微软雅黑），绘图会自动匹配 |

## 9. 大模型使用声明

本模块的代码、文档与实验脚本在编写过程中使用了 AI 编程助手（DeepSeek）辅助生成与调试；
文中所有指标均来自本机实际运行结果（`results/metrics.csv`），提交内容由提交者负责。
