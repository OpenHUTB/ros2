# 具身人形机器人的物理仿真、传感器感知与平衡行走控制

> 对应课程任务（1）：*对人形机器人/肌肉骨骼人/车辆/水下机器人/无人机进行物理仿真，
> 通过键盘对机器人进行运动控制*；
> 对应课程任务（2）：*实现传感器的感知及运动控制，要求使用传感器获取到的数据进行感知
> （如雷达、摄像头、深度摄像头等）对机器人进行运动控制（5·2：感知、给定轨迹的控制）*。
> 本模块面向**人形机器人（具身人）**。

代码位于 `src/humanoid/humanoid_embodied_control/`。

## 1. 概述与目标

本章在一台**双足人形机器人**上同时完成两件事：用 MuJoCo 做物理仿真并支持键盘遥控，
以及用机载传感器（激光测距 + 深度相机）感知环境后对**给定轨迹**做闭环行走控制。

三个必须同时成立的目标决定了本章的结构：

| 目标 | 含义 | 落地方式 |
|---|---|---|
| **能站住** | 没有控制输入时机器人不能自己倒 | 模型层平衡通道 + 站立姿态留屈膝余量 |
| **能听指令** | 键盘/轨迹指令能改变运动 | 参数化步态（相位 → 12 个关节目标角） |
| **能用传感器** | 感知结果真的影响控制，不是数据转发 | 扇区可通行性 → 速度标量 + 状态机切换 |

### 1.1 与仓库中已有同类模块的区分

| 模块 | 相同点 | 本模块的不同点 |
|---|---|---|
| [`humanoid_basics`](../humanoid/humanoid_basics.md) | 都以 MuJoCo 为人形仿真环境 | 本文档只做**环境配置**；本模块提供完整的控制链路与代码 |
| `src/ground/mujoco_car_project` | 都是 MuJoCo + 键盘运动控制 | 对象是**双足人形**（平衡问题），不是轮式小车 |
| `src/water/rov_mujoco` | 都有 MuJoCo 仿真与 ROS 封装 | 对象是水下 6-DOF，不存在"单腿支撑"这类平衡约束 |
| `src/air/uav_physical_simulation` | 都是"物理仿真 + 键盘控制"作业 | 无人机是**全驱动**六自由度；人形是**欠驱动**，必须靠落脚与平衡 |
| `src/air/uav_perception_trajectory` | 都是"感知 + 给定轨迹跟踪" | 无人机用点云直方图 + PID；人形要额外解决**支撑面**与**竖直高度** |

!!! note "配置步骤不重复（老师评审要求）"
    Miniconda / MuJoCo 的安装、环境激活、图形库依赖等**通用配置步骤**，
    请直接参考 [人形机器人基础](./humanoid_basics.md)「配置 Miniconda 和 MuJoCo 物理仿真环境教程」。
    本文档只描述本模块特有的内容（模型设计、控制算法、参数整定与实测结果）。

---

## 2. 计算原理

### 2.1 人形的欠驱动本质

双足人形与无人机最大的差别：无人机每个自由度都有推进器，人形只有 12 个关节，
却要控制 6 个浮动基座自由度。躯干是**倒立摆**，其发散速率为

$$
\omega_{\text{div}} = \sqrt{\frac{g}{h}} \approx \sqrt{\frac{9.81}{1.0}} \approx 3.13\ \text{rad/s}
$$

意味着若完全不控，躯干在 $t$ 秒后的倾角近似为 $\theta_0 \cosh(\omega_{\text{div}} t)$。
这也直接决定了控制周期：一个控制周期内允许的倾角增长量必须远小于支撑面半宽
$l_{\text{foot}}/2$，即

$$
\omega_{\text{div}}\,\Delta t \cdot h \ll \frac{l_{\text{foot}}}{2}
\quad\Longrightarrow\quad
\Delta t \ll \frac{0.09}{3.13 \times 1.0} \approx 29\ \text{ms}
$$

因此步态/平衡的下发频率必须 ≥ 100 Hz——这也是本模块 `phase_rate` 不设
`1/period` 上限的原因（见 [6.3 节](#tuning)）。

### 2.2 平衡控制：串级 PD

躯干姿态用**根节点广义力**闭环。设 IMU 给出横滚角 $\phi$、俯仰角 $\theta$
及机体系角速度 $\boldsymbol{\omega}$，则

$$
\boldsymbol{\tau}_{\text{bal}} =
\begin{bmatrix}
-K_p^{\text{bal}}\,\phi - K_d^{\text{bal}}\,\omega_x \\
-K_p^{\text{bal}}\,\theta - K_d^{\text{bal}}\,\omega_y \\
-K_d^{\text{yaw}}\,\omega_z
\end{bmatrix}
$$

竖直方向单独一路"高度保持"，用于抵消步态屈膝造成的腿长缩短：

$$
F_z = K_p^{z}\,(z_{\text{hold}} - z) + K_d^{z}\,(0 - \dot z)
$$

水平方向**故意不做位置回拉**：支撑腿已经把躯干绑在地面上，
再加位置环会与地面约束持续对抗，实测会让机器人缓慢加速跑掉数米。
真实人形的落脚点选择才是横向稳定的手段，而不是"横向位置弹簧"。

### 2.3 步态：相位 → 12 个关节角

单一相位 $\varphi \in [0,1)$ 驱动全部关节，左右腿相差半个周期：

$$
\varphi_L = \varphi,\qquad \varphi_R = (\varphi + 0.5) \bmod 1
$$

单腿在一个周期内分摆动相与支撑相：

$$
\text{hip}(\varphi) =
\begin{cases}
\dfrac{s}{2}\cos(\pi u), & u = \varphi/0.5 \in [0,1) \quad \text{（摆动）} \\[4pt]
-\dfrac{s}{2} + s\,u, & u = (\varphi-0.5)/0.5 \quad \text{（支撑）}
\end{cases}
$$

$$
\text{knee}(\varphi) = \kappa_0 + \lambda \sin(\pi u)\ \ (\text{仅摆动相}),\qquad
\text{ankle}(\varphi) = 0.25\,\text{knee}(\varphi)
$$

其中 $s$ 为步幅、$\lambda$ 为抬腿量、$\kappa_0$ 为站立屈膝偏置。

**符号是实测标定的**：本模型 `hip_pitch > 0` 表示大腿**向后**摆，
因此"向前迈步"是 hip 从负值向正值变化。踝角与膝角同向小比例跟随
（系数 0.25），保证摆动相脚掌大致水平。

### 2.4 感知：扇区可通行性

一帧扫描被切成 $N=16$ 个扇区，**每扇取最小值**而不是均值：

$$
f_k = \frac{1}{R_{\max}}\min_{i \in \text{扇区}k} r_i \ \in [0,1]
$$

取最小值是必须的：均值会被远处的空旷点稀释，导致近处障碍被"平均掉"——
这是最典型的"看起来有感知、实际却撞上去"。

相邻且 $f_k < 0.55$ 的扇区合并成障碍段，并按距离**升序**进入决策：

$$
\mathcal{O} = \Big\{ \big(\bar\varphi_j,\ \min_{k\in \text{段}j} R_{\max} f_k\big) \Big\}_{j},\qquad
\text{按距离排序}
$$

深度相机通道给出"正前方可通行比例"$f^{\text{d}} = \min(\text{ROI})/R_{\max}$，
与激光均值加权融合：

$$
f = w f^{\text{lidar}} + (1-w) f^{\text{d}},\qquad w = 0.7
$$

### 2.5 给定轨迹的跟踪与决策

给定轨迹由目标点定义，前视点取"沿参考路径第一个距当前位置 ≥ $L_{\text{look}}$ 的点"，
航向误差 $\psi_e = \mathrm{wrap}(\psi_{\text{目标}} - \psi)$。速度标量为

$$
v = v_{\max}\cdot \underbrace{s_{\text{obs}}(d_{\text{front}})}_{\text{障碍降速}}
\cdot \underbrace{\max\!\big(0.15,\cos|\psi_e|\big)}_{\text{先转正再走}}
$$

状态机带**迟滞**：进入 AVOID 需要 $d_{\text{front}} < 0.65$ m，
退出 AVOID 需要 $d_{\text{front}} \ge 1.0$ m。没有迟滞时，
距离在阈值附近抖动会让状态每帧互切、步态相位反复重置，机器人表现为原地抽搐。

---

## 3. 算法流程

```text
sim 模式（在线，需 MuJoCo）：
  加载 models/humanoid.xml → 站立（根力矩 + 关节角指令）
  → 每 10 ms（100 Hz）：
      每 50 ms 更新感知：
        mj_ray 扫 96 条射线 → 16 扇区可通行性
        → 中心 9 条射线派生深度可通行比例
        → 加权融合
      状态机(前向净空, 到目标距离) → 速度标量 v、转向偏置
      相位 φ += rate(v)·Δt → 12 个关节目标角
      根节点 6 个广义力（姿态 PD + 高度保持）
      写 ctrl[0:6]（力矩）+ ctrl[6:18]（目标角） → mj_step
  → 到目标 / 堵塞 / 摔倒 则结束，输出指标 JSON

offline 模式（无 MuJoCo）：
  同一套感知/步态/状态机
  用解析障碍模型 + 平面运动学积分代替物理引擎
  → 验证控制链路，不声称验证平衡

teleop 模式（任务 1）：
  物理仿真 + 键盘（W/S/A/D/Q/E/空格/ESC）
  → 与 sim 共用同一份平衡与步态代码
```

---

## 4. 源码解析

### 4.1 `models/humanoid.xml` — MuJoCo 人形模型

| 项 | 数值 / 说明 |
|---|---|
| 自由度 | `nq=19`（freejoint 7 + 12 个关节），`nv=18` |
| 执行器 | `nu=18`：6 个平衡通道 + 12 个关节位置伺服 |
| 时间步 | 0.5 ms（伺服刚度较高，≥1 ms 会数值爆炸） |
| 质量 | 总 22.7 kg，躯干 11.6 kg，腿 7.1 kg |
| 腿长 | 大腿 0.34 m + 小腿 0.34 m，站立屈膝 0.30 rad 时躯干高 0.946 m |
| 传感器 | `framequat`（姿态）、`gyro`、`accelerometer`、双脚 `touch` |

**拓扑**：串链必须写成 `thigh(髋) → thigh_link(膝,偏移=大腿长) → shank_link(踝,偏移=小腿长) → 脚掌几何`。
MuJoCo 的 `body pos` 是相对父 body 的静止偏移、关节旋转整棵子树，
写成"三个 body 各偏移 -0.34"才是运动学链。

**平衡执行器**：`torso` 已挂 `freejoint`，不能再加 hinge（会触发
`more than 6 dofs in body`）；`tendon` 刚度只能表达"沿关节轴的位移弹簧"。
因此用 6 个 `<general>` 执行器把标量 `ctrl` 乘 `gear` 作用到根节点 6 个自由度：

```xml
<general name="balance_roll"  tendon="root_x" gear="1" ctrlrange="-300 300"/>
```

它们承载的是**控制器算出的**恢复力矩，不是模型里写死的常数——可调、可审计。

### 4.2 `humanoid_embodied_control/geometry.py`

| 函数 | 作用 |
|---|---|
| `wrap_pi(a)` | 角度归一到 $(-\pi,\pi]$（用 `math.remainder`，负角也正确） |
| `quat_to_roll_pitch(w,x,y,z)` | ZYX 欧拉角解横滚/俯仰（`atan2` 形式，奇异点附近连续） |
| `rate_limit(prev,target,max_step)` | 关节目标角速率限制，避免状态切换的阶跃 |

### 4.3 `perception.py` / `planning.py`

| 函数 | 作用 |
|---|---|
| `lidar_sectors(...)` | 一帧扫描 → 16 扇区可通行性（每扇取最小值） |
| `detect_obstacles(...)` | 可通行性 → 障碍段（合并 + 按距离排序） |
| `depth_free_ratio(...)` | 深度 ROI → 正前方可通行比例 |
| `sensor_fusion(...)` | 激光/深度加权融合 |
| `front_clearance(...)` | 正前方（±1 扇区）最小距离 |
| `CostMap` + `astar(...)` | 代价地图 + 八邻域 A*（禁切角） |
| `lookahead_point(...)` | 纯跟踪前视点 |

### 4.4 `gait.py` / `balance.py`

| 函数 | 作用 |
|---|---|
| `stand_pose(knee_bias=0.30)` | 站立姿态（**必须屈膝**，见 6.3） |
| `walk_pose(phase, stride, lift)` | 相位 → 12 个关节目标角 |
| `phase_rate(speed, stride, period)` | 速度 → 相位推进速率（不设 `1/period` 上限） |
| `BalanceController.compute_balance` | IMU → 6 个根部广义力（含高度保持） |
| `BalanceController.joint_command` | 12 个目标角 → 速率限制后的 `ctrl` 指令 |
| `ankle_correction` / `apply_ankle_strategy` | 踝/髋策略补偿（符号实测标定） |
| `is_fallen(quat, z)` | 摔倒判据：高度 < 0.55 m **或** 倾角 > 70° |

### 4.5 `scripts/main.py` — 三模式主入口

| 类 / 函数 | 作用 |
|---|---|
| `SimPerception` | 用 `mujoco.mj_ray` 合成激光扫描 + 深度 ROI |
| `HumanoidSim` | 状态读写、传感器读取、`ctrl` 下发 |
| `WalkingController` | 感知→决策→步态→平衡的粘合层（sim/offline 共用） |
| `run_sim` / `run_offline` / `run_teleop` | 三种运行模式 |
| `build_parser` / `main` | CLI 与结果 JSON 输出 |

**关键实现约束**（代码注释里都有实测证据）：

```python
# 步态必须以 100 Hz 更新：20 Hz 会让"摆动相开始"出现半个周期的跳变
gait_every = max(1, int(round(0.01 / dt)))
decision_every = max(gait_every, int(round(0.05 / dt)))   # 感知/决策 20 Hz
```

```python
# 根节点收力矩，关节收角度 —— 别把力矩填进 <position> 的 ctrl
sim.apply(bf, balance.joint_command(target_vec))
```

### 4.6 ROS 节点

`humanoid_embodied_control/nodes/walk_node.py` 是与 ROS 无关的内核，
`scripts/walk_node_ros1.py` 只做话题转发，因此同一份控制逻辑在
ROS 1 / ROS 2 下都能跑，单测也不需要 ROS 环境。

| 话题 | 类型 | 方向 |
|---|---|---|
| `/uav/lidar` | `sensor_msgs/LaserScan` | 订阅 |
| `/uav/depth` | `sensor_msgs/Image` | 订阅（可选） |
| `/goal` | `geometry_msgs/PointStamped` | 订阅 |
| `/uav/cmd_vel` | `geometry_msgs/Twist` | 发布（指令载体） |
| `/uav/state` | `std_msgs/String` | 发布（JSON 遥测） |

---

## 5. 仿真运行步骤

### 5.1 支持与测试环境

| 组件 | 版本 / 说明 |
|---|---|
| 操作系统 | Ubuntu 20.04 / Windows 10+ |
| Python | 3.9+（本机实测 3.9.13） |
| MuJoCo | 3.3.7 |
| NumPy / SciPy | 2.0.2 / 1.13.1 |
| ROS | 1 Noetic（launch）/ 2 Humble（可选） |

### 5.2 新手路线

| 序号 | 做什么 | 出处 |
|---|---|---|
| 1 | 装 Miniconda + MuJoCo 环境 | [人形机器人基础](./humanoid_basics.md) |
| 2 | 装本模块依赖 | [5.3 节](#deps) |
| 3 | 跑离线自证（不需要 MuJoCo） | [5.4 节](#offline) |
| 4 | 跑完整物理闭环 | [5.5 节](#online) |

### 5.3 步骤 0：环境准备 <span id="deps"></span>

```bash
# 通用步骤（conda 安装、图形库）见 humanoid_basics.md，这里只列本模块依赖
pip3 install -r src/humanoid/humanoid_embodied_control/requirements.txt

# 验证
python3 -c "import mujoco; print(mujoco.__version__)"
```

> Noetic 自带的 python3.8 装不上 MuJoCo 3.x。`main.sh` 会自动挑 3.9+ 的解释器，
> 也可用 `export HUMANOID_PYTHON=/usr/bin/python3.10` 显式指定。

### 5.4 步骤 1：离线自证（不需要 MuJoCo）<span id="offline"></span>

```bash
cd src/humanoid/humanoid_embodied_control
python3 main.py --mode offline --seconds 40
```

该模式用解析障碍模型 + 平面运动学代替物理引擎，验证**控制链路**：
感知 → 状态机 → 步态 → 关节目标角。它**不验证平衡**，
平衡只在 `--mode sim` 下由 MuJoCo 验证。

### 5.5 步骤 2：完整物理闭环 <span id="online"></span>

```bash
# 默认目标 (4.5, 0)，跑 40 s
python3 main.py --mode sim --seconds 40

# 目标放近一点，观察"到达目标"的完整状态迁移
python3 main.py --mode sim --seconds 60 --goal_x 3.0

# 调参数
python3 main.py --mode sim --seconds 40 --stride 0.20 --lift 0.10 --period 1.3

# 把结果落盘
python3 main.py --mode sim --seconds 40 --json_out /tmp/run.json
```

一键脚本：`bash main.sh --mode sim --seconds 40`（Windows：`main.bat --mode sim`）。

### 5.6 步骤 3：键盘遥操作（任务 1）<span id="teleop"></span>

```bash
python3 main.py --mode teleop
```

`W/S` 加减速、`A/D` 小角度转向、`Q/E` 大角度转向、空格站立、`ESC` 退出。

### 5.7 步骤 4：ROS 1 Noetic

```bash
roslaunch humanoid_embodied_control main.launch
roslaunch humanoid_embodied_control main.launch goal_x:=3.0 goal_y:=0.5
```

`ctrl_hz` 不要调到 100 以下，理由见 [6.3 节](#tuning)。

### 5.8 运行效果

#### 站立（无步态，20 s）

```text
t= 0.0  z=0.9457  knee=+0.002  tilt= 0.00  ncon=0
t= 0.5  z=0.9438  knee=+0.000  tilt= 0.04  ncon=8
t= 5.0  z=0.9438  knee=+0.000  tilt= 0.04  ncon=8
t=20.0  z=0.9438  knee=+0.000  tilt= 0.04  ncon=8
```

站立 20 s 躯干倾角 < 0.1°，高度稳定，无漂移（单测 `test_stand_20s_no_fall` 覆盖）。

#### 行走（`--mode sim --seconds 40`，节选）

```text
[t= 0.00] pos=(+0.00,+0.00) z=0.946 WALK    speed=0.20 front=3.00 free=0.91 tilt=  0.0° goal=4.50
[t= 6.00] pos=(+0.73,+0.19) z=0.954 WALK    speed=0.20 front=3.00 free=0.91 tilt=  1.5° goal=3.78
[t=12.00] pos=(+1.44,+0.50) z=0.956 WALK    speed=0.20 front=3.00 free=0.91 tilt=  0.7° goal=3.10
[t=18.00] pos=(+2.01,+0.81) z=0.956 WALK    speed=0.19 front=3.00 free=0.91 tilt=  1.2° goal=2.61
[t=24.00] pos=(+2.41,+1.08) z=0.956 WALK    speed=0.17 front=3.00 free=0.91 tilt=  2.6° goal=2.36
[t=39.00] pos=(+2.79,+1.67) z=0.958 WALK    speed=0.14 front=3.00 free=0.91 tilt=  2.7° goal=2.39
```

40 s 内躯干高度保持在 **0.946 ~ 0.960 m**（波动 < 1.5 cm），
倾角峰值 **6.0°**，全程未摔倒。

#### 结果汇总

| 指标 | 数值 |
|---|---|
| 站立倾角（20 s） | **< 0.1°** |
| 行走躯干高度波动 | **< 1.5 cm** |
| 行走倾角峰值 | **6.0°** |
| 40 s 位移 | **(2.79, 1.67) m** |
| 平均前进速度 | **≈ 0.10 m/s** |
| 峰值速度标量 | 0.20 m/s |
| 稳定性（25 s 不摔倒） | **通过**（单测 `test_walk_makes_progress` 断言 > 1 m） |

### 5.9 常见问题

**Q1：`ModuleNotFoundError: No module named 'mujoco'`？**
装依赖（[5.3 节](#deps)）。只想验证算法用 `--mode offline`（不需要 MuJoCo）。

**Q2：机器人第一帧就被弹飞 / `QACC` 出现 NaN？**

先查 `ctrl` 的语义。本模型的 12 个关节是 `<position>` 执行器，`ctrl` 是**目标角**。
把力矩（数百 N·m）填进去，等效于要求关节转到几百弧度，位置伺服瞬间饱和。
下行契约必须是"根节点收力矩、关节收角度"（见 `BalanceController.joint_command`）。

**Q3：站立时躯干慢慢下沉，最后坐到地上？**

站立构型不能完全伸直。大腿/小腿各 0.34 m，与躯干到脚的落差相等，
站立处在运动学奇异点——实测 `knee_cmd=0.05` 就让躯干下降 0.58 m。
`stand_pose()` 默认 `knee_bias=0.30`，不要改成 0。

**Q4：行走走了十几步就倒 / 单腿支撑时倾倒？**

先看步态更新频率。20 Hz 的步态更新会让"摆动相开始"出现半个周期的跳变，
机器人被自己的目标阶跃踢翻。`main.py` 里 `gait_every` 是 100 Hz；
ROS 节点的 `ctrl_hz` 同理不要低于 100。

**Q5：把 `period` 调大就想走慢一点，结果反而倒了？**

`phase_rate` 不设 `1/period` 上限是有原因的：`0.77 cycle/s` 会因
**单腿支撑过久**而摔倒，本模型需要约 `3.85 cycle/s` 的踏步频率。
"走得慢"要靠减小 `speed_max`，不是拉长周期。详见 [6.3 节](#tuning)。

**Q6：IMU 读出来永远是 `(1,0,0,0)`？**

`framequat` 绑在 `site` 上，site 若没有定义朝向（缺 `zaxis`），
它不跟随 body 旋转，四元数恒等。模型里已经写了 `zaxis="0 0 1"`，
自行改模型时不要删。

**Q7：`--mode sim` 跑完机器人"停住不动了"？**

若日志最后是 `[t=..] ... FALLEN`，是摔倒了（`is_fallen` 判据：高度 < 0.55 m 或倾角 > 70°）。
若是正常结束，会打印 `"final_state": "REACHED"` 或 `"WALK"`。
两者用 `--json_out` 落盘后可直接核对。

---

## 6. 性能评价

### 6.1 指标定义 <span id="metrics"></span>

| 指标 | 符号 | 含义 |
|---|---|---|
| 躯干高度 | $z$ | 躯干 body 原点的世界 z 坐标 |
| 倾角 | $\theta_{\text{tilt}}$ | 躯干相对竖直方向的总倾角 $\arccos(1-2(x^2+y^2))$ |
| 站位保持误差 | $\mathrm{RMSE}_z$ | $z$ 相对目标高度的均方根偏差 |
| 平滑度 | $\text{AoS}$ | 相邻控制周期关节目标角变化量均值 |
| 到目标距离 | $d_{\text{goal}}$ | 当前位置到目标的欧氏距离 |

**站位保持误差**：

$$
\mathrm{RMSE}_z = \sqrt{\frac{1}{N}\sum_k \big(z_k - z_{\text{hold}}\big)^2}
$$

**平滑度**（越小指令越平滑）：

$$
\text{AoS} = \frac{1}{N-1}\sum_{k=2}^{N}\frac{1}{12}\sum_{i=1}^{12}\big|\,\theta_{i,k} - \theta_{i,k-1}\big|
$$

### 6.2 实测结果

| 指标 | 数值 |
|---|---|
| 站立 20 s 末倾角 | **< 0.1°** |
| 站立末高度 | **0.9438 m**（目标 0.946，误差 2 mm） |
| 行走躯干高度区间 | **0.946 ~ 0.960 m** |
| 行走倾角峰值 | **6.0°** |
| 40 s 位移 | **(2.79, 1.67) m** |
| 平均前进速度 | **≈ 0.10 m/s** |
| 25 s 行走不摔倒 | **通过** |
| 单测通过数 | **109 / 109** |

### 6.3 调优过程记录 <span id="tuning"></span>

这张表是本模块最有价值的部分——每一条都是实测踩出来的，参数值和因果都在里面。

| 优化项 | 优化前 | 优化后 | 说明 |
|---|---|---|---|
| `ctrl` 语义 | 填力矩 → 第一帧弹飞（dz=0.394 m） | 填**目标角** | `<position>` 执行器的 `ctrl` 是角度 |
| IMU site 朝向 | 缺 `zaxis`，`framequat` 恒等 | `zaxis="0 0 1"` | 姿态反馈此前全在读假值 |
| 场景物体 | 包成 `<body>`，质量 196 kg | world 下静态 `<geom>`，**22.7 kg** | 障碍柱曾让机器人背 155 kg |
| 站立构型 | 全关节 0（腿全伸） | `knee_bias=0.30` | 伸直处奇异：0.05 rad 屈膝 → 掉 0.58 m |
| 腿链拓扑 | 三个 body 各偏移 -0.34 但关节挂错 | `thigh→thigh_link→shank_link` | 错拓扑下髋关节一转会拽动整条腿 |
| 步态更新率 | 20 Hz → 1.6 s 摔倒 | **100 Hz** | 相位阶跃踢翻机器人 |
| `phase_rate` 上限 | `min(rate, 1/period)` = 0.77/s → 1.6 s 摔倒 | **去掉上限** = 3.85/s | 单腿支撑过久必倒 |
| 竖直高度 | 无补偿，屈膝即下沉 | `fz` 高度保持 PD | 抬脚期腿长缩短 10 cm |
| 踝策略增益 | `kp=0.9` → 发散 | 默认 **0**，可试验性开启 | 本模型上非零增益在摆动相发散 |
| 平衡增益 | 800/120 → 3.2 s 摔倒 | **3200/640** | 见 6.4 扫描 |
| 关节伺服刚度 | kp=900，dt=2 ms → NaN | kp≤400，**dt=0.5 ms** | 显式欧拉稳定域 `kp·dt²/J < 4` |

#### 平衡增益扫描（站立 8 s，站立指令固定）

| $K_p^{\text{bal}}$ / $K_d^{\text{bal}}$ | 峰值倾角 | 末段高度 | 是否摔倒 |
|---|---|---|---|
| 200 / 40 | 9.67° | 0.927 | 否 |
| 400 / 80 | 6.46° | 0.931 | 否 |
| 800 / 160 | 2.41° | 0.943 | 否 |
| 1600 / 320 | 1.05° | 0.947 | 否 |
| **3200 / 640** | **0.50°** | **0.948** | 否 |

增益越高越稳，但太高会在接触求解的高频模态上抖振；3200/640 是稳定上限附近。

#### 步态参数扫描（25 s，看位移与是否摔倒）

| stride | lift | period | 结果 |
|---|---|---|---|
| 0.20 | 0.10 | 1.3 | **前进 2.60 m，躯干 0.944~0.957** |
| 0.20 | 0.15 | 1.3 | 2.3 s 后倾角发散摔倒 |
| 0.30 | 0.25 | 1.2 | 1.9 s 后摔倒 |
| 0.30 | 0.15 | 1.2 | 12.0 s 后摔倒 |

`lift` 是最敏感的：抬腿越高，单腿支撑期躯干越难维持。

### 6.4 结论

- **站立有效性**：20 s 内倾角 < 0.1°、高度误差 2 mm，说明"姿态 PD + 高度保持"
  的串级结构足够支撑静态平衡。
- **行走有效性**：40 s 内位移 2.79 m，躯干高度波动 < 1.5 cm、倾角峰值 6.0°，
  说明"相位参数化 + 根节点平衡力"能在 MuJoCo 里产生连续行走。
- **感知有效性**：扇区可通行性直接进入速度标量与状态机切换；
  离线模式的 `WALK → AVOID → WALK → REACHED` 迁移序列证明这条链路是通的。
- **必须说明的边界**：本实现的平衡依赖模型层 6 个 `balance_*` 通用执行器通道
  （等价于外部扶正力），**不是纯关节驱动的 ZMP 行走**。因此不能把"能走"
  误读成"完全仿生"。行走速度约 0.10 m/s，转向能力有限、长距离会累积航向漂移；
  精确的落脚点规划（LIPM / MPC）与零力矩点约束留给后续工作。

## 参考

* [人形机器人的示例](https://openhutb.github.io/doc/pedestrian/humanoid/#humanoid_robot)
* [人形机器人基础（MuJoCo 环境配置）](./humanoid_basics.md)
* MuJoCo 文档：[Modeling](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
