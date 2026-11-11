# drivetrain_torsion_state_feedback 双惯量传动扭振的状态反馈极点配置

电机经弹性轴驱动车轮时会产生扭振（本算例 25.7 Hz、阻尼比仅 0.134）。本模块用
**Ackermann 极点配置**设计状态反馈，把闭环阻尼比提到 0.7，并量化调节时间从 0.18 s
降到 0.038 s。

与仓库已有模块的区别：

- `lateral_stability_margin`、`rollover_margin` 讨论整车稳态转向与侧翻裕度；
- `ride_comfort_transmissibility` 讨论悬架传递率（垂向）；
- 本模块讨论的是**传动链扭转模态**的控制，属于动力总成而不是底盘。

## 1. 运行环境

- 操作系统：Ubuntu 22.04.5 LTS（实测）
- ROS 版本：ROS 2 Humble
- Python：3.10.12
- 依赖：`rclpy`、`std_msgs`；离线入口与测试只用标准库

## 2. 编译与测试

```bash
cd ~/ros2 && source /opt/ros/humble/setup.bash
colcon build --packages-select drivetrain_torsion_state_feedback
source install/setup.bash
colcon test --packages-select drivetrain_torsion_state_feedback
```

## 3. 运行

```bash
cd src/ground/drivetrain_torsion_state_feedback
python3 -m drivetrain_torsion_state_feedback.main
```

实测输出：

```text
双惯量传动：Jm=0.05 kg·m²，Jw=0.6 kg·m²，k=1200.0 Nm/rad，c=2.0 N·m·s/rad
扭振固有频率 161.2 rad/s = 25.66 Hz，开环阻尼比 0.134
开环极点 -21.667+159.783j / -21.667-159.783j

能控性
  能控性矩阵 [B AB] = [[0.0000, 21.6667], [21.6667, -938.8889]]
  det = -469.4444  →  可控

期望极点 -112.872+115.152j / -112.872-115.152j（zeta=0.70）
Ackermann 反馈增益 K = [-0.0000, 8.4189]
闭环极点 -112.872+115.152j / -112.872-115.152j  → zeta=0.700，wn=161.2 rad/s

自由响应对比（初始扭角 0.01 rad，u=0 vs u=-Kx）
方案             峰值扭角(rad)     调节时间(s)     阻尼比     残余扭角(rad)
开环                  0.010000        0.1800     0.134        0.000012
状态反馈              0.010000        0.0380     0.700        0.000000
```

ROS 2 节点：

```bash
ros2 launch drivetrain_torsion_state_feedback main.launch.py
# 关掉反馈做开环对比
ros2 launch drivetrain_torsion_state_feedback main.launch.py feedback_enabled:=false
```

- `/drivetrain_torsion/theta`（`Float64`）：轴扭转角（rad）
- `/drivetrain_torsion/torque`（`Float64`）：反馈转矩（Nm）
- `/drivetrain_torsion/status`（`String`）：开/闭环阻尼比、极点、增益

## 4. 模型与设计

```text
x1' = x2
x2' = S*(u - k*x1 - c*x2),   S = 1/Jm + 1/Jw
```

- `A = [[0,1], [-S·k, -S·c]]`，`B = [[0],[S]]`
- `wn = sqrt(S·k) = 161.2 rad/s`，`2·zeta·wn = S·c` → `zeta = 0.134`
- 能控性矩阵 `[B AB]` 的 `det = -S²·k ≠ 0`，系统可控
- Ackermann：`K = [0 1]·[B AB]⁻¹·(A² + a1·A + a0·I)`

**一个值得注意的结果**：因为目标只是补阻尼、保持固有频率不变，`K` 的第一项恰好为 0，
闭环实际只用了相对转速反馈——工程上的含义是「用电气阻尼替代机械阻尼」，
这正是主动扭振抑制的做法。

## 5. 为什么不用加大机械阻尼

把阻尼比补到临界阻尼需要轴阻尼 `c = 14.884 N·m·s/rad`，是当前的 **7.4 倍**；
状态反馈不需要改机械结构就能达到同样的目标阻尼，这是本模块的工程动机。

## 6. 简化假设

- 只保留扭振模态，忽略整车纵向运动与齿轮间隙、齿隙死区；
- 轴刚度与阻尼视为常数，不含温度/老化影响；
- 状态反馈假设**扭转角与相对转速都可测**（实际工程中扭转角往往要靠观测器估计，
  见仓库里的 `wheel_speed_observer` 模块）；
- 反馈转矩未考虑电机电流环带宽与电压限幅，仿真中允许任意大转矩；
- 未做参数摄动（k、c、J 变化）下的鲁棒性验证。

## 7. 大模型使用声明

使用大模型辅助生成代码、测试与文档；作者已复核模型推导、Ackermann 实现与实测数据，
并对提交内容负责。
