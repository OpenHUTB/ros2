# 双惯量传动扭振的状态反馈极点配置

电机经弹性轴驱动车轮，会形成一个 25 Hz 左右的扭转模态。本页用 Ackermann 极点配置
把它从阻尼比 0.134 提到 0.7，调节时间从 0.18 s 降到 0.038 s。

## 1. 为什么会有扭振

电机转子（Jm=0.05 kg·m²）与车轮（Jw=0.6 kg·m²）之间靠一根有刚度的轴连接，
轴刚度 k=1200 Nm/rad、阻尼 c=2 N·m·s/rad。相对转速一旦被激励，两者就会反相振荡，
整车表现为起步/换挡时的"抖动"，严重时会触发齿面敲击。

## 2. 模型

```text
x1 = 轴扭转角 theta（rad）
x2 = 相对转速 omega_m - omega_w（rad/s）
u  = 电机转矩指令（Nm）
S  = 1/Jm + 1/Jw = 21.667

x1' = x2
x2' = S*(u - k*x1 - c*x2)

A = [[0, 1], [-S*k, -S*c]] = [[0,1],[-26000,-43.333]]
B = [[0],[S]] = [[0],[21.667]]
wn = sqrt(S*k) = 161.2 rad/s = 25.66 Hz
2*zeta*wn = S*c  →  zeta = 0.134
```

## 3. 极点配置

能控性矩阵 `[B AB] = [[0, 21.667],[21.667, -938.889]]`，`det = -469.44 ≠ 0`，可控。
Ackermann 公式 `K = [0 1]·[B AB]⁻¹·(A² + a1·A + a0·I)`，期望极点取
`-112.87 ± 115.15j`（wn 不变、ζ=0.7）：

```text
K = [-0.0000, 8.4189]
闭环极点 -112.872 ± 115.152j  →  ζ = 0.700，wn = 161.2 rad/s
```

`K` 的第一项恰为 0，说明**保持固有频率时只需要相对转速反馈**——用电气阻尼替代机械阻尼，
这正是主动扭振抑制的工程含义。

## 4. 效果

| 方案 | 峰值扭角 | 调节时间（±2%） | 阻尼比 | 残余扭角 |
| --- | --- | --- | --- | --- |
| 开环 | 0.010000 rad | 0.1800 s | 0.134 | 0.000012 rad |
| 状态反馈 | 0.010000 rad | **0.0380 s** | 0.700 | 0.000000 rad |

如果靠机械阻尼达到临界阻尼，需要 `c = 14.884 N·m·s/rad`，是现在的 **7.4 倍**。

## 5. 构建与运行

```bash
cd ~/ros2 && source /opt/ros/humble/setup.bash
colcon build --packages-select drivetrain_torsion_state_feedback && source install/setup.bash
ros2 launch drivetrain_torsion_state_feedback main.launch.py
ros2 launch drivetrain_torsion_state_feedback main.launch.py feedback_enabled:=false   # 开环对比
cd src/ground/drivetrain_torsion_state_feedback && python3 -m drivetrain_torsion_state_feedback.main
```

话题：`/drivetrain_torsion/theta`、`/drivetrain_torsion/torque`、`/drivetrain_torsion/status`。

## 6. 已知限制

1. 只保留扭振模态，忽略整车纵向运动、齿隙与齿轮间隙；
2. k、c 视为常数，不含温度与老化；
3. 假设扭转角与相对转速都可测，实际工程中扭转角要靠观测器估计；
4. 反馈转矩未考虑电流环带宽与电压限幅；
5. 未做 k、c、J 摄动下的鲁棒性验证。

## 7. 大模型使用声明

使用大模型辅助生成代码、测试与文档；作者已复核模型推导与实测数据，并对提交内容负责。
