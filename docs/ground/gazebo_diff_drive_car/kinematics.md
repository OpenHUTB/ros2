# 差速驱动运动学

---

## 5.1 差速驱动原理

差速驱动（Differential Drive）是最常见的轮式移动机器人构型。
其核心思想是：**通过左右两侧驱动轮的转速差实现转向**，
无需独立的转向机构。

```
                 俯视图

              v_l                 v_r
               ↑                   ↑
               │                   │
        ┌──────┴───────────────────┴──────┐
        │                                 │
        │          L（轮间距）             │
        │  ●─────────────────────────●    │
        │  左轮                     右轮   │
        │                                 │
        └─────────────────────────────────┘
                       │
                       ▼  v（车体线速度）

   v_l = v_r  →  直线前进
   v_l < v_r  →  向左转弯
   v_l > v_r  →  向右转弯
   v_l = -v_r →  原地旋转
```

---

## 5.2 正运动学

### 5.2.1 速度合成

已知左右轮线速度 $v_l, v_r$，求车体线速度 $v$ 与角速度 $\omega$：

$$
v = \frac{v_r + v_l}{2}
$$

$$
\omega = \frac{v_r - v_l}{L}
$$

其中 $L$ 为左右轮中心距。

**推导**：

设车辆绕**瞬时旋转中心**（ICR）转动，该中心位于车轴延长线上，
距车体中心 $R$ 处。

```
                    ICR（瞬时旋转中心）
                     ●
                    ╱│
                   ╱ │
              R+L/2  │ R
                 ╱   │
                ╱    │
        ┌──────╱─────┼─────╲──────┐
        │     ●      │      ●    │
        │   左轮     │     右轮   │
        └───────────┴────────────┘
              R-L/2
```

由刚体转动的线速度关系 $v = \omega R$：

- 左轮：$v_l = \omega (R - L/2)$
- 右轮：$v_r = \omega (R + L/2)$
- 车体：$v = \omega R$

两式相减：$v_r - v_l = \omega L$，即 $\omega = (v_r - v_l)/L$ ✓

两式相加：$v_r + v_l = 2\omega R = 2v$，即 $v = (v_r + v_l)/2$ ✓

### 5.2.2 转向半径

由 $R = v/\omega$，代入上式：

$$
R = \frac{(v_r + v_l)/2}{(v_r - v_l)/L} = \frac{L(v_r + v_l)}{2(v_r - v_l)}
$$

**特殊情况**：

| 条件 | 转向半径 | 运动形式 |
|---|---|---|
| $v_l = v_r$ | $R \to \infty$ | 直线行驶 |
| $v_l = -v_r$ | $R = 0$ | 原地旋转 |
| $v_l = 0$ | $R = L/2$ | 绕左轮转动 |
| $\lvert v_l \rvert \ne \lvert v_r \rvert$ 且同号 | $R > L/2$ | 大半径转弯 |

---

## 5.3 逆运动学

### 5.3.1 由车体速度求轮速

实际控制中，我们给定车体速度 $(v, \omega)$，需要反解左右轮速：

$$
v_l = v - \frac{\omega L}{2}, \qquad v_r = v + \frac{\omega L}{2}
$$

再由轮半径 $r$ 换算为**角速度**：

$$
\omega_l = \frac{v_l}{r} = \frac{v - \omega L/2}{r}
$$

$$
\omega_r = \frac{v_r}{r} = \frac{v + \omega L/2}{r}
$$

### 5.3.2 本项目的实现方式

本项目**不直接计算轮速**，而是发布 `geometry_msgs/Twist` 消息
到 `/cmd_vel`，由 `gazebo_ros_diff_drive` 插件完成上述解算：

```xml
<plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">
  <leftJoint>left_wheel_joint</leftJoint>
  <rightJoint>right_wheel_joint</rightJoint>
  <wheelSeparation>0.335</wheelSeparation>   <!-- L -->
  <wheelDiameter>0.195</wheelDiameter>        <!-- 2r -->
  <commandTopic>cmd_vel</commandTopic>
  <odometryTopic>odom</odometryTopic>
  <robotBaseFrame>base_footprint</robotBaseFrame>
  <odometryFrame>odom</odometryFrame>
</plugin>
```

**这样做的好处**：控制器逻辑与具体底盘解耦，更换底盘只需改 URDF 参数。

### 5.3.3 代入本车参数

本车 $L = 0.335$ m，$r = 0.0975$ m：

$$
\omega_l = \frac{v - 0.1675\,\omega}{0.0975}, \qquad
\omega_r = \frac{v + 0.1675\,\omega}{0.0975}
$$

**验算**：若 $v = 0.2$ m/s，$\omega = 0$：

$$
\omega_l = \omega_r = \frac{0.2}{0.0975} = 2.051 \text{ rad/s}
$$

这与实测的车轮角速度 **2.052 rad/s** 高度吻合 ✓

---

## 5.4 位姿积分

### 5.4.1 连续形式

车体位姿 $(x, y, \theta)$ 的演化方程：

$$
\begin{aligned}
\dot{x} &= v \cos\theta \\
\dot{y} &= v \sin\theta \\
\dot{\theta} &= \omega
\end{aligned}
$$

### 5.4.2 离散积分

Gazebo 以固定步长 $\Delta t$（本项目 1 ms）积分：

$$
\begin{aligned}
x_{k+1} &= x_k + v_k \cos\theta_k \cdot \Delta t \\
y_{k+1} &= y_k + v_k \sin\theta_k \cdot \Delta t \\
\theta_{k+1} &= \theta_k + \omega_k \cdot \Delta t
\end{aligned}
$$

> **注意**：这是**一阶欧拉积分**，在转向时存在累积误差。
> 更精确的做法是使用中点法或龙格-库塔法。由于 Gazebo 步长极小（1 ms），
> 一阶积分的误差在实际应用中可忽略。

### 5.4.3 里程计发布

`gazebo_ros_diff_drive` 插件订阅 `/cmd_vel` 后：

1. 解算左右轮角速度
2. 通过 `SetVelocity` 作用于车轮关节
3. 从 Gazebo 获取真实位姿（`odometrySource = 1` 表示世界坐标系）
4. 发布 `/odom`（`nav_msgs/Odometry`）
5. 发布 `odom → base_footprint` 的 TF 变换

---

## 5.5 运动约束与限制

### 5.5.1 非完整约束

差速驱动是**非完整约束系统**（nonholonomic）：

$$
\dot{x}\sin\theta - \dot{y}\cos\theta = 0
$$

物理含义：**车辆不能横向平移**（不能像全向轮那样侧滑）。
这决定了：

- 车辆无法直接沿 Y 方向移动
- 要到达侧向目标点，必须通过"前进 + 转向"的组合
- 路径规划时必须考虑车辆的最小转向半径

### 5.5.2 速度限制

| 限制 | 值 | 约束来源 |
|---|---|---|
| 最大线速度 | 0.8 m/s | 键盘控制上限（保证激光雷达来得及响应） |
| 最大角速度 | 1.2 rad/s | 键盘控制上限 |
| 最大线加速度 | 1.5 m/s² | 仿真参数 |
| 最大角加速度 | 1.5 rad/s² | 仿真参数 |
| 车轮最大扭矩 | 20 N·m | 仿真参数 |

**为什么限制线速度？**

车辆以速度 $v$ 行驶时，激光雷达扫描周期 $T = 100$ ms 内的位移为：

$$
\Delta s = v \cdot T
$$

- $v = 0.35$ m/s → $\Delta s = 0.035$ m
- $v = 0.8$ m/s → $\Delta s = 0.08$ m

安全阈值 $d_{\text{danger}} = 0.45$ m，即在 0.45 m 处触发紧急制动时，
高速情况下单周期仍只前进 0.08 m，留有充足的反应余量。

---

## 5.6 参数敏感性分析

### 5.6.1 轮间距 $L$ 的影响

由 $\omega = (v_r - v_l)/L$ 可知：

- $L$ **偏大** → 同样的轮速差产生的角速度偏小，转向迟钝
- $L$ **偏小** → 转向过于灵敏，容易震荡

URDF 中的 `wheelSeparation` 必须**准确对应**模型中的实际轮距，
否则会出现：

- 里程计角速度与实际不符 → 轨迹漂移
- TF 树与视觉模型不匹配

**本项目验证**：URDF 中 `wheel_separation = 0.335`，
与 `gazebo_ros_diff_drive` 插件的 `wheelSeparation` 一致。

### 5.6.2 轮半径 $r$ 的影响

由 $v = \omega_{\text{wheel}} \cdot r$ 可知，$r$ 直接影响里程计的
线速度估算。$r$ 设置偏大 → 里程计认为走得比实际远。

**本项目验证**：

- URDF 中 `wheel_radius = 0.0975` m
- 插件中 `wheelDiameter = 0.195` m（= 2r）
- 实测：发布 $v = 0.2$ m/s → 轮角速度 2.052 rad/s
- 理论：$0.2 / 0.0975 = 2.051$ rad/s ✓ **吻合**

---

## 5.7 实测验证

### 5.7.1 直线行驶验证

实验条件：$v = 0.2$ m/s，$\omega = 0$，运行 58.4 秒。

| 指标 | 理论值 | 实测值 | 偏差 |
|---|---|---|---|
| 行驶距离 | $0.2 \times 58.4 = 11.68$ m | 6.34 m | 偏小* |
| Y 方向漂移 | 0 | 0.16 m | 0.16 m |
| 路径效率 | 1.000 | **0.997** | 0.003 |

> \* 实测距离偏小是因为**前 22 秒用于启动 Gazebo 和生成模型**，
> 车辆尚未开始运动；实际有效运行时间约 32 秒，
> 对应理论距离 $0.2 \times 32 = 6.4$ m，与实测 6.34 m 吻合。

**结论**：运动学模型正确，直线行驶无侧滑。

### 5.7.2 转向验证

从激光雷达避障实验的轨迹可见，车辆完成了多次转向动作，
转向总量达 **51.05 rad**（约 8.1 圈），
轨迹 X 范围 −5.21 ~ 5.07 m，Y 范围 −5.26 ~ 1.12 m。

**结论**：差速转向控制有效，转向方向与算法指令一致。

### 5.7.3 轮速验证

```
$ rostopic echo -n 2 /joint_states
name: [left_wheel_joint, right_wheel_joint]
position: [168.925, 168.928]        ← 角度（rad）
velocity: [2.052]                     ← 角速度（rad/s）
```

发布 $v = 0.2$ m/s 时：

- 理论轮角速度 = $0.2 / 0.0975 = 2.051$ rad/s
- 实测轮角速度 = **2.052 rad/s**

**误差 0.05%**，验证了轮半径参数的正确性。
