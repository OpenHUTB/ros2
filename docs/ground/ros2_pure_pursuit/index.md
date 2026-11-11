# 地面载具 ROS2 差速机器人纯追踪路径跟踪与反应式避障

本文档介绍 `src/ground/ros2_pure_pursuit` 模块：一个差速移动机器人的**路径跟踪 + 激光反应式避障**实现。

与仓库中其他地面载具模块的区别：本模块**不依赖 CARLA / Gazebo / 任何仿真器**，
核心算法与闭环仿真只用 NumPy，因此在没有 GPU、没有仿真环境的机器上也能完整复现；
同时提供 ROS 2 (Humble) 节点与 `ros2 launch` 入口。

* 源代码目录：[`src/ground/ros2_pure_pursuit`](https://github.com/OpenHUTB/ros2/tree/master/src/ground/ros2_pure_pursuit)
* 一键复现：`cd src/ground/ros2_pure_pursuit && python3 sim/run_sim.py --path eight`
* 一键检查：`bash ci/run_ci.sh`（语法 → 依赖 → 测试 → 仿真 → 基准，五阶段）

## 1. 目标与系统架构

在 ROS2 环境下实现差速移动机器人的自主跟踪能力：

1. 沿任意给定的离散参考路径稳定行驶；
2. 路径上存在静态障碍时安全通过，不发生碰撞；
3. 代码在无 ROS 环境中同样可运行、可验证，便于在课程环境中快速复现。

```
                 ┌──────────────────────────────┐
   /scan  ──────▶│  ObstacleAvoider             │
                 │  前向扇区最小测距 → 危险度 r   │──▶ turn_bias
                 └──────────────────────────────┘   slow_factor
                                                      │
   /odom  ──────▶┌──────────────────────────────┐    │      ┌──────────┐
   参考路径 ─────▶│  PurePursuitController       │──▶ ω_pure ─▶│  /cmd_vel │
                 │  前视点 → 曲率 → 角速度        │──▶ v_ref ─▶└──────────┘
                 └──────────────────────────────┘
```

跟踪层（Pure Pursuit）负责几何意义上的「贴紧路径」；避障层（Obstacle Avoider）
负责安全。两层输出通过角速度叠加、线速度相乘的方式融合，无状态耦合，便于单独调参。

## 2. 运动学模型

差速驱动机器人在平面上的离散运动学：

$$x_{k+1} = x_k + v_k\cos\theta_k\,\Delta t$$

$$y_{k+1} = y_k + v_k\sin\theta_k\,\Delta t$$

$$\theta_{k+1} = \theta_k + \omega_k\,\Delta t$$

底层轮速由 $(v, \omega)$ 反解（`wheel_base` 为轮距 $L$，轮半径 $r$）：

$$\dot\varphi_L = \frac{v - \omega L/2}{r},\qquad \dot\varphi_R = \frac{v + \omega L/2}{r}$$

实现中加入了**加速度限幅**：$\Delta v$ 每周期不超过 $a_{\max}\Delta t$，
避免速度指令阶跃导致轮胎打滑，也便于在真车上做平滑控制。

## 3. 纯追踪控制器

### 3.1 前视点选取

在参考路径上寻找距离当前位置 $L_d$ 的点 $(x_g, y_g)$，并用线性插值保证
目标点始终落在路径折线上，而不是落在某个离散步之间。

前视距离有两层自适应——先按速度取基准值，再按前视点附近的路径曲率缩短：

$$L_{d,\text{speed}} = \mathrm{clip}(k|v| + L_{d0},\ L_{d,\min},\ L_{d,\max})$$

$$L_d = \mathrm{clip}\!\left(\frac{L_{d,\text{speed}}}{1 + g\,|\kappa_{\text{path}}|\,s},\ L_{d,\min},\ L_{d,\max}\right)$$

其中 $\kappa_{\text{path}}$ 是用前视点附近弧长 $s$ 上三点外接圆估出来的路径曲率，
$g$ 为 `curve_gain`（取 0 即退回纯速度自适应，用于对照实验）。

参数取值（`PurePursuitController`）：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `ld0` | 0.6 | 基础前视距离 (m) |
| `k` | 0.45 | 速度系数 (s) |
| `ld_min` | 0.4 | 前视距离下限 (m) |
| `ld_max` | 2.5 | 前视距离上限 (m) |
| `goal_tol` | 0.25 | 终点判定半径 (m) |
| `curve_gain` | 2.5 | 曲率自适应增益，0 = 关闭 |
| `curve_span` | 0.8 | 估计曲率所用的弧长窗口 (m) |

### 3.2 曲率与角速度

将前视点变换到车体坐标系（前向 $x$，左向 $y$）：

$$x_g' = \Delta x\cos\theta + \Delta y\sin\theta,\qquad y_g' = -\Delta x\sin\theta + \Delta y\cos\theta$$

航向偏差 $\alpha = \operatorname{atan2}(y_g', x_g')$，由纯追踪的经典几何关系，
过前视点的圆弧曲率为 $\kappa = 2\sin\alpha / L_d$，于是

$$\omega = \kappa v = \frac{2v\sin\alpha}{L_d}$$

### 3.3 稳定性说明

纯追踪在 $L_d$ 固定时对路径曲率变化是稳定的；前视距离随速度增大而增大，
本质上等价于在高速时降低控制增益，抑制了差速机器人在大曲率路径上的转向震荡。
本实现中 $L_d \ge L_{d,\min} = 0.4$ m 大于车长 0.36 m，可避免前视点落在车身内部导致的奇异。

### 3.4 曲率自适应前视（本版新增）

只按速度取 $L_d$ 时，S 形路径在曲率反号处会出现横向误差尖峰：前视点跨在弯道外侧，
机器人倾向于"抄近路"切内弯。改进办法是让 $L_d$ 随路径曲率缩短（上节公式中的第二层），
弯道上前视点自动拉近，直道上保持原值。

曲率用前视点附近弧长 `curve_span` 上三点外接圆估计（`Path.curvature()`），
比取相邻三点更抗离散噪声；闭合路径按取模回绕。

在期望速度 0.9 m/s、开启避障的条件下扫参（`sim/benchmark.py --curve-gain g`）：

| `curve_gain` | circle RMS | eight RMS | s_curve RMS | s_curve 最大误差 |
| ---: | ---: | ---: | ---: | ---: |
| 0（旧） | 0.0766 | 0.0699 | 0.0991 | 0.3198 |
| 2.5（默认） | 0.0680 | 0.0613 | 0.0871 | 0.2994 |
| 4.0 | 0.0632 | 0.0585 | 0.0821 | 0.2919 |
| 6.0 | 0.0579 | 0.0539 | 0.0765 | 0.2776 |

默认取 2.5：再往上收益变缓，但前视距离更短、弯道耗时略增（步数 636 → 671）。
速度提到 1.5 / 2.0 m/s 时同样单调改善且未出现震荡（$L_{d,\min}$ 兜底，前视点不会退到车身内部），
因此默认值在 0.9–2.0 m/s 区间内都可用。

## 4. 反应式避障

### 4.1 危险度

取车头前方 $\pm 60°$ 扇区内的最小测距 $d_{\min}$：

$$r = \mathrm{clip}\left(\frac{d_{\text{safe}} - d_{\min}}{d_{\text{safe}} - d_{\text{danger}}},\ 0,\ 1\right)$$

其中 $d_{\text{safe}} = 1.2$ m、$d_{\text{danger}} = 0.45$ m。

### 4.2 转向策略

比较扇区左右两半的平均余量，向**更空旷的一侧**施加偏置：

$$\omega_{\text{bias}} = \sigma \cdot g \cdot r,\qquad \sigma = \mathrm{sign}\left(\bar d_L - \bar d_R\right)$$

$g = 1.6$ rad/s。使用半扇区均值而不是最近点的单点距离，是为了避免被单个噪点带偏。

### 4.3 速度耦合

$$v \leftarrow v \cdot (1 - 0.85\,r)$$

危险度为 1 时速度降至 15%。这一点很关键：反应式避障的转向能力与速度直接相关，
转弯半径 $R = v/\omega$；不减速则 $R$ 过大，机器人来不及绕过障碍。

### 4.4 危险距离内的急停

仅靠减速仍然存在下限：上式在 $r=1$ 时仍保留 15% 速度，机器人会继续逼近障碍。
因此当最小测距进入危险距离之内时，直接判定为急停：

$$d_{\min} \le d_{\text{danger}} \;\Rightarrow\; v = 0$$

此时**保留转向偏置**，机器人会在原地转向直至前方让出通路，再恢复前进，
避免在障碍正前方「停死」。该行为由 `test_arbitration_emergency_stop_zero_speed_keeps_turn`
与 `test_simulation_survives_emergency_stop` 两项测试锁定。

## 5. 控制仲裁与安全兜底

每个控制周期最终下发哪一组 $(v, \omega)$ 由 `arbitrate()` 统一裁决——
节点层不再出现多个分支各自发布速度的情况，所有决策只有一个出口：

| 优先级 | 条件 | 结果 | `reason` |
| :---: | --- | --- | --- |
| 0 | `/odom` 超过 `odom_timeout`(0.5 s) 未更新，或从未收到 | 停车 $(0,0)$ | `stale_odom` |
| 1 | 到达路径终点 | 停车 $(0,0)$ | `finished` |
| 2 | 正常跟踪 | 采纳纯追踪输出 | `pursuit` |
| 3 | 前向有障碍 | 叠加转向偏置与减速 | `avoidance` |
| 4 | 前向测距 $\le d_{\text{danger}}$ | 急停 $v=0$，保留转向 | `emergency_stop` |

出口处统一把角速度截断到 $\pm$ `max_angular`(1.8 rad/s)，防止叠加偏置后
下发车辆执行不了的角速度。

三点设计考量：

1. **定位超时的优先级高于终点判断**：位姿都不可信时，「是否到达终点」本身无法确认，
   因此 `stale_odom` 排在 `finished` 之前；
2. **激光超时只停用避障、不停机**：`/scan` 断流后节点不再用过期扫描做安全决策，
   退化为纯跟踪并每 2 s 告警一次，而不是让机器人急停；
3. **可测试性**：`arbitrate()` 与 `safety.FreshnessTracker` 都不依赖 rclpy，
   上述全部规则可以在无 ROS 环境下被单元测试直接覆盖，
   离线仿真 `sim/run_sim.py` 也复用同一个 `arbitrate()`，保证仿真与节点行为一致。

节点同时对 `target_speed` 做合法性校验（NaN/非正数回退 0.8 m/s，过大值截断到 3.0 m/s），
并对空 `ranges` 显式停用避障。

## 6. 运行步骤

### 6.1 安装依赖

```bash
cd src/ground/ros2_pure_pursuit
pip3 install -r requirements.txt    # numpy, matplotlib
```

### 6.2 离线仿真（无需 ROS）

```bash
python3 sim/run_sim.py --path eight        # 可选 circle / eight / s_curve / straight
python3 sim/run_sim.py --path eight --no-avoid   # 关闭避障的对照组
python3 sim/benchmark.py                   # 生成 4 路径 × 有无避障 的基准矩阵
```

### 6.3 ROS2 中部署

```bash
colcon build --packages-select ros2_pure_pursuit
source install/setup.bash
ros2 launch ros2_pure_pursuit nav.launch.py
```

节点 `pure_pursuit_node` 以 20 Hz 运行（定时器周期 0.05 s），订阅 `/odom` 与 `/scan`，
发布 `/cmd_vel`。里程计四元数到 yaw 的转换使用标准公式：

$$\psi = \operatorname{atan2}\left(2(wz+xy),\ 1-2(y^2+z^2)\right)$$

## 7. 实验结果

### 7.1 场景设置

- 参考路径四种：圆形（$R=8$ m）、8 字形（利萨如曲线 $a=10$ m）、S 形（$L=24$ m，$A=3$ m）、直线（$L=20$ m）
- 障碍：沿路径法线方向偏移 0.80 m、半径 0.50 m 布置两处；
  该位置满足 $0.80 < 0.50 + 0.35$，即**关闭避障必然碰撞**、开启避障必须绕行
- 期望速度 0.9 m/s，控制周期 0.05 s，机器人等效外接半径 0.35 m

### 7.2 对照实验数据

| 路径 | 避障 | 最大横向误差 (m) | RMS (m) | 前向最小测距 (m) | 步数 | 碰撞 |
| --- | --- | ---: | ---: | ---: | ---: | :---: |
| circle | 开 | 0.279 | 0.068 | 0.705 | 1153 | 否 |
| circle | 关 | 0.052 | 0.004 | — | 283 | **是** |
| eight | 开 | 0.286 | 0.061 | 0.688 | 1386 | 否 |
| eight | 关 | 0.094 | 0.007 | — | 343 | **是** |
| s_curve | 开 | 0.299 | 0.087 | 0.659 | 651 | 否 |
| s_curve | 关 | 0.003 | 0.002 | — | 157 | **是** |
| straight | 开 | 0.317 | 0.116 | 0.727 | 475 | 否 |
| straight | 关 | 0.000 | 0.000 | — | 116 | **是** |

> 表中数据由 `python3 sim/benchmark.py`（默认 `curve_gain=2.5`）在本机生成；
> 与上一版（`curve_gain=0`，纯速度自适应）相比，RMS 误差下降 11%~12%，
> 最大横向误差下降 6%~12%，逐条增益对比见 3.4 节的扫参表。

### 7.3 轨迹与误差曲线（开启避障）

![8 字形路径跟踪轨迹与横向误差](eight.png)

![圆形路径跟踪轨迹与横向误差](circle.png)

![S 形路径跟踪轨迹与横向误差](s_curve.png)

![直线路径跟踪轨迹与横向误差](straight.png)

### 7.4 对照组（关闭避障）

![8 字形 - 关闭避障发生碰撞](eight_noavoid.png)

![圆形 - 关闭避障发生碰撞](circle_noavoid.png)

### 7.5 讨论

1. **跟踪精度**：开启避障时 RMS 误差 0.06~0.12 m，主要来自绕行阶段的横向偏移，
   绕行结束后误差迅速回落到厘米级；相比纯速度自适应（0.07~0.12 m）整体下降约 11%；
2. **安全性**：关闭避障时四条路径全部在第 116~343 步发生碰撞，
   开启后全部安全跑完，前向最小测距稳定在 0.66~0.73 m（大于机器人半径 0.35 m）；
3. **弯道性能**：S 形路径曲率反号处曾是误差尖峰（旧版最大 0.32 m）。启用曲率自适应前视后
   最大误差降到 0.299 m、RMS 从 0.099 降到 0.087；直线路径的最大误差（0.317 m）不随自适应变化，
   因为它来自绕障时的横向绕行，而不是纯追踪本身。

## 8. 测试与可复现性

`python3 tests/test_core.py` 共 **30 项**，全部通过；`bash ci/run_ci.sh` 一条命令跑完
语法 → 依赖 → 测试 → 仿真 → 基准 五个阶段。除运动学与几何外，其中包含针对上文
安全规则与曲率自适应的回归测试：

| 测试 | 锁定的行为 |
| --- | --- |
| `test_arbitration_finished_wins_over_avoidance` | 到达终点必须停车，即使避障正要求转向 |
| `test_arbitration_emergency_stop_zero_speed_keeps_turn` | 危险距离内 $v=0$ 且保留转向 |
| `test_avoider_emergency_stop_inside_danger_distance` | $d_{\min} \le d_{\text{danger}}$ 时急停 |
| `test_arbitration_stale_odom_stops_robot` | 定位超时必须停车 |
| `test_freshness_tracker_detects_stale_and_fresh` | 超时判定对多路数据源互不干扰 |
| `test_sanitize_target_speed_rejects_bad_values` | 非法速度参数回退/截断 |
| `test_arbitration_angular_speed_is_clamped` | 角速度不超过 `max_angular` |
| `test_simulation_survives_emergency_stop` | 急停介入后闭环仿真仍不碰撞 |
| `test_path_curvature_matches_circle_and_straight` | 曲率估计：半径 8 m 圆 ≈ 0.125，直线 ≈ 0 |
| `test_lookahead_shrinks_on_curvature` | 弯道前视距离小于直道，且不低于 `ld_min` |
| `test_lookahead_curve_gain_zero_keeps_speed_only` | `curve_gain=0` 与旧行为逐位一致 |

## 9. 局限与展望

- 避障为反应式，只处理前方扇区内的障碍，无法处理被遮挡后突然出现的物体；
- 未做全局重规划，绕行后仍沿原路径回归，可能与静态地图产生冲突；
- 仿真使用合成激光，尚未在 Gazebo / CARLA 中接入真实话题验证；
- 未考虑车辆动力学（轮胎侧滑、电机力矩饱和），真车部署前需补充 PID 速度环。
