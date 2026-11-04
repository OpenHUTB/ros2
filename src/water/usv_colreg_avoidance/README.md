# usv_colreg_avoidance

无人水面艇（USV）按《国际海上避碰规则》（COLREG）判断会遇局面并自动避让。

和陆上避障的区别在于：海上不是"看到障碍就绕"，而是**先判断两船构成哪种局面、
谁该让路**，再由让路船按规定的大幅度右转，直航船保持航向航速。规则判错了，
转向再漂亮也是错的。

## 覆盖的规则

| 局面 | 判定条件 | 本船角色 | 动作 |
|---|---|---|---|
| 对遇 Rule 14 | 目标在正前方 ±6°，航向大致相反（≥170°） | 双方均让路 | 各自向右转 |
| 交叉 Rule 15（右舷） | 目标相对方位 6°～112.5°，右舷为正 | 让路船 | 右转，从对方船尾通过 |
| 交叉 Rule 15（左舷） | 相对方位 -6°～-112.5° | 直航船 | 保向保速 |
| 追越 Rule 13 | 目标在正横后方 22.5° 扇区内 | 追越船让路 | 小角度右转超越 |
| 紧迫危险 Rule 17(b) | 直航船被迫逼近 D_emergency 仍未见对方动作 | 直航船 | 自行右转 |

转向幅度不是拍脑袋定的：按 `DCPA ≥ (1+margin)·D_safe` 反解最小转向角，
同时满足 Rule 16 的「大幅度」下限（对遇/交叉 30°，追越 15°）。

## 快速开始

不需要 ROS，只用 Python 标准库：

```bash
cd src/water/usv_colreg_avoidance
python3 -m usv_colreg_avoidance.main --offline --output results
```

会跑完四个场景，输出轨迹图、动画和一张结果表：

| 场景 | 不避让 | 避让后 | 碰撞 |
|---|---|---|---|
| 对遇 | 1.2 m | 754.8 m | 是 → 否 |
| 交叉 | 0.0 m | 438.1 m | 是 → 否 |
| 追越 | 0.6 m | 414.1 m | 是 → 否 |
| 三船 | 0.0 m | 438.2 m | 是 → 否 |

单元测试：

```bash
python3 -m pytest tests -v      # 20 项
```

## ROS 2 下运行

```bash
colcon build --packages-select usv_colreg_avoidance
source install/setup.bash
ros2 launch usv_colreg_avoidance main.launch.py
ros2 launch usv_colreg_avoidance main.launch.py scenario:=head_on
```

场景可选 `head_on` / `crossing` / `overtaking` / `multi`。

| 话题 / 服务 | 类型 | 说明 |
|---|---|---|
| `/usv_colreg/markers` | `visualization_msgs/MarkerArray` | 船位、轨迹、安全圈、角色标签 |
| `/usv_colreg/track_a` | `nav_msgs/Path` | 主船完整轨迹 |
| `/usv_colreg/status` | `std_msgs/String` | 逐帧的角色与局面文本 |
| `/usv_colreg/replay` | `std_srvs/Trigger` | 从头回放 |

参数：`scenario`（场景）、`d_safe`（安全会遇距离，默认 400 m）、
`playback_rate`（回放帧率）、`frame_step`（每帧推进的仿真步数）。

## 模型

- **船舶运动**：Nomoto 一阶操舵响应 `T·ṙ + r = K·δ`，T = 3 s、K = 0.17 (deg/s)/deg、
  最大舵角 35°，稳态转向率约 6°/s；只做转向机动，未建模主机减速。
- **决策**：每条船独立观测、独立判断，没有集中调度，所以能看到两船互相配合。
- **避让锁定**：动作一旦开始就锁定避让航向，直到目标过正横（≥110°）才回原航线，
  避免 DCPA 刚回到安全值就把舵回过来形成一连串小改动（Rule 8 明确禁止）。
- **让清判据**：追越时刚超过对方几十米就切回航线等于横穿对方船首，
  所以解除条件取「过正横 110° 且拉开距离」，而不是只看 DCPA。

## 已知限制

- 只做二维平面运动，未建模横摇、纵摇与波浪干扰；
- 目标是匀速直航的假设贯穿始终，对方机动时需要重算（真实系统靠持续雷达跟踪）；
- 只处理开阔水域，未考虑航道边界、浅水效应与分道通航制（TSS）；
- 未建模减速与倒车，实际避让中减速往往比转向更有效。
