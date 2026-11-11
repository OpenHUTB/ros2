# gear_shift_policy —— 换挡策略与扭矩需求

把「该用几挡、要多少扭矩」从**经验**变成可复算的判据：给定一条纵向速度剖面，
按**发动机转速**而不是车速决定升降挡，并反推每一帧的扭矩需求，用四条断言判定策略是否合理。

纯 Python 标准库实现，**不需要 CARLA、不需要 GPU、不需要显示器**，输出全部是终端文本。

## 为什么需要这一层

`twist_to_control`（#614）把速度指令换成转角与轮速，`braking_distance_check`、`speed_profile_check`、
`turning_sweep_check` 分别算了「停得住吗」「怎么排这段路」「转弯占多宽」——
但都默认**动力可以随便取**。真实车辆上，动力来自一个有限扭矩、有限转速区间的发动机，
而挡位又决定「同样车速下发动机转多快」。这一层就是补这个缺口：

| 问题 | 本包的答案 |
|---|---|
| 什么时候升挡 | 转速超过 `upshift_rpm`（默认 2600） |
| 什么时候降挡 | 转速低于 `downshift_rpm`（默认 1500） |
| 为什么不会在临界点来回换 | 两条线之间有迟滞带，再加 `min_dwell` 最短保持时间 |
| 这一帧需要多少扭矩 | `F = m·a + ½ρCdA·v² + m·g·Crr` → 车轮扭矩 → 除以总传动比与效率 |
| 需求会不会超出引擎能力 | 与简化外特性 `max_torque(rpm)` 比，超出即判 `[FAIL] 扭矩上限` |

## 目录结构

```
gear_shift_policy/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── gear_shift_policy.yaml     # 24 项参数
├── launch/
│   └── main.launch                # 12 个带 doc 的 arg
└── scripts/
    ├── main.py                    # 入口（仓库约定以 main. 开头）
    ├── gear_core.py               # 纯逻辑：无 rospy、无 numpy、无 yaml
    └── gear_node.py               # ROS 收发
```

## 运行

```bash
roslaunch gear_shift_policy main.launch
```

默认参数（1.0 m/s² 加速到 30 m/s、保持 2 s、2.0 m/s² 减速）的真实输出：

```
换挡策略与扭矩需求（按转速判据 + 保持时间）
挡位传动比：3.55 / 2.10 / 1.40 / 1.00，主减速比 4.10，效率 0.92，车轮半径 0.31 m
升挡线 2600 rpm，降挡线 1500 rpm，最短保持 1.40 s
------------------------------------------------------------
帧数：941，时长：47.00 s，最高车速 30.00 m/s，最高转速 3789 rpm
换挡 6 次：
  t=5.80 s  1 -> 2  （5.80 m/s，2600 -> 1538 rpm）
  t=9.85 s  2 -> 3  （9.85 m/s，2612 -> 1742 rpm）
  t=14.75 s  3 -> 4  （14.75 m/s，2608 -> 1863 rpm）
  t=41.10 s  4 -> 3  （11.80 m/s，1490 -> 2086 rpm）
  t=42.80 s  3 -> 2  （8.40 m/s，1485 -> 2228 rpm）
  t=44.20 s  2 -> 1  （5.60 m/s，1485 -> 2511 rpm）
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 挡位覆盖 用到的挡位 [1, 2, 3, 4]（期望至少 4 个）
  [PASS] 换挡平顺 最短保持 1.40 s（要求 >= 0.50 s），1.0 s 内来回换挡 0 次
  [PASS] 转速区间 越界 0 帧（怠速 800，红线 6500，实测最高 3789）
  [PASS] 扭矩上限 超限 0 帧（峰值需求 172.7 N·m）
结论：PASSED
```

命令行覆盖示例：

```bash
# 升挡更晚（更运动），其余默认
roslaunch gear_shift_policy main.launch upshift_rpm:=4000.0

# 只跑 5 m/s 的短剖面：挡位覆盖必然失败
roslaunch gear_shift_policy main.launch max_speed:=5.0

# 红线压到 2000 rpm：转速区间必然失败
roslaunch gear_shift_policy main.launch redline_rpm:=2000.0

# 加速度提到 3.0：扭矩上限必然失败
roslaunch gear_shift_policy main.launch accel:=3.0
```

## 参数

`config/gear_shift_policy.yaml` 里共 24 项，launch 暴露了其中 12 项：

| 参数 | 默认 | 含义 |
|---|---|---|
| `policy/upshift_rpm` | 2600.0 | 升挡转速线 |
| `policy/downshift_rpm` | 1500.0 | 降挡转速线（必须低于升挡线） |
| `policy/min_dwell` | 0.50 | 换挡后最短保持时间 s |
| `policy/idle_rpm` | 800.0 | 怠速（低于此转速由离合器打滑补足） |
| `policy/redline_rpm` | 6500.0 | 红线转速 |
| `profile/accel` | 1.00 | 加速段加速度 m/s² |
| `profile/max_speed` | 30.00 | 定速段车速 m/s |
| `profile/hold_seconds` | 2.00 | 定速保持时长 s |
| `profile/decel` | 2.00 | 减速段减速度 m/s² |
| `engine/peak_torque` | 220.0 | 峰值扭矩 N·m |
| `engine/peak_rpm` | 4200.0 | 峰值扭矩转速 |
| `gearbox/final_drive` | 4.10 | 主减速比 |
| `gearbox/efficiency` | 0.92 | 传动效率 |
| `gearbox/ratio_1..4` | 3.55 / 2.10 / 1.40 / 1.00 | 各挡传动比 |
| `vehicle/mass` | 1500.0 | 整备质量 kg |
| `vehicle/wheel_radius` | 0.31 | 车轮半径 m |
| `sim/step` | 0.05 | 仿真步长 s |
| `rate/publish_hz` | 10.0 | 保活与重发频率 Hz |

## 话题

| 话题 | 类型 | 内容 |
|---|---|---|
| `/gear/current` | `std_msgs/Int32` | 末帧挡位 |
| `/gear/report` | `std_msgs/String` | 完整文字报告 |
| `/gear/curve` | `std_msgs/String` | 挡位条带 + 扭矩柱（字符图） |
| `/gear/shift` | `std_msgs/String` | 换挡事件表（t / from / to / 车速 / 前后转速） |

四个话题都是 latched，节点打报告后会**保持挂起**，方便另一个终端用 `rosparam get` 或
`rostopic echo` 取证，Ctrl+C 退出。

## 四条断言

| 断言 | 判据 | 失败示例 |
|---|---|---|
| `挡位覆盖` | 至少用到 4 个挡 | `max_speed:=5.0` → 只用 1 挡 |
| `换挡平顺` | 相邻换挡间隔 ≥ `min_dwell`，且窗口内无 A→B→A | `upshift_rpm:=1600 downshift_rpm:=1500` → 26 次换挡、振荡 20 次 |
| `转速区间` | 全程转速 ∈ [0.5×怠速, 红线] | `redline_rpm:=2000` → 631 帧越界 |
| `扭矩上限` | 每帧需求 ≤ `max_torque(rpm)` | `accel:=3.0` → 峰值需求 419.2 N·m |

## 离线回归测试

```bash
python3 tests/test_gear_shift_policy_local.py     # Python 3
pytest tests/test_gear_shift_policy_local.py      # pytest 也会真失败
```

只用标准库，不需要 ROS、不需要 CARLA、不需要显示器。

## 与其他模块的关系

| 模块 | 算什么 | 与本包的分工 |
|---|---|---|
| `twist_to_control`（#614） | 速度指令 → 转角 / 轮速 | 只做运动学换算，不管动力 |
| `speed_profile_check` | 加速/匀速/减速三段怎么排 | 输出速度曲线，本包接它的速度剖面 |
| `braking_distance_check` | 从看见到停住要几米 | 只算制动，本包算加速侧 |
| `steering_actuator_lag` | 转向执行器的滞后与回正 | 管横向执行器，本包管纵向动力源 |
| **本包** | **该用几挡、需要多少扭矩、够不够** | 把「动力够不够」变成可断言的四条判据 |

## 常见问题

**Q：为什么升挡判据用转速而不是车速？**
A：同样 60 km/h，一挡会 7800 rpm、四挡只有 2200 rpm。真实变速箱读的是发动机转速，
用转速判据能自动适配不同主减速比与轮胎半径。

**Q：为什么限定「换挡后最短保持」？**
A：只有迟滞带、没有保持时间时，在换挡线附近小幅波动仍可能连续换挡
（`upshift_rpm:=1600 downshift_rpm:=1500` 就是这种病态配置，振荡 20 次）。

**Q：为什么减速段扭矩是 0？**
A：减速靠制动器，发动机处于倒拖状态，本模型把负扭矩截到 0，避免把制动器的功算到发动机头上。

**Q：Melodic 上日志报 `UnicodeEncodeError`？**
A：Python 2.7 的 rosgraph 会对非 ASCII 的 unicode 调 `str()`，节点内的 `_log()` 已统一转成
byte str；自己加日志时请走 `_log("info", ...)` 而不是直接 `rospy.loginfo(u"中文")`。

---

使用大模型辅助代码编写、测试编写与文档整理，提交内容由本人负责。

## 闭环执行（默认模式）

从 `1.0.0` 起，节点默认走闭环：只给**目标车速剖面**，油门与刹车由比例控制器
给出，扭矩受发动机外特性限制，车速由「牵引力 − 阻力 − 制动力」积分得到。
换挡会反过来影响加速度 —— 挡位挂错、扭矩不够，车速就跟不上目标，这一点是
开环算法（先有车速、反推扭矩）看不出来的。

```text
$ roslaunch gear_shift_policy main.launch
换挡策略与扭矩需求（闭环执行：踏板 → 受限扭矩 → 车速积分，不依赖 CARLA）
挡位传动比：3.55 / 2.10 / 1.40 / 1.00，主减速比 4.10，效率 0.92，车轮半径 0.31 m
升挡线 2600 rpm，降挡线 1500 rpm，最短保持 1.40 s
------------------------------------------------------------
帧数：941，时长：47.00 s，行驶 734.59 m，最高车速 29.99 m/s，最高转速 3788 rpm
跟踪：最大偏差 0.90 m/s，平均偏差 0.57 m/s（最差第 939 帧）
换挡 6 次：
  t=6.25 s  1 -> 2  （5.80 m/s，2600 -> 1538 rpm）
  t=10.30 s  2 -> 3  （9.85 m/s，2612 -> 1742 rpm）
  t=15.20 s  3 -> 4  （14.75 m/s，2608 -> 1863 rpm）
  t=41.55 s  4 -> 3  （11.80 m/s，1490 -> 2086 rpm）
  t=43.25 s  3 -> 2  （8.40 m/s，1485 -> 2228 rpm）
  t=44.65 s  2 -> 1  （5.60 m/s，1485 -> 2511 rpm）
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 挡位覆盖 用到的挡位 [1, 2, 3, 4]（期望至少 4 个）
  [PASS] 换挡平顺 最短保持 1.40 s（要求 >= 0.50 s），1.0 s 内来回换挡 0 次
  [PASS] 跟踪误差 最大偏差 0.90 m/s（容差 2.50），最差在 t=46.90 s（目标 0.20，实到 1.10）
  [PASS] 转速区间 越界 0 帧（怠速 800，红线 6500，实测最高 3788）
结论：PASSED
```

四条断言也随之换成闭环版本：**挡位覆盖 / 换挡平顺 / 跟踪误差 / 转速区间**
（开环的「扭矩上限」在闭环里由 `clamp` 保证，不再需要断言）。

想回到开环（先有车速、反推扭矩、只判扭矩是否超限）：

```bash
roslaunch gear_shift_policy main.launch closed_loop:=false
```

闭环的两个增益与原话容差可调：

```bash
roslaunch gear_shift_policy main.launch driver/throttle_gain 5.0 \
                                    driver/brake_gain 3.0 \
                                    check/track_tolerance 1.0
```
