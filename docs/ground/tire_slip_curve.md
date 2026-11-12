# 轮胎滑移率-附着曲线

## 这一页解决什么问题

纵向控制里有个常被跳过的前提：**附着系数不是常数**。

车轮从自由滚动到完全抱死，纵向力先升后降 —— 峰值出现在一个很窄的滑移率区间里，
过了峰值附着反而变差，抱死时只剩峰值的两三成。ABS 的全部设计要点都在这条曲线上：
不是「不让车轮滑」，而是「把滑移率压在峰值附近」。

本页给出一个可以离线跑、可以断言、不需要 CARLA 的实现，把这条曲线画在终端里，
把峰值、锁死值和目标带都算成具体数字。

## 曲线模型

以归一化滑移率 `ŝ = s / s_peak` 表示：

```
μ(ŝ) = μ_peak · 2·ŝ / (1 + ŝ²)
```

| 位置 | 取值 | 含义 |
|---|---|---|
| `s = 0` | `μ = 0` | 自由滚动 |
| `s = s_peak` | `μ = μ_peak` | 峰值附着（唯一极大） |
| `s = 1.0` | `μ_peak · 2k/(1+k²)`，`k = 1/s_peak` | 车轮抱死 |

闭式派生量：

- 初段刚度：`dμ/ds|₀ = 2·μ_peak / s_peak`
- ABS 目标带：`[s_peak(1−band), s_peak(1+band)]`
- 锁死占比：`μ(1.0) / μ_peak`

默认参数（`μ_peak = 0.90`、`s_peak = 0.12`、`band = 0.20`）下：

| 量 | 值 |
|---|---|
| 峰值附着 | `0.9000` @ `s = 0.1200` |
| 锁死附着 | `0.2129`（占峰值 **23.7%**） |
| 初段刚度 | `15.000 1/s` |
| ABS 目标带 | `[0.0960, 0.1440]` |

对照：`s = 0.06`（半峰值滑移）时 `μ = 0.72`，正好是峰值的 80% —— 滑移率只有峰值的一半，
纵向力已经掉了两成，这就是「轮胎工况窗口很窄」的量化说法。

## 四种路面

`road/surface` 只影响报告标注与默认值选择，实际数字仍由 `peak_slip` / `peak_friction` 决定。
常用量级：

| 路面 | `peak_slip` 典型值 | `peak_friction` 典型值 |
|---|---|---|
| `dry_asphalt` | 0.10 ~ 0.15 | 0.85 ~ 1.00 |
| `wet_asphalt` | 0.12 ~ 0.20 | 0.55 ~ 0.75 |
| `snow` | 0.20 ~ 0.40 | 0.25 ~ 0.40 |
| `ice` | 0.30 ~ 0.50 | 0.08 ~ 0.15 |

## 四条断言

| 断言 | 判据 | 失败意味着 |
|---|---|---|
| **峰值唯一** | 采样点局部极大只有一个，且落在 `(s_peak, μ_peak)` 上 | 采样太粗抓不到峰值，或曲线形状被改坏 |
| **锁死衰减** | `μ(1.0)/μ_peak ≤ lock_friction_ratio` | 曲线没体现出「过度滑移导致附着下降」 |
| **初段线性** | `s→0` 时 `μ ≈ (2μ_peak/s_peak)·s`，相对误差 ≤ `linear_tolerance` | 曲线在原点附近不光滑（真车会有初始刚度段） |
| **目标带覆盖峰值** | 目标带必须包含 `s_peak` | 参数组合自相矛盾，ABS 会控制在错误的滑移区间 |

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `tire/peak_slip` | `0.12` | 峰值滑移率（必须 `(0, 1]`） |
| `tire/peak_friction` | `0.90` | 峰值附着系数（必须 > 0） |
| `tire/abs_band` | `0.20` | 目标带宽比例（必须 `[0, 1)`） |
| `road/surface` | `dry_asphalt` | 路面标注 |
| `curve/samples` | `51` | 采样点数（≥5，默认步长 0.02） |
| `check/lock_friction_ratio` | `0.85` | 锁死附着上限比例 |
| `check/linear_tolerance` | `0.02` | 初段线性度容差 |
| `check/peak_tolerance` | `0.01` | 峰值容差 |
| `rate/publish_hz` | `10.0` | 挂起时循环频率 |
| `topic/report` | `/tire/report` | 报告话题 |
| `topic/curve` | `/tire/curve` | 曲线表话题 |
| `topic/peak_slip` | `/tire/peak_slip` | 峰值滑移率话题 |

## 话题

| 话题 | 类型 | 内容 |
|---|---|---|
| `/tire/report` | `std_msgs/String`（latched） | 完整文本报告（含字符图与四条断言） |
| `/tire/curve` | `std_msgs/String`（latched） | 曲线表 CSV：`slip,friction,ratio`，51 行 |
| `/tire/peak_slip` | `std_msgs/Float64`（latched） | 峰值滑移率 |

## 运行

```bash
cd ~/catkin_ws16
catkin_make
source devel/setup.bash
roslaunch tire_slip_curve main.launch
```

改参数：

```bash
roslaunch tire_slip_curve main.launch peak_slip:=0.30 peak_friction:=0.35 surface:=snow
```

另一终端核对：

```bash
rosparam get /tire/peak_slip
rosparam get /check/lock_friction_ratio
rosparam get /road/surface
```

## 实际输出（默认参数）

```
路面：dry_asphalt，峰值滑移率 0.120，峰值附着 0.900
锁死点：s=1.00，附着 0.2129（占峰值 23.7%）
初段刚度：15.000 1/s，ABS 目标带 [0.0960, 0.1440]（带宽 20%）
采样：51 点（s 从 0 到 1）
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 峰值唯一  峰值 0.9000 @ s=0.1200（期望 0.9000 @ 0.1200），局部极大 1 个
  [PASS] 锁死衰减  锁死附着 0.2129，占峰值 23.7%（要求 <= 85.0%）
  [PASS] 初段线性  初段刚度 15.000，最大相对误差 0.99% @ s=0.0120（要求 <= 2.00%）
  [PASS] 目标带覆盖峰值  目标带 [0.0960, 0.1440] 覆盖峰值滑移 0.1200
结论：PASSED
```

## 两个反例

**采样太粗（`samples:=5`）**

```bash
roslaunch tire_slip_curve main.launch samples:=5
```

只有 5 个采样点（步长 0.25），峰值的 `s = 0.12` 落在两个采样点之间，抓到的最大值是
`s = 0.25` 处的 `0.7020` —— 比真峰值低 22%。于是 `峰值唯一` 失败：

```
[FAIL] 峰值唯一  峰值 0.7020 @ s=0.2500（期望 0.9000 @ 0.1200），局部极大 1 个
```

**锁死衰减不达标（`lock_friction_ratio:=0.20`）**

```bash
roslaunch tire_slip_curve main.launch lock_friction_ratio:=0.20
```

实测锁死附着占峰值 23.7%，高于要求的 20%：

```
[FAIL] 锁死衰减  锁死附着 0.2129，占峰值 23.7%（要求 <= 20.0%）
```

## 离线测试

```bash
cd ~/catkin_ws16
python3 tests/test_tire_slip_curve_local.py
```

测试只依赖标准库（不需要 ROS / CARLA / 显示器）。在仓库根目录执行时还会检查
`docs/ground/tire_slip_curve.md` 是否存在、参数表与命令是否完整。

## 与已有模块的分工

| 模块 | 关心什么 |
|---|---|
| `braking_distance_check` | 给定附着系数算制动距离 |
| `turning_sweep_check` | 给定转角与轴距算转弯包络 |
| `reverse_parking_planner` | 倒车入库轨迹 |
| **`tire_slip_curve`（本页）** | **附着系数本身：它由滑移率决定，有峰值也有锁死衰减** |

本包是上游 —— 其它模块把这里的 `peak_friction` 当输入用。

## 常见问题

**Q：为什么不用魔术公式（Pacejka）？**
A：魔术公式需要拟合 4 个系数、没有解析极大值，断言只能靠数值搜索。本页要讲清的是
「峰值 - 锁死 - 目标带」这三个概念，有闭式解的曲线更容易校验。真车标定时把
`peak_slip` / `peak_friction` 换成实测值即可，0~0.4 区间形状足够接近。

**Q：Melodic 上为什么不能直接用 `rospy.loginfo(fmt, *args)`？**
A：rosgraph 的格式化会对非 ASCII 的 unicode 调 `str()`，在 Python 2.7 下抛
`UnicodeEncodeError`。节点里的 `_log()` 把消息统一编码成 byte str 后再发。

**Q：为什么打印完节点不退出？**
A：留出时间让另一个终端 `rosparam get` 核对参数（launch 的 arg 覆盖是否生效）。
按 `Ctrl+C` 退出。

---

所使用具体的大模型声明：使用大模型辅助代码编写、测试编写与文档整理，提交内容由本人负责。
