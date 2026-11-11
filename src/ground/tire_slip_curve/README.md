# tire_slip_curve —— 轮胎滑移率-附着曲线核算

把一个「车轮在滑还是没在滑」的问题，做成一条**有闭式解**的附着曲线，用终端就能看清
三件事：峰值附着在哪、车轮抱死后附着掉多少、ABS 为什么要把滑移率圈在一个窄带里。

纯标准库实现，不依赖 CARLA、numpy、GUI；在 Ubuntu 18.04 + ROS Melodic（Python 2.7）
和 Ubuntu 20.04 + Noetic（Python 3）上都能直接跑。

## 为什么需要这个模块

纵向动力学里最容易被忽略的一条：**附着系数不是常数，它是滑移率的函数**。

- 滑移率太小时轮胎还没「咬住」路面，纵向力上不去；
- 滑移率跨过峰值之后附着开始下降，车轮彻底抱死时只剩峰值的两三成；
- 所以 ABS 的目标从来不是「不滑」，而是「把滑移率压在峰值附近的那条带子里」。

这三个量都能用一条曲线讲清楚，而这条曲线的极大值、初段斜率、锁死值都可以解析求出，
于是可以被断言精确校验，不必依赖实测拟合。

## 曲线模型

以归一化滑移率 `ŝ = s / s_peak` 表示：

```
μ(ŝ) = μ_peak · 2·ŝ / (1 + ŝ²)
```

| 位置 | 取值 | 含义 |
|---|---|---|
| `ŝ = 0`（s = 0） | `μ = 0` | 自由滚动，没有纵向力 |
| `ŝ = 1`（s = s_peak） | `μ = μ_peak` | 峰值附着，曲线唯一极大 |
| `s = 1.0`（车轮抱死） | `μ = μ_peak · 2·k / (1 + k²)`，`k = 1/s_peak` | 锁死附着 |

派生量也都是闭式解：

- 初段刚度 `dμ/ds|₀ = 2·μ_peak / s_peak`
- 峰值滑移率 `s_peak`
- ABS 目标带 `[s_peak·(1−band), s_peak·(1+band)]`

默认参数（`μ_peak = 0.90`、`s_peak = 0.12`）下：锁死附着 `0.2129`（只剩峰值的 **23.7%**），
初段刚度 `15.0`，目标带 `[0.096, 0.144]`。

## 目录结构

```
src/ground/tire_slip_curve/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/tire_slip_curve.yaml     # 12 个参数
├── launch/main.launch              # 12 个带 doc 的 arg
└── scripts/
    ├── main.py                     # 入口（符合 main.* 约定）
    ├── tire_core.py                # 纯数学，不 import rospy
    └── tire_node.py                # ROS 收发
tests/test_tire_slip_curve_local.py # 离线回归测试
docs/ground/tire_slip_curve.md      # 文档页
```

## 四条断言

| 断言 | 判据 |
|---|---|
| **峰值唯一** | 采样点里的局部极大只有一个，且落在 `(s_peak, μ_peak)` 上（容差 `peak_tolerance`） |
| **锁死衰减** | `μ(1.0) / μ_peak ≤ lock_friction_ratio`（默认 0.85） |
| **初段线性** | `s → 0` 时 `μ ≈ (dμ/ds|₀)·s`，相对误差 ≤ `linear_tolerance`（默认 2%） |
| **目标带覆盖峰值** | ABS 目标带必须包含 `s_peak` |

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `tire/peak_slip` | `0.12` | 峰值附着对应的滑移率（干路 0.10~0.15，冰雪 0.20~0.40） |
| `tire/peak_friction` | `0.90` | 峰值附着系数 |
| `tire/abs_band` | `0.20` | ABS 目标带宽比例 |
| `road/surface` | `dry_asphalt` | 路面标注：`dry_asphalt` / `wet_asphalt` / `snow` / `ice` |
| `curve/samples` | `51` | 曲线采样点数（≥5） |
| `check/lock_friction_ratio` | `0.85` | 锁死附着的上限比例 |
| `check/linear_tolerance` | `0.02` | 初段线性度容差 |
| `check/peak_tolerance` | `0.01` | 峰值位置/大小容差 |
| `rate/publish_hz` | `10.0` | 打印完后保持挂起的循环频率 |
| `topic/report` | `/tire/report` | 文本报告（latched, `std_msgs/String`） |
| `topic/curve` | `/tire/curve` | 曲线表 CSV 文本（latched） |
| `topic/peak_slip` | `/tire/peak_slip` | 峰值滑移率数值（latched, `std_msgs/Float64`） |

## 运行

```bash
roslaunch tire_slip_curve main.launch
```

改参数（示例）：

```bash
roslaunch tire_slip_curve main.launch peak_slip:=0.30 peak_friction:=0.35 surface:=snow
roslaunch tire_slip_curve main.launch lock_friction_ratio:=0.20
roslaunch tire_slip_curve main.launch samples:=5
```

打印完节点会保持挂起，方便另一个终端核对参数：

```bash
rosparam get /tire/peak_slip
rosparam get /check/lock_friction_ratio
```

## 实测输出（默认参数）

```
路面：dry_asphalt，峰值滑移率 0.120，峰值附着 0.900
锁死点：s=1.00，附着 0.2129（占峰值 23.7%）
初段刚度：15.000 1/s，ABS 目标带 [0.0960, 0.1440]（带宽 20%）
采样：51 点（s 从 0 到 1）
------------------------------------------------------------
附着曲线（纵轴 0 ~ 0.95）：
     *******
    *       **
   *          ***
   |             ****
   |                 ****
  *                      *******
  |                             **********
 *                                        *****************
 |                                                         *
 |
*     ...
0                                                滑移率 → 1.0
------------------------------------------------------------
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 峰值唯一  峰值 0.9000 @ s=0.1200（期望 0.9000 @ 0.1200），局部极大 1 个
  [PASS] 锁死衰减  锁死附着 0.2129，占峰值 23.7%（要求 <= 85.0%）
  [PASS] 初段线性  初段刚度 15.000，最大相对误差 0.99% @ s=0.0120（要求 <= 2.00%）
  [PASS] 目标带覆盖峰值  目标带 [0.0960, 0.1440] 覆盖峰值滑移 0.1200
结论：PASSED
```

## 两个反例

```bash
roslaunch tire_slip_curve main.launch samples:=5          # 采样太粗 → 峰值唯一失败
roslaunch tire_slip_curve main.launch lock_friction_ratio:=0.20   # 实测 23.7% > 20% → 锁死衰减失败
```

## 离线测试

```bash
cd ~/catkin_ws16
python3 tests/test_tire_slip_curve_local.py
```

不需要 ROS、不需要 CARLA、不需要显示器；在仓库根目录执行时还会顺带校验文档页。

## 与已有模块的分工

| 模块 | 关心什么 |
|---|---|
| `braking_distance_check` | 给定附着系数，算制动距离 |
| `turning_sweep_check` | 给定转角与轴距，算转弯时车身占多宽 |
| **`tire_slip_curve`（本包）** | **附着系数从哪来：滑移率决定它，曲线给出峰值与锁死衰减** |

换句话说：本包是上游，其它包把它算出的 `μ_peak` 当输入。

## 常见问题

**Q：为什么用这条简单的曲线，不用魔术公式（Pacejka）？**
A：魔术公式要拟合 4 个系数，没有解析极大值，断言只能靠数值搜索；本包要讲的是
「峰值 - 锁死 - 目标带」这三个概念，用有闭式解的曲线反而更容易被校验和讲清。真车
标定时把 `peak_slip`/`peak_friction` 换成实测值即可，曲线形状在 0~0.4 区间足够像。

**Q：Melodic 上运行时报 `UnicodeEncodeError`？**
A：rosgraph 的日志格式化对非 ASCII 的 unicode 会调 `str()`，节点里已经通过 `_log()`
统一转成 byte str，如果自己改代码请沿用这个函数。

---

所使用具体的大模型声明：使用大模型辅助代码编写、测试编写与文档整理，提交内容由本人负责。
