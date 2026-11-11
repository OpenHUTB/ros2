# rollover_margin —— 横向载荷转移与侧翻裕度

一辆车以 60 km/h 过半径 30 m 的弯，内侧车轮还剩多少垂向力？再快多少会抬轮？
这个包用三个参数（轮距、质心高度、质量）把答案算成一组可以断言的数字。

纯标准库，不依赖 CARLA / numpy / GUI；Melodic（Python 2.7）与 Noetic（Python 3）都能跑。

## 为什么需要这个模块

侧翻不是「车太轻」或「路太滑」，而是**横向载荷转移超过了内侧轮的静态载荷**。
准静态下只有三个量在起作用：

```
静态稳定因子    SSF    = track / (2·h_cg)
侧翻临界加速度  a_crit = SSF · g
载荷转移        ΔF_z   = m·a_y·h_cg / track
```

于是内侧轮 `F_inner = m·g/2 − ΔF_z`，抬轮恰好发生在 `ΔF_z = m·g/2`，
即 `a_y = g·track/(2h_cg) = a_crit` —— 两条定义自洽，`ΔF_z/F_z0` 恒等于 `a_y/a_crit`。
给定半径还能反推极限车速 `v_max = sqrt(a_crit·R)`。

这些全是代数式，可以被断言精确校验，不需要仿真器。

## 默认参数下的数字

车型：轮距 `1.54 m`、质心高 `0.55 m`、质量 `1500 kg`；工况：半径 `30 m`、车速 `60 km/h`。

| 量 | 值 |
|---|---|
| SSF | `1.4000` |
| 临界加速度 | `13.734 m/s²`（1.40 g） |
| 侧向加速度 | `9.259 m/s²` |
| 裕度 `a_y/a_crit` | `0.674` |
| 载荷转移 | `4960.3 N`（占静态 67.4%） |
| 内侧轮 / 外侧轮 | `2397.2 N` / `12317.8 N` |
| 该半径极限车速 | `73.1 km/h`（余量 13.1 km/h） |

车速扫描（R = 30 m）：

| 车速 | `a_y` | 裕度 | 内侧轮 | 抬轮 |
|---|---|---|---|---|
| 30 | 2.315 | 0.169 | 6117.4 | 否 |
| 40 | 4.115 | 0.300 | 5152.9 | 否 |
| 50 | 6.430 | 0.468 | 3912.8 | 否 |
| 60 | 9.259 | 0.674 | 2397.2 | 否 |
| 70 | 12.603 | 0.918 | 606.0 | 否 |
| 80 | 16.461 | 1.199 | −1460.8 | **是** |
| 90 | 20.833 | 1.517 | −3803.2 | **是** |

抬轮落在 70~80 km/h 之间，与极限车速 73.1 km/h 吻合。

## 四条断言

| 断言 | 判据 | 失败意味着 |
|---|---|---|
| **侧翻裕度** | `a_y/a_crit ≤ rollover_margin`（默认 0.70） | 过弯太快或车太高 |
| **内侧轮载荷** | `F_inner > 0` | 已抬轮，准静态模型失效 |
| **极限车速** | `v ≤ sqrt(a_crit·R)` | 超过该半径的物理上限 |
| **载荷转移** | `|ΔF_z/F_z0 − a_y/a_crit| ≤ tolerance` | 实现不自洽 |

前三条讲安全性，第四条讲实现正确性 —— 所以抬高质心会让前三条一起翻，第四条仍然通过。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `vehicle/track` | `1.54` | 轮距（m） |
| `vehicle/cg_height` | `0.55` | 质心高度（m） |
| `vehicle/mass` | `1500.0` | 质量（kg） |
| `vehicle/gravity` | `9.81` | 重力加速度 |
| `corner/radius` | `30.0` | 弯道半径（m） |
| `corner/speed_kmh` | `60.0` | 过弯车速（km/h） |
| `check/rollover_margin` | `0.70` | 裕度上限 |
| `check/transfer_tolerance` | `0.02` | 自洽性容差 |
| `rate/publish_hz` | `10.0` | 打印完保持挂起的循环频率 |
| `topic/report` | `/rollover/report` | 文本报告（latched） |
| `topic/loads` | `/rollover/loads` | 车速扫描 CSV（latched） |
| `topic/critical_speed` | `/rollover/critical_speed` | 极限车速 Float64（km/h） |

## 运行

```bash
roslaunch rollover_margin main.launch
roslaunch rollover_margin main.launch speed_kmh:=80
roslaunch rollover_margin main.launch rollover_margin:=0.5
roslaunch rollover_margin main.launch cg_height:=0.90
```

打印完节点保持挂起，方便另一个终端核对：

```bash
rosparam get /vehicle/track
rosparam get /check/rollover_margin
rosparam get /vehicle/cg_height
```

## 实测输出（默认）

```
车辆：轮距 1.54 m，质心高 0.55 m，质量 1500 kg
SSF = 1.4000，侧翻临界加速度 13.734 m/s²（1.40 g）
工况：半径 30.0 m，车速 60.0 km/h（16.67 m/s）
侧向加速度 9.259 m/s²，裕度比值 0.674
载荷转移 4960.3 N（占比 0.674），内侧 2397.2 N / 外侧 12317.8 N
该半径极限车速 73.1 km/h（余量 13.1 km/h）
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 侧翻裕度  a_y 9.259 m/s²，a_crit 13.734 m/s²，比值 0.674（上限 0.70）
  [PASS] 内侧轮载荷  内侧轮 2397.2 N，外侧轮 12317.8 N（静态 7357.5 N）
  [PASS] 极限车速  当前 60.0 km/h，该半径极限 73.1 km/h（余量 13.1 km/h）
  [PASS] 载荷转移  ΔF_z/F_z0 = 0.6742，a_y/a_crit = 0.6742
结论：PASSED
```

## 两个反例

```bash
roslaunch rollover_margin main.launch rollover_margin:=0.5   # 只翻「侧翻裕度」
roslaunch rollover_margin main.launch cg_height:=0.90        # 裕度 + 内侧轮 + 极限车速 一起翻
```

第二个更有意思：质心抬到 0.90 m 后 SSF 掉到 `0.8556`，临界加速度 `8.393 m/s²`
低于 60 km/h 所需的 `9.259`，于是内侧轮载荷变成 **−759.4 N**（物理上已经翻了），
极限车速掉到 `57.1 km/h`；而「载荷转移」依然通过，因为它校验的是模型自洽而非安全性。

## 离线测试

```bash
cd ~/catkin_ws17
python3 tests/test_rollover_margin_local.py
```

不需要 ROS / CARLA / 显示器。

## 与已有模块的分工

| 模块 | 关心什么 |
|---|---|
| `tire_slip_curve` | 附着系数从哪来（滑移率决定 μ） |
| `braking_distance_check` | 纵向：从看到到停住要几米 |
| `turning_sweep_check` | 转弯时车身占多宽（几何包络） |
| **`rollover_margin`（本包）** | **横向：载荷转移多少、何时抬轮、极限车速** |

`turning_sweep_check` 回答「这个弯转得过去吗（不压线）」，本包回答「这个弯转得过去吗（不翻车）」。

## 常见问题

**Q：为什么不建模悬架和轮胎侧偏？**
A：准静态阈值主要由 `track/2h_cg` 决定，悬架柔度只让实际阈值低 5~10%。本包要把公式讲清楚，
闭式解更容易校验；真车标定时把 `rollover_margin` 收到 0.6 即可覆盖这一偏差。

**Q：内侧轮载荷为负代表什么？**
A：代表该侧车轮已离地，准静态模型到此失效（真实车辆此时绕外侧轮倾覆）。所以断言只判 `> 0`。

---

所使用具体的大模型声明：使用大模型辅助代码编写、测试编写与文档整理，提交内容由本人负责。
