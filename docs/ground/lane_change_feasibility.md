# 换道可行性判定

## 这一页解决什么问题

高速上换道的三种典型误判——「后视镜没车就行」「前车还远够用」「慢慢挪更安全」——都不是手的问题，而是**判断**的问题。

这一页给出一个不依赖 CARLA 的算法包：**把「现在能不能换道」算成一个可以打印、可以回归的结论**，并给出「不能换的话，差多少、要等多久、要降到多少速」。

## 换道剖面

用正弦横向加速度剖面，保证起点与终点的横向速度、横向加速度都为零，方向盘不回甩：

```
a_y(t) = A · sin(2πt/T)
y(t)   = (A·T / 2π) · ( t − (T/2π)·sin(2πt/T) )
y(T) = L   ⇒   A = 2πL / T²   ⇒   T = sqrt(2πL / A)
```

- 峰值横向速度 = `2L/T`
- 换道期间自车沿车道方向推进 `distance = v_ego · T`
- 横向位移 `L` 即车道中心到中心（默认 3.5 m）

取 `L = 3.5 m`、`A = 1.5 m/s²` 得 `T = 3.8289 s`；把 `A` 降到 0.5，`T` 涨到 `6.6319 s`，前向占用从 85 m 涨到 147 m —— 这就是「慢慢挪更危险」的量化版本。

## 三条约束（四条断言）

| 断言 | 判据 | 默认阈值 |
|---|---|---|
| 换道时长 | `T ≤ duration_limit` | 5.00 s |
| 横向加速度 | `A ≤ max_lateral_accel` | 1.50 m/s² |
| 前向间隙 | `front_gap ≥ max(0, v_ego−v_front)·T + front_margin` | 余量 5.00 m |
| 后向 TTC | `rear_gap / (v_rear−v_ego) ≥ ttc_min`（后车不快于自车时恒通过） | 3.00 s |

后向所需间隙同理：`rear_gap ≥ max(0, v_rear−v_ego)·T + rear_margin`。

## 参数（14 项）

| 参数 | 默认 | 含义 |
|---|---|---|
| `lane/width` | 3.50 | 车道中心距，即横向位移 (m) |
| `lane/max_lateral_accel` | 1.50 | 横向加速度上限 (m/s²) |
| `lane/duration_limit` | 5.00 | 换道时长上限 (s) |
| `lane/direction` | 1 | 1 向左，-1 向右 |
| `safety/ttc_min` | 3.00 | 后方 TTC 下限 (s) |
| `safety/front_margin` | 5.00 | 前向余量 (m) |
| `safety/rear_margin` | 5.00 | 后向余量 (m) |
| `traffic/ego_speed_kmh` | 80.0 | 自车速度 |
| `traffic/front_speed_kmh` | 60.0 | 前车速度 |
| `traffic/front_gap` | 40.0 | 前车间距 (m) |
| `traffic/rear_speed_kmh` | 90.0 | 后车速度 |
| `traffic/rear_gap` | 30.0 | 后车间距 (m) |
| `sim/step` | 0.02 | 采样步长 (s) |
| `rate/publish_hz` | 10.0 | 发完报告后的保活频率 (Hz) |

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/lane_change/report` | `std_msgs/String` | 完整报告（latched） |
| `/lane_change/lane` | `std_msgs/String` | 三车道字符俯视图（latched） |
| `/lane_change/decision` | `std_msgs/Bool` | `true` = 可以换道 |
| `/lane_change/duration` | `std_msgs/Float64` | 换道时长 (s) |

## 运行

```bash
roslaunch lane_change_feasibility main.launch
```

默认输出（真机实测）：

```
换道方向：向左，车道宽度 3.50 m，横向加速度上限 1.50 m/s^2
自车 80.0 km/h（22.22 m/s），目标车道前车 60.0 km/h 距 40.0 m，后车 90.0 km/h 距 30.0 m
------------------------------------------------------------
换道时长：3.829 s（上限 5.00 s）
换道距离：85.088 m（自车 22.22 m/s 乘以时长）
峰值横向加速度：1.500 m/s^2，峰值横向速度：1.828 m/s
所需前向间隙：26.272 m（实际 40.000 m）
所需后向间隙：15.636 m（实际 30.000 m）
前车 TTC：7.200 s
后车 TTC：10.800 s（下限 3.00 s）
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 换道时长   换道用时 3.83 s，上限 5.00 s
  [PASS] 横向加速度  峰值横向加速度 1.50 m/s^2，上限 1.50 m/s^2
  [PASS] 前向间隙   前向间隙 40.00 m，需要 26.27 m（差 -13.73 m）
  [PASS] 后向 TTC 后车 TTC 10.80 s，下限 3.00 s
决策：可以换道
建议：当前时机可行，按 3.83 s 的剖面执行，换道期间推进 85.1 m
结论：PASSED
```

## 两个反例

**前车太近**（`ego_speed_kmh:=100 front_gap:=40`）：

```
换道距离：106.359 m（自车 27.78 m/s 乘以时长）
所需前向间隙：47.544 m（实际 40.000 m）
断言：3 通过 / 1 失败（共 4 条）
  [FAIL] 前向间隙   前向间隙 40.00 m，需要 47.54 m（差 7.54 m）
决策：不建议换道
  - 前向间隙 40.00 m 不足（需要 47.54 m）
建议：前向间隙还差 7.5 m：把车速降到 92.9 km/h 以下，或等前车拉开后再换
结论：FAILED
```

**后车逼近**（`rear_kmh:=120 rear_gap:=12`）：

```
所需后向间隙：47.544 m（实际 12.000 m）
后车 TTC：1.080 s（下限 3.00 s）
断言：3 通过 / 1 失败（共 4 条）
  [FAIL] 后向 TTC 后车 TTC 1.08 s，下限 3.00 s
决策：不建议换道
  - 后向间隙 12.00 m 不足（需要 47.54 m）
  - 后车 TTC 1.08 s 低于下限 3.00 s
建议：后车 120.0 km/h 正在逼近，TTC 只有 1.08 s：等它拉开到 33.3 m 以外，或等它超车通过
建议：提速到 113.4 km/h 以上可以缩小后向占用
结论：FAILED
```

## 离线测试

```bash
python3 tests/test_lane_change_feasibility_local.py
```

只用标准库：不需要 ROS、不需要 CARLA、不需要显示器；Python 2.7 与 3.x 都能跑。

## 与已有模块的分工

| 模块 | 负责 |
|---|---|
| `twist_to_control` | Twist 指令 → 转角与轮速 |
| `turning_sweep_check` | 转弯时车身占多宽 |
| `braking_distance_check` | 从看见到停住要几米 |
| `speed_profile_check` | 一段路上的时间-速度曲线 |
| **`lane_change_feasibility`** | **横向挪一个车道，前后车给不给你这个机会** |

## 常见问题

**Q：为什么不做成可视化？**
A：这个包要回答的是「数值上能不能换」，终端报告加上字符俯视图已经够读；可视化会引入 GUI 依赖，反而不利于在其他机器上复现。

**Q：换道时长上限为什么是 5 s？**
A：常见工程经验值。横向加速度 1.5 m/s² 时 3.5 m 车道约 3.83 s，留一点余量；想更保守就 `duration_limit:=4.0`。

**Q：TTC 为什么只看后车？**
A：前车的风险是「追尾」，用间隙与相对速度衡量更直观；后车的风险是「被追」，用 TTC 更直观。两者分开算，方便定位是哪一侧不满足。

**Q：Melodic 上日志报编码错？**
A：节点里所有日志都经 `_log()` 转成 byte str 再交给 rospy，不要改成 `rospy.loginfo(fmt, *args)` 形式。

---

所使用具体的大模型声明：使用大模型辅助代码编写、测试编写与文档整理，提交内容由本人负责。

## TF 广播（ROS 原生形态）

节点算完轨迹后会持续广播变换（`tf/enabled` 默认 `true`，频率 `tf/rate_hz` 默认 10 Hz），
因此除了看终端报告，还可以用 ROS 自带的工具核对位姿：

```
$ rosrun tf tf_echo map ego
$ rosrun tf view_frames     # 导出一份 tf 树到 frames.pdf
```

| parent | child | 含义 |
|---|---|---|
| `map` | `map -> ego` | 自车在全局地图里的位姿（换道过程中的关键帧） |


广播的是换道过程中的 **24 个关键帧**循环播放：`x` 从 0 均匀涨到换道距离
`85.09 m`，`y` 从 `0` 涨到 `3.50 m`（车道宽度）再回到 `0`（这里是一次换道，
所以只涨不回），`yaw` 由 `atan2(横向速度, 纵向速度)` 得出，两端为 0、
中途最大约 `0.08 rad`。用 `tf_echo` 连看几行就能看出这条横向曲线。

依赖：`tf2_ros`、`geometry_msgs`（`package.xml` 已声明）。关掉广播用
`roslaunch lane_change_feasibility main.launch tf_enabled:=false`。
