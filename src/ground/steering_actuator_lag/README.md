# steering_actuator_lag —— 转向执行器的滞后、死区与回正

一个**不依赖 CARLA**的转向执行器核算包：给定执行器的物理特性，算清"打一把方向"
到底要多久、停在哪、松手回正又要多久，并用四条断言把结果判成 PASS / FAIL。

## 为什么需要这一层

上层规划给出的转角是"瞬间到位"的理想值，真实的转向执行器不是：

| 真实特性 | 后果 | 本包怎么建模 |
|---|---|---|
| 电机/液压有惯性 | 打角要 0.3~1 s 才跟上 | 一阶惯性 `τ` |
| 连杆/齿条有间隙 | 指令只差一点时根本不动 | 死区 `±dead_band` |
| 电机有转速上限 | 大角度指令会被"限速" | 速率上限 `max_rate` |
| 回正靠回位弹簧/回正电机 | 回正往往比打角慢 | 独立的 `center_rate` |

所以"这条转向曲线行不行"不能被假定，得算出来 —— 这就是本包做的事。

## 模型

每一帧先算误差，再按一阶惯性 + 死区 + 速率限制推进：

```
误差    e = command − angle
死区    |e| < dead_band  →  不动
期望速率 v = e / τ
限速    v = clamp(v, ±cap)          cap = center_rate（回正段）或 max_rate（打角段）
推进    angle += v · dt,  angle = clamp(angle, ±max_angle)
```

`command` 在 `[0, dwell)` 内是阶跃幅值，之后归零（松开方向盘回正）。

## 四条断言

| 断言 | 判据 | 默认阈值 |
|---|---|---|
| 上升时间 | 角度到阶跃的 90% 所需时间 | ≤ 1.00 s |
| 稳态误差 | 保持段末端的 `\|command − angle\|` | ≤ 0.50 度 |
| 速率上限 | 全程实测最大角速度 | ≤ max(打角, 回正) 上限 |
| 回正时间 | 指令归零后回到 0.50 度以内所需时间 | ≤ 1.50 s |

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `actuator/time_constant` | 0.25 | 一阶惯性时间常数 (s) |
| `actuator/dead_band_deg` | 0.30 | 死区半宽 (度) |
| `actuator/max_rate_deg` | 60.0 | 打角速率上限 (度/s) |
| `actuator/center_rate_deg` | 40.0 | 回正速率上限 (度/s) |
| `actuator/max_angle_deg` | 35.0 | 机械最大转角 (度) |
| `test/step_deg` | 20.0 | 阶跃指令幅值 (度) |
| `test/dwell_seconds` | 5.0 | 阶跃保持时长 (s) |
| `test/steady_tolerance_deg` | 0.50 | 稳态误差容差 (度) |
| `test/rise_limit_seconds` | 1.00 | 上升时间上限 (s) |
| `test/center_limit_seconds` | 1.50 | 回正时间上限 (s) |
| `test/center_tolerance_deg` | 0.50 | 回正判定容差 (度) |
| `sim/step` | 0.01 | 积分步长 (s) |
| `rate/publish_hz` | 10.0 | 保活频率 (Hz) |

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/steering/report` | `std_msgs/String` | 完整报告（latched） |
| `/steering/curve` | `std_msgs/String` | 指令/实际角度的字符曲线（latched） |
| `/steering/rise_time` | `std_msgs/Float64` | 上升时间，未达到 90% 时发 −1.0（latched） |

## 运行

```bash
roslaunch steering_actuator_lag main.launch
```

默认输出（Ubuntu 18.04 + ROS Melodic 实测）：

```
工况：阶跃 20.00 度，保持 5.00 s，时间常数 0.25 s
死区：±0.30 度，打角上限 60.0 度/s，回正上限 40.0 度/s
---
到 90% 用时：0.580 s，超调 0.000 度
稳态角度：19.310 度，稳态误差 0.290 度
最大角速度：60.00 度/s
回正：归零后 0.970 s 回到容差内，最终 0.007 度
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 上升时间     到 90% 用时 0.580 s，上限 1.00 s
  [PASS] 稳态误差     稳态误差 0.290 度（死区 0.30 度），容差 0.50 度
  [PASS] 速率上限     最大角速度 60.00 度/s，上限 60.00 度/s（打角 60.0、回正 40.0）
  [PASS] 回正时间     回正到 0.50 度以内用时 0.970 s，上限 1.50 s
结论：PASSED
```

稳态角度 19.31 而不是 20.00，是因为死区 0.30 度：误差小于 0.30 时执行器不再动作，
所以停在 20 − 0.29 ≈ 19.31 度。这正是"死区"在真实系统里的表现。

## 三个反例

```bash
# 慢执行器：上升 2.30 s（超 1.00）、回正 3.65 s（超 1.50）→ 2 通过 / 2 失败
roslaunch steering_actuator_lag main.launch time_constant:=1.0

# 大死区：稳态误差 1.98 度（超 0.50）→ 3 通过 / 1 失败
roslaunch steering_actuator_lag main.launch dead_band_deg:=2.0

# 回正很弱：回正段在保活时间内没回到 0.50 度以内 → 3 通过 / 1 失败
roslaunch steering_actuator_lag main.launch center_rate_deg:=5.0
```

三个反例各只翻一条断言，说明四条断言是独立判的。

## 离线测试

```bash
python3 tests/test_steering_actuator_lag_local.py
```

只用标准库，不需要 ROS / 显示器，兼容 Python 2.7 与 3.x：

```
通过 190 项，失败 0 项
全部通过
```

也可以用 pytest 跑（每个分组是真断言，失败会红）：

```bash
python3 -m pytest tests/test_steering_actuator_lag_local.py -q
```

## 与已有模块的分工

| 模块 | 算的是 |
|---|---|
| `twist_to_control` | Twist 速度指令 → 转角与轮速（几何换算） |
| `turning_sweep_check` | 转弯时车身的横向包络（空间） |
| `trailer_kinematics` | 牵引车-挂车折刀（双状态量） |
| **本包** | 执行器**时间响应**：滞后、死区、回正 |

## 常见问题

**为什么稳态角度不等于阶跃幅值？** 死区。误差进入 `±dead_band` 后执行器不再动作，
这是真实齿条间隙/连杆虚位的数学表达；想让稳态更准就调小 `dead_band_deg`。

**为什么 `max_rate_deg` 调小以后速率上限还是 40 度/s？** 因为回正段用的是
`center_rate_deg`，断言取两者较大值。要整体限速需同时调小两个。

**Melodic 上日志要特别注意什么？** 发给 rospy 的日志消息必须是 byte str。
本包所有日志都经过 `_log()`，先在 Python 2.7 下把 unicode 编码成 utf-8 再交给
rospy，否则 rosgraph 的格式化会对 `str(unicode)` 抛 `UnicodeEncodeError`。

**ROS 参数为什么会被当成字符串？** `rosparam` 读出来都是 `str`。本包在
`ActuatorParams.__init__` 里统一用 `_as_float` 转换，避免 Python 2.7 下
`max(1.0, "10.0")` 直接 `TypeError`。

---

所使用具体的大模型声明：使用大模型辅助代码编写、测试编写与文档整理，提交内容由本人负责。
