# 转向执行器滞后核算

## 这一页解决什么问题

上层规划给出的转角是"瞬间到位"的理想值，真实的转向执行器不是：

- 电机、减速器、液压助力都有惯性，打一把方向要 0.3~1 s 才跟上；
- 齿条与连杆之间存在间隙和虚位，指令只差一点点时执行器根本不动；
- 电机有转速上限，大角度指令会被"限速"；
- 回正往往靠回位弹簧或独立回正电机，比打角慢。

判断"这条转向曲线行不行"不能靠假定，要把它算出来。本页对应的
`src/ground/steering_actuator_lag` 包就是做这件事的：给执行器的物理特性，
算上升时间、稳态误差、角速度上限与回正时间，并给出 PASS / FAIL 结论。
纯 Python 标准库实现，**不需要 CARLA、不需要 GPU、不需要显示器**。

## 模型

每一帧先算误差，再按一阶惯性 + 死区 + 速率限制推进：

```
误差    e = command − angle
死区    |e| < dead_band        →  不动
期望速率 v = e / τ
限速    v = clamp(v, ±cap)     cap = center_rate（回正段）或 max_rate（打角段）
推进    angle += v · dt，angle = clamp(angle, ±max_angle)
```

指令在 `[0, dwell)` 内是阶跃幅值，之后归零（相当于松开方向盘回正）。
一阶惯性意味着**没有超调**，这是本包实测 `超调 0.000 度` 的原因。

## 四条断言

| 断言 | 判据 | 默认阈值 |
|---|---|---|
| 上升时间 | 角度到阶跃的 90% 所需时间 | ≤ 1.00 s |
| 稳态误差 | 保持段末端的 `\|command − angle\|` | ≤ 0.50 度 |
| 速率上限 | 全程实测最大角速度 | ≤ max(打角, 回正) 上限 |
| 回正时间 | 指令归零后回到 0.50 度以内所需时间 | ≤ 1.50 s |

四条断言各自独立：三个反例分别只翻其中一条，说明它们没有被同一个量牵着走。

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

全部参数在 `config/steering_actuator_lag.yaml` 里有默认值，`launch/main.launch`
把它们逐个暴露成同名 `<arg>`，所以命令行可以覆盖：

```bash
roslaunch steering_actuator_lag main.launch time_constant:=1.0 dead_band_deg:=2.0
```

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/steering/report` | `std_msgs/String` | 完整报告（latched） |
| `/steering/curve` | `std_msgs/String` | 指令/实际角度的字符曲线（latched） |
| `/steering/rise_time` | `std_msgs/Float64` | 上升时间；未达到 90% 时发 −1.0（latched） |

节点算完一次就常驻（按 `rate/publish_hz` 保活），`Ctrl+C` 退出，不需要额外的定时器。

## 运行与实际输出

```bash
roslaunch steering_actuator_lag main.launch
```

Ubuntu 18.04 + ROS Melodic 实测：

```
转向执行器节点已启动：时间常数 0.25 s，死区 ±0.30 度，打角上限 60.0 度/s，回正上限 40.0 度/s
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

稳态角度是 19.31 而非 20.00，差值 0.29 度正是死区的宽度 —— 误差小到 0.30 度以内
执行器就不再动作。这是真实系统里最容易忽略、也最容易被误判成"标定不准"的现象。

## 三个反例

| 参数 | 结果 | 说明 |
|---|---|---|
| `time_constant:=1.0` | `2 通过 / 2 失败` | 上升 2.30 s（超 1.00）、回正 3.65 s（超 1.50） |
| `dead_band_deg:=2.0` | `3 通过 / 1 失败` | 稳态误差 1.98 度（超 0.50） |
| `center_rate_deg:=5.0` | `3 通过 / 1 失败` | 回正段在保活时间内没回到 0.50 度以内 |

## 离线测试

```bash
python3 tests/test_steering_actuator_lag_local.py
```

不需要 ROS，兼容 Python 2.7 / 3.x。也可以用 pytest：

```bash
python3 -m pytest tests/test_steering_actuator_lag_local.py -q
```

每个分组都是真断言，失败会抛 `AssertionError`，不会出现"pytest 全绿但脚本非零退出"的假通过。

## 与已有模块的分工

| 模块 | 算的是 |
|---|---|
| `twist_to_control` | Twist 速度指令 → 转角与轮速（几何换算） |
| `turning_sweep_check` | 转弯时车身的横向包络（空间） |
| `trailer_kinematics` | 牵引车-挂车的折刀与稳定性（双状态量） |
| `reverse_parking_planner` | 倒车入库的轨迹几何 |
| **本页对应模块** | 执行器的**时间响应**：滞后、死区、回正 |

前者回答"方向盘该打多少度"，本包回答"这 20 度要多久才能到位、松手多久能回正"。

## 常见问题

**为什么稳态角度不等于阶跃幅值？** 死区。误差进入 `±dead_band` 后执行器不再动作，
这是齿条间隙/连杆虚位的数学表达。想让稳态更准，就调小 `actuator/dead_band_deg`。

**`max_rate_deg` 调小以后，为什么速率上限仍能到 40 度/s？** 因为回正段用的是
`center_rate_deg`，而"速率上限"断言取两者较大值。要整体限速需同时调小两个。

**为什么上升时间断言用 90% 而不是 100%？** 一阶惯性理论上永远到不了目标值
（指数逼近），工程上统一用 90% 作为上升时间定义。

**Melodic 上运行要注意什么？** 发给 rospy 的日志消息必须是 byte str。本包所有日志
都经过 `_log()`，在 Python 2.7 下先把 unicode 编码成 utf-8 再交给 rospy，否则
rosgraph 的格式化会对 `str(unicode)` 抛 `UnicodeEncodeError`。

**ROS 参数为什么会被当成字符串？** `rosparam` 读出来都是 `str`。本包在
`ActuatorParams.__init__` 里统一经 `_as_float` 转换，避免 Python 2.7 下
`max(1.0, "10.0")` 直接 `TypeError`。

---

所使用具体的大模型声明：使用大模型辅助代码编写、测试编写与文档整理，提交内容由本人负责。
