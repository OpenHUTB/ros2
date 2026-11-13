# trailer_jackknife_guard —— 半挂车折叠风险与倒车限速保护

半挂车倒车时，只要方向盘有一点转角，铰接角就会**指数发散**，最后「折」成 V 形
（jackknife）。本模块给出可实算的预测与保护：

- 铰接角速率 `phi' = -(v/L2)·sin(phi) - (v·tan(delta))/L1`；
- 给定当前铰接角与转角，预测**多久会折**、**允许的最大倒车速度**；
- 三档风险（注意 / 危险 / 折叠）+ 三组对照场景（无保护倒车、带保护倒车、前进自稳）。

## 1. 为什么前进稳、倒车折

对铰接角求导：

```
phi' = -(v / L2) * sin(phi) - (v * tan(delta)) / L1
```

- 前进（`v > 0`）：`d(phi')/d(phi) = -(v/L2) < 0` → 负反馈，铰接角收敛到平衡点
  `phi_eq = asin(-tan(delta) * L2 / L1)`；
- 倒车（`v < 0`）：导数变正 → 正反馈，铰接角按指数增长，这就是折叠。

## 2. 用法

```bash
# 离线：三个场景的对照报告
python3 -m trailer_jackknife_guard.main

# 指定仿真时长
python3 -m trailer_jackknife_guard.main --duration 20

# ROS 2 节点（订阅 /trailer/state，发布 /trailer/guard）
ros2 run trailer_jackknife_guard main --node
ros2 launch trailer_jackknife_guard main.launch.py horizon_s:=3.0
```

话题：

| 话题 | 类型 | 方向 | 数据 |
| --- | --- | --- | --- |
| `/trailer/state` | `std_msgs/Float32MultiArray` | 订阅 | `[铰接角 deg, 转角 deg, 请求倒车速度 m/s]` |
| `/trailer/guard` | `std_msgs/Float32MultiArray` | 发布 | `[允许速度 m/s, 风险等级, 铰接角 deg, 转角 deg]` |

参数：`horizon_s`（预测时域，默认 5.0）、`jackknife_angle_deg`（默认 75）、
`tractor_wheelbase_m`（3.6）、`trailer_wheelbase_m`（8.0）。

## 3. 实测结果（WSL2 Ubuntu 24.04 + Python 3.12，倒车 1.2 m/s、转角 6°）

```
[reverse_unguarded] 折叠: 是，13.21 s 达到折叠角；最大铰接角 75.02°
[reverse_guarded]   折叠: 否；最大铰接角 63.27°（保护余量 0.85×75°）
[forward_stable]    折叠: 否；铰接角收敛到 -13.34°（理论平衡点 -13.5°）
```

ROS 2 节点实测（发布 `[70°, 8°, 1.2 m/s]`）：

```
[INFO] risk=2 限速: 请求 1.20 m/s 超过允许 0.00 m/s（铰接角 70.0°，转角 8.0°）
```

## 4. 测试

```bash
python3 -m pytest -q test/     # 15 项，纯 Python，不需要 ROS
```

覆盖：前进自稳/倒车发散、折叠时间有限与无限、风险分级边界、允许速度随转角与铰接角
单调下降、已折叠时禁止倒车、保护后不越过折叠角、三场景预期结果、参数校验。

## 5. 文件结构

```
trailer_jackknife_guard/
├── trailer_jackknife_guard/
│   ├── model.py        # 铰接角速率、折叠时间、最大安全倒车速度、风险分级
│   ├── simulation.py   # 三组对照场景 + 报告
│   ├── node.py         # ROS 2 节点（/trailer/state -> /trailer/guard）
│   └── main.py         # 入口：默认离线报告，--node 启动节点
├── launch/main.launch.py
├── test/test_model.py
└── README.md
```

## 6. 限制

- 运动学模型（无轮胎侧偏、无质量转移），高速/低附着时的折叠会比模型更早；
- 只保护倒车方向；前进时铰接角自稳，不做额外限制；
- 转角按前轮转角直接给出，未建模转向执行器滞后；
- 保护是「限速」而不是「主动回正」，实际车辆还需要驾驶员/控制器配合打回方向。
