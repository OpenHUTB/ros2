# train_ato_stop

列车 ATO 自动运行与站台精确停车仿真。

给定线路限速、坡度和停车点，先生成一条"跑得起来又一定停得住"的目标速度曲线，
再按这条曲线闭环跟踪；站台前几十米切换成距离闭环，把停车误差压进 0.3 m 标尺。
另外单独实现一层 ATP 超速监督，用满紧急制动能力算允许速度，ATO 正常时永远在它
下面，被顶上去才接管。

除 ROS 2 节点外只用 Python 标准库（含 PNG / GIF 编码），没有 numpy 也能跑。

## 快速开始

```bash
cd src/ground/train_ato_stop
python3 -m train_ato_stop.main --offline --output results
python3 -m pytest tests -v      # 20 项
```

会跑完四个场景，输出速度-距离曲线、停车误差对比图、过程动画和一张结果表：

| 场景 | 不按曲线跑 / 按曲线跑 | 停车误差 |
|---|---|---|
| 平直道 | — | +0.03 m |
| 中途限速 | — | +0.02 m |
| 连续坡道 | — | -0.18 m |
| 晚制动（对照） | 制动点推迟到距站 520 m | +76.78 m |

## 曲线是怎么来的

1. **反向推包络**：从停车点倒着走，每个位置算出"以规划减速度还能刹住的最大速度"，
   与静态限速取小。反推时把基本阻力和坡道分力算进去 —— 12‰ 下坡能抵掉约
   0.09 m/s² 的制动力，按平直道算会把制动点算晚。
2. **正向积牵引**：从起点按当前速度下的牵引能力和阻力积分，与上一步的包络取小。
   出限速区后速度只能按牵引能力慢慢爬，不会瞬间跳上去。
3. **规划减速度打折**：用 0.8 m/s² 规划，实际常用制动是 1.0 m/s²。差的那 0.2 就是给
   0.8 s 制动力上升时间和控制滞后的余量。

## 停车精度

最后 40 m 且速度低于 0.8 m/s 时切到位置闭环：

```
a = kp * 剩余距离 - kd * 当前速度
```

只给距离项等于一个无阻尼弹簧，12‰ 下坡上一定会冲过头（早期版本误差 0.42 m）。
加上速度阻尼项之后三组都收在 0.2 m 以内。

## ATP 与 ATO 的分层

| | 规划减速度 | 作用 |
|---|---|---|
| ATO | 0.8 m/s² | 决定怎么开，跟踪曲线 |
| ATP | 1.2 m/s²（紧急） | 决定不许越过，超 4 km/h 直接施加紧急制动 |

ATP 的允许速度更"贴边"，所以正常运行时 ATO 永远在它下面。把两层混在一起就分不清
"没停准"是控制没做好还是安全防护没起作用。

## ROS 2

```bash
ros2 launch train_ato_stop main.launch.py
ros2 launch train_ato_stop main.launch.py scenario:=grade
```

| 话题 | 类型 | 说明 |
|---|---|---|
| `/train_ato/markers` | `visualization_msgs/MarkerArray` | 线路、限速段、坡度、速度曲线、列车 |
| `/train_ato/status` | `std_msgs/String` | `T=..S V=..KM/H REMAIN=..M` |
| `/train_ato/path` | `nav_msgs/Path` | 已行驶区段 |
| `/train_ato/replay` | `std_srvs/Trigger` | 从头重放 |

RViz2 里横轴是线路里程，纵轴不是高度而是速度（按 `speed_scale` 折算），
所以一张图里能同时看到"车在哪"和"速度曲线长什么样"。

`speed_scale` 默认 4.0，即 160 km/h 画到线路以上 640 m。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `scenario` | `flat` | `flat` / `restriction` / `grade` / `late_brake` |
| `speed_scale` | `4.0` | 速度折算到高度的系数，m/(km/h) |
| `playback_rate` | `1.0` | 回放倍速 |
| `frame_step` | `10` | 每个周期跨过的仿真帧数（单帧 0.02 s） |
