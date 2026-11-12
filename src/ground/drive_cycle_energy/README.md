# drive_cycle_energy —— 工况能耗与续航核算

给定一段行驶工况（每段「时长 + 起始车速 + 结束车速」），把电池的能量花到哪里算清楚：
滚阻、风阻、制动损失、动能各占多少，再生制动能收回多少，满电能跑多远。

**纯标准库、纯计算**：不需要 CARLA、不需要 GPU、不需要显示器，`python3` 直接就能跑离线测试。

## 为什么做这个包

仓库里已有三个和「能耗」擦边的模块，但都不回答「这段工况电都花在哪了」：

| 已有模块 | 它回答什么 |
|---|---|
| `speed_profile_check` | 一段距离内怎么分配加减速时间（运动学，不算阻力） |
| `braking_distance_check` | 从看见到停住要几米（只算制动段） |
| `lateral_stability_margin` | 转弯稳不稳（横向） |
| **本包** | **纵向能量流：滚阻 / 风阻 / 制动 / 动能的分配，以及续航** |

关键点是**能量分解可以写成闭式恒等式**，于是「实现对不对」能被第四条断言直接卡住：

```
E_drive − E_brake = E_roll + E_aero + ΔKE
E_roll  = m·g·c_rr · 行驶距离          （滚阻力与车速无关）
E_aero  = 0.5·ρ·Cd·A · ∫v³ dt          （风阻力与车速平方成正比）
ΔKE     = 0.5·m·(v_end² − v_start²)
```

## 安装与运行

本包是 catkin 包，放进工作空间即可（不需要编译 C++）：

```bash
cd ~/catkin_ws18
catkin_make
source /opt/ros/melodic/setup.bash
source devel/setup.bash
```

跑起来（参数全部在 `config/drive_cycle_energy.yaml`，命令行可覆盖）：

```bash
roslaunch drive_cycle_energy main.launch
roslaunch drive_cycle_energy main.launch regen_eff:=0.0 regen_limit:=0.5
```

看发布出来的东西：

```bash
rostopic echo -n 1 /energy/report       # 文字报告（latched）
rostopic echo -n 1 /energy/breakdown    # 逐项能量 CSV（latched）
rostopic echo -n 1 /energy/range        # 满电续航 km（latched）
rosparam get /vehicle/battery_kwh       # 确认参数生效
```

## 参数（12 个）

| 参数 | 默认 | 含义 |
|---|---|---|
| `vehicle/mass` | 1800.0 | 整备质量 kg |
| `vehicle/c_rr` | 0.012 | 滚动阻力系数 |
| `vehicle/cd` | 0.31 | 风阻系数 |
| `vehicle/area` | 2.2 | 迎风面积 m² |
| `vehicle/battery_kwh` | 60.0 | 电池可用容量 kWh |
| `vehicle/regen_eff` | 0.6 | 再生制动回收效率 0~1 |
| `check/breakdown_tolerance` | 0.02 | 能量分解相对偏差上限 |
| `check/regen_limit` | 1.0 | 回收 / 制动 的上限 |
| `rate/publish_hz` | 10.0 | 保活循环频率 |
| `topic/report` | `/energy/report` | 文字报告话题 |
| `topic/breakdown` | `/energy/breakdown` | 逐项能量 CSV |
| `topic/range` | `/energy/range` | 满电续航 km |

## 话题（3 路，都是 latched）

| 话题 | 类型 | 内容 |
|---|---|---|
| `/energy/report` | `std_msgs/String` | 完整报告（工况、能量、分解、续航、四条断言） |
| `/energy/breakdown` | `std_msgs/String` | `item,joules` 七行 CSV |
| `/energy/range` | `std_msgs/Float64` | 满电续航 km |

## 默认工况与结果

默认工况 8 段共 60 s：0→30、30 匀速、30→50、50 匀速、50→0、怠速、0→40、40 匀速，行驶 512.5 m。

| 量 | 值 |
|---|---|
| 驱动能量 | **407.3 kJ** |
| 制动能量 | 159.6 kJ |
| 回收能量（η=0.6） | **95.8 kJ** |
| 净耗 | **311.5 kJ** |
| 分解 | 滚阻 108.6 kJ（26.7%）/ 风阻 28.0 kJ（6.9%）/ 制动 159.6 kJ（39.2%）/ 动能 111.1 kJ（27.3%）|
| 百公里电耗 | **16.89 kWh** |
| 满电续航 | **355.3 km** |

四条断言全过：

```
[PASS] 能量分解  滚阻 108.6 + 风阻 28.0 + 制动 159.6 + 动能 111.1 = 407.3 kJ（偏差 0.0000）
[PASS] 回收上限  回收 95.8 / 制动 159.6 = 0.6000（上限 1.00，η=0.60）
[PASS] 续航自洽  百公里 16.885 kWh，电池 60.0 kWh → 续航 355.3 km
[PASS] 风阻上升  10 m/s 40.9 N < 30 m/s 368.3 N
```

## 两个反例

1. `regen_limit:=0.5` → **只翻「回收上限」**（0.6000 > 0.50），其余三条仍通过；
2. `regen_eff:=0.0` → 净耗从 311.5 kJ 涨到 **407.3 kJ**、续航从 355.3 km 掉到 **271.8 km**，
   四条断言依然全过 —— 说明断言校验的是「自洽」，不是「车好不好」；
   要看回收有没有收益，得对比这两次运行。

## 离线测试

```bash
python3 tests/test_drive_cycle_energy_local.py
```

**通过 152 项，失败 0 项**，覆盖：参数校验、阻力计算、工况解析、积分、四条断言、
能量恒等式、续航与百公里、字符图、报告文本、参数对象、工程文件约定。

其中 K 组专门回归本项目踩过的坑：逐个文件检查 `scripts/*.py` 的
**第 1 行是 shebang、第 2 行是 `# -*- coding: utf-8 -*-`**。
Melodic 的 Python 2.7 要求编码声明在前两行，而 `sed` 改 shebang 时若整行覆盖，
编码声明就会丢，节点一启动就 `SyntaxError: Non-ASCII character ... no encoding declared`。

## 与其他模块的分工

- `speed_profile_check`：只算运动学（距离、时间、加减速），不含阻力；
- 本包：在运动学之上加阻力模型与电池模型，算能量与续航；
- `braking_distance_check`：只关心「停得住吗」，不含回收；
- `lateral_stability_margin`：横向稳定性，与本包的纵向能量互不重叠。

## 常见问题

**Q：`rostopic echo` 一直卡着不动？**
latched 话题会一直重发最后一条；加 `-n 1` 只取一条。

**Q：数字和 README 对不上？**
先 `rosparam get /vehicle/mass` 等确认没被命令行或其它 yaml 覆盖。

**Q：能换成自己的工况吗？**
改 `energy_core.DEFAULT_CYCLE`（每段 `(时长, 起始 km/h, 结束 km/h)`），或在代码里
`core.simulate(spec, 你自己的段列表)`。

> 所使用具体的大模型声明：使用大模型辅助代码编写、测试编写与文档整理，提交内容由本人负责。
