# 工况能耗与续航核算（`drive_cycle_energy`）

> 一段行驶工况里，电池的电到底花在哪里？滚阻、风阻、制动损失、动能各占多少？
> 再生制动能收回多少？满电能跑多远？
> 本模块用纵向动力学把这几件事算成一组**可断言的数字**，并在终端打印能量条。

## 模块概述

输入是一段**工况**（每段写成「时长 + 起始车速 + 结束车速」）和一组整车参数，
输出是：

- 驱动能量、制动能量、回收能量、净耗电量；
- 能量分解：滚阻 / 风阻 / 制动损失 / 动能净变化；
- 百公里电耗与满电续航；
- 四条断言（能量分解自洽、回收上限、续航自洽、风阻随车速上升）。

**为什么值得单独做一个模块**：运动学模块（如 `speed_profile_check`）能算出
「什么时候该加速、什么时候该减速」，但它不算阻力，也就无法回答「这么开费多少电」。
本模块补上纵向阻力与电池模型，并且因为能量分解可以写成闭式恒等式，
「实现对不对」能被断言直接卡住，不需要仿真器。

## 构建与运行

本包是纯 Python 的 catkin 包，`catkin_make` 只做安装，不编译 C++：

```bash
cd ~/catkin_ws18
catkin_make
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roslaunch drive_cycle_energy main.launch
```

命令行覆盖参数（优先级：命令行 > yaml > 代码默认值）：

```bash
roslaunch drive_cycle_energy main.launch regen_eff:=0.0
roslaunch drive_cycle_energy main.launch regen_limit:=0.5
roslaunch drive_cycle_energy main.launch battery_kwh:=30.0
```

## 模型

纵向动力学（忽略坡度，纵向加速度记为 a）：

```
驱动力  F     = m·a + m·g·c_rr + 0.5·ρ·Cd·A·v²
滚阻力  F_rr  = m·g·c_rr
风阻力  F_aero = 0.5·ρ·Cd·A·v²
```

能量按时间积分：

```
E_drive = ∫ max(F, 0)·v dt        驱动能量
E_brake = ∫ max(−F, 0)·v dt       制动能量（被消耗掉的机械能）
E_regen = η · E_brake             再生制动回收
E_net   = E_drive − E_regen       净耗电
```

核心恒等式（第一条断言的依据）：

```
E_drive = E_roll + E_aero + E_brake + ΔKE
```

其中 `ΔKE = 0.5·m·(v_end² − v_start²)` 是整段工况的动能净变化。
滚阻与风阻都能写成速度的幂函数（`v` 与 `v³`），所以三项都可以逐点积出来再对账。

续航：

```
百公里电耗 = E_net / 3.6e6 / (距离 / 100 km)   [kWh/100km]
续航       = 电池容量 / 百公里电耗 × 100        [km]
```

## 四条断言

| 断言 | 判据 | 失败意味着 |
|---|---|---|
| **能量分解** | 滚阻+风阻+制动+动能 与驱动能量的相对偏差 ≤ `breakdown_tolerance` | 积分或阻力模型实现错了 |
| **回收上限** | `E_regen / E_brake ≤ regen_limit` | 回收效率配置不合理（η 越界或断言被放宽） |
| **续航自洽** | 由百公里电耗反推的续航与直接算的一致 | 折算公式实现不一致 |
| **风阻上升** | 30 m/s 风阻 > 10 m/s 风阻 | 风阻没按 v² 算（最常见是把线性项当二次项） |

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `vehicle/mass` | 1800.0 | 整备质量 kg |
| `vehicle/c_rr` | 0.012 | 滚动阻力系数 |
| `vehicle/cd` | 0.31 | 风阻系数 |
| `vehicle/area` | 2.2 | 迎风面积 m² |
| `vehicle/battery_kwh` | 60.0 | 电池可用容量 kWh |
| `vehicle/regen_eff` | 0.6 | 再生制动效率 0~1 |
| `check/breakdown_tolerance` | 0.02 | 能量分解相对偏差上限 |
| `check/regen_limit` | 1.0 | 回收 / 制动 上限 |
| `rate/publish_hz` | 10.0 | 保活循环频率 Hz |
| `topic/report` | `/energy/report` | 文字报告话题 |
| `topic/breakdown` | `/energy/breakdown` | 逐项能量 CSV |
| `topic/range` | `/energy/range` | 满电续航 km |

`launch/main.launch` 为每个参数都声明了带 `doc` 的 `<arg>` 并写回同名 `<param>`，
所以命令行覆盖一定生效（不像写错的 `$(rospack find …)` 那样静默失败）。

## 话题

| 话题 | 类型 | 内容 |
|---|---|---|
| `/energy/report` | `std_msgs/String` | 完整文字报告（latched） |
| `/energy/breakdown` | `std_msgs/String` | `item,joules` 七行 CSV（latched） |
| `/energy/range` | `std_msgs/Float64` | 满电续航 km（latched） |

```bash
rostopic echo -n 1 /energy/report
rostopic echo -n 1 /energy/breakdown
rostopic echo -n 1 /energy/range
```

## 实测输出（默认工况）

默认工况 8 段共 60 s（0→30、30 匀速、30→50、50 匀速、50→0、怠速、0→40、40 匀速），
行驶 512.5 m，最高 50 km/h：

```
工况：60.0 s，行驶 512.5 m，最高 50.0 km/h
整车：质量 1800 kg，滚阻 0.012，Cd·A 0.68 m²，电池 60.0 kWh，回收 η 0.60
能量：驱动 407.3 kJ，制动 159.6 kJ，回收 95.8 kJ，净耗 311.5 kJ
分解：滚阻 108.6 kJ（26.7%），风阻 28.0 kJ（6.9%），制动 159.6 kJ（39.2%），动能 111.1 kJ（27.3%）
续航：百公里 16.89 kWh，满电 355.3 km
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 能量分解  … 相对偏差 0.0000，容差 0.02
  [PASS] 回收上限  回收 95.8 / 制动 159.6 = 0.6000（上限 1.00，η=0.60）
  [PASS] 续航自洽  百公里 16.885 kWh，电池 60.0 kWh → 续航 355.3 km
  [PASS] 风阻上升  10 m/s 风阻 40.9 N，30 m/s 风阻 368.3 N（期望上升）
结论：PASSED
```

能量条（驱动为满条，其余按比例）：

```
滚阻|###########|    108.6 kJ
风阻|###|     28.0 kJ
制动|################|    159.6 kJ
动能|###########|    111.1 kJ
驱动|##########################################|    407.3 kJ
回收|##########|     95.8 kJ
```

## 两个反例

**反例一**：`regen_limit:=0.5` —— 回收比 0.6000 超过上限，只翻「回收上限」，其余三条通过。

**反例二**：`regen_eff:=0.0` —— 没有回收：

| 量 | 默认（η=0.6） | 无回收（η=0） |
|---|---|---|
| 净耗 | 311.5 kJ | **407.3 kJ** |
| 满电续航 | 355.3 km | **271.8 km** |

注意四条断言**依然全过**：它们校验的是自洽性，不是「这台车好不好」。
想知道回收值不值，就是要像上表这样对比两次运行 —— 这正是把断言和结论分开的价值。

## 离线测试

```bash
python3 tests/test_drive_cycle_energy_local.py
```

**通过 152 项，失败 0 项**（Python 3.6.9 / 3.12 均可），11 组：
参数校验、阻力计算、工况解析、工况积分、四条断言、能量恒等式、续航与百公里、
字符图、报告文本、参数对象、工程文件约定。

其中 K 组是本项目踩坑后的回归：**逐个检查 `scripts/*.py` 的第 1 行是 shebang、
第 2 行是 `# -*- coding: utf-8 -*-`**。Melodic 的 Python 2.7 只在前两行找编码声明，
用 `sed -i '1s|.*|#!/usr/bin/env python2|'` 整行覆盖会把编码声明顶掉，
节点一启动就报 `SyntaxError: Non-ASCII character ... but no encoding declared`。
现在的写法是带锚点的 `sed -i '1s|^#!/usr/bin/env python$|#!/usr/bin/env python2|'`，
并且三个脚本都保证「第 1 行 shebang、第 2 行编码声明」。

## 与其他模块的分工

| 模块 | 回答的问题 |
|---|---|
| `speed_profile_check` | 一段距离怎么分配加减速时间（运动学，无阻力） |
| `braking_distance_check` | 从看见到停住要几米（纵向制动，无回收） |
| `lateral_stability_margin` | 转弯稳不稳（横向，与能耗无关） |
| **本模块** | **纵向能量流：滚阻 / 风阻 / 制动 / 动能分配 + 续航** |

## 常见问题

**Q：`rostopic echo` 卡住了？**
latched 话题会持续重发最后一条，加 `-n 1` 取一条。

**Q：`rosparam get` 提示 `is not set`？**
说明 `roslaunch` 已经退出；重跑 launch，让它挂着别关。

**Q：想用自己的工况？**
改 `scripts/energy_core.py` 末尾的 `DEFAULT_CYCLE`，每段是
`(时长秒, 起始 km/h, 结束 km/h)`；或在交互式 Python 里直接
`core.simulate(spec, [(5.0, 0.0, 60.0)])` 试算。

**Q：为什么没有 slope（坡度）？**
为了让能量分解保持闭式、可断言；坡度会把 `m·g·sinθ·v` 这一项混进来，
需要在 `traction_force` 里再加一个参数，属于可扩展项，不影响当前结论。

> 所使用具体的大模型声明：使用大模型辅助代码编写、测试编写与文档整理，提交内容由本人负责。
