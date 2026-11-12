# lateral_stability_margin：不足转向梯度与侧向稳定裕度

同一台车，换一套轮胎或挪一下质心，方向盘的手感与高速稳定性会完全不同。
这个模块把这件事算成四个能直接读的数：**不足转向梯度 K、特征车速 / 临界车速、
稳态转向角 δ、转向灵敏度 a_y/δ**，并按四条断言给出结论。

不依赖 CARLA、不需要显示器，纯 Python 标准库，Python 2.7 / 3.x 都能跑。

## 它回答的问题

| 问题 | 输出 |
|---|---|
| 这车是推头还是甩尾？ | 梯度符号：`K > 0` 不足转向、`K < 0` 过多转向 |
| 高速上还有多少余量？ | 不足转向给特征车速，过多转向给临界车速 |
| 半径 50 m 的弯，方向盘要打多少？ | 稳态转向角 δ（rad 与 °） |
| 同样的方向盘角度，车能侧向加速度多少？ | 转向灵敏度 a_y/δ |

## 模型

```
轴荷        W_f = m·g·b/L                 b = 质心到后轴
            W_r = m·g·a/L                 a = 质心到前轴

侧偏角      α_f = W_f·a_y/(g·C_f)
            α_r = W_r·a_y/(g·C_r)

不足转向梯度
            K = (W_f/C_f − W_r/C_r) / g        [rad/(m/s²)]
                K > 0 不足转向（推头，稳定）
                K = 0 中性转向
                K < 0 过多转向（甩尾，超过临界车速会发散）

稳态转向角  δ = L/R + K·a_y                 [rad]，a_y = v²/R
转向灵敏度  a_y/δ = v² / (L + K·v²)

特征车速    v_char = sqrt(L / K)    （K > 0；灵敏度降到中性转向的一半）
临界车速    v_crit = sqrt(−L / K)   （K < 0；灵敏度发散）
```

注意 `K` 里那一步 **除以 g**：`W/C` 的量纲是 rad，乘上 `a_y` 再除 `g` 才是 rad。
漏掉它，特征车速会从 100 km/h 掉到 32 km/h —— 这是最容易错的一步。

## 默认数字（`config/lateral_stability_margin.yaml`）

| 项 | 值 |
|---|---|
| 整车质量 / 轴距 / 质心到前轴 | 1500 kg / 2.7 m / 1.2 m（到后轴 1.5 m） |
| 前 / 后轴侧偏刚度 | 80 000 / 95 000 N/rad |
| 前轴 / 后轴静态载荷 | **8175.0 N / 6540.0 N**（合计 14 715.0 N） |
| 不足转向梯度 K | **0.003399 rad/(m/s²)**，特性「不足转向」 |
| 特征车速 | **28.184 m/s = 101.5 km/h** |
| 工况 R = 50 m、40 km/h | a_y 2.469 m/s²，δ **0.062393 rad（3.57°）**，阿克曼 0.054000 rad（3.09°），超出 0.008393 rad |
| 转向灵敏度 | 39.574 1/s |

车速扫描（R = 50 m）：

| 车速 km/h | a_y m/s² | δ rad | δ ° | 灵敏度 1/s |
|---|---|---|---|---|
| 20 | 0.617 | 0.0561 | 3.21 | 11.00 |
| 40 | 2.469 | 0.0624 | 3.57 | 39.57 |
| 60 | 5.556 | 0.0729 | 4.18 | 76.22 |
| 80 | 9.877 | 0.0876 | 5.02 | 112.78 |
| 100 | 15.432 | 0.1065 | 6.10 | 144.96 |
| 120 | 22.222 | 0.1295 | 7.42 | 171.55 |

转向角随车速单调增大、灵敏度单调上升，正是不足转向车的特征。

## 四条断言

| 断言 | 判据 | 默认结果 |
|---|---|---|
| 轴荷分配 | `W_f + W_r = m·g`，且 `W_f/W_r = b/a` | PASS |
| 不足转向裕度 | `K ≥ understeer_min`（默认 0） | PASS |
| 低速阿克曼 | 低速（默认 2 km/h）时 `δ` 与 `L/R` 的相对偏差 ≤ 2% | PASS（0.04%） |
| 稳态转向角 | 由 `δ` 反算 `a_y` 再正算，闭环相对偏差 ≤ 1e-6 | PASS（1.8e-16） |

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `/vehicle/mass` | 1500.0 | 整车质量 kg |
| `/vehicle/wheelbase` | 2.7 | 轴距 m |
| `/vehicle/cg_to_front` | 1.2 | 质心到前轴距离 m（必须 < 轴距） |
| `/vehicle/gravity` | 9.81 | 重力加速度 |
| `/tire/front_cornering` | 80000.0 | 前轴侧偏刚度 N/rad |
| `/tire/rear_cornering` | 95000.0 | 后轴侧偏刚度 N/rad |
| `/scene/radius` | 50.0 | 转弯半径 m |
| `/scene/speed_kmh` | 40.0 | 稳态圆周车速 km/h |
| `/check/understeer_min` | 0.0 | 梯度下限，0 = 不允许过多转向 |
| `/check/ackermann_speed_kmh` | 2.0 | 校验阿克曼角用的低速 |
| `/check/ackermann_tolerance` | 0.02 | 低速偏差上限 |
| `/rate/publish_hz` | 1.0 | 保活重发频率 Hz |

## 话题

| 话题 | 类型 | 内容 |
|---|---|---|
| `/stability/report` | `std_msgs/String` | 整份报告（参数 + 扫描表 + 断言 + 结论） |
| `/stability/cornering` | `std_msgs/Float64MultiArray` | `[K, a_y, δ, 阿克曼角, 超出量]` |
| `/stability/limit` | `std_msgs/Float64MultiArray` | `[特征车速, 临界车速]`，不存在的那个给 −1 |

三条都是 latched，`rostopic echo` 一上来就能拿到。

## 跑起来

```bash
# ① 编译
cd ~/catkin_ws20
catkin_make

# ② 跑
source /opt/ros/melodic/setup.bash
source ~/catkin_ws20/devel/setup.bash
roslaunch lateral_stability_margin main.launch

# ③ 反例：换成硬前软后的甩尾车
roslaunch lateral_stability_margin main.launch front_cornering:=160000 rear_cornering:=40000
```

默认车 `结论：PASSED`；换成 `front_cornering:=160000 rear_cornering:=40000` 后变成
`过多转向`、临界车速 **15.350 m/s（55.3 km/h）**，只翻「不足转向裕度」这一条：

```
[FAIL] 不足转向裕度  K = -0.011458 rad/(m/s²)（下限 0.0000），
                    特性「过多转向」，临界车速 15.350 m/s（55.3 km/h）
```

## 离线测试

不启动 ROS 也能跑，152+ 项覆盖几何、梯度、圆周、断言、扫描、字符图、参数、
报告、闭式核对、工程文件与编码声明：

```bash
python3 tests/test_lateral_stability_margin_local.py
```

## 与已有模块的分工

| 模块 | 关注点 | 本模块的区别 |
|---|---|---|
| `tire_slip_curve` | 纵向附着随滑移率的变化 | 管纵向（刹车/驱动），本模块管侧向 |
| `rollover_margin` | 侧翻（抬轮） | 侧翻是**载荷**问题，本模块是**转向响应**问题 |
| `turning_sweep_check` | 转弯扫掠包络（几何） | 只算占多宽，不算车会不会甩尾 |
| `steering_actuator_lag` | 转向执行器动态 | 管“打不打得到”，本模块管“该打多少” |

## 常见问题

**1. 为什么我算出的特征车速特别低？**
九成是 `K` 忘了除以 `g`。`W/C` 出来是 rad，`δ = L/R + K·a_y` 里的 `K` 单位是
rad/(m/s²)，必须除以 9.81。

**2. 低速阿克曼那条为什么卡在 2 km/h？**
`δ = L/R + K·a_y`，只要车速不为 0，`K·a_y` 就不为 0。2 km/h 时它只占 0.04%，
放到 60 km/h 就占 8 倍 —— 手册里的第二个反例就是这么翻的。

**3. 我把 `understeer_min` 调大有什么用？**
它是一条验收线。乘用车通常要求至少轻微不足转向；把它设成 0.002 就能卡掉
“几乎中性”的调校。

## 大模型声明

本模块的代码编写、测试编写与文档整理使用了 AI 编程助手辅助，提交内容由本人负责。
