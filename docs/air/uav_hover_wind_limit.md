# 悬停抗风能力核算：倾角、推重比余量与蒲福风级

多旋翼在风里能不能悬停，取决于**姿态能倾多少**和**推力还剩多少**，不是只看电机最大拉力。
本模块把这两个约束写成闭式解：给定质量、阻力面积 $C_dA$ 和最大倾角，直接算出能悬停的最大风速、
对应的蒲福风级、需要预留的推重比，以及阵风来时的额外倾角。

默认机型（2 kg、$C_dA=0.15\ \mathrm{m^2}$、最大倾角 25°、海平面标准大气）的结论：

| 指标 | 数值 |
| --- | --- |
| 悬停重力 | 19.61 N |
| 最大抗风（倾角 25°） | **9.98 m/s**（蒲福 5 级，8.0–10.7 m/s） |
| 该倾角所需推重比 | $T/W = 1.1034$，比竖直悬停多 10.3% 推力 |
| 阵风校验（+3 m/s） | 需要额外倾斜 13.27°，到 38.27° |

![倾角-风速曲线](../img/air/uav_hover_wind_limit/wind_vs_tilt.png)

## 1. 运行环境

只要 Python 3.8 及以上、NumPy 与 Matplotlib（见 `requirements.txt`），不需要 ROS、飞控或仿真器：

```bash
cd src/air/uav_hover_wind_limit
python3 -m pip install -r requirements.txt
python3 main.py --output outputs
```

实测环境：Windows 11 `10.0.26200`、Python `3.12.14`、NumPy `2.3.5`、Matplotlib `3.11.2`。
实际输出（`main.py` 默认参数）：

```text
==> uav_hover_wind_limit
    参数: m=2.00 kg, Cd*A=0.150 m^2, 最大倾角=25.0 deg, 海拔=0 m, rho=1.2250 kg/m^3
    悬停重力: 19.61 N
    最大抗风: 9.98 m/s (蒲福 5 级)
    该倾角所需推重比 T/W = 1.1034 (比竖直悬停多 10.3% 推力)
    阵风校验: 在 9.98 m/s 上再加 3.0 m/s, 需要额外倾斜 13.27 deg (共 38.27 deg)

    [倾角-风速] 倾角(deg)   可悬停风速(m/s)   蒲福风级
                  0.00           0.000            0
                  4.17           3.944            3
                  8.33           5.592            4
                 12.50           6.880            4
                 16.67           7.994            4
                 20.83           9.013            5
                 25.00           9.977            5
```

换机型/换高度只需改参数：

```bash
python3 main.py --mass 3.0 --drag-area 0.20 --max-tilt 30 --altitude 1500
```

`--altitude` 用 ISA 对流层近似把海拔换算成空气密度（只支持 -500 ~ 11000 m，超出直接报错），
高原空气稀薄，同样倾角能扛的风速更大。

## 2. 模型

准静态假设：悬停时气流速度等于风速，机体为刚性体，气动合力只有阻力
$D = \tfrac12 \rho (C_dA) V^2$ 且与来流同向；推力沿机体 $z$ 轴。垂直与水平方向平衡：

$$T\cos\theta = mg,\qquad T\sin\theta = D(V) = \tfrac12\rho (C_dA) V^2 .$$

两式相除消掉推力：

$$\tan\theta = \frac{\rho (C_dA) V^2}{2mg}.$$

于是给定最大倾角 $\theta_{max}$ 时的抗风上限与所需推重比为

$$V_{max} = \sqrt{\frac{2mg\tan\theta_{max}}{\rho (C_dA)}},\qquad
\frac{T}{W}=\frac{1}{\cos\theta_{max}} .$$

反解也成立：给定风速 $V$，所需倾角 $\theta(V)=\arctan\!\big(\rho (C_dA)V^2/(2mg)\big)$，
阵风增量 $\Delta V$ 只要再算一次 $\theta(V+\Delta V)-\theta(V)$。

## 3. 敏感性

![阻力面积与倾角上限的影响](../img/air/uav_hover_wind_limit/capability.png)

从公式可以直接读出三条缩放关系，测试里都钉住了：

- 抗风上限 $\propto \sqrt{m/(\rho C_dA)}$：质量翻 4 倍，抗风翻倍；阻力面积翻 4 倍，抗风减半；
- 抗风上限 $\propto \sqrt{\tan\theta_{max}}$：倾角从 25° 放到 30° / 35°，抗风从 9.98 涨到 11.10 / 12.23 m/s；
- 海拔升高 $\rho$ 变小，抗风按 $1/\sqrt{\rho}$ 变大（1500 m 约 1.08 倍，2000 m 约 1.10 倍，3000 m 约 1.16 倍）。

推重比只和倾角有关：$T/W = 1/\cos\theta_{max}$，25° 是 1.103、30° 是 1.155、35° 是 1.221。
要留姿态余量给阵风，就得按「最大倾角 + 阵风余量」来选 $T/W$。

## 4. 蒲福风级对照

风速换算按 WMO 分级（0 级 <0.3 m/s，1 级 0.3–1.5，……12 级 ≥32.7 m/s），结果里同时给出
`max_wind_beaufort`，方便和作业规范里的「几级风能飞」对照。默认机型的 9.98 m/s 落在 5 级，
6 级（10.8–13.8 m/s）就超出 25° 姿态能力了——要么提高倾角上限，要么减小 $C_dA$。

## 5. 适用范围与限制

- 准静态、单自由度姿态：不建模姿态环动态、电机响应和阵风频谱，结论用于**选型与作业窗口**评估；
- 只算气动阻力，不含机体升力、旋翼入流遮挡、螺旋桨效率随来流的变化；
- $C_dA$ 需要实测或查厂家数据，取错会直接让 $V_{max}$ 偏 $\sqrt{1/C_dA}$；
- 未考虑地面效应与近壁流动；风切变、下击暴流等极端风况不适用；
- 纯离线计算，没有 ROS 节点：它只输出评估结果，不参与飞行控制；需要话题接口时可以另加一层。

## 6. 测试与验证

```bash
cd src/air/uav_hover_wind_limit
python3 -m pip install -r requirements-dev.txt
python3 -m pytest test -v
```

13 项检查全部通过，期望值都能手算核对：

- $\tan\theta = D/(mg)$ 恒等式、倾角与风速互为反函数、$V_{max}$ 闭式解；
- $T/W = 1/\cos\theta_{max}$ 与「多付多少推力」；
- 两条缩放律（质量 ×4 / 阻力面积 ×4）与海拔对密度、抗风的影响；
- 蒲福风级 18 个边界点（0.3、1.6、3.4 … 32.7 m/s）；
- 阵风附加倾角、report 字段自洽、参数校验（质量/阻力面积/倾角/密度/重力/风速/高度范围）；
- GBK 控制台下打印不抛 `UnicodeEncodeError`（Windows 默认中文代码页）；
- `main.py` 冒烟：真的写出 2 张 PNG 与 `report.json`。

CI：`.github/workflows/uav_hover_wind_limit_tests.yml` 在 Ubuntu + Python 3.8 上按
`requirements-dev.txt` 装依赖、跑 pytest、跑 `main.py` 冒烟与 `PYTHONIOENCODING=gbk` 复跑，
另有一个 Windows 任务用 GBK stdio 再跑一遍。

## 使用声明

使用 DeepSeek Harness（deepseek-flash）辅助实现、测试与文档；作者对提交内容负责。
