# 悬停抗风能力核算（倾角 / 推重比余量 / 蒲福风级）

给定整机质量、阻力面积 $C_dA$ 与最大倾角，算清楚三件事：

1. 这台多旋翼在悬停时能扛多大的风（m/s，以及蒲福几级）；
2. 需要为姿态预留多少推重比（$T/W=1/\cos\theta_{max}$）；
3. 阵风再大一点时需要额外倾斜多少度。

公式、敏感性、适用范围见[完整说明](../../../docs/air/uav_hover_wind_limit.md)。

## 环境与运行

Python 3.8 及以上，只需 NumPy 与 Matplotlib，不需要 ROS、飞控或仿真器：

```bash
cd src/air/uav_hover_wind_limit
python3 -m pip install -r requirements.txt
python3 main.py --output outputs
python3 main.py --mass 3.0 --drag-area 0.20 --max-tilt 30 --altitude 1500
```

默认参数：2 kg 四旋翼、$C_dA=0.15\ \mathrm{m^2}$、最大倾角 25°、海平面标准大气。输出
`outputs/wind_vs_tilt.png`、`outputs/capability.png` 与 `outputs/report.json`（`outputs/`
已进 `.gitignore`）。

实测环境：Windows 11 / Python 3.12.14 / NumPy 2.3.5 / Matplotlib 3.11.2。控制台输出只用
`m/s`、`deg`、`m^2` 这类 ASCII 单位，并在入口把 stdout/stderr 的 `errors` 放宽为 `replace`，
Windows 默认 GBK 代码页下也不会抛 `UnicodeEncodeError`。

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--mass` | 2.0 | 整机质量 kg |
| `--drag-area` | 0.15 | 阻力面积 $C_dA$，m² |
| `--max-tilt` | 25.0 | 最大倾角 deg，必须在 (0, 90) |
| `--altitude` | 0.0 | 海拔 m，用 ISA 对流层近似换算空气密度（-500~11000） |
| `--output` | `outputs` | 输出目录 |

## 输出字段

`report.json` 里包含 `parameters`、`weight_n`、`max_wind_m_s`、`max_wind_beaufort`、
`thrust_to_weight`、`thrust_increase_percent`、`tilt_sweep`（倾角-风速表）与 `gust_test`
（阵风增量与总倾角）。

## 测试

```bash
python3 -m pip install -r requirements-dev.txt
python3 -m pytest test -v      # 13 项
```

覆盖 $\tan\theta=D/(mg)$ 恒等式、倾角与风速互逆、$V_{max}$ 闭式解、$T/W=1/\cos\theta_{max}$、
质量与阻力面积的缩放律、海拔对密度与抗风的影响、蒲福风级边界、阵风附加倾角、report 自洽、
参数校验、GBK 打印，以及 `main.py` 冒烟（真的写出 2 张 PNG 与 JSON）。

## 适用范围

准静态模型：只算气动阻力与姿态倾角，不含姿态环动态、电机响应、旋翼入流遮挡、机体升力与
风切变；$C_dA$ 需要实测或厂家数据。结论用于选型与作业窗口评估，不代替整机气动仿真或试飞。
本模块是纯离线计算，不提供 ROS 节点（它不参与飞行控制）；需要话题接口时可以另加一层。

## 使用声明

使用 DeepSeek Harness（deepseek-flash）辅助实现、测试与文档；作者对提交内容负责。
