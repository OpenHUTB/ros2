# uav_l1_altitude_stabilization：无人机 L1 自适应高度控制仿真

竖直一维质点（质量 1 kg）+ L1 自适应控制器（位置 PD 外环 + 状态预测器 + 速度误差驱动的
扰动估计 + 一阶低通）的离线仿真。高度指令 0 → 2.0 m → 3.5 m 两段阶跃，t=5–8 s 下沉阵风
−2.0 m/s²，t>13 s 往复风扰 0.8 m/s²（0.6 Hz）。同时给出「同增益、同限幅、无自适应」的
PD 基线作对照。不依赖 ROS / PX4 / Gazebo。

## 目录结构

```
uav_l1_altitude_stabilization/
├── main.py                                  # 入口，转发到 scripts/uav_l1_altitude_stabilization.py
├── requirements.txt
├── README.md
├── scripts/
│   └── uav_l1_altitude_stabilization.py      # 唯一实现：被控对象 + L1 + 指令/扰动 + 指标 + 出图
└── tests/
    └── test_uav_l1_altitude_stabilization.py # 10 项离线回归测试
```

## 运行

```bash
pip install -r requirements.txt
python main.py
```

可选参数：`--out-dir`（默认 `results`）、`--dt`（默认 0.001 s）、`--t-end`（默认 20 s）。

## 产物

- `results/altitude_response.png`：高度曲线 + 推力曲线 + 真实风扰与估计量对照
- `results/altitude_demo.gif`：60 帧、10 fps 动图（高度曲线 + 推力 + 竖直剖面）
- `results/summary.json`：L1 与 PD 基线的全部指标数值

## 参数

| 参数 | 取值 | 说明 |
| --- | --- | --- |
| `m` | 1.0 kg | 竖直一维等效质量 |
| `kp` / `kd` | 4.0 / 4.0 | 位置 PD，`ωn = 2 rad/s`、`ζ = 1` |
| `gamma` | 150 | 自适应增益，`γ·dt = 0.15` |
| `k_pred` (`k1 = k2`) | 4.0 | 预测器注入增益 |
| `lpf_omega` | 5.0 rad/s | 估计量低通带宽 |
| `sigma_limit` | ±25 m/s² | 扰动估计投影限幅 |
| `thrust_limit` | ±20 N | 推力限幅 |

## 测试

```bash
python -m pytest tests -v
```

覆盖：指令分段、扰动窗口、悬停平衡、估计初值、投影限幅、推力限幅、阶跃响应、
与 PD 基线的抗扰对比、往复风扰与末高度、产物写出。

文档见 [`docs/air/uav_l1_altitude_stabilization.md`](../../../docs/air/uav_l1_altitude_stabilization.md)。
