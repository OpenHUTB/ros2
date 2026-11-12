# uav_yaw_stabilize：无人机航向角 PD 稳定控制仿真

偏航单轴刚体（Izz = 0.02 kg·m²）+ 航向 PD（微分作用在角速率上、最短弧误差、力矩限幅）的离线
仿真，航向指令 +45° → −30° 两段阶跃，t=5–6 s 阵风力矩、t>10 s 往复风扰。不依赖 ROS / PX4 / Gazebo。

## 目录结构

```
uav_yaw_stabilize/
├── main.py                         # 入口，转发到 scripts/uav_yaw_stabilize.py
├── requirements.txt
├── README.md
├── scripts/
│   └── uav_yaw_stabilize.py        # 唯一实现：被控对象 + PD + 指令/扰动 + 指标 + 出图
└── tests/
    └── test_uav_yaw_stabilize.py   # 9 项离线回归测试
```

## 运行

```bash
pip install -r requirements.txt
python main.py
```

可选参数：`--out-dir`（默认 `results`）、`--dt`（默认 0.01 s）、`--t-end`（默认 15 s）。

## 产物

- `results/yaw_response.png`：航向角曲线 + 力矩曲线
- `results/yaw_demo.gif`：60 帧、10 fps 动图（曲线 + 俯视罗盘）
- `results/summary.json`：全部指标数值

## 测试

```bash
python -m pytest tests -v
```

文档见 [`docs/air/uav_yaw_stabilize.md`](../../../docs/air/uav_yaw_stabilize.md)。
