# rov_depth_hold：ROV 深度保持 PID 仿真

竖直一维刚体（等效质量 5 kg）+ 深度 PID（微分作用在测量上、积分与推力双限幅）的离线仿真，
深度指令 0 → 2.0 m 阶跃，t=5–8 s 负载上浮扰动、t>12 s 波浪扰动。不依赖 ROS / Gazebo。

## 目录结构

```
rov_depth_hold/
├── main.py                      # 入口，转发到 scripts/rov_depth_hold.py
├── requirements.txt
├── README.md
├── scripts/
│   └── rov_depth_hold.py        # 唯一实现：被控对象 + PID + 指令/扰动 + 指标 + 出图
└── tests/
    └── test_rov_depth_hold.py   # 9 项离线回归测试
```

## 运行

```bash
pip install -r requirements.txt
python main.py
```

可选参数：`--out-dir`（默认 `results`）、`--dt`（默认 0.01 s）、`--t-end`（默认 20 s）。

## 产物

- `results/depth_response.png`：深度跟踪曲线 + 推力曲线
- `results/depth_demo.gif`：60 帧、10 fps 动图（左曲线、右竖直剖面）
- `results/summary.json`：全部指标数值

## 测试

```bash
python -m pytest tests -v
```

文档见 [`docs/water/rov_depth_hold.md`](../../../docs/water/rov_depth_hold.md)。
