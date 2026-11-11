# ugv_cruise_pid：地面车 PID 巡航速度跟踪仿真

一维质点车辆 + 速度 PID（积分限幅、加速度限幅）的离线仿真，速度指令按 0 → 5 → 8 → 3 m/s
四段阶跃给出，t=12–14 s 叠加 −1.0 m/s² 逆风阵风。不依赖 ROS / Gazebo / CARLA。

## 目录结构

```
ugv_cruise_pid/
├── main.py                      # 入口，转发到 scripts/cruise_pid_node.py
├── requirements.txt
├── README.md
├── scripts/
│   ├── cruise_pid_node.py       # 唯一实现：被控对象 + PID + 指令/扰动 + 指标 + 出图
│   └── ugv_cruise_pid.py        # 兼容入口，同样转发到 cruise_pid_node.py
└── tests/
    └── test_cruise_pid.py       # 9 项离线回归测试
```

## 运行

```bash
pip install -r requirements.txt
python main.py
```

可选参数：`--out-dir`（默认 `results`）、`--dt`（默认 0.01 s）、`--t-end`（默认 20 s）。

## 产物

- `results/cruise_response.png`：速度跟踪曲线 + 加速度指令曲线
- `results/cruise_demo.gif`：60 帧、10 fps 动图
- `results/summary.json`：全部指标数值

## 测试

```bash
python -m pytest tests -v
```

文档见 [`docs/ground/ugv_cruise_pid.md`](../../../docs/ground/ugv_cruise_pid.md)。
