# uav_l1_altitude_stabilization

无人机阵风扰动下的 L1 自适应高度控制器仿真与阶跃响应评测。

## 依赖

- Ubuntu 20.04 / 22.04
- Python 3.8+
- NumPy, Matplotlib

## 目录结构

```
uav_l1_altitude_stabilization/
├── main.sh
├── run_demo.sh
├── scripts/
│   └── l1_altitude_node.py
└── README.md
```

## 运行

```bash
bash main.sh
bash run_demo.sh
python3 scripts/l1_altitude_node.py
```

## 已验证话题

- /uav/altitude_setpoint
- /uav/altitude_state
- /uav/vertical_thrust
- /uav/gust_disturbance

## 已知限制

- 仿真采用简化点质量模型，未包含姿态内环
- 阵风模型为阶跃+正弦叠加，真实风场更复杂
- 仅验证了高度通道，横向位置通道未接入
