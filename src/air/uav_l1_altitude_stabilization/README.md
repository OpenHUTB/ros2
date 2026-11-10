# uav_l1_altitude_stabilization

无人机阵风扰动下的 L1 自适应高度控制器仿真与阶跃响应评测。

## 依赖

- Ubuntu 20.04 / 22.04
- ROS Noetic / ROS 2 Humble
- Python 3.8+
- NumPy, Matplotlib

## 目录结构

```
uav_l1_altitude_stabilization/
├── main.sh              # 一键入口
├── run_demo.sh          # 演示运行脚本
├── scripts/
│   └── l1_altitude_node.py  # L1 高度控制节点
└── README.md
```

## 运行

```bash
# 一键运行
bash main.sh

# 或分步运行
bash run_demo.sh          # 带阵风扰动的阶跃响应仿真
python3 scripts/l1_altitude_node.py  # 直接运行节点
```

## 已验证话题

- `/uav/altitude_setpoint` (std_msgs/Float64)：高度设定值 (m)
- `/uav/altitude_state` (std_msgs/Float64)：当前高度 (m)
- `/uav/vertical_thrust` (std_msgs/Float64)：垂直推力指令 (N)
- `/uav/gust_disturbance` (std_msgs/Float64)：阵风扰动估计 (m/s^2)

## 已知限制

- 仿真采用简化点质量模型，未包含姿态内环
- 阵风模型为阶跃+正弦叠加，真实风场更复杂
- 仅验证了高度通道，横向位置通道未接入
