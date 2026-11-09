# ackermann_mpc_tracking — 带约束的 MPC 轨迹跟踪

用线性 MPC 跟踪参考路径：在长度为 N 的预测时域上最小化「横向/航向误差 + 转角 +
转角增量」，并显式满足前轮转角幅值与转向速率约束。同一场景下与 Stanley 对照。

## 目录

```
ackermann_mpc_tracking/
├── scripts/
│   ├── mpc.py              # 自行车模型、路径投影、线性 MPC（坐标下降解 QP）
│   ├── stanley_baseline.py # 对照基线
│   ├── simulate.py         # 闭环仿真、指标、参数扫描
│   ├── main.py             # 打印对比 + 生成 PNG
│   └── ros_node.py         # 订阅 /odom，发布 /cmd_vel
├── tests/test_mpc.py       # 24 项单元测试
└── launch/ackermann_mpc_tracking.launch.py
```

## 快速开始

```bash
python3 scripts/main.py    # 打印对比表并生成 3 张 PNG
python3 tests/test_mpc.py  # 24 项单元测试
```

纯标准库实现，QP 用坐标下降 + 箱式投影手写，不依赖 cvxpy / numpy。

## 场景与结果

参考路径：直线 6 m → 右转 R=6 m 90° → 左转 R=6 m 90° → 直线 8 m，总长 32.85 m。
车辆轴距 2.5 m，速度 5 m/s（18 km/h），初始横向偏差 0.8 m，转角上限 0.6 rad，
转向速率上限 1.6 rad/s（两者完全相同）。

| 控制器 | RMSE | 最大误差 | 末值误差 | 峰值转角 |
|---|---|---|---|---|
| MPC | 0.2182 m | 0.8000 m（不超调） | 0.0009 m | 0.483 rad |
| Stanley（k=1.5） | 0.5502 m | 1.2638 m（超调 58%） | 0.1654 m | 0.600 rad（饱和） |

参数鲁棒性：MPC 的 `q_y` 从 2 变到 60，RMSE 只在 0.218~0.231 之间；Stanley 的 `k`
取 1.5 时 RMSE 0.55，取 10 时恶化到 2.11。

## 在 ROS 2 中运行

```bash
colcon build --packages-select ackermann_mpc_tracking
source install/setup.bash
ros2 launch ackermann_mpc_tracking ackermann_mpc_tracking.launch.py
ros2 topic echo /cmd_vel
ros2 topic echo /mpc/debug   # [e_y, e_psi, kappa_ref, delta, s]
```

订阅 `/odom`（nav_msgs/Odometry），发布 `/cmd_vel`（geometry_msgs/Twist，
`angular.z = v * tan(delta) / L`）。

## 关于验证范围

模型、QP、闭环与对比数值均在本机实跑（`tests/test_mpc.py` 24 项全过）。
`colcon build` 与 `ros2 launch` 未在装了 ROS 2 发行版的机器上实测。
