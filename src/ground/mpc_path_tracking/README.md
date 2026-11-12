# mpc_path_tracking

线性时变 MPC 路径跟踪 vs 纯追踪：把单向车在每个参考点线性化，按预测步长拼成带箱约束的二次规划，
用坐标下降求解；与纯追踪在同一条 S 形参考、同一个初始偏差下对比精度、收敛与计算量。

## 环境

- 离线：Windows 11、Python 3.12.14、NumPy 2.3.5、Matplotlib 3.11.2（仅绘图）
- ROS 2：Ubuntu 22.04 + ROS 2 Humble

## 构建与运行（ROS 2）

```bash
source /opt/ros/humble/setup.bash
cd ~/hutb_ws
colcon build --symlink-install --packages-select mpc_path_tracking
source install/setup.bash
ros2 launch mpc_path_tracking main.launch.py duration:=20
```

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/mpc/reference` | `nav_msgs/Path` | 输出 | 参考路径 |
| `/mpc/state` | `geometry_msgs/PoseStamped` | 输出 | 闭环轨迹（预报） |
| `/mpc/status` | `std_msgs/String` | 输出 | 步数与配置（JSON） |

## 离线实验

```bash
cd src/ground/mpc_path_tracking
python3 main.py --out results --figure-dir ../../../docs/img/ground/mpc_path_tracking
python3 -m unittest discover -s tests -v
```

## 主要结果（40 s，10 Hz，初始横向偏差 1.2 m / 航向偏差 20°）

| 指标 | 纯追踪 | MPC |
|---|---|---|
| 横向 RMSE | 0.215 m | **0.212 m** |
| 稳态 RMSE（t > 5 s） | 0.013 m | **0.008 m** |
| 收敛到 0.15 m 的时间 | 2.5 s | **2.2 s** |
| 偏航角速度最大值 | 1.200 rad/s（贴限幅） | **0.904 rad/s** |
| 单步求解耗时 | 0.06 ms | 27.2 ms |

详见 `docs/ground/mpc_path_tracking.md`。
