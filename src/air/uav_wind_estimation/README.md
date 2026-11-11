# uav_wind_estimation

不用空速量测、只用航向方向约束估计风：只用航向方向约束：真风满足 $(I-\mathbf{u}\mathbf{u}^\top)(\mathbf{v}_g-\mathbf{w})=0$，与空速大小无关；
把候选风代入后"垂直残差"应为零，于是化成两参数最小二乘。模块同时给出可观测性分析，
对比悬停、直线、对飞、圆周四种机动的估计效果。

## 环境

- 离线：Windows 11、Python 3.12.14、NumPy 2.3.5、Matplotlib 3.11.2（仅绘图）
- ROS 2：Ubuntu 22.04 + ROS 2 Humble

## 构建与运行（ROS 2）

```bash
source /opt/ros/humble/setup.bash
cd ~/hutb_ws
colcon build --symlink-install --packages-select uav_wind_estimation
source install/setup.bash
ros2 launch uav_wind_estimation main.launch.py maneuver:=circle
```

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/uav/ground_velocity` | `geometry_msgs/TwistStamped` | 输出（仿真） | 地速与航向 |
| `/uav/airspeed` | `std_msgs/Float64` | 输出（仿真） | 空速 |
| `/uav/wind` | `std_msgs/String` | 输出（估计） | 风速、风向、协方差与可观测性（JSON） |

## 离线实验

```bash
cd src/air/uav_wind_estimation
python3 main.py --out results --figure-dir ../../../docs/img/air/uav_wind_estimation
python3 -m unittest discover -s tests -v
```

## 主要结果（真值风 3.0 / −2.0 m/s，即 3.6 m/s）

| 机动 | 可观测 | 最小二乘风速 RMSE | 对飞平均法 RMSE |
|---|---|---|---|
| 悬停 | 否（cond 3600） | 不可估 | 11.996 m/s |
| 直线 | 否（cond 2914） | 不可估 | 11.996 m/s |
| 对飞 | 否（cond 3945） | 不可估 | **0.287 m/s** |
| 圆周 | **是（cond 1.0）** | **0.049 m/s** | 0.230 m/s |

详见 `docs/air/uav_wind_estimation.md`。
