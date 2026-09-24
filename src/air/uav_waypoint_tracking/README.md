# uav_waypoint_tracking

无人机航点轨迹跟踪实验包：用自研 PID 控制器驱动 AirSim 中的四旋翼依次飞过给定航点，
记录真实轨迹，并与 AirSim 内置位置接口在同一条件下对比，输出到达时间、稳态误差、
超调量与轨迹均方根误差等量化指标。

完整实验说明见站点文档：`docs/air/waypoint_tracking.md`

## 文件结构

| 文件 | 说明 |
| --- | --- |
| `scripts/pid_tracker.py` | 主节点：起飞、PID 跟踪航点、记录 CSV 日志、输出指标 |
| `scripts/compare_builtin.py` | 对照组：AirSim 内置位置接口 `moveToPositionAsync` |
| `scripts/plot_tracking.py` | 读取日志绘制轨迹对比图、误差曲线、指标柱状图、增益对比图 |
| `config/params.yaml` | 连接参数、飞行参数、PID 增益、三套航点序列 |
| `launch/main.launch` | 启动入口，支持覆盖任务名与 PID 增益 |

## 依赖

- ROS Noetic（`rospy`、`nav_msgs`、`geometry_msgs`、`visualization_msgs`、`tf2_ros`）
- AirSim 1.8.1 模拟器（宿主机）与 `airsim` Python 客户端（客户机）
- `matplotlib`（仅出图脚本需要，缺失时 `sudo apt install -y python3-matplotlib`）

## 快速开始

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch uav_waypoint_tracking main.launch mission:=rectangle