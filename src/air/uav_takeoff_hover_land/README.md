# uav_takeoff_hover_land — 无人机起飞—悬停—降落状态机

订阅 `/uav/pose`（`geometry_msgs/PoseStamped`，ENU），按「起飞 → 悬停 → 降落」
状态机计算速度指令，发布到 `/uav/cmd_vel`（`geometry_msgs/Twist`），由桥接节点、
PX4/MAVROS 或仿真器执行。配套教程见
[docs/air/uav_takeoff_hover_land.md](../../docs/air/uav_takeoff_hover_land.md)。

## 运行环境

| 项目 | 配置 |
| ---- | ---- |
| 操作系统 | Ubuntu 20.04（ROS 节点） |
| ROS | ROS Noetic（Python 3） |
| 依赖 | rospy、geometry_msgs |
| 离线验证 | Windows / Linux 均可，Python 3 + numpy + matplotlib ≥ 3.1（3D 绘图依赖 mplot3d，不需要 ROS） |

## 运行步骤

### 方式一：ROS 节点（需要先有仿真器提供位姿）

```bash
# 1. 拷贝到 catkin 工作空间并编译
mkdir -p ~/catkin_ws/src
cp -r <本仓库>/src/air/uav_takeoff_hover_land ~/catkin_ws/src/
chmod +x ~/catkin_ws/src/uav_takeoff_hover_land/main.py
cd ~/catkin_ws && catkin_make
source devel/setup.bash

# 2. 先启动提供 /uav/pose 的仿真器或桥接，例如仓库里的 carlair_ros_bridge
#    确认位姿话题有数据：
rostopic echo -n1 /uav/pose

# 3. 启动起飞—悬停—降落节点
roslaunch uav_takeoff_hover_land main.launch
```

终端会依次打印 `待机 -> 起飞 -> 悬停 -> 降落 -> 完成` 的状态切换日志，
结束后自动发布零速度。

### 方式二：离线仿真（不需要 ROS，用于快速验证控制逻辑）

```bash
cd <本仓库>
python -m pip install "matplotlib>=3.1" numpy
python src/air/uav_takeoff_hover_land/offline_sim.py
```

脚本会运行同样的状态机和比例控制律，输出三张图和一组指标到终端，
图片写到 `docs/img/air/uav_takeoff_hover_land/`。

## 参数

| 参数 | 默认值 | 说明 |
| ---- | ---- | ---- |
| `~takeoff_altitude` | 2.0 | 起飞目标高度（m） |
| `~hover_x` / `~hover_y` | 1.0 / 0.5 | 悬停点相对起飞点的水平偏移（m） |
| `~hover_time` | 5.0 | 到点后悬停时间（s） |
| `~position_tolerance` | 0.15 | 判定到点的位置误差（m） |
| `~max_horizontal_speed` | 1.0 | 水平速度限幅（m/s） |
| `~max_vertical_speed` | 0.5 | 垂直速度限幅（m/s） |
| `~kp_horizontal` | 0.8 | 水平位置误差到速度的比例系数 |
| `~kp_vertical` | 1.0 | 垂直位置误差到速度的比例系数 |
| `~land_altitude` | 0.05 | 判定触地的高度（m） |
| `~pose_timeout` | 1.0 | 位姿超时（s），超时后悬停等待 |

## 话题接口

| 方向 | 话题 | 类型 | 说明 |
| ---- | ---- | ---- | ---- |
| 订阅 | `/uav/pose` | `geometry_msgs/PoseStamped` | 无人机位置（ENU，z 向上） |
| 发布 | `/uav/cmd_vel` | `geometry_msgs/Twist` | 机体系速度指令，`linear.z` 向上 |

## 预期效果

无人机从当前位置起飞到 2 m，平移到起飞点前侧 1.0 m、左侧 0.5 m 的悬停点，
保持 5 s 后原地下降到地面，总飞行时间约 15 s；悬停阶段位置误差小于 0.1 m
（离线仿真结果见文档）。
