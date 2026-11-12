# AirSim ROS 2 Humble 最小桥接

本模块把 AirSim 多旋翼的位姿和速度控制接入 ROS 2 Humble。现有
[CarlaAir ROS 桥接](./carlair_ros_bridge.md)面向 ROS Noetic，本模块只实现
ROS 2 闭环所需的最小接口，便于在 Ubuntu 22.04 上继续开发感知、规划和控制。

## 1. 接口

| 类型 | 名称 | 消息或服务 |
| --- | --- | --- |
| 发布 | `/uav/odom` | `nav_msgs/msg/Odometry` |
| 发布 | `/tf` | `world -> base_link` |
| 订阅 | `/uav/cmd_vel` | `geometry_msgs/msg/Twist` |
| 服务 | `/uav/takeoff` | `std_srvs/srv/Trigger` |
| 服务 | `/uav/land` | `std_srvs/srv/Trigger` |

默认速度命令使用 ROS 机体系 FLU：`linear.x` 向前、`linear.y` 向左、
`linear.z` 向上，`angular.z` 为偏航角速度。也可以把
`cmd_vel_frame` 改为 `world_enu` 下发世界系速度。

## 2. 坐标转换

AirSim 位置使用世界 NED，姿态的机体系是 FRD；ROS 使用世界 ENU 和机体系 FLU。

```text
x_enu = y_ned
y_enu = x_ned
z_enu = -z_ned

R_world_enu_body_flu =
    C_ned_to_enu * R_world_ned_body_frd * diag(1, -1, -1)
```

桥接器发布的位置、TF、里程计速度都使用 ENU/FLU 约定。AirSim 中机头朝北且水平时，
ROS 偏航角为 `+90°`。

## 3. 构建

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/hutb_ws/src
ln -s /path/to/ros2/src/air/airsim_ros2_bringup \
  ~/hutb_ws/src/airsim_ros2_bringup
cd ~/hutb_ws
colcon build --symlink-install
source install/setup.bash
```

AirSim Python 客户端：

```bash
python3 -m pip install airsim
```

## 4. 运行

先启动 AirSim/CarlaAir，并确认 Windows 端 `41451` 端口可从 WSL2 访问。

```bash
ros2 launch airsim_ros2_bringup bringup.launch.py \
  host:=<Windows_IP> port:=41451 vehicle_name:=""
```

检查位姿、TF 和频率：

```bash
ros2 topic hz /uav/odom
ros2 topic echo /uav/odom --once
ros2 run tf2_ros tf2_echo world base_link
```

起飞、悬停控制与降落：

```bash
ros2 service call /uav/takeoff std_srvs/srv/Trigger
ros2 topic pub -r 10 /uav/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.2}}"
ros2 service call /uav/land std_srvs/srv/Trigger
```

停止发布 `/uav/cmd_vel` 超过 `command_timeout` 后，桥接器自动调用悬停。

## 5. 自动测试

坐标转换测试不需要 AirSim、ROS 2 或 GPU：

```bash
python3 -m pytest src/air/airsim_ros2_bringup/tests -q
```

ROS 2 构建与测试：

```bash
colcon test --packages-select airsim_ros2_bringup
colcon test-result --verbose
```

## 6. 范围与限制

- 当前只提供里程计、TF、速度控制和起降服务；
- 相机、深度、IMU、LiDAR 和点云接口留待独立 PR；
- AirSim 不发布 `/clock`，因此 `use_sim_time` 默认关闭；
- 仿真器断连时节点会停止发送旧指令并定期重连，但不会自动重新解锁飞行器。

## 7. ROS 2 与仓库贡献规范

本模块遵循 [ROS 2 Humble 文档](https://docs.ros.org/en/humble/) 的节点、参数、QoS、
Launch、TF2 和消息接口约定，坐标轴方向遵循
[ROS REP-103](https://www.ros.org/reps/rep-0103.html)。模块入口为 `main`，并提供
ROS 2 launch 文件。

按照 [OpenHUTB/ros2 提交注意事项](https://github.com/OpenHUTB/ros2#注意事项) 和
[OpenHUTB 贡献指南](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md)：

- 源码位于 `src/air/airsim_ros2_bringup`；
- 文档位于 `docs/air/airsim_ros2_bringup.md`；
- 首页和 `mkdocs.yml` 已加入链接；
- 提交前需运行 `mkdocs serve --livereload`；
- PR 需附真实运行截图或 GIF，并请另一名贡献者在另一台机器复核；
- 合并前解决与主分支的冲突，并在变更页逐项检查文件。

## AI 辅助声明

本模块、测试和文档由大语言模型辅助整理；提交者应实际运行 AirSim/Humble
闭环并在 PR 中说明验证环境与结果。
