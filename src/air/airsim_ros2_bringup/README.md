# airsim_ros2_bringup

ROS 2 Humble 的 AirSim 多旋翼最小桥接。它补齐仓库中现有
`carlair_ros_bridge` 和 `drone_airsim_bringup` 仅支持 ROS Noetic 的空白。

## 功能范围

- 发布 `/uav/odom`，包含 ENU 位姿和机体系 FLU 速度
- 发布 `world -> base_link` TF
- 订阅 `/uav/cmd_vel`，支持机体系 FLU 或世界系 ENU 速度
- 提供 `/uav/takeoff`、`/uav/land` 服务
- 指令超时后自动切换为悬停
- AirSim 断连后自动重连

视觉、激光雷达、SLAM 和控制器不在本最小桥接范围内。

## 坐标系

AirSim 使用世界 NED、机体 FRD，ROS REP-103 使用世界 ENU、机体 FLU：

```text
position_enu = (east, north, -down)
body_flu     = (forward, -right, -down)
R_ros        = C_ned_to_enu * R_airsim * diag(1, -1, -1)
```

因此 AirSim 中机头朝北的水平姿态，在 ROS 中对应 `+90°` ENU 偏航。转换函数位于
`airsim_ros2_bringup/transforms.py`，测试不依赖 ROS 或模拟器。

## 构建

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/hutb_ws/src
ln -s /path/to/ros2/src/air/airsim_ros2_bringup \
  ~/hutb_ws/src/airsim_ros2_bringup
cd ~/hutb_ws
colcon build --symlink-install
source install/setup.bash
```

安装 AirSim Python 客户端：

```bash
python3 -m pip install airsim
```

## 启动与验证

```bash
ros2 launch airsim_ros2_bringup bringup.launch.py \
  host:=192.168.1.100 port:=41451
```

```bash
ros2 topic hz /uav/odom
ros2 run tf2_ros tf2_echo world base_link
ros2 topic echo /uav/odom
ros2 service call /uav/takeoff std_srvs/srv/Trigger
ros2 service call /uav/land std_srvs/srv/Trigger
```

速度指令遵循 ROS 机体系 FLU：

```bash
ros2 topic pub -r 10 /uav/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.2}}"
```

超过 `command_timeout` 没有新指令时，桥接节点调用 `hoverAsync`。`use_sim_time`
默认关闭；接入 `/clock` 后可显式设为 `true`。

## 测试

```bash
python3 -m pytest src/air/airsim_ros2_bringup/tests -q
colcon test --packages-select airsim_ros2_bringup
```

## ROS 2 与仓库贡献约定

- 使用 ROS 2 Humble、`rclpy`、ament_python、标准消息和 `std_srvs/srv/Trigger`；
- TF 使用 `tf2_ros`，坐标遵循 ROS REP-103 的 ENU/FLU 约定；
- 模块入口统一为 `main`，并同时提供 `launch/bringup.launch.py`；
- 文档已在 `mkdocs.yml` 和 `docs/index.md` 添加入口；
- 提交前必须运行 `mkdocs serve --livereload` 检查网页；
- PR 需要提供真实 AirSim 运行截图或 GIF，并由另一台机器复核。

参考：

- [OpenHUTB/ros2 提交注意事项](https://github.com/OpenHUTB/ros2#注意事项)
- [OpenHUTB 贡献指南](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md)
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
- [ROS 2 launch 教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ROS 2 TF2 教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [ROS REP-103 坐标系约定](https://www.ros.org/reps/rep-0103.html)

## AI 辅助声明

本模块、测试和文档由大语言模型辅助整理；提交者应实际运行并确认结果。
