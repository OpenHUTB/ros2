# airsim_ros2_imu

ROS 2 Humble 的 AirSim IMU 桥接。仓库现有
`carlair_ros_bridge/scripts/imu_pub.py` 面向 ROS Noetic，本模块提供 ROS 2
标准 `sensor_msgs/msg/Imu` 接口，可直接用于状态估计、振动分析和控制调试。

## 功能范围

- 从 AirSim `getImuData` 读取姿态、角速度和线加速度；
- 发布 `/uav/imu`，消息类型为 `sensor_msgs/msg/Imu`；
- 将世界 NED、机体 FRD 转换为 ROS REP-103 的 ENU、FLU；
- 支持多旋翼、IMU 名称、话题、坐标系和发布频率参数；
- AirSim 断连后停止发布旧数据并定期重连；
- 提供统一的 `main` 入口和 ROS 2 launch。

本模块不进行姿态融合、偏置标定、重力补偿、滤波或故障诊断。

## 坐标转换

AirSim 的 IMU 角速度和线加速度位于机体 FRD 坐标系：

```text
forward_flu = forward_frd
left_flu    = -right_frd
up_flu      = -down_frd
```

姿态从世界 NED、机体 FRD 转换为世界 ENU、机体 FLU。转换函数位于
`airsim_ros2_imu/transforms.py`，可脱离 ROS 和模拟器单独测试。

## 构建

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/hutb_ws/src
ln -s /path/to/ros2/src/air/airsim_ros2_imu \
  ~/hutb_ws/src/airsim_ros2_imu
cd ~/hutb_ws
colcon build --symlink-install --packages-select airsim_ros2_imu
source install/setup.bash
```

安装 AirSim Python 客户端：

```bash
python3 -m pip install -r \
  src/air/airsim_ros2_imu/requirements.txt
```

## 运行

先启动 AirSim/OpenHUTB 场景，再执行：

```bash
ros2 launch airsim_ros2_imu bringup.launch.py \
  host:=<Windows_IP> port:=41451 vehicle_name:=""
```

检查消息和频率：

```bash
ros2 topic hz /uav/imu
ros2 topic echo /uav/imu --once
```

默认消息坐标系为 `base_link`，`use_sim_time` 默认关闭，因为当前 AirSim 场景
没有发布 `/clock`。

节点读取 AirSim IMU 的 `time_stamp`（纳秒）并作为消息 header 时间戳；如果
`use_source_timestamp:=true`。重复的时间戳不会再次发布，超过 `stale_timeout`
没有新样本时会记录陈旧数据警告。若需要改用 ROS 当前时间，可设置
`use_source_timestamp:=false`。

## 测试

```bash
python3 -m pytest src/air/airsim_ros2_imu/tests -q
colcon test --packages-select airsim_ros2_imu
```

纯 Python 测试覆盖坐标和四元数转换。启动测试会实际运行 launch，确认节点在
AirSim 不在线时仍能启动，并能发现 `/uav/imu` 话题。消息测试还会直接检查
四元数、FRD→FLU 数值、协方差、frame_id 和 source timestamp。

## ROS 2 与仓库贡献约定

- 使用 ROS 2 Humble、`rclpy`、ament_python 和标准 `sensor_msgs/msg/Imu`；
- 高频 IMU 话题使用传感器数据 QoS；
- 模块入口统一为 `main`，并提供 `launch/bringup.launch.py`；
- 源码和教程分别位于 `src/air`、`docs/air`；
- 首页和 `mkdocs.yml` 已加入导航；
- PR 需要补充真实 AirSim IMU 频率、数值和实际运行截图/GIF，并由另一台机器复核。

参考：

- [OpenHUTB/ros2 提交注意事项](https://github.com/OpenHUTB/ros2#注意事项)
- [OpenHUTB 贡献指南](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md)
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
- [ROS REP-103 坐标系约定](https://www.ros.org/reps/rep-0103.html)
- [sensor_msgs/Imu 接口](https://docs.ros.org/en/humble/p/sensor_msgs/msg/Imu.html)

## AI 辅助声明

本模块、测试和文档由大语言模型辅助整理；提交者应实际连接 AirSim，检查 IMU
数值、方向、频率和静态噪声，并对提交内容负责。
