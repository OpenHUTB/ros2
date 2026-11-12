# airsim_ros2_barometer

ROS 2 Humble 的 AirSim 气压计桥接。模块将 AirSim `getBarometerData` 的绝对
压力、高度和海平面基准压力分别发布为 ROS 2 标准或通用消息，为高度估计、
定高控制、气压计噪声分析和传感器融合提供数据源。

## 功能范围

- 从 AirSim `getBarometerData` 读取压力、高度和 QNH；
- 发布 `/uav/barometer/pressure`，类型为
  `sensor_msgs/msg/FluidPressure`；
- 发布 `/uav/barometer/altitude`，类型为 `std_msgs/msg/Float64`，单位为米；
- 发布 `/uav/barometer/qnh`，类型为 `std_msgs/msg/Float64`，单位为 Pa；
- 检查压力、高度和 QNH 是否为有限有效值；
- 支持车辆、气压计名称、话题、坐标系、频率和压力方差参数；
- AirSim 断连后停止发布旧数据并定期重连；
- 提供统一的 `main` 入口和 ROS 2 launch。

本模块不进行温度补偿、偏置标定、滤波、高度融合或 QNH 动态估计。

## 接口

| 类型 | 名称 | 消息 | 单位 |
| --- | --- | --- | --- |
| 发布 | `/uav/barometer/pressure` | `sensor_msgs/msg/FluidPressure` | Pa |
| 发布 | `/uav/barometer/altitude` | `std_msgs/msg/Float64` | m |
| 发布 | `/uav/barometer/qnh` | `std_msgs/msg/Float64` | Pa |

`FluidPressure.variance` 默认取 `1.0 Pa²`，可通过
`pressure_variance` 参数调整。

AirSim 1.8.1 的 QNH 原始单位为 hPa，模块在发布前乘以 `100` 转为 Pa。
例如场景默认值 `1013.25 hPa` 会发布为 `101325 Pa`。

AirSim 的连接和读取 RPC 在同一个后台工作线程执行，`rpc_timeout` 默认
`1.0` 秒，不会阻塞 ROS 定时器；`reconnect_period` 默认 `2.0` 秒。

## 构建

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/hutb_ws/src
ln -sfn ~/ros2/src/air/airsim_ros2_barometer \
  ~/hutb_ws/src/airsim_ros2_barometer
python3 -m pip install -r \
  ~/hutb_ws/src/airsim_ros2_barometer/requirements.txt
cd ~/hutb_ws
colcon build --symlink-install --packages-select airsim_ros2_barometer
source install/setup.bash
```

## 运行

先启动 AirSim/OpenHUTB 场景：

```bash
ros2 launch airsim_ros2_barometer bringup.launch.py \
  host:=<Windows_IP> port:=41451 vehicle_name:=""
```

检查气压、高度和频率：

```bash
ros2 topic echo /uav/barometer/pressure --once
ros2 topic echo /uav/barometer/altitude --once
ros2 topic hz /uav/barometer/pressure
```

默认坐标系为 `base_link`，`publish_rate` 为 `20.0` Hz，表示最大调度和发布
频率；AirSim RPC 变慢时实际频率会下降。AirSim 场景未发布 `/clock` 时保持
`use_sim_time=false`。

## 测试

```bash
python3 -m pytest ~/hutb_ws/src/airsim_ros2_barometer/test -q
colcon test --packages-select airsim_ros2_barometer
colcon test-result --verbose
```

20 个用例覆盖有效样本、非法压力/高度/QNH、消息字段映射、AirSim 客户端
缺失提示和启动检查。启动测试会执行 launch，确认 AirSim 不在线时节点保持
运行，并能发现三个气压计话题。连接测试覆盖 1–4 次读取失败、第 5 次断开、
后台重连恢复，以及连接和读取阻塞时 ROS 定时器不被占用。仓库 Air bridge
workflow 会在 Humble 容器中构建并测试本包。

## 范围与限制

- 除 QNH 的 hPa→Pa 转换外只转发 AirSim 数据，不执行海拔解算、温度补偿或 Kalman 融合；
- 不保证 QNH 与真实气象基准一致，使用前应检查场景配置；
- 不替代 IMU/高度融合、气压计故障检测或飞控内部 EKF；
- 真实压力、高度、QNH、噪声和频率仍需在 AirSim/OpenHUTB 场景中验证。

## ROS 2 与仓库贡献约定

- 使用 ROS 2 Humble、`rclpy`、ament_python 和标准 ROS 消息；
- 高频气压话题使用传感器数据 QoS；
- 模块入口统一为 `main`，并提供 `launch/bringup.launch.py`；
- 源码和教程分别位于 `src/air`、`docs/air`；
- 首页和 `mkdocs.yml` 已加入导航；
- PR 需要补充真实气压、高度、QNH 和实际运行截图/GIF，并由另一台机器复核。

参考：

- [OpenHUTB/ros2 提交注意事项](https://github.com/OpenHUTB/ros2#注意事项)
- [OpenHUTB 贡献指南](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md)
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
- [sensor_msgs/FluidPressure 接口](https://docs.ros.org/en/humble/p/sensor_msgs/msg/FluidPressure.html)

## AI 辅助声明

本模块、测试和文档由大语言模型辅助整理；提交者应连接 AirSim，检查压力、
高度、QNH、噪声、频率和坐标系，并对提交内容负责。
