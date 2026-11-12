# airsim_ros2_gnss

ROS 2 Humble 的 AirSim GNSS/GPS 桥接。仓库现有
`carlair_ros_bridge` 面向 ROS Noetic，`uav_gps_integrity_guard` 负责消费 GPS
样本做完整性判断；本模块补齐 ROS 2 侧从 AirSim 获取 GNSS 位置并发布标准
`sensor_msgs/msg/NavSatFix` 的数据源。

## 功能范围

- 从 AirSim `getGpsData` 读取经纬度、高度、定位有效性、水平和垂直精度；
- 发布 `/uav/gps/fix`，消息类型为 `sensor_msgs/msg/NavSatFix`；
- 将 AirSim GNSS fix type 和有效性映射为 ROS `NavSatStatus`；
- 根据 `eph`、`epv` 填充对角位置协方差；
- 兼容 AirSim 新旧 `GpsData` 字段布局；
- 支持车辆、GPS 名称、话题、坐标系、发布频率和最小 fix type 参数；
- AirSim 断连后停止发布旧数据，并在后台定期重连；
- 提供统一的 `main` 入口和 ROS 2 launch。

本模块不进行坐标投影、RTK 解算、GPS/INS 融合、完整性判断或欺骗检测。

## 接口

| 类型 | 名称 | 消息 |
| --- | --- | --- |
| 发布 | `/uav/gps/fix` | `sensor_msgs/msg/NavSatFix` |

`NavSatFix.status.status` 使用 ROS 标准值：

- `STATUS_NO_FIX`：AirSim 数据无效或 fix type 不足；
- `STATUS_FIX`：有效 2D/3D GNSS 定位；
- `position_covariance_type=COVARIANCE_TYPE_DIAGONAL_KNOWN` 表示协方差有效。

## 构建

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/hutb_ws/src
ln -s /path/to/ros2/src/air/airsim_ros2_gnss \
  ~/hutb_ws/src/airsim_ros2_gnss
cd ~/hutb_ws
colcon build --symlink-install --packages-select airsim_ros2_gnss
source install/setup.bash
python3 -m pip install -r \
  src/air/airsim_ros2_gnss/requirements.txt
```

## 运行

先启动 AirSim/OpenHUTB 场景：

```bash
ros2 launch airsim_ros2_gnss bringup.launch.py \
  host:=<Windows_IP> port:=41451 vehicle_name:=""
```

检查定位消息和频率：

```bash
ros2 topic hz /uav/gps/fix
ros2 topic echo /uav/gps/fix --once
```

默认坐标系为 `gps_link`，`publish_rate` 为 `5.0` Hz。AirSim 场景未发布
`/clock` 时保持 `use_sim_time=false`。

连接尝试在独立线程执行，`connection_timeout` 默认 `1.0` 秒，不会阻塞 ROS
定时器和其他回调；`reconnect_period` 默认 `2.0` 秒。

## 测试

```bash
python3 -m pytest src/air/airsim_ros2_gnss/tests -q
colcon test --packages-select airsim_ros2_gnss
```

测试覆盖新旧 AirSim `GpsData` 解析、fix type 映射、协方差计算和非法经纬度。
启动测试会实际运行 launch，确认 AirSim 不在线时节点保持运行，并能通过 ROS 2
发现 `/uav/gps/fix` 话题。连接测试使用故意阻塞的假 AirSim 客户端，验证连接
调用不会阻塞节点。

## 范围与限制

- 只发布 AirSim 提供的 GNSS 位置，不包含速度、卫星数或原始伪距；
- 不执行 ENU 投影和 GPS 坐标到局部地图的转换；
- 不替代 `uav_gps_integrity_guard` 的漂移、跳变或欺骗判断；
- 真实定位精度、坐标、协方差和频率仍需在 AirSim/OpenHUTB 场景中验证。

## ROS 2 与仓库贡献约定

- 使用 ROS 2 Humble、`rclpy`、ament_python 和标准 `sensor_msgs/msg/NavSatFix`；
- 高频 GNSS 话题使用传感器数据 QoS；
- 模块入口统一为 `main`，并提供 `launch/bringup.launch.py`；
- 源码和教程分别位于 `src/air`、`docs/air`；
- 首页和 `mkdocs.yml` 已加入导航；
- PR 需要补充真实 GNSS 位置、协方差和实际运行截图/GIF，并由另一台机器复核。

参考：

- [OpenHUTB/ros2 提交注意事项](https://github.com/OpenHUTB/ros2#注意事项)
- [OpenHUTB 贡献指南](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md)
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
- [sensor_msgs/NavSatFix 接口](https://docs.ros.org/en/humble/p/sensor_msgs/msg/NavSatFix.html)

## AI 辅助声明

本模块、测试和文档由大语言模型辅助整理；提交者应实际连接 AirSim，检查
经纬度、高度、fix status、协方差和频率，并对提交内容负责。
