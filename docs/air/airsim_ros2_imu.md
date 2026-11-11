# AirSim ROS 2 Humble IMU 桥接

本模块把 AirSim 多旋翼 IMU 接入 ROS 2 Humble，并发布标准
`sensor_msgs/msg/Imu`。现有 `carlair_ros_bridge` 的 IMU 发布代码面向 ROS
Noetic；本模块提供 ROS 2 原生入口，便于状态估计、振动分析和控制器调试。

## 1. 接口

| 类型 | 名称 | 消息 |
| --- | --- | --- |
| 发布 | `/uav/imu` | `sensor_msgs/msg/Imu` |

消息包含 AirSim IMU 的姿态、角速度和线加速度，默认坐标系为 `base_link`。
高频数据使用传感器数据 QoS。

## 2. 坐标转换

AirSim 的 IMU 角速度和线加速度位于机体 FRD 坐标系，ROS REP-103 使用机体 FLU：

```text
x_flu = x_frd
y_flu = -y_frd
z_flu = -z_frd
```

姿态从世界 NED、机体 FRD 转换到世界 ENU、机体 FLU。AirSim 中机头朝北时，
ROS ENU 偏航为 `+90°`。转换逻辑位于 `airsim_ros2_imu/transforms.py`。

## 3. 构建

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/hutb_ws/src
ln -s /path/to/ros2/src/air/airsim_ros2_imu \
  ~/hutb_ws/src/airsim_ros2_imu
cd ~/hutb_ws
colcon build --symlink-install --packages-select airsim_ros2_imu
source install/setup.bash
python3 -m pip install -r \
  src/air/airsim_ros2_imu/requirements.txt
```

## 4. 运行

先启动 AirSim/OpenHUTB 场景：

```bash
ros2 launch airsim_ros2_imu bringup.launch.py \
  host:=<Windows_IP> port:=41451 vehicle_name:=""
```

检查频率和消息：

```bash
ros2 topic hz /uav/imu
ros2 topic echo /uav/imu --once
```

重要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `host` | `127.0.0.1` | AirSim RPC 地址 |
| `port` | `41451` | AirSim RPC 端口 |
| `vehicle_name` | 空 | 多旋翼名称 |
| `imu_name` | 空 | IMU 名称 |
| `imu_topic` | `/uav/imu` | 输出话题 |
| `frame_id` | `base_link` | IMU 坐标系 |
| `publish_rate` | `50.0` | 读取和发布频率 |
| `stale_timeout` | `0.25` | 无新时间戳时报告陈旧的秒数 |
| `use_source_timestamp` | `true` | 使用 AirSim 纳秒时间戳作为 header |
| `orientation_variance` | `0.001` | 姿态方差 |
| `angular_velocity_variance` | `0.0001` | 角速度方差 |
| `linear_acceleration_variance` | `0.01` | 线加速度方差 |

## 5. 测试

```bash
python3 -m pytest src/air/airsim_ros2_imu/tests -q
colcon test --packages-select airsim_ros2_imu
colcon test-result --verbose
```

测试覆盖 FRD/FLU 向量转换、NED/ENU 姿态转换、四元数归一化和非法输入。
启动测试执行 `bringup.launch.py`，在 AirSim 不在线时检查节点保持运行，并能
通过 ROS 2 发现 `/uav/imu` 话题。消息测试直接使用固定 ImuData 样本检查
四元数、FRD→FLU、协方差、frame_id 和 source timestamp。

## 6. 范围与限制

- 只发布原始 IMU，不进行姿态融合、滤波、偏置标定或振动抑制；
- 不执行重力补偿，AirSim 的线加速度语义以实际固件配置为准；
- 当前 AirSim 场景不发布 `/clock`，`use_sim_time` 默认关闭；
- `time_stamp` 按纳秒映射到 ROS header；重复样本会被抑制，
  `stale_timeout` 用于报告无新样本；
- 真实 IMU 数值、方向和频率仍需在 AirSim/OpenHUTB 场景中验证。

## 7. ROS 2 与仓库贡献规范

本模块遵循 [ROS 2 Humble 文档](https://docs.ros.org/en/humble/) 的节点、参数、
QoS、launch 和消息接口约定，坐标遵循
[ROS REP-103](https://www.ros.org/reps/rep-0103.html)。模块入口为 `main`，
并提供 ROS 2 launch 文件。

按照 [OpenHUTB/ros2 提交注意事项](https://github.com/OpenHUTB/ros2#注意事项) 和
[OpenHUTB 贡献指南](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md)：

- 源码位于 `src/air/airsim_ros2_imu`；
- 文档位于 `docs/air/airsim_ros2_imu.md`；
- 首页和 `mkdocs.yml` 已加入链接；
- 提交前需运行 `mkdocs serve --livereload`；
- PR 需附真实运行截图或 GIF，并请另一名贡献者在另一台机器复核；
- 合并前解决与主分支的冲突，并在变更页逐项检查文件。

## AI 辅助声明

本模块、测试和文档由大语言模型辅助整理；提交者应连接实际 AirSim 场景，检查
IMU 数值方向、频率和静态噪声，并对提交内容负责。
