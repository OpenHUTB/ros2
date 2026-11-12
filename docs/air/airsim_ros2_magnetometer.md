# AirSim ROS 2 Humble 磁力计桥接

本模块把 AirSim 多旋翼磁力计接入 ROS 2 Humble，发布
`sensor_msgs/msg/MagneticField`，用于航向估计和磁场分析。

## 1. 接口

| 类型 | 名称 | 消息 |
| --- | --- | --- |
| 发布 | `/uav/magnetometer` | `sensor_msgs/msg/MagneticField` |

AirSim 磁场位于机体 FRD，模块转换为 ROS FLU：

```text
x_flu = x_frd
y_flu = -y_frd
z_flu = -z_frd
```

`field_scale` 默认 `1.0`，不会静默改变 AirSim 原始数值；若场景使用高斯而
下游需要 Tesla，应显式设置 `field_scale:=0.0001`。使用前必须确认场景单位。
发布端使用可靠 QoS；AirSim 未提供协方差时，消息协方差保持全零，表示未知。

![磁力计桥接验证成功](../img/air/airsim_ros2_magnetometer/preview.png)

## 2. 构建与运行

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/hutb_ws/src
ln -s /path/to/ros2/src/air/airsim_ros2_magnetometer \
  ~/hutb_ws/src/airsim_ros2_magnetometer
cd ~/hutb_ws
colcon build --symlink-install --packages-select airsim_ros2_magnetometer
source install/setup.bash
ros2 launch airsim_ros2_magnetometer bringup.launch.py \
  host:=<Windows_IP> port:=41451 vehicle_name:=""
```

```bash
ros2 topic hz /uav/magnetometer
ros2 topic echo /uav/magnetometer --once
```

连接在后台执行，`connection_timeout` 默认 `1.0` 秒，不会阻塞 ROS 定时器；
`reconnect_period` 默认 `2.0` 秒。

## 3. 测试

```bash
python3 -m pytest src/air/airsim_ros2_magnetometer/tests -q
colcon test --packages-select airsim_ros2_magnetometer
```

测试覆盖 FRD→FLU、尺度换算、协方差、非法样本、阻塞连接和启动检查。

## 4. 范围与限制

- 不执行磁偏角补偿、椭球拟合、硬铁/软铁标定或姿态融合；
- 不假设 AirSim 原始磁场单位，必须通过 `field_scale` 明确单位；
- 真实磁场、方向、噪声和频率仍需在 AirSim/OpenHUTB 中验证。

## 5. ROS 2 与仓库贡献规范

本模块遵循 [ROS 2 Humble 文档](https://docs.ros.org/en/humble/) 的节点、参数、
QoS、launch 和消息接口约定，并提供 `main` 入口、launch、README、首页与导航。

## AI 辅助声明

本模块、测试和文档由大语言模型辅助整理；提交者应实际连接 AirSim，检查磁场
方向、量级、单位、噪声和频率，并对提交内容负责。
