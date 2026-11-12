# AirSim ROS 2 Humble 气压计桥接

本模块把 AirSim 气压计数据接入 ROS 2 Humble，发布标准
`sensor_msgs/msg/FluidPressure` 以及气压高度和 QNH，为高度估计、定高控制和
传感器噪声分析提供数据源。

## 1. 接口

| 类型 | 名称 | 消息 | 单位 |
| --- | --- | --- | --- |
| 发布 | `/uav/barometer/pressure` | `sensor_msgs/msg/FluidPressure` | Pa |
| 发布 | `/uav/barometer/altitude` | `std_msgs/msg/Float64` | m |
| 发布 | `/uav/barometer/qnh` | `std_msgs/msg/Float64` | Pa |

压力消息包含采样时间、坐标系和方差。默认坐标系为 `base_link`，默认频率为
`20 Hz`。

## 2. 数据检查

节点会拒绝以下样本：

- 高度不是有限值；
- 压力不是有限正数；
- QNH 不是有限非负数。

连续读取失败 5 次后，节点停止发布旧数据并重新连接 AirSim。

## 3. 构建

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/hutb_ws/src
ln -s /path/to/ros2/src/air/airsim_ros2_barometer \
  ~/hutb_ws/src/airsim_ros2_barometer
cd ~/hutb_ws
colcon build --symlink-install --packages-select airsim_ros2_barometer
source install/setup.bash
python3 -m pip install -r \
  src/air/airsim_ros2_barometer/requirements.txt
```

## 4. 运行

```bash
ros2 launch airsim_ros2_barometer bringup.launch.py \
  host:=<Windows_IP> port:=41451 vehicle_name:=""
```

检查数据：

```bash
ros2 topic echo /uav/barometer/pressure --once
ros2 topic echo /uav/barometer/altitude --once
ros2 topic echo /uav/barometer/qnh --once
ros2 topic hz /uav/barometer/pressure
```

AirSim 连接在独立线程执行，`connection_timeout` 默认 `1.0` 秒；
`reconnect_period` 默认 `2.0` 秒。连接阻塞时 ROS 定时器仍可执行。

重要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `host` | `127.0.0.1` | AirSim RPC 地址 |
| `port` | `41451` | AirSim RPC 端口 |
| `vehicle_name` | 空 | 多旋翼名称 |
| `barometer_name` | 空 | 气压计名称 |
| `pressure_topic` | `/uav/barometer/pressure` | 压力话题 |
| `altitude_topic` | `/uav/barometer/altitude` | 气压高度话题 |
| `qnh_topic` | `/uav/barometer/qnh` | QNH 话题 |
| `publish_rate` | `20.0` | 读取和发布频率 |
| `connection_timeout` | `1.0` | AirSim RPC 连接超时 |
| `reconnect_period` | `2.0` | 失败后的重连间隔 |
| `pressure_variance` | `1.0` | 压力方差 Pa² |

## 5. 测试

```bash
python3 -m pytest src/air/airsim_ros2_barometer/tests -q
colcon test --packages-select airsim_ros2_barometer
colcon test-result --verbose
```

测试覆盖有效样本、非法压力/高度/QNH 和启动检查。启动测试执行
`bringup.launch.py`，在 AirSim 不在线时检查节点保持运行，并发现三个气压计
话题。连接测试覆盖 1–4 次读取失败、第 5 次断开以及后台重连恢复。

## 6. 范围与限制

- 只转发 AirSim 气压计数据，不执行温度补偿、偏置标定或滤波；
- 不使用气压公式重新计算高度，不替代飞控 EKF 或高度融合模块；
- QNH 来自仿真场景配置，不能假设与真实气象数据一致；
- 真实压力、高度、QNH、噪声和频率仍需在 AirSim/OpenHUTB 场景中验证。

## 7. ROS 2 与仓库贡献规范

本模块遵循 [ROS 2 Humble 文档](https://docs.ros.org/en/humble/) 的节点、参数、
QoS、launch 和消息接口约定。模块入口为 `main`，并提供 ROS 2 launch 文件。

按照 [OpenHUTB/ros2 提交注意事项](https://github.com/OpenHUTB/ros2#注意事项) 和
[OpenHUTB 贡献指南](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md)：

- 源码位于 `src/air/airsim_ros2_barometer`；
- 文档位于 `docs/air/airsim_ros2_barometer.md`；
- 首页和 `mkdocs.yml` 已加入链接；
- 提交前需运行 `mkdocs serve --livereload`；
- PR 需附真实运行截图或 GIF，并请另一名贡献者在另一台机器复核；
- 合并前解决与主分支的冲突，并在变更页逐项检查文件。

## AI 辅助声明

本模块、测试和文档由大语言模型辅助整理；提交者应连接实际 AirSim 场景，检查
压力、高度、QNH、噪声和频率，并对提交内容负责。
