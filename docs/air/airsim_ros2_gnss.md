# AirSim ROS 2 Humble GNSS 桥接

本模块把 AirSim 多旋翼 GNSS/GPS 数据接入 ROS 2 Humble，并发布标准
`sensor_msgs/msg/NavSatFix`。现有 `uav_gps_integrity_guard` 负责消费 GPS
样本进行完整性和欺骗判断，本模块补齐 ROS 2 侧的数据来源。

## 1. 接口

| 类型 | 名称 | 消息 |
| --- | --- | --- |
| 发布 | `/uav/gps/fix` | `sensor_msgs/msg/NavSatFix` |

`NavSatFix.status.status` 映射规则：

- AirSim `is_valid=false` 或 fix type 小于阈值：`STATUS_NO_FIX`；
- 有效 2D/3D 定位：`STATUS_FIX`；
- 位置协方差为 `[eph², eph², epv²]` 对角阵；
- `eph` 或 `epv` 无效时使用配置中的回退方差。

## 2. 数据兼容性

AirSim 不同版本存在两种 GPS 数据布局：

```text
GpsData.gnss.geo_point.{latitude, longitude, altitude}
GpsData.{latitude, longitude, altitude}
```

模块同时兼容两者，并将 fix type、`eph`、`epv` 和 `is_valid` 转换为 ROS
`NavSatFix`。模块不进行 ENU 投影、RTK 解算或 GPS/INS 融合。

## 3. 构建

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

## 4. 运行

```bash
ros2 launch airsim_ros2_gnss bringup.launch.py \
  host:=<Windows_IP> port:=41451 vehicle_name:=""
```

检查定位消息：

```bash
ros2 topic hz /uav/gps/fix
ros2 topic echo /uav/gps/fix --once
```

连接尝试在独立线程执行，AirSim RPC 使用 `connection_timeout` 限时，默认
`1.0` 秒；`reconnect_period` 默认 `2.0` 秒。AirSim 离线或连接阻塞时，ROS
定时器仍可继续执行，不会把执行器卡住数小时。

重要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `host` | `127.0.0.1` | AirSim RPC 地址 |
| `port` | `41451` | AirSim RPC 端口 |
| `vehicle_name` | 空 | 多旋翼名称 |
| `gps_name` | 空 | GPS 名称 |
| `fix_topic` | `/uav/gps/fix` | NavSatFix 话题 |
| `frame_id` | `gps_link` | GNSS 坐标系 |
| `publish_rate` | `5.0` | 读取和发布频率 |
| `connection_timeout` | `1.0` | AirSim RPC 连接超时 |
| `minimum_fix_type` | `2` | 最低有效 fix type |

## 5. 测试

```bash
python3 -m pytest src/air/airsim_ros2_gnss/tests -q
colcon test --packages-select airsim_ros2_gnss
colcon test-result --verbose
```

单元测试覆盖新旧 AirSim GPS 布局、fix status、协方差和非法经纬度。启动测试
执行 `bringup.launch.py`，在 AirSim 不在线时检查节点保持运行，并发现
`/uav/gps/fix` 话题。连接测试使用阻塞式假客户端验证后台连接不会阻塞节点。

## 6. 范围与限制

- 只发布经纬度、高度、定位状态和水平/垂直精度；
- 不发布 GNSS 速度、卫星数、DOP 或原始伪距；
- 不替代 GPS 完整性、漂移、跳变和欺骗检测模块；
- 真实经纬度、高度、协方差和频率仍需在 AirSim/OpenHUTB 场景中验证。

## 7. ROS 2 与仓库贡献规范

本模块遵循 [ROS 2 Humble 文档](https://docs.ros.org/en/humble/) 的节点、参数、
QoS、launch 和消息接口约定。模块入口为 `main`，并提供 ROS 2 launch 文件。

按照 [OpenHUTB/ros2 提交注意事项](https://github.com/OpenHUTB/ros2#注意事项) 和
[OpenHUTB 贡献指南](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md)：

- 源码位于 `src/air/airsim_ros2_gnss`；
- 文档位于 `docs/air/airsim_ros2_gnss.md`；
- 首页和 `mkdocs.yml` 已加入链接；
- 提交前需运行 `mkdocs serve --livereload`；
- PR 需附真实运行截图或 GIF，并请另一名贡献者在另一台机器复核；
- 合并前解决与主分支的冲突，并在变更页逐项检查文件。

## AI 辅助声明

本模块、测试和文档由大语言模型辅助整理；提交者应连接实际 AirSim 场景，检查
GNSS 坐标、fix status、协方差和频率，并对提交内容负责。
