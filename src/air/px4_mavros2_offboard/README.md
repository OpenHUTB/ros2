# px4_mavros2_offboard

ROS 2 Humble 的 PX4 SITL + MAVROS2 Offboard 起降基线。它补齐 RflySim/PX4
SITL 教程中“ROS 话题控制尚未实施”的缺口，重点验证通信、模式切换、解锁、
起飞、悬停、降落和失效保护，不包含视觉目标追踪。

## 状态机

```text
IDLE
  -> PRESTREAM -> REQUEST_OFFBOARD -> ARMING
  -> TAKEOFF -> HOVER -> LANDING -> DONE
```

下列情况进入 `FAILSAFE` 并请求 `AUTO.LAND`：

- MAVROS 状态超过 `state_timeout` 没有更新；
- 进入 OFFBOARD 超时；
- 自动解锁超时；
- 起飞或降落超时；
- 操作员调用 `/uav/offboard/abort`。

## 依赖

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras
```

运行 GeographicLib 数据集安装脚本（首次安装 MAVROS2 时）：

```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

## 构建

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/hutb_ws/src
ln -s /path/to/ros2/src/air/px4_mavros2_offboard \
  ~/hutb_ws/src/px4_mavros2_offboard
cd ~/hutb_ws
colcon build --symlink-install
source install/setup.bash
```

## 运行

先启动 RflySim/PX4 SITL，并等待 QGroundControl 显示 `Ready To Fly`。
`fcu_url` 必须填写当前仿真环境对外提供的 MAVLink 端点。默认值适用于常见的
本机 PX4 SITL UDP 14540/14557 组合。

```bash
ros2 launch px4_mavros2_offboard offboard.launch.py \
  fcu_url:=udp://:14540@127.0.0.1:14557
```

如果已经单独启动 MAVROS2：

```bash
ros2 launch px4_mavros2_offboard offboard.launch.py start_mavros:=false
```

确认状态和位置：

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/odom --once
```

启动安全序列：

```bash
ros2 service call /uav/offboard/start std_srvs/srv/Trigger
ros2 topic echo /uav/offboard/status
```

中止并请求自动降落：

```bash
ros2 service call /uav/offboard/abort std_srvs/srv/Trigger
```

## 测试

```bash
python3 -m pytest src/air/px4_mavros2_offboard/tests -q
colcon test --packages-select px4_mavros2_offboard
```

单元测试覆盖预推流、模式确认、解锁确认、起飞、悬停、降落、超时和操作员中止。
`test_node_startup.py` 还会执行
`ros2 launch px4_mavros2_offboard offboard.launch.py start_mavros:=false`，
确认节点保持运行并可通过 ROS 2 发现 `/uav/offboard/start` 和
`/uav/offboard/abort` 服务；该检查不要求 PX4 或 MAVROS 在线。

## 范围与限制

- 只使用 MAVROS2 的位置设定点，不包含 Offboard 姿态/角速度控制；
- 不自动覆盖遥控器或飞控内部失效保护；
- 不验证真实气动模型，所有结论需注明 SITL 环境；
- 该模块与 `PX4+ROS2+OpenCV` 视觉追踪 PR 的职责不同。

## ROS 2 与仓库贡献约定

- 使用 ROS 2 Humble、`rclpy`、ament_python、标准消息和 `std_srvs/srv/Trigger`；
- MAVROS 状态使用 Humble 支持的显式可靠 QoS（depth 10），本地位置使用传感器 QoS；
- 模块入口统一为 `main`，并同时提供 `launch/offboard.launch.py`；
- 文档已在 `mkdocs.yml` 和 `docs/index.md` 添加入口；
- 提交前必须运行 `mkdocs serve --livereload` 检查网页；
- PR 需要提供真实 SITL 运行截图或 GIF，并由另一台机器复核。

参考：

- [OpenHUTB/ros2 提交注意事项](https://github.com/OpenHUTB/ros2#注意事项)
- [OpenHUTB 贡献指南](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md)
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
- [ROS 2 QoS 设置](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [MAVROS2 文档](https://docs.ros.org/en/humble/p/mavros/)

## AI 辅助声明

本模块、测试和文档由大语言模型辅助整理；提交者应在 RflySim/PX4 SITL
中完成闭环测试并在 PR 中记录实际日志。
