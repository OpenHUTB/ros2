# PX4 SITL + MAVROS2 Offboard 起降与失效保护

本模块面向 ROS 2 Humble，把 RflySim/PX4 SITL 的 MAVLink 接口接入
MAVROS2，并实现受保护的 Offboard 起飞、悬停和降落流程。

现有 [RflySim 单机起飞实验](./rflysim_sitl_first_flight.md) 已完成
QGroundControl 连接和三维显示，但尚未接入 ROS 控制。本模块只补齐这部分，
不涉及视觉目标追踪、姿态 Offboard 或多机协同。

## 1. 状态机

```text
IDLE
  -> PRESTREAM
  -> REQUEST_OFFBOARD
  -> ARMING
  -> TAKEOFF
  -> HOVER
  -> LANDING
  -> DONE
```

`FAILSAFE` 可以由 MAVROS 状态超时、模式/解锁超时、起飞/降落超时或用户中止触发。
触发后持续请求 `AUTO.LAND`，待飞行器上锁且高度回到起飞高度附近后结束。

## 2. ROS 接口

| 类型 | 名称 | 说明 |
| --- | --- | --- |
| 订阅 | `/mavros/state` | MAVROS 连接、模式和上锁状态 |
| 订阅 | `/mavros/local_position/odom` | 本地 ENU 位置 |
| 发布 | `/mavros/setpoint_position/local` | 位置设定点，默认 20 Hz |
| 发布 | `/uav/offboard/status` | 当前状态机文本 |
| 服务 | `/uav/offboard/start` | 启动序列 |
| 服务 | `/uav/offboard/abort` | 进入失效保护并请求降落 |

## 3. 依赖与构建

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

source /opt/ros/humble/setup.bash
mkdir -p ~/hutb_ws/src
ln -s /path/to/ros2/src/air/px4_mavros2_offboard \
  ~/hutb_ws/src/px4_mavros2_offboard
cd ~/hutb_ws
colcon build --symlink-install
source install/setup.bash
```

## 4. RflySim/PX4 SITL 运行

先启动 RflySim，等待 QGroundControl 显示 `Ready To Fly`。确认当前环境的
MAVLink 端点，然后启动：

```bash
ros2 launch px4_mavros2_offboard offboard.launch.py \
  fcu_url:=udp://:14540@127.0.0.1:14557
```

默认 `fcu_url` 对应常见的本机 PX4 SITL UDP 组合；RflySim 或虚拟机网络
使用其他端口时，以实际 MAVLink 路由配置为准。

启动前检查：

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/odom --once
```

执行：

```bash
ros2 service call /uav/offboard/start std_srvs/srv/Trigger
ros2 topic echo /uav/offboard/status
```

中止：

```bash
ros2 service call /uav/offboard/abort std_srvs/srv/Trigger
```

## 5. 测试

```bash
python3 -m pytest src/air/px4_mavros2_offboard/tests -q
colcon test --packages-select px4_mavros2_offboard
colcon test-result --verbose
```

状态机测试不依赖 MAVROS、PX4 或模拟器，覆盖：

- 2 秒设定点预推流；
- OFFBOARD 模式确认后才解锁；
- 解锁确认后才起飞；
- 到达目标高度后悬停；
- 悬停结束后请求 `AUTO.LAND`；
- 状态超时和操作员中止进入 `FAILSAFE`。

节点启动测试会以 `start_mavros:=false` 执行 launch，检查节点进程持续存活，
并通过 ROS 2 发现 `/uav/offboard/start` 和 `/uav/offboard/abort` 服务。
该测试覆盖实际入口导入、节点创建和 ROS 服务注册，不要求 PX4 或 MAVROS 在线。

## 6. 提交 PR 时应补充的真实证据

单元测试不能代替 SITL 闭环。PR 应提供：

1. QGroundControl 的 `Ready To Fly` 截图；
2. MAVROS `/mavros/state` 和本地位置话题输出；
3. `/uav/offboard/status` 从预推流到 `DONE` 的日志；
4. 实际起飞高度、悬停时间和降落结果；
5. 主动取消或状态超时触发 `FAILSAFE` 的记录。

## 7. ROS 2 与仓库贡献规范

本模块遵循 [ROS 2 Humble 文档](https://docs.ros.org/en/humble/) 的节点、参数、QoS、
Launch 和消息接口约定。安全关键的 `/mavros/state` 使用 Humble 支持的显式可靠
QoS（depth 10），高频本地位置使用传感器 QoS。模块入口为 `main`，并提供
ROS 2 launch 文件。

按照 [OpenHUTB/ros2 提交注意事项](https://github.com/OpenHUTB/ros2#注意事项) 和
[OpenHUTB 贡献指南](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md)：

- 源码位于 `src/air/px4_mavros2_offboard`；
- 文档位于 `docs/air/px4_mavros2_offboard.md`；
- 首页和 `mkdocs.yml` 已加入链接；
- 提交前需运行 `mkdocs serve --livereload`；
- PR 需附真实运行截图或 GIF，并请另一名贡献者在另一台机器复核；
- 合并前解决与主分支的冲突，并在变更页逐项检查文件。

## AI 辅助声明

本模块、测试和文档由大语言模型辅助整理；提交者应在 RflySim/PX4 SITL
中实际验证并负责提交内容。
