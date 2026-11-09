# 地面移动底盘速度指令安全监督

## 问题

移动底盘的 `/cmd_vel` 通常直接来自遥控、导航栈或算法节点。一旦上游发出超限速度、卡死不再更新，或者需要紧急停车，底盘会继续按最后一条指令运动，容易撞墙、冲出场地。安全做法是在 `/cmd_vel` 之前插入一个监督节点，对所有速度指令做统一处理。

本模块实现三道保护：

1. **限幅**：线速度与角速度超过参数上限时截断，防止底盘超速；
2. **急停**：`/emergency_stop` 为 `true` 时立即输出零速度，解除后恢复；
3. **超时保护**：超过 `cmd_timeout` 秒没有收到新指令时自动停车，防止上游崩溃后底盘继续运动。

## 环境要求

- ROS 2 Humble
- 只用 `rclpy`、`geometry_msgs`、`std_msgs`，**不需要 Gazebo 或任何仿真器**

## 文件结构

```
src/ground/cmd_vel_guard/
├── resource/cmd_vel_guard
├── cmd_vel_guard/
│   ├── __init__.py
│   └── cmd_vel_guard_node.py    # 监督节点
├── launch/
│   └── main.launch.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 话题与参数

| 方向 | 话题 | 类型 |
|---|---|---|
| 订阅 | `/cmd_vel_raw` | `geometry_msgs/Twist` |
| 订阅 | `/emergency_stop` | `std_msgs/Bool` |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` |
| 发布 | `/guard_status` | `std_msgs/String` |

| 参数 | 默认值 | 说明 |
|---|---|---|
| `max_linear_x` | 0.3 | 前进/后退速度上限，m/s |
| `max_linear_y` | 0.0 | 横向速度上限，差速底盘为 0 |
| `max_angular_z` | 0.8 | 角速度上限，rad/s |
| `cmd_timeout` | 2.0 | 指令超时时间，秒 |
| `publish_rate` | 20.0 | 输出频率，Hz |

`/guard_status` 取值：`waiting`（尚未收到指令）、`ok`、`clamped`（已截断）、`timeout`、`estop`。

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select cmd_vel_guard
source install/setup.bash
```

## 运行

```bash
ros2 launch cmd_vel_guard main.launch.py
```

带参数启动，例如把速度上限改成 0.2 m/s、超时改成 1 秒：

```bash
ros2 launch cmd_vel_guard main.launch.py max_linear_x:=0.2 cmd_timeout:=1.0
```

## 演示步骤

打开三个终端，都先执行 `source ~/ros2_ws/install/setup.bash`。

**终端 1**：启动监督节点

```bash
ros2 launch cmd_vel_guard main.launch.py
```

**终端 2**：以 10 Hz 发布超速指令（0.8 m/s，上限 0.3）

```bash
ros2 topic pub -r 10 /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.8}}"
```

**终端 3**：查看经过保护后的指令

```bash
ros2 topic echo /cmd_vel
```

预期结果：

- 终端 1 打印 `指令超限已截断: x 0.80->0.30`（1 秒最多一次，避免刷屏）
- 终端 3 中 `linear.x` 稳定为 `0.3`，`/guard_status` 为 `clamped`

**急停演示**（终端 4）：

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
ros2 topic echo /cmd_vel    # 应持续为全零，/guard_status 为 estop
```

**超时演示**：在终端 2 按 `Ctrl+C` 停止发布，2 秒后终端 1 打印 `未收到速度指令，已自动停车`，`/cmd_vel` 变为全零，`/guard_status` 为 `timeout`。

## 验收标准

| 检查项 | 预期 |
|---|---|
| 正常指令 | 输入 0.2 m/s，输出 0.2 m/s，`/guard_status` 为 `ok` |
| 限幅 | 输入 0.8 m/s，输出 0.3 m/s，状态 `clamped` |
| 急停 | `/emergency_stop` 为 true 时输出全零，日志提示急停 |
| 超时 | 停止发布 2 秒后输出全零，状态 `timeout` |
| 频率 | `/cmd_vel` 稳定在 20 Hz |
| 依赖 | 除 ROS 2 Humble 外无额外依赖，不需要仿真器 |

## 扩展方向

- 把 `/guard_status` 换成 `diagnostic_msgs/DiagnosticArray`，接入 `rqt_robot_monitor`；
- 增加 `cmd_vel` 平滑滤波（一阶低通），减少速度突变；
- 加入地理位置限速区：订阅 `/odom` 或定位话题，在特定区域内自动降速。
