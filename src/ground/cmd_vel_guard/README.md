# cmd_vel_guard

地面移动底盘的 `/cmd_vel` 安全监督节点。订阅原始速度指令 `/cmd_vel_raw`，经过限幅、急停、超时三道保护后发布到 `/cmd_vel`。不需要 Gazebo 或任何仿真环境。

## 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/cmd_vel_raw` | `geometry_msgs/Twist` | 上游（遥控、导航、算法）发出的原始速度 |
| 订阅 | `/emergency_stop` | `std_msgs/Bool` | `true` 立即输出零速度 |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | 经过保护后的安全速度 |
| 发布 | `/guard_status` | `std_msgs/String` | `waiting` / `ok` / `clamped` / `timeout` / `estop` |

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `max_linear_x` | 0.3 | 前进/后退速度上限，m/s |
| `max_linear_y` | 0.0 | 横向速度上限，普通差速底盘为 0 |
| `max_angular_z` | 0.8 | 角速度上限，rad/s |
| `cmd_timeout` | 2.0 | 指令超时时间，秒 |
| `publish_rate` | 20.0 | 输出频率，Hz |

## 编译与运行

```bash
colcon build --packages-select cmd_vel_guard
source install/setup.bash
ros2 launch cmd_vel_guard main.launch.py
```

## 快速验证

```bash
# 终端 2：发布超速指令（0.8 m/s，超过 0.3 上限）
ros2 topic pub -r 10 /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.8}}"

# 终端 3：查看限幅后的输出
ros2 topic echo /cmd_vel
```

终端 1 应打印 `指令超限已截断: x 0.80->0.30`，终端 3 中的 `linear.x` 稳定在 `0.3`。

急停验证：

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
```

超时验证：`Ctrl+C` 停掉终端 2，2 秒后终端 1 打印 `2.0 秒未收到速度指令，已自动停车`，`/cmd_vel` 变为全零。
