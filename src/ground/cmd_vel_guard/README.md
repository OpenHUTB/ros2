# cmd_vel_guard

地面移动底盘的 `/cmd_vel` 安全监督节点。订阅原始速度指令 `/cmd_vel_raw`，经过限幅、急停、超时三道保护后发布到 `/cmd_vel`。不需要 Gazebo 或任何仿真环境。

## 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/cmd_vel_raw` | `geometry_msgs/Twist` | 上游（遥控、导航、算法）发出的原始速度 |
| 订阅 | `/emergency_stop` | `std_msgs/Bool` | `true` 立即输出零速度 |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | 经过保护后的安全速度 |
| 发布 | `/guard_status` | `std_msgs/String` | `waiting` / `ok` / `clamped` / `timeout` / `estop` / `invalid` |

`invalid` 表示收到含 `NaN` 或 `Inf` 的指令，节点直接输出零速度，不会把非法值当成上限速度。

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `max_linear_x` | 0.3 | 前进/后退速度上限，m/s，须为非负有限值 |
| `max_linear_y` | 0.0 | 横向速度上限，普通差速底盘为 0 |
| `max_angular_z` | 0.8 | 角速度上限，rad/s，须为非负有限值 |
| `cmd_timeout` | 2.0 | 指令超时时间，秒，须大于 0 |
| `publish_rate` | 20.0 | 输出频率，Hz，须大于 0 |

参数非法时节点打印错误日志后退出，不会带着错误配置运行。

## 编译与运行

```bash
colcon build --packages-select cmd_vel_guard
source install/setup.bash
ros2 launch cmd_vel_guard main.launch.py
```

带参数启动，例如上限 0.2 m/s、超时 1 秒、输出 10 Hz：

```bash
ros2 launch cmd_vel_guard main.launch.py \
  max_linear_x:=0.2 cmd_timeout:=1.0 publish_rate:=10.0
```

## 快速验证

```bash
# 终端 2：发布超速指令（0.8 m/s，超过 0.3 上限）
ros2 topic pub -r 10 /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.8}}"

# 终端 3：查看限幅后的输出
ros2 topic echo /cmd_vel
```

终端 1 会打印 `指令超限已截断: x 0.80->0.30, z 0.00->0.00`（每秒最多一次），终端 3 中的 `linear.x` 稳定在 `0.3`，`/guard_status` 为 `clamped`。

急停验证：

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
```

此时 `/cmd_vel` 持续为全零，`/guard_status` 为 `estop`。急停触发时节点会清除缓存的旧指令，急停期间收到的指令同样被丢弃；解除急停（发布 `false`）后状态变为 `waiting` 并保持零速，必须重新发送 `/cmd_vel_raw` 才会恢复输出，避免底盘接着执行急停前的旧速度。

非法值验证：用 `nan` 作为速度分量发布，`/cmd_vel` 仍为全零，`/guard_status` 为 `invalid`。

超时验证：`Ctrl+C` 停掉终端 2，超过 `cmd_timeout` 后终端 1 打印 `2.1 秒未收到速度指令，已自动停车`（秒数为实际耗时，保留一位小数），`/cmd_vel` 变为全零。

## 测试

```bash
colcon test --packages-select cmd_vel_guard
colcon test-result --verbose
```

用例覆盖 `_clamp` 的正常值、边界值与 `NaN`/`Inf`，急停优先于超时、超时输出全零、非法指令输出全零、正常指令原样透传，急停清除缓存后「解除但不再发送新指令」保持零速、急停期间收到的指令被丢弃，以及非法参数被拒绝。
