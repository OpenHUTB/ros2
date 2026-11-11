# uav_keyboard_control

终端键盘 → 机体系速度指令 `/uav/cmd_vel`，由 `carlair_ros_bridge` 发给仿真器执行。**ROS2 Humble (ament_python)**。

## 按键

```
W / S   前进 / 后退      forward_speed m/s
A / D   左移 / 右移      side_speed    m/s
R / F   上升 / 下降      vert_speed    m/s
Q / E   右转 / 左转      yaw_rate      rad/s
Ctrl-C             退出
```

松开按键后保持 `hold_timeout`（默认 0.3s），超时自动发零速悬停。

## 运行

```bash
# 终端 1：起 bridge（Noetic 那边）
ros2 launch carlair_ros_bridge main.launch.py

# 终端 2：起本节点
ros2 launch uav_keyboard_control keyboard.launch.py
# 或者
ros2 run uav_keyboard_control keyboard
```

注意：节点直接读 /dev/tty 读键，不依赖 stdin，ros2 launch 也能交互。

## 参数

`config/keyboard.yaml`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `forward_speed` | 2.0 | W/S m/s |
| `side_speed` | 0.5 | A/D m/s |
| `vert_speed` | 1.0 | R/F m/s |
| `yaw_rate` | 0.4 | Q/E rad/s |
| `hold_timeout` | 0.3 | 松开后保持秒数 |
| `publish_hz` | 30.0 | 发布频率 |
| `cmd_vel_topic` | `/uav/cmd_vel` | 速度话题 |

## 文件

```
uav_keyboard_control/
  logic.py            纯逻辑（按键映射、KeyLatch 锁存），不依赖 ROS，可单测
  keyboard_node.py    rclpy 节点 + /dev/tty raw 读键
launch/keyboard.launch.py
config/keyboard.yaml
test/test_keyboard_local.py
```

## 测试

```bash
python3 test/test_keyboard_local.py   # 19 个用例，不依赖 ROS / 终端
```

CI 直接跑。

## 限制

- 只支持 Linux 终端（`/dev/tty` + `termios` + `tty.setraw`），Windows 跑不了——ROS 本来就在 Linux 上。
- 方向键是 `\x1b[A` 这类转义序列，按未知键处理（不刷新锁存），只认 WASD/RF/QE 字母键。
