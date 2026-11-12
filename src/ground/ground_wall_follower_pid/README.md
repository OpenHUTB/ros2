# ground_wall_follower_pid

差速小车右侧墙 PID 跟随 + 前向避障的 ROS 2 仿真示例。

## 运行环境

- Ubuntu 22.04 / ROS 2 Humble（其他 rclpy 发行版理论上也可）
- Python 3.10+

## 构建

```bash
cd ~/ros2_ws
# 将本目录放到 src/ground/ground_wall_follower_pid
colcon build --packages-select ground_wall_follower_pid
source install/setup.bash
```

## 运行

```bash
ros2 launch ground_wall_follower_pid main.launch.py
```

另开终端查看话题：

```bash
ros2 topic echo /wall_follower/scan
ros2 topic echo /wall_follower/cmd_vel
ros2 topic echo /wall_follower/state
```

按 `Ctrl-C` 退出。

## 运行测试

```bash
python3 -m unittest discover -s tests -v
```

## 接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/wall_follower/scan` | `std_msgs/String` (JSON) | 输出 | `{"side": 0.5, "forward": 2.0}` |
| `/wall_follower/cmd_vel` | `std_msgs/String` (JSON) | 输出 | `{"linear": 0.2, "angular": 0.05}` |
| `/wall_follower/state` | `std_msgs/String` (JSON) | 输出 | `{"state": "FOLLOW", ...}` |

## 状态机

- `FOLLOW`：右侧墙距离误差经 PID 输出角速度，前进速度随角速度衰减。
- `AVOID`：前向距离小于 `forward_safety`（默认 0.4 m）时原地旋转，直到前向距离恢复到 `1.5 * forward_safety`。

## 边界与说明

- `ScanSimNode` 是一个极简的 2D 差速运动学积分器，右侧墙在 `y=0`、前向走廊终点在 `x=10`，仅用于闭环验证，不代表真实传感器。
- 控制器核心（`core.py`）不依赖 rclpy，可单独单元测试。
