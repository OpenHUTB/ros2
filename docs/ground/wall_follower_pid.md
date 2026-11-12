# 差速小车右侧墙 PID 跟随与前向避障仿真

本模块给出一个最小可运行的 ROS 2 差速小车右侧墙跟随示例。控制器使用纯 Python 实现的 PID，不依赖 Gazebo / CARLA / MuJoCo，配套一个 2D 运动学积分的虚拟测距节点，因此在只有 rclpy 的环境里也能跑通闭环。

与仓库内已有地面载具模块的区别：

- `diff_drive_astar_odom_nav` 关注全局 A* 路径规划；本模块只做局部反应式墙跟随。
- `mujoco_car_avoidance` / `carla_aeb_braking` 依赖 MuJoCo / CARLA 物理仿真；本模块为零依赖教学示例。
- `vehicle_pygame_teleop` 是键盘遥控；本模块是自动闭环。

## 1. 控制律

右侧墙距离误差：

\\[
e = d_{\\text{target}} - d_{\\text{side}}
\\]

角速度由 PID 输出：

\\[
\\omega = K_p e + K_i \\int e\\,dt + K_d \\frac{de}{dt}
\\]

前进速度随角速度衰减：

\\[
v = v_{\\max} \\cdot \\max\\bigl(0.2,\\; 1 - |\\omega|/\\omega_{\\max}\\bigr)
\\]

状态机：

| 状态 | 触发条件 | 行为 |
|---|---|---|
| `FOLLOW` | 前向距离 ≥ `forward_safety` | 沿右侧墙定距前进 |
| `AVOID` | 前向距离 < `forward_safety` | 原地旋转，直到前向距离 ≥ `1.5 * forward_safety` |

## 2. ROS 2 接口

话题均为 `std_msgs/String` 承载 JSON，避免引入额外消息包：

| 话题 | 方向 | 说明 |
|---|---|---|
| `/wall_follower/scan` | 输入到控制器 | `{"side": 0.5, "forward": 2.0}` |
| `/wall_follower/cmd_vel` | 输出到仿真器 | `{"linear": 0.2, "angular": 0.05}` |
| `/wall_follower/state` | 输出 | `{"state": "FOLLOW", "linear": ..., "angular": ...}` |

## 3. 构建与运行

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ground_wall_follower_pid
source install/setup.bash
ros2 launch ground_wall_follower_pid main.launch.py
```

另开终端：

```bash
ros2 topic echo /wall_follower/state
ros2 topic echo /wall_follower/cmd_vel
```

按 `Ctrl-C` 结束。

## 4. 单元测试

控制器核心 `core.py` 不依赖 rclpy，可直接用系统 Python 测试：

```bash
python3 -m unittest discover -s tests -v
```

覆盖：比例项计算、积分复位、前向障碍进入 `AVOID`、离墙过近时转正向角速度、离墙过远时转负向角速度、缺测侧墙距离时保持设定点。

## 5. 验证边界

- `ScanSimNode` 是手写的 2D 差速运动学积分器，右侧墙固定在 `y=0`、前向走廊终点在 `x=10`，不代表真实激光雷达或超声。
- PID 参数为教学默认值，未针对具体底盘标定；实际部署应替换为真实传感器话题并重新调参。
- 仿真只覆盖平直墙 + 前方一堵墙的场景，未验证墙角、动态障碍和打滑。

## 6. 大模型使用声明

使用豆包 / Codex 辅助生成代码骨架、文档与测试用例；作者已逐项复核控制器逻辑、话题接口与测试输出，并对提交内容、结果真实性和后续维护负责。
