# diff_drive_corridor_centering

差速小车双侧墙通道居中 PD 巡航教学示例。控制器只依赖左侧墙、右侧墙和正前方
三路测距，用 PD 把车体保持在直通道中线上；配套一个 2D 运动学积分的虚拟测距
节点，不依赖 Gazebo / CARLA / MuJoCo，在只有 rclpy 的环境即可跑通闭环。

## 目录结构

```
diff_drive_corridor_centering/
├── diff_drive_corridor_centering/
│   ├── __init__.py
│   ├── centering.py      # 纯 Python 控制核心（不依赖 rclpy，可离线测试）
│   └── nodes.py          # ROS 2 虚拟测距节点 + 居中控制器节点
├── launch/
│   └── main.launch.py    # 一键启动两个节点
├── config/
│   └── params.yaml       # PD 增益、速度限幅与受阻阈值
├── resource/             # ament_python 资源标记
├── tests/
│   └── test_centering.py # 控制器核心离线单元测试
├── offline_sim.py        # 2D 离线仿真，生成轨迹/距离/速度三张效果图
├── package.xml
├── setup.py
└── setup.cfg
```

## ROS 2 构建与运行

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select diff_drive_corridor_centering
source install/setup.bash
ros2 launch diff_drive_corridor_centering main.launch.py
```

另开终端观测：

```bash
ros2 topic echo /corridor/state
ros2 topic echo /corridor/cmd_vel
```

## 离线仿真与单元测试

控制核心不依赖 ROS 2，可直接用系统 Python 运行：

```bash
# 在仓库根目录
python3 src/ground/diff_drive_corridor_centering/offline_sim.py
PYTHONPATH=src/ground/diff_drive_corridor_centering \
  python3 -m unittest discover -s src/ground/diff_drive_corridor_centering/tests -v
```

## ROS 2 接口

话题均为 `std_msgs/String` 承载 JSON，避免引入额外消息包：

| 话题 | 方向 | 内容 |
| --- | --- | --- |
| `/corridor/scan` | 仿真 -> 控制器 | `{"left": 1.0, "right": 1.0, "forward": 6.0}` |
| `/corridor/cmd_vel` | 控制器 -> 仿真器 | `{"linear": 0.3, "angular": 0.05}` |
| `/corridor/state` | 控制器 -> 观测 | `{"state": "CENTER", "linear": ..., "angular": ...}` |

## 控制律

- 横向偏差：`e = (d_left - d_right) / 2`，居中时 `e = 0`；
- 角速度：`ω = Kp·e + Kd·de/dt`，并限幅到 `w_max`；
- 线速度：`v = v_max · max(v_floor, 1 - |ω|/w_max)`，转角越大车速越低；
- 前方距离小于 `forward_safety` 进入 `BLOCKED` 零速停车，恢复到
  `forward_recover` 后继续巡航。

## 已知限制

- 离线仿真是理想直通道 + 简化运动学，不含轮子打滑、激光噪声和墙面凹凸；
- 本机没有 ROS 2 / Gazebo，ROS 端联调尚未在本机验证，需另一台机器复验。
