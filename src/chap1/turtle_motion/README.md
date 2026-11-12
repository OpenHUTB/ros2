# turtle_motion —— 小海龟画圆（ROS 2 版）

对应文档 [`docs/chapter/chap1/turtle_circle.md`](../../../docs/chapter/chap1/turtle_circle.md)：
向 `/turtle1/cmd_vel` 发速度指令、订阅 `/turtle1/pose` 累计转角，画满一圈（2π）后停止。

> **与文档的差异**：文档的实验环境是 Ubuntu 20.04 + ROS Noetic（`rospy`）。本仓库
> 的 CI 与现有 `src/chap1/service_demo` 都是 ROS 2，所以这里用 **rclpy** 实现同样的
> 话题与算法；消息类型、话题名、控制律都与文档一致。

## 算法

- 圆周运动：`omega = v / r`，半径由 `v / omega` 决定；
- 转角累计：对相邻位姿的 `theta` 取最短角差（`wrap_to_pi`）后**有符号**累加，
  再加 1e-3 rad 死区，静止时的浮点抖动不会被当成转动；
- 累计到 `2π` 后发一帧零速度并停止发布。

## 运行

```bash
colcon build --packages-select turtle_motion
source install/setup.bash
ros2 run turtle_motion main            # 或 ros2 launch turtle_motion turtle_circle.launch.py            # 默认 r=2.0 m, v=2.0 m/s
ros2 run turtle_motion main            # 或 ros2 launch turtle_motion turtle_circle.launch.py radius:=1.5 linear_speed:=1.0
```

无显示器的环境用 `QT_QPA_PLATFORM=offscreen`（turtlesim 仍会发布 `/turtle1/pose`）。

参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `radius` | 2.0 | 圆半径 [m] |
| `linear_speed` | 2.0 | 线速度 [m/s] |
| `turtle` | `turtle1` | 海龟名字 |
| `publish_hz` | 20.0 | 速度发布频率 [Hz] |

## 实测（WSL2 Ubuntu 24.04 + ROS 2 Jazzy，无显示器）

启动 launch 后用独立订阅者记录 `/turtle1/pose` 并拟合圆：

```
$ QT_QPA_PLATFORM=offscreen ros2 run turtle_motion main            # 或 ros2 launch turtle_motion turtle_circle.launch.py &
$ python3 measure_circle.py
samples: 730
center=(5.528, 7.544) radius=2.000 m turned=6.29 rad

[circle_draw-2] [INFO] [...] finished: turned 6.288 rad
```

半径 2.000 m（目标 2.0 m），累计转角 6.29 rad（目标 2π ≈ 6.283），画满一圈后自动停止。

## 测试

```bash
cd src/chap1/turtle_motion
python3 -m pytest -q test/          # 13 项，纯函数，不需要 ROS
```

覆盖：`omega = v/r`、半径反比、非法参数、角度归一化（±π、3π、2π+0.1）、
有符号累计、±π 跳变、死区抗噪、满圈判定、重置。

## 文件结构

```
turtle_motion/
├── turtle_motion/
│   ├── circle.py        # 纯函数：速度换算、wrap_to_pi、TurnAccumulator
│   └── circle_node.py   # rclpy 节点：发布 cmd_vel、订阅 pose、满圈停止
├── launch/turtle_circle.launch.py
├── test/test_circle.py
├── package.xml / setup.py / setup.cfg
└── README.md
```

## 已知限制

- 只在 Jazzy 的 turtlesim 上验证，没有在文档原环境（Noetic）跑过；
- 海龟撞墙时 turtlesim 会自行限位，本模块没有处理撞墙；
- 没有画多圈/变半径的能力（文档任务只要求一圈定半径）。
