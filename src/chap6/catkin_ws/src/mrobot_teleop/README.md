# mrobot_teleop

mrobot 键盘遥控节点。节点启动后读取键盘，把速度指令（`geometry_msgs/Twist`）发布到 `/cmd_vel`，驱动 Gazebo 中的差速小车，用于 SLAM 建图时手动遍历房间。

## 目录结构

```text
mrobot_teleop/
├── launch/
│   └── mrobot_teleop.launch   # 启动键盘节点，scale_linear=0.1，scale_angular=0.4
└── scripts/
    └── mrobot_teleop.py        # Python3 键盘脚本（已加可执行权限）
```

## 运行

```bash
roslaunch mrobot_teleop mrobot_teleop.launch
```

## 按键说明

| 按键 | 作用 |
|---|---|
| `i` | 前进 |
| `,` | 后退 |
| `j` / `l` | 原地左转 / 右转 |
| `u` / `o` | 前进并左转 / 右转 |
| `m` / `.` | 后退并左转 / 右转 |
| `k` 或空格 | 强制停止 |
| `q` / `z` | 同时增大 / 减小线速度和角速度 10% |
| `w` / `x` | 只增大 / 减小线速度 10% |
| `e` / `c` | 只增大 / 减小角速度 10% |
| `Ctrl+C` | 退出 |

## Noetic 注意事项

- 原始脚本是 Python2 代码（`#!/usr/bin/env python`、`print msg` 等），在 Noetic（Python3）下无法运行；本包已改为 Python3 语法并将 shebang 改为 `python3`。
- 脚本必须具备可执行权限，否则 `roslaunch` 会报 `can't locate node ... in package ...`；本包已 `chmod +x`。
- 建图时不要误触 `q/w/e` 加速键，速度过快会导致地图出现放射状噪点和重影。
