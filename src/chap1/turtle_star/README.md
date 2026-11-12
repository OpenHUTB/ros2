# turtle_star —— 小海龟画五角星（ROS 2 版）

对应文档 [`docs/chapter/chap1/turtle_star.md`](../../../docs/chapter/chap1/turtle_star.md)：
循环 5 次「直行 L → 原地左转 144°」，并在每条边开始前用 `/turtle1/set_pen` 换颜色。

> **与文档的差异**
> 1. 文档环境是 Ubuntu 20.04 + ROS Noetic（rospy），本仓库现有 `src/chap1/*` 与 CI 都是
>    ROS 2，因此用 **rclpy** 实现，话题/服务名与文档一致。
> 2. 文档把画圆与画星放在同一个包 `turtle_motion`；为了两个实验能分 PR 提交、互不冲突，
>    这里单独做成 `turtle_star` 包，算法与文档一致。

## 几何

- 五角星外角 `144° = 4π/5`，5 条边总转角 `5 × 144° = 720° = 4π`；
- 每次「直行 L + 左转 144°」后，海龟回到起点（闭合），朝向回到 0（mod 2π）；
- 直行时间 `L / v`，转角时间 `(4π/5) / ω`。

## 运行

```bash
colcon build --packages-select turtle_star
source install/setup.bash
ros2 run turtle_star main              # 或 ros2 launch turtle_star turtle_star.launch.py                     # L=1.0 m, v=1.0, ω=1.0
ros2 run turtle_star main              # 或 ros2 launch turtle_star turtle_star.launch.py side_length:=0.8 angular_speed:=1.5
```

无显示器时设 `QT_QPA_PLATFORM=offscreen`。

参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `side_length` | 1.0 | 边长 [m] |
| `linear_speed` | 1.0 | 直行速度 [m/s] |
| `angular_speed` | 1.0 | 原地转角速度 [rad/s] |
| `turtle` | `turtle1` | 海龟名字 |
| `tick` | 0.02 | 状态机节拍 [s] |

## 实测（WSL2 Ubuntu 24.04 + ROS 2 Jazzy，无显示器）

```
$ QT_QPA_PLATFORM=offscreen ros2 run turtle_star main              # 或 ros2 launch turtle_star turtle_star.launch.py &
$ python3 measure_star.py
samples: 1353
start=(5.544, 5.544) final=(5.539, 5.563) theta=0.010
closure_error=0.0194 m  turned=12.576 rad (4pi=12.566)

[star_draw-2] edge 1..5 颜色依次打印
[star_draw-2] finished: 5 edges, final theta=-0.02 rad
```

两次独立运行结果一致：终点 (5.551, 5.554) 对起点 (5.544, 5.544) 的闭合误差
**1.2 cm**，末帧航向 **-0.01 rad**（目标回到 0），5 条边颜色依次切换。

> 这里修过一次：最初用「定时开环」（按 `L/v`、`144°/ω` 计时），在无显示器、
> 位姿流被调度拖慢时五角星会越走越偏（同样代码有时 18 s 闭合、有时 40 s 还没走完）。
> 现在改成**绝对位姿反馈**：直行按「离本边起点的直线距离」判停，转向按
> 「目标航向 = 本边起始航向 + 144°」判停，容忍 0.01 rad；并在退出时补发一帧
> 零速度，避免节点被中断后 turtlesim 继续执行最后指令。

## 测试

```bash
cd src/chap1/turtle_star
python3 -m pytest -q test/        # 10 项，纯函数，不需要 ROS
```

覆盖：144° 与 4π、五边形闭合、顶点数、首边方向、各边等长、坐标有界、
直行/转角时间、非法参数、五色调色板。

## 文件结构

```
turtle_star/
├── turtle_star/
│   ├── star.py        # 纯函数：顶点、闭合误差、各阶段时长、调色板
│   └── star_node.py   # rclpy 节点：5×(直行→停顿→转 144°)，每条边前 set_pen
├── launch/turtle_star.launch.py
├── test/test_star.py
├── package.xml / setup.py / setup.cfg
└── README.md
```

## 已知限制

- 只在 Jazzy 的 turtlesim 上验证，没有在文档原环境（Noetic）跑过；
- 闭合误差 1.9 cm 来自「按时间开环走边 + 服务调用耗时」，不是闭环几何；
- 撞墙时 turtlesim 会限位，本模块没有处理撞墙。
