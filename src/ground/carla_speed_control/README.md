# carla_speed_control

按道路曲率自动限速，并把目标车速闭环跟踪住的纵向速度控制（ROS Noetic）。

地面载具已有的纵向模块都是**事件触发**的：ACC 神经网络跟车看前车距离、AEB 看障碍物
距离、悬崖制动看前方落差。它们回答「遇到某物该减速多少」，不回答「道路本身弯了该开
多快」。本模块补这一层：由路径曲率反解不使横向加速度超限的车速，再做预瞄制动与
速度跟踪，顺带把加速度和 jerk 限幅。

算法纯 numpy 实现，不依赖 OpenCV；限速、预瞄、PID 都是可直接单测的纯函数。

## 目录

| 文件 | 职责 |
|---|---|
| `main.py` | 包根入口，转调 `scripts/main.py` |
| `scripts/main.py` | 统一入口：`node` / `demo` / `check` 三种模式 |
| `scripts/speed_control.py` | 算法核心：限速剖面、可达性扫描、预瞄、PID |
| `scripts/speed_controller.py` | ROS 节点：订阅曲率与里程计，发布限速/加速度/速度指令 |
| `scripts/simple_plot.py` | 从零写的 PNG 生成器（numpy + zlib），避免依赖 matplotlib |
| `config/speed_params.yaml` | 全部可调参数 |
| `launch/main.launch` | 拉起控制节点 |
| `tests/` | 单元测试与离线演示 |

## 运行环境

| 项 | 版本 |
|---|---|
| 操作系统 | Ubuntu 20.04（虚拟机） |
| ROS | Noetic（ROS1） |
| Python | 3.8.10 |
| 运行依赖 | `numpy`；`rospy` / `std_msgs` / `geometry_msgs` / `nav_msgs` |
| 测试与演示依赖 | **只需 numpy**，不需要 ROS、CARLA、OpenCV、matplotlib |
| 车速来源 | `/carla/ego_vehicle/odom`（`nav_msgs/Odometry`） |

## 运行步骤

```bash
# 0. 放进工作空间并编译
cd ~/catkin_ws && cp -r <本模块> src/ && catkin_make && source devel/setup.bash

# 1. 不需要 ROS 就能验证（推荐先跑这个）
cd src/ground/carla_speed_control
python3 -m unittest discover -s tests -v     # 预期 Ran 90 tests ... OK
python3 tests/demo_speed_control.py          # 端到端仿真，输出量化对比与两张图
python3 main.py --mode check                 # 打印不同曲率下的限速表

# 2. 启动节点（定速巡航）
python3 main.py --mode node
# 或
roslaunch carla_speed_control main.launch target_speed:=12.0
```

与车道线检测联动时，把 `curvature_topic` 指向 `carla_lane_keeping` 发布的
`/carla/ego_vehicle/lane/curvature`，即形成「前视相机 → 车道线 → 曲率 → 限速」链路。

## 话题

订阅：

| 话题 | 类型 | 说明 |
|---|---|---|
| `speed/curvature` | `std_msgs/Float32` | 前方路径曲率（1/m），负值右转 |
| `speed/target` | `std_msgs/Float32` | 目标速度（m/s），可选 |
| `/carla/ego_vehicle/odom` | `nav_msgs/Odometry` | 当前车速 |

发布：

| 话题 | 类型 | 说明 |
|---|---|---|
| `speed/limit` | `std_msgs/Float32` | 当前限速（含曲率约束） |
| `speed/command` | `std_msgs/Float32` | 加速度指令（m/s²），负值减速 |
| `speed/control` | `geometry_msgs/Twist` | 纵向速度指令（`linear.x`） |
| `speed/diagnostics` | `std_msgs/Float32MultiArray` | 内部量，调试用 |

## 参数

见 `config/speed_params.yaml`。**换车后必须重新标定**载具能力那一组：

| 参数 | 默认 | 说明 |
|---|---|---|
| `target_speed` | 12.0 | 无曲率限制时的巡航速度（m/s） |
| `kp` / `ki` / `kd` | 1.0 / 0.15 / 0.35 | 速度 PID |
| `kff` | 1.0 | 加速度前馈增益 |
| `max_accel` / `max_decel` | 3.0 / 6.0 | 加减速度上限（m/s²） |
| `max_jerk` | 4.0 | 加速度变化率上限（m/s³） |
| `max_lateral_accel` | 3.0 | 横向加速度上限（m/s²），决定过弯车速 |
| `lookahead_time` | 1.5 | 前瞻时间（s） |
| `curvature_timeout` | 0.5 | 曲率话题超时后按直道处理（s） |

## 实测结果

S 形路径（直道 100 m → 半径 40 m 左弯 → 直道 → 右弯 → 直道，全程 400.3 m）：

| 指标 | 无曲率限速 | 有本模块 |
|---|---|---|
| 最大横向加速度 | 3.600 m/s²（超限 20%） | **2.977 m/s²** |
| 最大超速 | — | 0.000 m/s |
| 最大 \|jerk\| | — | 4.00 m/s³ |
| 稳态速度误差 | — | 0.0124 m/s |
| 平均速度 / 总时间 | 12.00 m/s / 33.3 s | 10.47 m/s / 38.25 s |

多花 4.9 s 换取弯道不失稳。完整曲线与推导见
`docs/ground/carla_speed_control/speed_control.md`。

## 测试

```bash
python3 -m unittest discover -s tests -v
```

90 个用例：`test_speed_control.py` 68 个（限速公式、前瞻取最大曲率、预瞄制动、
jerk 限制、PID、端到端仿真），`test_ros_node.py` 22 个（参数加载、曲率超时、
发布字段、安全层限幅，内置 rospy 与消息桩）。

## 已知限制

- `max_lateral_accel` 与 `max_accel`/`max_decel` 是**车辆相关**参数，换车必须重新标定，
  否则限速会偏乐观（打滑）或偏保守（白慢）。
- 曲率来自上游感知，感知中断超过 `curvature_timeout` 后按直道处理，此时会有短暂的
  限速偏高，靠兜底超速制动补偿。
- 曲率突变（如路口直角弯）时预瞄窗口内最大曲率可能高估限速需求，表现为速度短暂
  下降到低于必要值，属保守侧偏差。
