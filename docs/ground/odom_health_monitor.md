# 里程计健康监视

## 问题

底盘跑起来之后，「发出去的指令」和「轮子实际走的速度」往往不一致：

- 打滑时实测速度明显大于期望速度，里程计被放大；
- 轮子卡住、驱动掉线时实测速度为 0，但指令还在发，底盘看起来「在动」；
- 电机接线反接或标定符号错时，实测方向与指令相反。

这些异常不会让节点报错，只会让定位和导航缓慢漂移，事后很难定位原因。本模块用一组速度比值判据，把「打滑 / 堵转 / 反向」直接从数据里识别出来。

## 与已有模块的区别

仓库里已有四个与里程计、打滑相关的模块，目标都不是「在线判断运动是否可信」：

| 已有模块 | 目标 | 与本模块的区别 |
|---|---|---|
| `mujoco_car_drive_fault` | 在 MuJoCo 中检测单侧驱动失效并做差速补偿 | 仿真内闭环补偿，需要模型与执行器；本模块只做判断，不做补偿 |
| `mujoco_diff_drive_slip_compensation` | 学习轮滑并补偿轨迹 | 依赖 MuJoCo 与训练，产出是补偿后的轨迹 |
| `encoder_quantization_velocity` | 编码器量化噪声与三种滤波器的估计质量对比 | 讨论速度估计的信号质量，离线实验，不比较指令与实测 |
| `odom_trace_recorder` | 记录 `/odom` 轨迹 | 只记录数据，不产生判断 |

本模块是一个纯 ROS 2 在线监视节点：不建模、不补偿、不依赖仿真，只回答「当前这段运动与指令是否一致」，输出 `ok` / `slip` / `stuck` / `reverse` 等状态供上层决策。

## 环境要求

- ROS 2 Humble
- 只用 `rclpy`、`geometry_msgs`、`nav_msgs`、`std_msgs`，**不需要 Gazebo 或任何仿真器**

## 文件结构

```
src/ground/odom_health_monitor/
├── resource/odom_health_monitor
├── odom_health_monitor/
│   ├── __init__.py
│   └── odom_health_monitor_node.py    # 监视节点
├── launch/
│   └── main.launch.py
├── test/
│   └── test_odom_health_monitor.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 话题与参数

| 方向 | 话题 | 类型 |
|---|---|---|
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` |
| 订阅 | `/odom` | `nav_msgs/Odometry` |
| 发布 | `/odom_health` | `std_msgs/String` |
| 发布 | `/slip_ratio` | `std_msgs/Float32` |

速度比较只用平面线速度（`linear.x`、`linear.y` 的模长），`/odom` 取 `twist.twist`。`/slip_ratio` 为实测与期望速度之比，无法判断时输出 `-1`。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `velocity_deadband` | 0.02 | 速度死区，m/s，低于此值不做判断 |
| `stuck_ratio` | 0.1 | 比值低于此值视为堵转候选 |
| `slip_ratio` | 1.5 | 比值高于此值视为打滑 |
| `stuck_duration` | 0.5 | 堵转需持续的时间，秒 |
| `cmd_timeout` | 0.5 | 指令超时时间，秒 |
| `odom_timeout` | 0.5 | 里程计超时时间，秒 |
| `publish_rate` | 10.0 | 输出频率，Hz |

`/odom_health` 取值：

| 状态 | 条件 |
|---|---|
| `ok` | 比值在 `[stuck_ratio, slip_ratio]` 内，或堵转尚未达到 `stuck_duration` |
| `slip` | 比值 > `slip_ratio` |
| `stuck` | 比值 < `stuck_ratio` 且持续超过 `stuck_duration` |
| `reverse` | 实测线速度与指令方向相反 |
| `idle` | 指令速度在死区内，不做判断 |
| `no_cmd` | 从未收到 `/cmd_vel` 或已超过 `cmd_timeout` |
| `no_odom` | 从未收到 `/odom` 或已超过 `odom_timeout` |

判定顺序固定为 `no_odom` → `no_cmd` → `idle` → `reverse` → `slip` → `stuck`（或 `ok`），数据缺失优先，方向异常先于幅值异常。

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select odom_health_monitor
source install/setup.bash
```

## 运行

```bash
ros2 launch odom_health_monitor main.launch.py
```

带参数启动，例如把打滑阈值放宽到 2.0：

```bash
ros2 launch odom_health_monitor main.launch.py slip_ratio:=2.0
```

## 演示步骤

打开三个终端，都先执行 `source ~/ros2_ws/install/setup.bash`。

**终端 1**：启动监视节点

```bash
ros2 launch odom_health_monitor main.launch.py
```

**终端 2**：持续发布速度指令（0.2 m/s）

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

**终端 3**：先按正常速度回放里程计，再分别构造三种异常

```bash
# 正常：实测 0.2 m/s，/odom_health 为 ok，/slip_ratio 约 1.0
ros2 topic pub -r 10 /odom nav_msgs/msg/Odometry "{twist: {twist: {linear: {x: 0.2}}}}"

# 打滑：实测 0.5 m/s，状态变为 slip，/slip_ratio 约 2.5
ros2 topic pub -r 10 /odom nav_msgs/msg/Odometry "{twist: {twist: {linear: {x: 0.5}}}}"

# 堵转：实测 0，0.5 秒后状态变为 stuck
ros2 topic pub -r 10 /odom nav_msgs/msg/Odometry "{twist: {twist: {linear: {x: 0.0}}}}"

# 反向：实测 -0.2 m/s，状态变为 reverse
ros2 topic pub -r 10 /odom nav_msgs/msg/Odometry "{twist: {twist: {linear: {x: -0.2}}}}"
```

每次按 `Ctrl+C` 结束上一条，再发下一条。指令停止后 0.5 秒，状态变为 `no_cmd`。

## 验收标准

| 检查项 | 预期 |
|---|---|
| 正常匹配 | 实测与指令一致，状态 `ok`，比值约 1.0 |
| 打滑 | 实测明显偏大，状态 `slip`，比值同步变化 |
| 堵转 | 实测为 0 且持续 `stuck_duration`，状态 `stuck` |
| 反向 | 实测方向与指令相反，状态 `reverse` |
| 指令过小 | 指令在死区内，状态 `idle` |
| 数据缺失 | 缺 `/odom` 或 `/cmd_vel` 超过超时时间，状态分别为 `no_odom`、`no_cmd` |
| 频率 | `/odom_health` 与 `/slip_ratio` 稳定在 `publish_rate` |
| 依赖 | 除 ROS 2 Humble 外无额外依赖，不需要仿真器 |

## 扩展方向

- 把 `/odom_health` 换成 `diagnostic_msgs/DiagnosticArray`，接入 `rqt_robot_monitor`；
- 检测到 `stuck` 时联动 `cmd_vel_guard` 的 `/emergency_stop`，直接切断运动；
- 用滑动窗口估计打滑比例，替代单帧比值，减少瞬时噪声误判。
