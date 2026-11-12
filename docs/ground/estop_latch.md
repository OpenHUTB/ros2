# 急停闩锁

## 问题

底盘上通常有多个安全模块会向 `/emergency_stop` 发布 `true`：限幅模块发现非法指令、
里程计监视发现打滑、心跳看门狗发现上游失联。这些模块的做法都是「发现问题就置 `true`，
问题消失就置 `false`」。

这对真实车辆并不够：

- 某个安全模块因为丢了一帧数据而瞬间触发急停，下一帧又恢复正常，底盘在没有人工确认的
  情况下自己重新加速；
- 故障原因可能是接线松动、驱动过热这类需要人去看一眼的问题，自动恢复等于把事故留给下一次；
- 上电瞬间所有模块都还没起来，`/emergency_stop` 上没有任何消息，底盘默认是「可以动」的。

所以需要一个专门负责「锁住」的节点：**急停一旦触发就保持，直到人确认现场安全再放行**。

## 与已有模块的区别

| 已有模块 | 关注点 |
|---|---|
| `cmd_vel_guard` | 指令数值是否合法：限幅、超时、NaN 拦截 |
| `odom_health_monitor` | 指令与实测速度是否一致：打滑、堵转、反向 |
| `heartbeat_watchdog` | 产生指令的节点是否还活着 |
| 本模块 | 安全事件发生后**是否已经由人确认解除** |

前面三个模块都是「检测并发布急停」，本模块是「把急停锁住并管理放行流程」，仓库里没有
急停闩锁 / 人工复位相关的实现。

## 环境要求

- ROS 2 Humble
- 只用 `rclpy`、`std_msgs`、`std_srvs`，**不需要 Gazebo 或任何仿真器**

## 文件结构

```
src/ground/estop_latch/
├── resource/estop_latch
├── estop_latch/
│   ├── __init__.py
│   └── estop_latch_node.py    # 闩锁节点
├── launch/
│   └── main.launch.py
├── test/
│   └── test_estop_latch.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 话题、服务与参数

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/emergency_stop` | `std_msgs/Bool` | 各安全模块的急停输出 |
| 发布 | `/estop` | `std_msgs/Bool` | 闩锁后的急停，接 `cmd_vel_guard` 的急停输入 |
| 发布 | `/estop_status` | `std_msgs/String` | `clear` / `latched: <原因>` |
| 服务 | `/estop_latch/reset` | `std_srvs/Trigger` | 人工复位，成功时返回 `success: true` |

| 参数 | 默认值 | 说明 |
|---|---|---|
| `input_timeout` | 0.0 | 链路监视，默认关闭。收到过输入后超过这么久没再收到就认为安全链路失效并闩锁；此时复位会被拒绝，直到上游再发任意消息。上游是周期发布（如 `heartbeat_watchdog` 10 Hz）时可设为 1.0，事件驱动上游请保持 0 |
| `release_hold_time` | 2.0 | 急停信号消失后至少保持这么久才允许复位，防止抖动导致误放行 |
| `status_rate` | 1.0 | 状态与急停输出的发布频率，Hz |
| `auto_release` | false | 为 true 时满足复位条件后自动解除，仅供演示，实车应保持 false |
| `start_latched` | true | 启动即闩锁，需要人工复位后才能运动 |

`latched` 的原因分三种：

| 状态 | 含义 |
|---|---|
| `latched: start` | 启动时按 `start_latched` 默认闩锁 |
| `latched: input` | 收到过 `true` 的急停信号 |
| `latched: input_timeout` | 收到过输入后链路静默超过 `input_timeout`，按失效安全闩锁 |

话题名在代码中是相对名（`emergency_stop`、`estop`、`estop_status`），在默认根命名空间下
等价于上表的绝对路径；若用 `namespace` 启动，实际话题会带上前缀。服务相对名会带上节点名，
因此默认是 `/estop_latch/reset`。

`/estop` 使用 `TRANSIENT_LOCAL`（depth 1）发布，最后一条急停值会保留，下游重启后能立刻
拿到当前状态，不必等一个 `status_rate` 周期。下游用默认 `volatile` 订阅也能收到新消息，
只是重启后要先等到下一次发布；如果需要「一连上就拿到当前值」，订阅端把 durability 设成
`transient_local` 即可。

输入与输出刻意分成两个话题：如果把 `estop` 也发布到 `emergency_stop`，节点会收到自己的
输出，`last_active_time` 被不断刷新，人工复位将永远无法通过。

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select estop_latch
source install/setup.bash
```

## 运行

```bash
ros2 launch estop_latch main.launch.py
```

启动后立即进入 `latched: start`，`/estop` 为 `true`。复位：

```bash
ros2 service call /estop_latch/reset std_srvs/srv/Trigger
```

急停信号仍在触发、或消失不足 `release_hold_time`、或输入链路超时（仅限收到过输入之后、
且开启了 `input_timeout`），服务返回 `success: false` 并在 `message` 里说明原因。
节点当前本来就没闩锁时，复位返回 `success: false`、`message` 为「当前未闩锁，无需复位」，
不会打印「急停解除」日志，避免现场误判。上游如果是事件驱动（平时不发消息、急停才发
`true`），保持 `input_timeout` 默认的 0 即可，不会因为「一直没消息」而自锁。

## 演示步骤

打开三个终端，都先执行 `source ~/ros2_ws/install/setup.bash`。

**终端 1**：启动闩锁节点，保持时间设成 0，方便演示

```bash
ros2 launch estop_latch main.launch.py release_hold_time:=0.0
```

**终端 2**：先告诉节点「当前没有急停」，再复位

```bash
ros2 topic pub -1 /emergency_stop std_msgs/msg/Bool "{data: false}"
ros2 service call /estop_latch/reset std_srvs/srv/Trigger
```

复位后终端 1 打印 `急停解除，允许运动`，`/estop` 变为 `false`。

**终端 3**：观察状态

```bash
ros2 topic echo /estop_status
```

在终端 2 发一条 `true`：

```bash
ros2 topic pub -1 /emergency_stop std_msgs/msg/Bool "{data: true}"
```

状态变为 `latched: input`，`/estop` 回到 `true`。此时再发 `false` 并调用复位，若
`release_hold_time` 不为 0，服务会返回 `success: false`，提示「急停信号消失不足」。

## 与其它安全模块级联

当前仓库里 `/emergency_stop` 的发布者是 `heartbeat_watchdog`（上游失联时发 `true`），
订阅者是 `cmd_vel_guard`（收到 `true` 立即输出零速度）：

```
heartbeat_watchdog ──/emergency_stop──> estop_latch ──/estop──> cmd_vel_guard 急停输入
```

本节点插在两者之间，把瞬时急停锁成「必须人工确认才解除」，因此下游要把
`cmd_vel_guard` 的急停输入 remap 到 `/estop`：

```bash
ros2 run cmd_vel_guard cmd_vel_guard --ros-args -r emergency_stop:=estop
```

如果后续有更多安全模块发布 `/emergency_stop`，只要话题相同就会一起被闩锁，不需要改代码。

## 验收标准

| 检查项 | 预期 |
|---|---|
| 启动 | 状态 `latched: start`，`/estop` 为 `true` |
| 复位（条件满足） | 服务返回 `success: true`，状态 `clear`，`/estop` 为 `false` |
| 复位（急停仍触发） | 服务返回 `success: false`，提示「急停信号仍处于触发状态」 |
| 复位（从未收到输入） | 服务返回 `success: true`，允许人工确认后放行 |
| 复位（本就未闩锁） | 服务返回 `success: false`，提示「当前未闩锁，无需复位」，不打解除日志 |
| 收到 `true` 的响应 | 立即发布 `/estop`，不等 `status_rate` 定时器 |
| 闩锁后再触发 | 重新进入 `latched: input` |
| 链路监视（`input_timeout>0`） | 输入中断后进入 `latched: input_timeout`，复位被拒，直到上游再发消息 |
| `/estop` 保留最后值 | 下游以 `transient_local` 订阅时，一连接就能拿到当前急停值 |
| 参数非法 | `release_hold_time` / `input_timeout` 为负、`status_rate` 不为正、bool 参数类型错误时，以退出码 1 结束并打印「参数非法」 |

## 扩展方向

- 把 `/estop_status` 换成 `diagnostic_msgs/DiagnosticArray`，接入 `rqt_robot_monitor`；
- 增加复位原因与操作人记录，写入日志或 `rosbag`，便于事后追责；
- 把 `Trigger` 换成带 `std_srvs/SetBool` 或自定义服务，要求同时按下物理按钮才允许复位。
