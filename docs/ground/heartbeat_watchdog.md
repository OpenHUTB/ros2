# 上游心跳看门狗

## 问题

底盘的速度指令通常由导航、遥控或算法节点产生。当这些节点崩溃、被 `Ctrl+C` 中断、或整个进程卡死时：

- 它们不再发布 `/cmd_vel`，而 `cmd_vel_guard` 只会把「没有新指令」处理成超时停车；
- 但如果上游是「进程还在、逻辑卡死」的状态，可能持续发送同一条速度；
- 更常见的是：底盘停在原地（指令为 0），上游其实已经死了，没人发现，直到下一次要动的时候才暴露。

这一类问题需要一层独立于指令内容的存活判断：**谁该发心跳、多久没发就算失联**。

## 与已有模块的区别

| 已有模块 | 关注点 |
|---|---|
| `cmd_vel_guard` | 指令的数值是否合法：限幅、急停、超时、NaN 拦截 |
| `odom_health_monitor` | 指令与实测速度是否一致：打滑、堵转、反向 |
| 本模块 | 产生指令的**节点是否还活着** |

`cmd_vel_guard` 的超时保护看的是「有没有新速度指令」，上游静止（一直发 0）时它永远认为是正常的；本模块看的是「心跳有没有按时到」，与指令内容无关。仓库目前没有其他心跳 / 看门狗类模块。

## 环境要求

- ROS 2 Humble
- 只用 `rclpy`、`std_msgs`，**不需要 Gazebo 或任何仿真器**

## 文件结构

```
src/ground/heartbeat_watchdog/
├── resource/heartbeat_watchdog
├── heartbeat_watchdog/
│   ├── __init__.py
│   └── heartbeat_watchdog_node.py    # 看门狗节点
├── launch/
│   └── main.launch.py
├── test/
│   └── test_heartbeat_watchdog.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 话题与参数

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/heartbeat` | `std_msgs/String` | `data` 填节点名，例如 `navigator` |
| 发布 | `/watchdog_status` | `std_msgs/String` | `waiting` / `ok` / `stale: <节点名>` |
| 发布 | `/emergency_stop` | `std_msgs/Bool` | 存在失联节点时为 `true` |

| 参数 | 默认值 | 说明 |
|---|---|---|
| `watched_nodes` | `navigator` | 要监视的节点名，多个用逗号分隔 |
| `heartbeat_timeout` | 1.0 | 心跳超时时间，秒 |
| `publish_rate` | 10.0 | 输出频率，Hz |
| `publish_estop` | true | 是否发布 `/emergency_stop` |

状态含义：

| 状态 | 条件 | 急停 |
|---|---|---|
| `waiting` | `watched_nodes` 中仍有节点**从未**上报过心跳 | `true`（失效安全） |
| `ok` | 名单内所有节点都在超时时间内上报过 | `false` |
| `stale: a, b` | 名单内节点都上报过，但至少一个已超时 | `true` |

只记录 `watched_nodes` 名单内的心跳，其他节点的心跳会被忽略，因此「有无关节点在发心跳、但我们要监视的节点还没起来」仍然停留在 `waiting`，不会被误判为 `stale`。

话题名在代码中是相对名（`heartbeat`、`watchdog_status`、`emergency_stop`），在默认根命名空间下等价于上表的绝对路径；若用 `namespace` 启动，实际话题会带上前缀。

启动阶段即输出 `waiting` + 急停，是为了避免「上游还没起来，底盘先动」；上游心跳到达后自动转为 `ok`。

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select heartbeat_watchdog
source install/setup.bash
```

## 运行

```bash
ros2 launch heartbeat_watchdog main.launch.py
```

同时监视两个节点、超时改成 0.5 秒：

```bash
ros2 launch heartbeat_watchdog main.launch.py \
    watched_nodes:=navigator,controller heartbeat_timeout:=0.5
```

## 演示步骤

打开三个终端，都先执行 `source ~/ros2_ws/install/setup.bash`。

**终端 1**：启动看门狗

```bash
ros2 launch heartbeat_watchdog main.launch.py heartbeat_timeout:=1.0
```

启动后先打印 `看门狗状态: waiting`，`/emergency_stop` 为 `true`。

**终端 2**：模拟上游节点持续发心跳（1 Hz）

```bash
ros2 topic pub -r 1 /heartbeat std_msgs/msg/String "{data: navigator}"
```

约 1 秒内状态变为 `ok`，`/emergency_stop` 变为 `false`。

**终端 3**：观察状态

```bash
ros2 topic echo /watchdog_status
```

在终端 2 按 `Ctrl+C` 停止心跳，1 秒后终端 1 打印 `看门狗状态: stale: navigator`，`/emergency_stop` 回到 `true`；重新启动终端 2 的心跳后恢复 `ok`。

## 与 cmd_vel_guard 级联

```
上游节点 --/cmd_vel--> cmd_vel_guard --/cmd_vel--> 底盘
     |                      ↑
     +--/heartbeat--> heartbeat_watchdog --/emergency_stop--+
```

两个 `/emergency_stop` 发布者（本节点、以及雷达等安全模块）可以同时接 `cmd_vel_guard` 的急停输入，任一为 `true` 即停车。

## 验收标准

| 检查项 | 预期 |
|---|---|
| 启动初期 | 状态 `waiting`，急停 `true` |
| 心跳正常 | 状态 `ok`，急停 `false` |
| 心跳中断 | 超过 `heartbeat_timeout` 后状态 `stale: <名称>`，急停 `true` |
| 心跳恢复 | 状态回到 `ok`，急停 `false` |
| 多节点 | 只报告失联的节点名，正常的节点不影响 |
| 空心跳 | `data` 为空或空白的消息被忽略 |
| 依赖 | 除 ROS 2 Humble 外无额外依赖，不需要仿真器 |

## 扩展方向

- 把 `/watchdog_status` 换成 `diagnostic_msgs/DiagnosticArray`，接入 `rqt_robot_monitor`；
- 增加 `stale_duration`：失联持续一段时间后才急停，避免偶发丢包误触发；
- 结合参数自动发现：用 `ros2 node list` 或生命周期状态代替手工名单。
