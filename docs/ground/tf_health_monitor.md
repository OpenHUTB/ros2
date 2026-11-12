# TF 链路健康监视

## 问题

TF 是整套导航栈里最不出声的一环。发布者掉线、时间源被重置、同一个 child 被两个
节点同时声明，都不会让任何节点报错，只会在下游表现为：

- `lookupTransform` 超时，`nav2` 打出 `Timed out waiting for transform`，但没人知道是哪条边；
- 话题还在发，但消息时间戳停在几秒前，tf2 丢掉数据并打 `TF_OLD_DATA`，补不上新的位姿；
- 同一个 `base_link` 被 `map` 和 `world` 同时挂上，TF 树变成两棵树，查询结果取决于时序。

这些异常的共同点是没有异常可捕获。本模块把每条 `(parent, child)` 边单独聚合，
给出「这条边当前是否可信」，并把异常边直接点名输出。

## 与已有模块的区别

仓库里已有的监视类模块，判断的都是运动与控制量，没有一处读取 TF：

| 已有模块 | 目标 | 与本模块的区别 |
|---|---|---|
| `odom_health_monitor` | 比较 `/cmd_vel` 与 `/odom` 的速度比值 | 判的是「实测运动是否跟得上指令」，输入是速度而非变换树 |
| `cmd_vel_guard` | 对 `/cmd_vel` 限幅、急停、超时拦截 | 判的是指令是否合法，不涉及坐标系 |
| `turtlebot3_relocalization` | AMCL 式粒子滤波重定位 | 产出位姿估计，不评估 TF 链路 |
| `odom_trace_recorder` | 记录 `/odom` 轨迹 | 只记录数据，不产生判断 |

全仓库此前没有 `tf2_msgs` 的使用记录。本模块是纯 ROS 2 在线监视：不发布变换、
不修改 TF、不依赖仿真器，只回答「哪条边断了、哪条边的时间戳不对」，
输出 `ok` / `stale` / `backwards` / `old_data` / `multi_parent` / `low_rate` 等状态。

## 环境要求

- ROS 2 Humble
- 只用 `rclpy`、`tf2_msgs`、`geometry_msgs`、`std_msgs`，**不需要 Gazebo 或任何仿真器**
- 判据部分（`core.py`）不导入 rclpy，可在无 ROS 环境用 pytest 回归

## 文件结构

```
src/ground/tf_health_monitor/
├── resource/tf_health_monitor
├── tf_health_monitor/
│   ├── __init__.py
│   ├── core.py                      # 判据，不依赖 ROS
│   ├── tf_health_monitor_node.py    # 监视节点
│   └── tf_demo_publisher.py         # 演示数据源
├── launch/
│   └── main.launch.py
├── test/
│   └── test_tf_health_monitor.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 话题与参数

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/tf` | `tf2_msgs/TFMessage` | `best_effort`，兼容 reliable 与 best_effort 两类发布者 |
| 订阅 | `/tf_static` | `tf2_msgs/TFMessage` | `transient_local`，晚加入也能补到历史 |
| 发布 | `/tf_health` | `std_msgs/String` | 总体状态 |
| 发布 | `/tf_health_edges` | `std_msgs/String` | 全部边：`parent->child:status:rate`，分号分隔 |
| 发布 | `/tf_health_problem` | `std_msgs/String` | 异常边：`parent->child(status)`，逗号分隔 |

| 参数 | 默认值 | 单位 | 说明 |
|---|---|---|---|
| `stale_timeout` | 0.5 | s | 超过此时长没有新观测判为 `stale` |
| `rate_window` | 2.0 | s | 频率统计窗口 |
| `min_rate` | 1.0 | Hz | 窗口内频率低于此值判为 `low_rate` |
| `min_samples` | 5 | 条 | 窗口内样本少于此数不下结论，输出 `unknown` |
| `backwards_hold` | 2.0 | s | 时间戳回退事件的保持时间，便于在回放中观察 |
| `stamp_lag` | 1.0 | s | 时间戳落后到达时刻超过此值判为 `old_data` |
| `publish_rate` | 5.0 | Hz | 输出频率 |
| `watched` | 空 | — | 逗号分隔的 `parent->child`；为空表示动态发现所有边 |

`/tf_health` 取值：

| 状态 | 条件 |
|---|---|
| `ok` | 新鲜、频率达标，且未发现结构性错误 |
| `stale` | 超过 `stale_timeout` 没有新观测 |
| `backwards` | 时间戳相对上一次不增反减，且发生在 `backwards_hold` 之内 |
| `old_data` | 消息在到达，但时间戳落后到达时刻超过 `stamp_lag`，对应 tf2 的 `TF_OLD_DATA` |
| `multi_parent` | 同一个 child 在 `stale_timeout` 内有两个及以上存活 parent |
| `low_rate` | 样本已达 `min_samples`，但窗口内频率低于 `min_rate` |
| `unknown` | 样本数量不足，暂不下结论 |
| `missing` | `watched` 中声明、但从未收到过 |
| `no_input` | 从未收到 `/tf` 或 `/tf_static` |

总体状态取最严重的一条，严重程度为
`missing` > `multi_parent` > `backwards` > `old_data` > `stale` > `low_rate` > `unknown` > `ok`。
单条边的判定顺序为：多 parent → 时间戳回退 → 断链 → 消息陈旧 → 频率 → 样本数量。

三条实现细节值得说明：

1. **静态变换不判时效。** `/tf_static` 上的边只发布一次，用新鲜度或频率去判它必然误报，
   因此只保留多 parent 与时间戳回退两项检查。
2. **时间戳为 0 视为未填写。** 用 `ros2 topic pub` 手工构造的消息通常没有 `header.stamp`，
   此时不参与 `old_data` 判定，避免把手搓数据的习惯判成故障。
3. **帧名去掉一个前导斜杠。** `/base_link` 与 `base_link` 在 tf2 里是两条不同的边，
   命名写法不一致会看起来像断链，这里统一归一化后再聚合。

频率的算法是窗口内平均到达频率：

\[
f=\frac{N-1}{t_{N}-t_{1}}
\]

其中 \(N\) 为窗口内样本数，\(t_1\)、\(t_N\) 为首末样本的到达时刻。陈旧程度用时间戳与到达时刻之差：

\[
\Delta = t_{\mathrm{now}} - t_{\mathrm{stamp}}
\]

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select tf_health_monitor
source install/setup.bash
```

## 运行

```bash
ros2 launch tf_health_monitor main.launch.py
```

带参数启动，例如只看两条指定边、把频率门限放宽到 0.5 Hz：

```bash
ros2 launch tf_health_monitor main.launch.py \
  min_rate:=0.5 watched:='map->odom,odom->base_footprint'
```

## 演示步骤

打开三个终端，都先执行 `source ~/ros2_ws/install/setup.bash`。

**终端 1**：启动监视节点

```bash
ros2 launch tf_health_monitor main.launch.py
```

**终端 2**：启动演示数据源，它按 8 s / 18 s / 26 s 三个节点依次制造异常

```bash
ros2 run tf_health_monitor tf_demo_publisher
```

**终端 3**：观察总体状态变化

```bash
ros2 topic echo /tf_health
```

观察 `/tf_health_edges` 可以看到每条边的状态与频率：

```bash
ros2 topic echo /tf_health_edges
```

演示数据源的时间表：

| 时段 | 演示数据源的行为 | 预期 `/tf_health` |
|---|---|---|
| 0–8 s | 正常发布 `map->odom`、`odom->base_footprint` | `ok`（启动瞬间为 `unknown`） |
| 8–18 s | 停发 `odom->base_footprint` | `stale` |
| 18–26 s | 恢复发布，但时间戳停在 8 s 前 | `old_data` |
| 26 s 以后 | 再挂一个 `world->base_footprint` | `multi_parent` |

`base_link->laser` 走 `/tf_static`，全程不应被判成 `stale`——这是静态边处理的验证点。

## 离线回放结果

在没有 ROS 的机器上，用 `core.TfHealthTracker` 回放 40 s、20 Hz 的合成观测流
（与演示数据源同一套时间表），得到的状态迁移如下：

| 时刻 (s) | 总体状态 | 异常边 |
|---|---|---|
| 0.00 | `unknown` | — |
| 0.20 | `ok` | — |
| 8.45 | `stale` | `odom->base_footprint(stale)` |
| 18.00 | `old_data` | `odom->base_footprint(old_data)` |
| 26.00 | `multi_parent` | `odom->base_footprint(multi_parent),world->base_footprint(multi_parent)` |

回放共 1681 条观测、4 条边、5 次状态迁移，与上表时间表的每个切换点一致
（`8.45` = 停发时刻 8 s + `stale_timeout` 0.5 s 内的一次采样周期）。

## 验收标准

| 检查项 | 预期 |
|---|---|
| 节点启动 | 打印 `TF 健康监视已启动`，`/tf_health` 在 `publish_rate` 上稳定发布 |
| 无数据 | 未启动数据源时状态为 `no_input`，并给出提示 |
| 正常时段 | 两条动态边为 `ok`，`/tf_health_edges` 中频率约 20 Hz |
| 断链 | 停发后约 0.5 s 内变为 `stale`，异常边点名 `odom->base_footprint` |
| 消息陈旧 | 恢复发布但时间戳滞后时变为 `old_data`，不误报 `backwards` |
| 多 parent | 出现第二个 parent 后两条边同时变为 `multi_parent` |
| 静态边 | `base_link->laser` 全程为 `ok`，不因只发一次被判 `stale` |
| 依赖 | 除 ROS 2 Humble 外无额外依赖，不需要仿真器 |

## 已知限制

- 只判断边的健康度，不提供 `lookupTransform` 能力，不替代 `tf2_ros` 的 `Buffer`；
- 频率用「消息到达时刻」统计，回调队列积压或网络抖动会直接影响读数；
- `old_data` 依赖收发两端使用同一时钟源；未启用 `use_sim_time` 时需要 NTP 对齐，
  否则时钟偏差会被当成消息陈旧；
- `multi_parent` 用「窗口内多个存活 parent」近似，无法区分真正的树冲突与
  发布者重启时的短暂重叠；
- 演示数据源 `tf_demo_publisher` 的提交机器无 ROS 环境，尚未实跑，
  逻辑与上面的离线回放同构，需在 ROS 机器上先验证再合并。

## 扩展方向

- 把 `/tf_health` 换成 `diagnostic_msgs/DiagnosticArray`，接入 `rqt_robot_monitor`；
- 检测到 `stale` 时联动 `cmd_vel_guard` 的急停接口，直接切断底盘运动；
- 用 `tf2_ros.Buffer` 实测一次查询的耗时与成功率，把「链路健康」升级成「查询健康」。
