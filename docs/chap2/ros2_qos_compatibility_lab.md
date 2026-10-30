# ROS 2 QoS 兼容性检查实验

本页介绍新增模块
[ros2_qos_compatibility_lab](https://github.com/OpenHUTB/ros2/tree/master/src/chap2/ros2_qos_compatibility_lab)。
当 ROS 2 发布者和订阅者都存在，但订阅端始终没有数据时，Reliability 或 Durability
的 offered/requested 策略不一致是常见原因之一。

## 1. 实验目标

完成本实验后，可以：

1. 解释 ROS 2 发布者提供（offered）的 QoS 和订阅者请求（requested）的 QoS；
2. 判断 Reliability 与 Durability 的基础兼容矩阵；
3. 使用检查器定位现有话题上的不兼容端点；
4. 复现 `BEST_EFFORT` 发布者无法匹配 `RELIABLE` 订阅者的现象；
5. 修改 QoS 参数后确认通信恢复。

## 2. 兼容规则

| 发布者提供 | 订阅者请求 | 结果 |
|---|---|---|
| `RELIABLE` | `RELIABLE` | 兼容 |
| `RELIABLE` | `BEST_EFFORT` | 兼容 |
| `BEST_EFFORT` | `BEST_EFFORT` | 兼容 |
| `BEST_EFFORT` | `RELIABLE` | 不兼容 |
| `TRANSIENT_LOCAL` | `TRANSIENT_LOCAL` | 兼容 |
| `TRANSIENT_LOCAL` | `VOLATILE` | 兼容 |
| `VOLATILE` | `VOLATILE` | 兼容 |
| `VOLATILE` | `TRANSIENT_LOCAL` | 不兼容 |

可以把它理解为：发布者提供的能力必须覆盖订阅者请求的最低要求。
`RELIABLE` 比 `BEST_EFFORT` 提供更多可靠性保证，`TRANSIENT_LOCAL` 比 `VOLATILE`
提供更多历史数据保留能力。

History 和 Depth 不在这两张兼容表的判断范围内。二者不同通常不会单独阻止
ROS 2 发布者与订阅者建立匹配。

## 3. 构建

```bash
mkdir -p ~/ros2_ws/src
cp -r <仓库路径>/src/chap2/ros2_qos_compatibility_lab ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_qos_compatibility_lab
source install/setup.bash
```

执行 `ros2 pkg executables ros2_qos_compatibility_lab` 后应看到：

```text
ros2_qos_compatibility_lab demo_publisher
ros2_qos_compatibility_lab demo_subscriber
ros2_qos_compatibility_lab qos_inspector
```

## 4. 复现 QoS 不兼容

默认 launch 故意把发布者设置为 `BEST_EFFORT`，把订阅者设置为 `RELIABLE`：

```bash
ros2 launch ros2_qos_compatibility_lab qos_demo.launch.py
```

此时三个节点都会启动，但订阅者收不到应用消息。检查器输出类似：

```text
QoS INCOMPATIBLE | /qos_lab/chatter |
publisher=/qos_demo_publisher subscriber=/qos_demo_subscriber
reliability=best_effort->reliable durability=volatile->volatile
  reliability: publisher offers best_effort, subscriber requests reliable.
  Use a RELIABLE publisher or a BEST_EFFORT subscriber.
```

在另一个终端中检查端点：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic info /qos_lab/chatter --verbose
```

输出中应同时存在发布者和订阅者，但发布者的 Reliability 为 `BEST_EFFORT`，
订阅者为 `RELIABLE`。这说明“话题存在”和“端点成功匹配”不是同一件事。

## 5. 恢复通信

停止演示后，把两个 Reliability 都设置为 `RELIABLE`：

```bash
ros2 launch ros2_qos_compatibility_lab qos_demo.launch.py \
  publisher_reliability:=reliable \
  subscriber_reliability:=reliable
```

订阅端开始打印：

```text
[qos_demo_subscriber]: Received: QoS lab message 0
[qos_demo_subscriber]: Received: QoS lab message 1
```

检查器报告：

```text
QoS COMPATIBLE | /qos_lab/chatter | ...
```

还可以复现 Durability 不匹配：

```bash
ros2 launch ros2_qos_compatibility_lab qos_demo.launch.py \
  publisher_reliability:=reliable \
  publisher_durability:=volatile \
  subscriber_reliability:=reliable \
  subscriber_durability:=transient_local
```

## 6. 检查已有话题

启动独立检查器：

```bash
ros2 launch ros2_qos_compatibility_lab qos_inspector.launch.py
```

用 `topic_filter` 缩小话题范围：

```bash
ros2 launch ros2_qos_compatibility_lab qos_inspector.launch.py \
  topic_filter:=/scan
```

也可以直接运行：

```bash
ros2 run ros2_qos_compatibility_lab qos_inspector --ros-args \
  -p scan_period_sec:=1.0 \
  -p include_compatible:=false
```

检查器只输出匹配状态发生变化的新组合，不会每秒重复打印同一对端点。

## 7. 测试与预期结果

纯 Python 测试不依赖 ROS 2：

```bash
cd src/chap2/ros2_qos_compatibility_lab
python3 -m unittest discover -s test -v
```

预期共运行 8 项测试，覆盖 Reliability、Durability 的完整基础兼容矩阵、
双策略同时不兼容和未知策略错误。

ROS 2 工作空间测试：

```bash
cd ~/ros2_ws
colcon test --packages-select ros2_qos_compatibility_lab
colcon test-result --verbose
```

本页中的 ROS 2 日志是运行预期格式。当前代码快照在 Windows 工作区完成了离线测试，
ROS 2 在线启动应由提交者在 Ubuntu + Humble 环境复测后补充真实截图。

## 8. 常见问题

| 现象 | 可能原因 | 处理方法 |
|---|---|---|
| 检查器没有输出 | 话题没有同时存在发布者和订阅者 | 用 `ros2 topic info <topic> --verbose` 确认端点 |
| 两个端点存在但检查器未发现 | DDS domain 或网络隔离不同 | 统一 `ROS_DOMAIN_ID` 和网络配置 |
| 已改为兼容 QoS 但仍无消息 | 话题名、消息类型或命名空间不同 | 用 `ros2 topic list -t` 和 verbose info 检查 |
| 传感器话题在 RViz 中无数据 | 订阅端请求了 `RELIABLE`，驱动发布 `BEST_EFFORT` | 按设备驱动要求修改订阅 QoS |

## 9. 范围与声明

当前版本只检查 Reliability 和 Durability，不代替 `ros2 doctor`、DDS 厂商诊断工具
或完整的 QoS 配置审计。检查器只能看到当前 DDS domain 中已发现的端点。

本页和对应代码在 Codex（基于 GPT-5）辅助下完成，提交者需对最终内容负责。
