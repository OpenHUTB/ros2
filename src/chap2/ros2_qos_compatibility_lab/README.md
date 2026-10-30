# ROS 2 QoS 兼容性检查实验

`ros2_qos_compatibility_lab` 是一个用于排查 ROS 2 发布者、订阅者已经同时出现，
但订阅端收不到消息的新增教学模块。它检查 DDS offered/requested 模型中的两个核心
策略：

| 策略 | 发布者能力 | 订阅者要求 | 是否兼容 |
|---|---|---|---|
| Reliability | `RELIABLE` | `RELIABLE` | 是 |
| Reliability | `RELIABLE` | `BEST_EFFORT` | 是 |
| Reliability | `BEST_EFFORT` | `BEST_EFFORT` | 是 |
| Reliability | `BEST_EFFORT` | `RELIABLE` | 否 |
| Durability | `TRANSIENT_LOCAL` | `TRANSIENT_LOCAL` | 是 |
| Durability | `TRANSIENT_LOCAL` | `VOLATILE` | 是 |
| Durability | `VOLATILE` | `VOLATILE` | 是 |
| Durability | `VOLATILE` | `TRANSIENT_LOCAL` | 否 |

模块包含：

* 纯 Python 兼容性判断核心，不依赖 ROS 2 即可进行单元测试；
* 在线检查器，自动发现同一话题上的发布者和订阅者并打印诊断；
* 可配置 QoS 的发布者、订阅者演示节点；
* 默认复现 `BEST_EFFORT -> RELIABLE` 不兼容场景的 launch；
* 检查现有 ROS 2 话题的独立 launch。

## 1. 验证环境

本次提交在当前 Windows 11 工作区完成了纯 Python 测试：

| 项目 | 版本 |
|---|---|
| 操作系统 | Windows 11 |
| Python | 3.10.11 |
| 测试框架 | Python 标准库 `unittest` |
| 离线测试结果 | 8 项通过 |

ROS 2 节点和 launch 的目标运行环境为：

| 项目 | 版本或说明 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble |
| Python | 3.10（同时避免使用 Python 3.9 以上专有类型语法） |

当前工作区没有安装 ROS 2，因此本 README 不把离线测试结果当作 ROS 2 在线通信验证。
提交 PR 前应在上述目标环境中运行启动和话题通信检查，并补充真实运行截图。

## 2. 构建

把模块放入工作空间：

```bash
mkdir -p ~/ros2_ws/src
cp -r <仓库路径>/src/chap2/ros2_qos_compatibility_lab ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_qos_compatibility_lab
source install/setup.bash
```

## 3. 复现不兼容场景

默认参数启动一个 `BEST_EFFORT` 发布者、一个 `RELIABLE` 订阅者和检查器：

```bash
ros2 launch ros2_qos_compatibility_lab qos_demo.launch.py
```

预期行为：

* 发布者每秒发布一条 `QoS lab message N`；
* 订阅者不打印 `Received`，因为 QoS 端点不兼容；
* 检查器打印类似下面的诊断：

```text
[WARN] [qos_compatibility_inspector]: QoS INCOMPATIBLE | /qos_lab/chatter | publisher=/qos_demo_publisher subscriber=/qos_demo_subscriber reliability=best_effort->reliable durability=volatile->volatile
[WARN] [qos_compatibility_inspector]:   reliability: publisher offers best_effort, subscriber requests reliable. Use a RELIABLE publisher or a BEST_EFFORT subscriber.
```

另开一个终端确认两个端点都存在：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic info /qos_lab/chatter --verbose
```

## 4. 验证通信恢复

停止默认演示，把发布者和订阅者都设为 `RELIABLE`：

```bash
ros2 launch ros2_qos_compatibility_lab qos_demo.launch.py \
  publisher_reliability:=reliable \
  subscriber_reliability:=reliable
```

预期行为：

* 检查器打印 `QoS COMPATIBLE`；
* 订阅者持续打印 `Received: QoS lab message N`；
* `ros2 topic hz /qos_lab/chatter` 的频率约为 1 Hz。

也可以单独验证 Durability 不兼容：

```bash
ros2 launch ros2_qos_compatibility_lab qos_demo.launch.py \
  publisher_reliability:=reliable \
  publisher_durability:=volatile \
  subscriber_reliability:=reliable \
  subscriber_durability:=transient_local
```

## 5. 检查已有话题

检查所有已发现的发布者/订阅者组合：

```bash
ros2 launch ros2_qos_compatibility_lab qos_inspector.launch.py
```

只检查名称中包含 `/scan` 的话题：

```bash
ros2 launch ros2_qos_compatibility_lab qos_inspector.launch.py \
  topic_filter:=/scan
```

也可以直接运行检查器：

```bash
ros2 run ros2_qos_compatibility_lab qos_inspector --ros-args \
  -p scan_period_sec:=1.0 \
  -p topic_filter:=/camera
```

`include_compatible` 参数默认为 `true`。设为 `false` 时只报告不兼容组合：

```bash
ros2 run ros2_qos_compatibility_lab qos_inspector --ros-args \
  -p include_compatible:=false
```

## 6. 直接运行 main.py

从源码目录运行：

```bash
cd ~/ros2_ws/src/ros2_qos_compatibility_lab
python3 main.py --ros-args -p scan_period_sec:=1.0
```

使用 `Ctrl+C` 停止。

## 7. 测试

纯 Python 单元测试不需要 ROS 2：

```bash
cd src/chap2/ros2_qos_compatibility_lab
python3 -m unittest discover -s test -v
```

在 ROS 2 工作空间中执行包测试：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select ros2_qos_compatibility_lab
colcon test-result --verbose
```

## 8. 范围与限制

本模块只判断对端点匹配影响最直接的 Reliability 和 Durability。History、Depth、
Deadline、Lifespan 和 Liveliness 不在当前检查范围内。特别是 History 与 Depth
不同并不会单独阻止 ROS 2 端点建立匹配，因此不能把深度不同直接判定为不兼容。

在线检查器通过 `rclpy` 的 ROS graph API 读取端点 QoS。它只能检查当前 DDS domain
中已经发现的端点；`ROS_DOMAIN_ID`、网络隔离或 DDS discovery 配置不同时，节点本身
就不会互相发现。

## 9. 大模型使用声明

本模块在 Codex（基于 GPT-5）辅助下完成，提交者需对代码、文档、测试和最终 PR
内容负责。
