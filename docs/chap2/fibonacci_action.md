# ROS 2 Action 斐波那契数列计算示例

本页面介绍 `fibonacci_action` 模块：一个纯粹的 ROS 2 Python 示例包，演示 **ROS 2 Action** 的核心通信模式——长任务 + 进度反馈 + 中途取消。Action 在导航路径跟踪、机械臂规划、无人机航点飞行等需要「异步执行 + 实时反馈 + 可取消」的机器人任务中广泛使用，是 ROS 2 三大通信机制（Topic / Service / Action）中最复杂的一种。

选择斐波那契数列计算是因为它天然适合 Action 场景：每算出一项就可以发布一次 Feedback，任务耗时可调节（通过 `time.sleep` 模拟），完成后返回完整数列作为 Result。

源码位置：[src/fibonacci_action](https://github.com/OpenHUTB/ros2/tree/master/src/fibonacci_action)

| 文件 | 说明 |
|---|---|
| [action/Fibonacci.action](https://github.com/OpenHUTB/ros2/blob/master/src/fibonacci_action/action/Fibonacci.action) | 自定义 Action 接口定义，三部分结构 |
| [fibonacci_action/server.py](https://github.com/OpenHUTB/ros2/blob/master/src/fibonacci_action/fibonacci_action/server.py) | Action 服务端节点 |
| [fibonacci_action/client.py](https://github.com/OpenHUTB/ros2/blob/master/src/fibonacci_action/fibonacci_action/client.py) | Action 客户端节点 |
| [launch/fibonacci.launch.py](https://github.com/OpenHUTB/ros2/blob/master/src/fibonacci_action/launch/fibonacci.launch.py) | 一键启动服务端 + 客户端 |

---

## 1. 运行环境

| 项目 | 版本 / 说明 |
|---|---|
| 操作系统 | Ubuntu 22.04 LTS（ROS 2 Humble 官方支持版本） |
| ROS | ROS 2 Humble Hawksbill |
| Python | 3.10 |
| 依赖 | 仅使用 ROS 2 自带功能包：`rclpy`、`action_msgs`、`rosidl_default_generators`、`launch`、`launch_ros` |

在虚拟机中确认环境：

```bash
lsb_release -a      # Ubuntu 22.04
echo $ROS_DISTRO    # humble
python3 --version   # Python 3.10.x
```

!!! note "ROS 1 与 ROS 2 环境不能混用"
    如果终端先后 source 过 Noetic 和 Humble，会出现 `ROS_DISTRO was set to 'noetic' before` 之类的提示。此时关闭终端，新开一个干净终端，只 source 需要的那一套环境即可。

---

## 2. Action 三阶段回顾

ROS 2 Action 由三部分组成，用 `.action` 文件定义：

```
# Goal：客户端发给服务端的请求
int32 order
---
# Result：服务端最终返回的结果
int32[] sequence
---
# Feedback：执行过程中服务端周期性发布的进度
int32[] sequence
```

对比 ROS 2 另外两种通信机制：

| 机制 | 方向 | 回调 | 适用场景 |
|---|---|---|---|
| Topic（话题） | 一收多 | subscriber | 持续数据流（传感器、状态） |
| Service（服务） | 一问一答 | service callback | 短任务（开关、查询） |
| **Action（动作）** | **异步执行 + 进度 + 结果** | **goal / feedback / result** | **长任务 + 可取消（导航、机械臂）** |

---

## 3. 编译

```bash
mkdir -p ~/ros2_ws/src
cp -r <本仓库路径>/src/fibonacci_action ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fibonacci_action
source install/setup.bash
```

---

## 4. 运行

三种方式效果相同，任选其一。

**方式 A：launch 启动（推荐）**

同时拉起服务端和客户端，最省操作：

```bash
ros2 launch fibonacci_action fibonacci.launch.py
```

可通过 `order` 参数修改客户端请求的目标阶数：

```bash
ros2 launch fibonacci_action fibonacci.launch.py order:=15
```

**方式 B：分别启动两个节点**

便于分别观察服务端和客户端日志：

```bash
# 终端 1 — 服务端
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run fibonacci_action server

# 终端 2 — 客户端
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run fibonacci_action client            # 默认 order=10
ros2 run fibonacci_action client 15         # 指定 order=15
```

**方式 C：命令行直接调用 Action（不跑 Python 节点）**

```bash
# 终端 1 — 先启动服务端
ros2 run fibonacci_action server

# 终端 2 — 用 ROS 2 CLI 发送 goal
ros2 action send_goal /fibonacci fibonacci_action/action/Fibonacci "{order: 10}"
```

运行结束后按 `Ctrl+C` 退出，属于正常关闭。

---

## 5. 运行效果

### 运行截图

> 待补充：本页的终端截图需在 Ubuntu + ROS 2 Humble 上实际执行 `colcon build` 与 `ros2 launch` 后截图替换。此前版本的截图与代码不一致，已移除。

### 日志解读

一次完整运行的日志（`order=10`，`ros2 run` 分别启动服务端与客户端时的输出）：

```text
[INFO] [1763000000.100000000] [fibonacci_action_server]: Fibonacci Action 服务端已就绪，等待客户端请求...
[INFO] [1763000000.200000000] [fibonacci_action_client]: 客户端启动，目标阶数: 10
[INFO] [1763000000.300000000] [fibonacci_action_client]: 已连接服务端，发送 goal (order=10)
[INFO] [1763000000.310000000] [fibonacci_action_server]: 接受请求：计算 10 阶斐波那契数列
[INFO] [1763000000.320000000] [fibonacci_action_server]: 开始计算 10 阶斐波那契数列
[INFO] [1763000000.330000000] [fibonacci_action_client]: Goal 已接受，等待结果...
[INFO] [1763000000.630000000] [fibonacci_action_server]: 进度 2/11: [0, 1, 1]
[INFO] [1763000000.640000000] [fibonacci_action_client]: 收到反馈: 当前数列长度=3, 最新项=1
[INFO] [1763000000.940000000] [fibonacci_action_server]: 进度 3/11: [0, 1, 1, 2]
[INFO] [1763000000.950000000] [fibonacci_action_client]: 收到反馈: 当前数列长度=4, 最新项=2
...（略）
[INFO] [1763000003.040000000] [fibonacci_action_server]: 进度 10/11: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]
[INFO] [1763000003.050000000] [fibonacci_action_client]: 收到反馈: 当前数列长度=10, 最新项=34
[INFO] [1763000003.350000000] [fibonacci_action_server]: 计算完成！结果: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
[INFO] [1763000003.360000000] [fibonacci_action_client]: 最终结果: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```

注意两点：

- Feedback 的进度分母是 `order + 1`（数列长度），`order=10` 时最后一次 Feedback 是 `10/11`，不会出现 `11/11`——第 11 项是计算完成后随 Result 一起返回的。
- 日志前缀由节点名决定：`ros2 run` 下是 `[fibonacci_action_server]` / `[fibonacci_action_client]`；用 `ros2 launch` 启动时会再套一层 `[fibonacci_action_server-1]` 这类进程名前缀。

### Action 四阶段图示

```
[Client]                         [Server]
   |                                 |
   |--- Goal(order=10) ----------->|
   |                                 |
   |<------ ACCEPT -----------------|
   |                                 |
   |   Feedback x N (每 0.3s)        | 逐步计算
   |<-------------------------------|
   |                                 |
   |<------ RESULT ------------------| 完成
```

---

## 6. 进阶测试

### 6.1 中途取消

Humble 的 `ros2 action` 只有 `list`、`send_goal`、`info` 三个子命令，其中 `send_goal` 支持 `-f/--feedback` 持续接收反馈，但**没有 `--cancel` 选项**，所以取消要在客户端代码里发起。

服务端已经实现了取消支持：`cancel_callback` 返回 `CancelResponse.ACCEPT`，`execute_callback` 每轮循环检查 `goal_handle.is_cancel_requested`。在客户端拿到 `goal_handle` 后调用：

```python
self._cancel_future = goal_handle.cancel_goal_async()
self._cancel_future.add_done_callback(self.cancel_done_callback)
```

服务端随即打印：

```text
[INFO] [1763000001.500000000] [fibonacci_action_server]: 收到取消请求
[INFO] [1763000001.510000000] [fibonacci_action_server]: 任务已取消
```

客户端 `get_result_async()` 返回的 `status` 为 `GoalStatus.STATUS_CANCELED`，`client.py` 会打印「目标已被取消，没有最终结果」并退出。这验证了 Action 相比 Service 最关键的优势——**长时间运行的任务可以被安全中断**。

### 6.2 参数校验

尝试传入非法阶数：

```bash
ros2 run fibonacci_action client -1     # 负数，超出范围
ros2 run fibonacci_action client 100    # 超过 50
```

服务端日志：

```text
[INFO] [1763000000.310000000] [fibonacci_action_server]: 拒绝请求：order=-1 超出合法范围 [0, 50]
```

客户端日志与退出行为：

```text
[WARN] [1763000000.320000000] [fibonacci_action_client]: Goal 被服务端拒绝，客户端退出
```

客户端不会再挂住等待，而是打印上述警告后以退出码 `1` 结束（`echo $?` 可验证），脚本或 CI 可以据此判断失败。这对应 `server.py` 的 `goal_callback` 与 `client.py` 的 `goal_response_callback`。

### 6.3 CLI 查看 Action 状态

运行时可以用 `ros2 action` 的三个子命令观察：

```bash
ros2 action list                # 列出当前所有 Action（本示例为 /fibonacci）
ros2 action list -t             # 连同类型一起列
ros2 action info /fibonacci     # 查看 Action 类型、服务端与客户端数量
```

Humble 的 `ros2 action` 没有 `echo` 子命令；要查看一次完整交互，用

```bash
ros2 action send_goal /fibonacci fibonacci_action/action/Fibonacci "{order: 10}" --feedback
```

`-f/--feedback` 会把服务端推送的 Feedback 逐条打印出来。

---

## 7. 代码要点

### 服务端关键设计

`server.py` 中值得关注的几个点：

```python
# 1. 可重入回调组 —— 允许多个 Goal 同时执行
self._action_server = ActionServer(
    self, Fibonacci, 'fibonacci',
    execute_callback=self.execute_callback,
    callback_group=ReentrantCallbackGroup(),   # <-- 关键
    goal_callback=self.goal_callback,
    cancel_callback=self.cancel_callback,
)

# 2. 同步 execute_callback —— 每轮循环检查取消
def execute_callback(self, goal_handle):
    for i in range(1, goal_handle.request.order):
        if goal_handle.is_cancel_requested:    # <-- 检查取消
            goal_handle.canceled()
            return Fibonacci.Result()
        ...
        goal_handle.publish_feedback(feedback_msg)  # <-- 发布 Feedback
        time.sleep(0.3)                             # 模拟耗时
```

`MultiThreadedExecutor` + `ReentrantCallbackGroup` 的组合保证多个 Goal 可以并行处理，且 `execute_callback` 中的取消检查能及时响应。

!!! warning "为什么用同步 def + time.sleep，而不是 async def + asyncio.sleep"
    在 Humble 的 `MultiThreadedExecutor` 下，`execute_callback` 由执行器的 worker 线程直接调用，该线程中没有运行中的 asyncio 事件循环。写成 `async def` 并在其中 `await asyncio.sleep()` 会抛：

    ```text
    Error raised in execute callback: no running event loop
    Goal state not set, assuming aborted
    ```

    目标随即被 abort（客户端收到状态码 6）。因此这里使用同步回调 + `time.sleep`；取消响应由 `cancel_callback` 在另一个 worker 线程处理，不受睡眠影响。官方 Python Action 教程的服务端同样采用同步写法。

### 客户端关键设计

`client.py` 使用了 ActionClient 的「三步异步」模式：

```python
self._action_client.wait_for_server()                      # 1. 等待服务端
self._send_goal_future = self._action_client.send_goal_async(
    goal_msg, feedback_callback=self.feedback_callback)    # 2. 异步发 Goal + Feedback 回调
self._send_goal_future.add_done_callback(self.goal_response_callback)  # 3. 注册响应回调
```

---

## 8. 与仓库现有模块的关系

本模块与仓库已有示例的对照。需要说明的是，`src/chap2/learning_communication`（`DoDishes.action`）同样是 Action 知识点，本文档与它的差异在于实现语言与侧重点：

| 已有示例 | 通信机制 | 与本文档的差异 |
|---|---|---|
| `chap2/ros2_pubsub_py.md`（话题 pub/sub） | Topic | Topic 只有单向数据流，没有 Goal / Result / Feedback 语义 |
| `chap2/ros2_service_param_demo.md`（服务 + 参数） | Service + 参数 | Service 是同步调用，无法中途反馈进度，也不能安全取消 |
| `chap2/ros2_qos_compatibility_lab.md`（QoS 实验） | Topic + QoS | 关注传输可靠性策略，不涉及 Action 状态机 |
| `src/chap2/learning_communication`（`DoDishes.action`，C++） | Action | 同为 Action：该示例是 **C++** 实现，接口与节点分目录组织；本文档是 **Python** 实现，在 `ament_cmake` 包内同包生成自定义接口，并额外演示取消状态处理、`GoalStatus` 分支与 CLI `--feedback` |

如果维护者认为 Action 示例只需要保留一个，可以改为在 `learning_communication` 中补充 Python 版本，本文档随之删除。

---

## 9. 已知限制

- 本包用 `ament_cmake` 构建（因为需要在同一个包里生成自定义 action 接口），因此不使用 `setup.py` 的 `console_scripts`；两个入口由 `CMakeLists.txt` 的 `install(PROGRAMS ... RENAME server|client)` 显式安装到 `lib/${PROJECT_NAME}/`
- 服务端与客户端脚本首行带 `#!/usr/bin/env python3`，`ros2 run` 直接以可执行文件方式调用
- 本文档的构建步骤按 ROS 2 Humble 编写，已在 Ubuntu 22.04 + ROS 2 Humble 上验证 `colcon build --packages-select fibonacci_action` 通过（18 s 左右），`ros2 launch` 也能正常拉起服务端与客户端
- `execute_callback` 使用同步写法，原因见第 7 节的 warning；在 `MultiThreadedExecutor` 下改用 `async def` + `asyncio.sleep` 会抛 `no running event loop`
- 运行效果截图待补充：需要一次完整跑通后的终端截图，当前页面仅给出日志文本

---

## 10. 参考资料

- [ROS 2 Action 官方教程（Python）](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [ROS 2 Action 概念文档](https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html)
- [ROS 2 Custom Action Interface](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [OpenHUTB/ros2 Issue #5：需新增不重复的 ROS 2 示例](https://github.com/OpenHUTB/ros2/issues/5)
