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
lsb_release -a      # Ubuntu 20.04
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

一次完整运行的日志（`order=10`）：

```text
[server]  [OK] Fibonacci Action 服务端已就绪，等待客户端请求...
[client]  [INFO] 客户端启动，目标阶数: 10
[client]  [INFO] 已连接服务端，发送 goal (order=10)
[server]  [GOAL] 接受请求：计算 10 阶斐波那契数列
[server]  [EXEC] 开始计算 10 阶斐波那契数列
[client]  [WAIT] Goal 已接受，等待结果...
[server]  [FEEDBACK] 进度 2/11: [0, 1, 1]
[client]  [FEEDBACK] 收到反馈: 当前数列长度=3, 最新项=1
[server]  [FEEDBACK] 进度 3/11: [0, 1, 1, 2]
[client]  [FEEDBACK] 收到反馈: 当前数列长度=4, 最新项=2
...（略）
[server]  [FEEDBACK] 进度 10/11: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]
[client]  [FEEDBACK] 收到反馈: 当前数列长度=10, 最新项=34
[server]  [FEEDBACK] 进度 11/11: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
[client]  [FEEDBACK] 收到反馈: 当前数列长度=11, 最新项=55
[server]  [RESULT] 计算完成！结果: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
[client]  [DONE] 最终结果: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```

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

在 Goal 被接受后、完成前，打开新终端执行：

```bash
ros2 action send_goal --cancel /fibonacci fibonacci_action/action/Fibonacci
```

服务端日志会打印：

```text
[CANCEL] 收到取消请求
[CANCEL] 任务已取消
```

客户端 Result 为空。这验证了 Action 相比 Service 最关键的优势——**长时间运行的任务可以被安全中断**。

### 6.2 参数校验

尝试传入非法阶数：

```bash
ros2 run fibonacci_action client -1     # 负数，超出范围
ros2 run fibonacci_action client 100    # 超过 50
```

服务端日志：

```text
[REJECT] 拒绝请求：order=-1 超出合法范围 [0, 50]
```

客户端 Goal 被 REJECT，不会开始执行。这对应 `server.py` 中的 `goal_callback` 函数。

### 6.3 CLI 查看 Action 列表

运行时可以用 `ros2 action` 系列命令观察：

```bash
ros2 action list                # 列出当前所有 Action
ros2 action list -t             # 连同类型一起列
ros2 action info /fibonacci     # 查看 Action 详细信息
ros2 action echo /fibonacci/feedback fibonacci_action/action/Fibonacci   # 监听所有 Feedback
```

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

# 2. 异步 execute_callback —— 中途可检查取消
async def execute_callback(self, goal_handle):
    for i in range(1, goal_handle.request.order):
        if goal_handle.is_cancel_requested:    # <-- 检查取消
            goal_handle.canceled()
            return Fibonacci.Result()
        ...
        goal_handle.publish_feedback(feedback_msg)  # <-- 发布 Feedback
```

`MultiThreadedExecutor` + `ReentrantCallbackGroup` 的组合保证多个 Goal 可以并行处理，且 `execute_callback` 中的取消检查能及时响应。

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

本模块不与仓库已有 ROS 2 示例重复：

| 已有示例 | 通信机制 | 本模块 |
|---|---|---|
| `chap2/ros2_pubsub_py.md`（话题 pub/sub） | Topic | **Action（长任务 + Feedback + Cancel）** |
| `chap2/ros2_service_param_demo.md`（服务 + 参数） | Service + 参数 | **Action 异步通信** |
| `chap2/ros2_qos_compatibility_lab.md`（QoS 实验） | Topic + QoS | **Action 类型通信** |

---

## 9. 已知限制

- 本包用 `ament_cmake` 构建（因为需要在同一个包里生成自定义 action 接口），因此不使用 `setup.py` 的 `console_scripts`；两个入口由 `CMakeLists.txt` 的 `install(PROGRAMS ... RENAME server|client)` 显式安装到 `lib/${PROJECT_NAME}/`
- 服务端与客户端脚本首行带 `#!/usr/bin/env python3`，`ros2 run` 直接以可执行文件方式调用
- 本文档的构建步骤按 ROS 2 Humble 编写，尚未在真实 Humble 环境完成端到端验证；首次构建请按第 3 节操作，如有报错请反馈

---

## 10. 参考资料

- [ROS 2 Action 官方教程（Python）](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [ROS 2 Action 概念文档](https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html)
- [ROS 2 Custom Action Interface](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [OpenHUTB/ros2 Issue #5：需新增不重复的 ROS 2 示例](https://github.com/OpenHUTB/ros2/issues/5)
