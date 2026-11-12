# ROS 2 Action 斐波那契数列计算示例

## 模块简介

本模块是一个纯粹的 ROS 2 Python 示例包，演示了 **ROS 2 Action** 的核心通信模式：

- **自定义 Action 接口**（.action 文件）
- **Action 服务端**：接收 Goal → 逐步执行 → 发布 Feedback → 返回 Result → 支持取消
- **Action 客户端**：发送 Goal → 接收 Feedback → 获取 Result
- **Launch 一键启动**：同时拉起服务端和客户端，无需手动多终端操作

选择斐波那契数列计算是因为它天然适合 Action 的"长任务+进度反馈"场景——每算出一项就可以发布一次 Feedback，任务可在中途取消，完成后返回完整数列作为 Result。

## 目录结构

```
src/fibonacci_action/
├── action/
│   └── Fibonacci.action      # 自定义 Action 接口定义
├── fibonacci_action/
│   ├── __init__.py
│   ├── server.py             # Action 服务端节点
│   └── client.py             # Action 客户端节点
├── launch/
│   └── fibonacci.launch.py   # 一键启动 launch 文件
├── resource/
│   └── fibonacci_action
├── CMakeLists.txt            # ament_cmake 构建（处理 action 生成）
├── package.xml               # 包描述
├── setup.cfg
└── setup.py                  # ament_python 构建
```

## 运行环境

| 项目 | 版本 |
|------|------|
| 操作系统 | Ubuntu 22.04 (Jammy Jellyfish) |
| ROS 2 发行版 | Humble Hawksbill |
| Python | 3.10 |
| 关键依赖 | rclpy, action_msgs, rosidl_default_generators |

> ROS 2 Humble 在 Ubuntu 22.04 上使用 apt 安装：`sudo apt install ros-humble-desktop ros-humble-rosidl-default-generators ros-humble-launch-ros`

## 构建与运行

以下命令均在工作区根目录（如 `~/ros2_ws`）执行。

### 1. 准备工作区并构建

```bash
# 先载入 ROS 2 环境（每次新终端都要执行）
source /opt/ros/humble/setup.bash

# 安装依赖（首次）
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 构建本包（--packages-select 只构建 fibonacci_action 及其依赖）
colcon build --packages-select fibonacci_action

# source 工作区环境
source install/setup.bash
```

### 2. 一键运行（推荐）

同时启动服务端和客户端：

```bash
ros2 launch fibonacci_action fibonacci.launch.py
```

可通过 `order` 参数修改客户端请求的目标阶数：

```bash
ros2 launch fibonacci_action fibonacci.launch.py order:=15
```

### 3. 分终端运行（便于观察日志）

终端 1 — 启动服务端：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run fibonacci_action server
```

终端 2 — 启动客户端：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run fibonacci_action client
# 或指定阶数
ros2 run fibonacci_action client 15
```

## 运行效果说明

### Action 四阶段

| 阶段 | 说明 | 日志特征 |
|------|------|----------|
| Goal 发送 | 客户端连接服务端并发送 order=N 的 Goal | 🚀 已连接服务端，发送 goal |
| Goal 响应 | 服务端校验 order ∈ [0,50] 后 ACCEPT | ✅ Goal 已接受 |
| 执行 + Feedback | 服务端逐步计算，每步发布 Feedback | 📊 进度 i/N: [0,1,1,2,...] |
| Result 返回 | 计算完成返回完整数列 | 🏁 最终结果: [0,1,1,2,3,5,8,...] |

### 取消测试

在客户端 Goal 被接受后、完成前，打开新终端执行：

```bash
ros2 action send_goal --cancel fibonacci
```

服务端日志会打印 `⛔ 任务已取消`，客户端 Result 为空。

### 参数校验

尝试传入非法阶数：

```bash
ros2 run fibonacci_action client -1     # 拒绝（负数）
ros2 run fibonacci_action client 100    # 拒绝（>50）
```

服务端日志会打印 `❌ 拒绝请求：order=N 超出合法范围 [0, 50]`，客户端 Goal 被 REJECT。

## 与仓库现有模块的关系

本模块位于 `src/fibonacci_action/` 和 `docs/fibonacci_action.md`，不与仓库已有模块重复：

| 已有 ROS2 示例 | 本模块 |
|---------------|--------|
| chap2/ros2_pubsub_py.md（话题 pub/sub） | Action（长任务 + 进度反馈 + 取消） |
| chap2/ros2_service_param_demo.md（服务 + 参数） | Action 类型通信 |

## 自定义 Action 接口

`action/Fibonacci.action` 定义了三部分：

```
int32 order              # Goal：请求的斐波那契阶数
---
int32[] sequence         # Result：完整数列
---
int32[] sequence         # Feedback：部分数列（实时进度）
```

ROS 2 构建时，CMakeLists.txt 中的 `rosidl_generate_interfaces` 会自动根据此文件生成 Python 消息类，生成路径形如 `install/fibonacci_action/lib/python3.10/site-packages/fibonacci_action/action/_fibonacci.py`。

## 已知限制

- 仅支持 ROS 2 Humble（Ubuntu 22.04），其他发行版需确认 Python 版本和消息生成器兼容性
- 斐波那契阶数限制在 [0, 50] 区间，主要考虑 Python `int32` 溢出（第 47 项约 29 亿，第 50 项约 125 亿会溢出）
- 代码中的 `time.sleep(0.3)` 用于演示反馈效果，实际应用应替换为真实计算耗时

## 参考资料

- [ROS 2 Action 官方教程（Python）](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [ROS 2 Action 概念文档](https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html)
- [ROS 2 Custom Action Interface](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
