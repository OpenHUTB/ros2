# ros2_pubsub_py

ROS 2 话题发布订阅（talker / listener）基础实验。本模块作为「基于强化学习的四旋翼无人机飞行控制」项目的**前置基础实验**，用于验证 ROS 2 节点之间的发布 / 订阅通信链路，为后续无人机传感器数据传输与强化学习控制指令下发做基础验证。

## 1. 实验目的

* 掌握 ROS 2 中发布者（publisher）与订阅者（subscriber）的创建方法；
* 验证两个节点通过 `chatter` 话题进行字符串消息收发的完整链路；
* 提供一条可复用的最小通信链路，作为无人机项目后续数据流的基础。

## 2. 运行环境

| 项目 | 版本 / 说明 |
|---|---|
| 操作系统 | Ubuntu 20.04.6 LTS（VMware 虚拟机），宿主为 Windows 11 |
| ROS | ROS 2 Humble |
| Python | 3.8.10（虚拟机内与 Humble 匹配的版本） |
| 依赖 | 仅使用 ROS 2 自带功能包：`rclpy`、`std_msgs`、`launch`、`launch_ros` |
| 硬件 | 无特殊要求，普通 PC 即可 |

虚拟机中确认环境：

```bash
lsb_release -a              # 应显示 Ubuntu 20.04
echo $ROS_DISTRO            # 应显示 humble
python3 --version           # 应显示 Python 3.8.x
```

> 注意：ROS 1（Noetic）与 ROS 2（Humble）的环境变量不能混用。若终端同时 source 过两套环境，请关闭终端后重新打开，只 source 需要的那一套。

## 3. 目录结构

```
ros2_pubsub_py/
├── main.py                          # 模块入口：python3 main.py 直接运行整个模块
├── launch/
│   └── pubsub_demo.launch.py        # ros2 launch 启动文件（节点与 main.py 一致）
├── ros2_pubsub_py/
│   ├── talker.py                    # 发布者节点
│   └── listener.py                  # 订阅者节点
├── resource/ros2_pubsub_py           # ament 资源索引标记文件
├── test/                            # ament_flake8 / ament_pep257 检查
├── package.xml
├── setup.py
└── setup.cfg
```

## 4. 运行步骤

### 4.1 编译

```bash
mkdir -p ~/ros2_ws/src
cp -r <本仓库路径>/src/chap2/ros2_pubsub_py ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_pubsub_py
source install/setup.bash
```

### 4.2 运行（三种方式任选其一，效果相同）

**方式 A：launch 启动（推荐，一条命令启动整个模块）**

```bash
ros2 launch ros2_pubsub_py pubsub_demo.launch.py
```

**方式 B：main 脚本入口**

```bash
python3 ~/ros2_ws/src/ros2_pubsub_py/main.py
```

**方式 C：分别启动两个节点（便于在两个终端中分别观察收发）**

```bash
# 终端 1
ros2 run ros2_pubsub_py talker
# 终端 2
ros2 run ros2_pubsub_py listener
```

### 4.3 观察与验证

* talker 终端持续输出 `发布: Hello ROS2, count=N`（10 Hz）；
* listener 终端持续输出 `收到消息: Hello ROS2, count=N`，序号与 talker 一致；
* 可在第三个终端检查话题：

```bash
ros2 topic list             # 应看到 /chatter
ros2 topic echo /chatter    # 打印话题内容
ros2 topic hz /chatter      # 应约为 10 Hz
```

### 4.4 结束运行

在运行终端按 `Ctrl+C`。此时可能看到 `KeyboardInterrupt` 相关的 Traceback，也可能只看到节点打印的 `talker 节点被手动终止` / `listener 节点被手动终止`，两者都是**正常的退出提示**，不是程序报错。退出时如果还跟着一行 `Failed to publish log message to rosout: publisher's context is invalid`，那是节点已销毁、rclpy 仍尝试向 `/rosout` 上报日志造成的，同样属于正常现象。

## 5. 运行效果

发布者节点（talker）持续发布消息：

![talker 发布消息](../../../docs/img/chap2/ros2_pubsub_py/talker_publish.gif)

订阅者节点（listener）稳定接收并打印消息：

![listener 接收消息](../../../docs/img/chap2/ros2_pubsub_py/listener_subscribe.gif)

按 `Ctrl+C` 结束程序：

![Ctrl+C 结束程序](../../../docs/img/chap2/ros2_pubsub_py/node_stop.png)

## 6. 常见问题

| 现象 | 原因 | 解决 |
|---|---|---|
| 终端提示 `ROS_DISTRO was set to 'noetic' before` | 同一终端里先后 source 了 Noetic 与 Humble 两套环境 | 关闭该终端，新开一个干净终端，只 `source /opt/ros/humble/setup.bash` |
| `ros2: command not found` | 未 source ROS 2 环境 | 执行 `source /opt/ros/humble/setup.bash`，或写入 `~/.bashrc` |
| listener 收不到消息 | 两个节点话题名或 ROS_DOMAIN_ID 不一致 | 用 `ros2 topic list` 确认两者都在 `/chatter` 上，且未设置不同的 `ROS_DOMAIN_ID` |
| 退出时出现 `Failed to publish log message to rosout: publisher's context is invalid` | 节点已销毁，rclpy 仍在向 `/rosout` 上报日志 | 属正常现象，可忽略（见 4.4 节） |

## 7. 说明

* `source` 命令只对当前终端生效，关闭终端后失效；
* 本模块为教学实验代码，可在不改变话题名 `chatter` 的前提下替换消息内容。

## 8. 测试

### 8.1 代码风格、docstring 与集成测试

```bash
cd ~/ros2_ws
source install/setup.bash
colcon test --packages-select ros2_pubsub_py
colcon test-result --verbose
```

`test/` 目录包含三项检查：

| 测试 | 内容 |
|---|---|
| `test_flake8.py` | 代码风格（PEP 8） |
| `test_pep257.py` | docstring 规范（第一行必须以英文句号 `.` 结尾） |
| `test_pubsub_integration.py` | **最小集成测试**：同时运行 talker 与 listener，断言 listener 至少收到一条 `/chatter` 消息 |

预期结果：全部通过且 `Summary ... 0 failures, 0 errors`。

### 8.2 人工稳定性测试

```bash
# 终端 1：启动整个模块
ros2 launch ros2_pubsub_py pubsub_demo.launch.py
# 终端 2：观察话题频率
ros2 topic hz /chatter
# 终端 3：观察消息内容与序号
ros2 topic echo /chatter
```

结果：连续运行 5 分钟以上，`/chatter` 频率稳定在约 10 Hz，listener 打印的序号与 talker 完全一致，无丢帧、无报错；`Ctrl+C` 退出后节点正常销毁。