# ros2_service_param_demo

ROS 2 **服务（service）与参数（parameter）** 基础实验，用一个「无人机模式节点」演示三件事：提供服务切换飞行模式、参数在线调整发布频率、周期发布状态话题。

本模块是「基于强化学习的四旋翼无人机飞行控制」项目的第二个前置基础实验：上一个实验（`ros2_pubsub_py`）验证了两个节点之间的单向消息流，本实验补齐真实无人机项目必须的两件基础设施 —— **模式切换**（起飞 / 悬停 / 降落，调用方需要拿到成功或失败）与 **在线调参**（不同飞行阶段要改发布频率或阈值，不能每次重启节点）。

> 与已有空域载具模块的区别：本模块只演示 ROS 2 服务与参数机制，**不连接仿真器、不发送任何飞控指令、不使用自定义消息**，与 `uav_keyboard_control`、`carlair_ros_bridge` 等模块没有功能重叠。

## 1. 实验目的

* 掌握 ROS 2 服务端与客户端的创建与调用方式（`std_srvs/SetBool`）；
* 掌握参数声明、读取与**运行中修改立即生效**的实现方式（参数回调）；
* 掌握「服务 + 参数 + 话题」三者配合的节点写法，作为后续控制器与强化学习策略的公共接口。

## 2. 运行环境

| 项目 | 版本 / 说明 |
|---|---|
| 操作系统 | Ubuntu 20.04.6 LTS（VMware 虚拟机），宿主为 Windows 11 |
| ROS | ROS 2 Humble |
| Python | 3.8.10（虚拟机内与 Humble 匹配的版本） |
| 依赖 | 仅使用 ROS 2 自带功能包：`rclpy`、`std_msgs`、`std_srvs`、`launch`、`launch_ros` |
| 硬件 | 无特殊要求，普通 PC 即可 |

虚拟机中确认环境：

```bash
lsb_release -a              # 应显示 Ubuntu 20.04
echo $ROS_DISTRO            # 应显示 humble
python3 --version           # 应显示 Python 3.8.x
```

> 注意：ROS 1（Noetic）与 ROS 2（Humble）的环境变量不能混用。若终端同时 source 过两套环境，请关闭终端后重新打开，只 source 需要的那一套。

### 2.1 环境准备（第一次执行一次）

如果虚拟机里还没有 ROS 2 Humble，先按仓库的 [安装 ROS](../../../docs/ubuntu20.04.md) 页面配置好。

确认装好后，把 ROS 2 环境写进 `~/.bashrc`，这样每开一个新终端都会自动加载：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo $ROS_DISTRO            # 应输出 humble
```

> 不要同时 source Noetic 与 Humble 两套环境，否则会提示 `ROS_DISTRO was set to 'noetic' before`。

## 3. 接口说明

| 接口 | 类型 | 说明 |
|---|---|---|
| 服务 `/uav/set_mode` | `std_srvs/SetBool` | `data: true` → 模式变 `FLYING`；`data: false` → 模式变 `IDLE`；返回 `success` 与 `message` |
| 参数 `publish_rate` | double，默认 `2.0` | 状态发布频率（Hz），运行中可改 |
| 参数 `start_mode` | string，默认 `IDLE` | 启动时的初始模式（只在启动时读取） |
| 话题 `/uav/status` | `std_msgs/String`，发布 | 内容形如 `mode=FLYING, count=12` |

## 4. 目录结构

```
ros2_service_param_demo/
├── main.py                                     # 模块入口：python3 main.py 直接运行整个模块
├── launch/
│   └── service_param_demo.launch.py            # ros2 launch 启动文件（含参数初值）
├── ros2_service_param_demo/
│   └── uav_mode_node.py                        # 唯一节点：服务 + 参数 + 话题发布
├── resource/ros2_service_param_demo             # ament 资源索引标记文件
├── test/                                       # ament_flake8 / ament_pep257 检查
├── package.xml
├── setup.py
└── setup.cfg
```

## 5. 运行步骤

### 5.1 编译

```bash
mkdir -p ~/ros2_ws/src
cp -r <本仓库路径>/src/chap2/ros2_service_param_demo ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_service_param_demo
source install/setup.bash
```

### 5.2 运行（三种方式任选其一，效果相同）

**方式 A：launch 启动（推荐）**

```bash
ros2 launch ros2_service_param_demo service_param_demo.launch.py
```

**方式 B：main 脚本入口**

```bash
python3 ~/ros2_ws/src/ros2_service_param_demo/main.py
```

**方式 C：单节点启动并指定参数**

```bash
ros2 run ros2_service_param_demo uav_mode_node \
  --ros-args -p publish_rate:=5.0 -p start_mode:=FLYING
```

节点启动后会输出：

```
[INFO] [...] [uav_mode_node]: uav_mode_node 已启动: publish_rate=2.0 Hz, mode=IDLE
[INFO] [...] [uav_mode_node]: 发布状态: mode=IDLE, count=0
[INFO] [...] [uav_mode_node]: 发布状态: mode=IDLE, count=1
```

### 5.3 调用服务切换模式

另开一个终端（记得 `source /opt/ros/humble/setup.bash` 与 `source ~/ros2_ws/install/setup.bash`）：

```bash
ros2 service list                                   # 应看到 /uav/set_mode
ros2 service call /uav/set_mode std_srvs/srv/SetBool "{data: true}"
ros2 service call /uav/set_mode std_srvs/srv/SetBool "{data: false}"
```

调用后节点终端会打印 `模式切换为 FLYING` / `模式切换为 IDLE`，`/uav/status` 的内容也随之改变。

### 5.4 在线修改参数

```bash
ros2 param list
ros2 param get /uav_mode_node publish_rate           # 显示 2.0
ros2 param set /uav_mode_node publish_rate 5.0       # 输出立刻变密
ros2 param set /uav_mode_node publish_rate 0.5       # 输出立刻变疏
ros2 param set /uav_mode_node publish_rate -1.0      # 被拒绝：publish_rate 必须大于 0
```

### 5.5 观察话题

```bash
ros2 topic list             # 应看到 /uav/status
ros2 topic echo /uav/status
ros2 topic hz /uav/status   # 频率应与 publish_rate 一致
```

### 5.6 结束运行

在运行终端按 `Ctrl+C`。此时可能看到 `KeyboardInterrupt` 相关的 Traceback，也可能只看到节点打印的 `uav_mode_node 被手动终止`，两者都是**正常的退出提示**，不是程序报错。退出时如果还跟着一行 `Failed to publish log message to rosout: publisher's context is invalid`，那是节点已销毁、rclpy 仍尝试向 `/rosout` 上报日志造成的，同样属于正常现象。

## 6. 运行效果

调用服务切换飞行模式：

![服务调用](../../../docs/img/chap2/ros2_service_param_demo/service_call.gif)

在线修改参数，输出频率立刻变化：

![参数修改](../../../docs/img/chap2/ros2_service_param_demo/param_change.gif)

launch 启动与话题频率：

![launch 启动](../../../docs/img/chap2/ros2_service_param_demo/launch_start.png)

## 7. 常见问题

| 现象 | 原因 | 解决 |
|---|---|---|
| `ros2: command not found` | 未 source ROS 2 环境 | 执行 `source /opt/ros/humble/setup.bash`，或写入 `~/.bashrc` |
| `Package 'ros2_service_param_demo' not found` | 未 source 工作空间环境 | 编译后执行 `source ~/ros2_ws/install/setup.bash` |
| 终端提示混用了 noetic 与 humble | 同一终端加载了两套 ROS 环境 | 新开干净终端，只加载 Humble |
| `ros2 param set` 之后频率没变 | 节点名写错，或参数没实现回调 | 用 `ros2 node list` 确认节点名是 `/uav_mode_node`；本模块已在参数回调里重建定时器 |
| `ros2 service call` 报类型错误 | 服务类型或引号写错 | 使用 5.3 节给出的原样命令 |
| 退出时出现 `Failed to publish log message to rosout ...` | 节点已销毁，rclpy 仍在向 `/rosout` 上报日志 | 属正常现象，可忽略（见 5.6 节） |

## 8. 说明

* `source` 命令只对当前终端生效，关闭终端后失效；
* 本模块为教学实验代码，演示的是 ROS 2 服务与参数机制，不涉及具体飞控算法；
* 后续的控制器与强化学习策略节点可以复用这条「服务切换模式 + 参数在线调整 + 话题发布状态」的接口约定。
