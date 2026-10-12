# 车辆键盘遥操作（CARLA 0.9.16）

键盘 → 平滑 Twist（`/carla/ego_vehicle/twist`，30Hz）→ `carla_twist_to_control` → CARLA 0.9.16 自车。
物理仿真由 CARLA 提供；源码位于 `src/vehicle_pygame_teleop`。

![控制链路](../img/ground/vehicle_pygame_teleop_pipeline.png)

![键盘控制窗口](../img/ground/vehicle_pygame_teleop_ui.png)

## 控制原理

按键 → 指令目标值，经一阶速率限制逐步逼近（30Hz）：

速度指令每帧（$\Delta t = 1/30\,\mathrm{s}$）按如下方式更新：

$$v_{k+1} = \mathrm{clamp}\big(v_k \pm a\,\Delta t,\; [-v_{\mathrm{rev}},\, v_{\mathrm{fwd}}]\big)$$

其中 $a$ 为油门加速度（4.0 m/s²）、刹车减速度（8.0 m/s²）或松键回落减速度
（3.0 m/s²），$v_{\mathrm{fwd}}=10$ m/s、$v_{\mathrm{rev}}=5$ m/s。转向指令同理：

$$\delta_{k+1} = \mathrm{clamp}\big(\delta_k \pm \omega_r\,\Delta t,\; [-\delta_{\max},\, \delta_{\max}]\big)$$

其中 $\omega_r = 3.0$ rad/s 为转向响应速率，$\delta_{\max} = 0.6$ rad 为最大转向指令。

`carla_twist_to_control` 的下游映射（见其源码）：

- 零 Twist（$v=0$ 且 $\delta=0$）→ `brake = 1`（全力刹车）；
- $v>0$ → `throttle = min(10, v)/10`；$v<0$ → `reverse = True`；
- `steer = -clamp(δ, ±δ_veh)/δ_veh`，其中 $δ_veh$ 为车辆最大前轮转角（由
  `/carla/ego_vehicle/vehicle_info` 获得）。

## 环境准备

- Ubuntu 20.04 + ROS Noetic（Python 3）
- CARLA 0.9.16（运行在有 GPU 的宿主机上）
- 安装 pygame 依赖：

```bash
sudo apt update
sudo apt install -y python3-pygame
# 或使用 pip 安装：
pip3 install pygame
```

## 构建与环境加载

将功能包放入 ROS 工作空间的 `src` 目录（例如 `~/catkin_ws/src`），并确保
`src/ground/ros-bridge` 中的 `carla_twist_to_control`、`carla_msgs`、
`ros_compatibility` 也在工作空间中，然后编译并加载环境：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 启动运行

前置步骤见 [设置并连接到 Carla 模拟器](../set_up_and_connect_to_carla.md)。完整链路：

1. 在装有 GPU 的宿主机上启动 CARLA 0.9.16 服务端（Windows 双击 `CarlaUE4.exe`；
   Linux 运行 `./CarlaUE4.sh`）；
2. 启动 ros-bridge 与示例自车（`host` 改为宿主机 IP）：

```bash
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch host:=192.168.8.1 timeout:=60000 town:='Town10HD_Opt'
```

3. 启动 Twist→车辆控制转换节点：

```bash
roslaunch carla_twist_to_control carla_twist_to_control.launch
```

4. 启动本功能包：

```bash
roslaunch vehicle_pygame_teleop main.launch
```

启动后点击窗口获取键盘焦点即可驾驶。

> 自车生成前，转换节点告警 `Did not yet receive vehicle info.` 并忽略指令。

## 运行效果（CARLA 0.9.16 实测）

![CARLA 实机运行](../img/ground/vehicle_pygame_teleop_carla.gif)

![CARLA 实机运行静帧](../img/ground/vehicle_pygame_teleop_carla.png)

实测数据：

| 项 | 实测 |
| --- | --- |
| 速度指令上限 | 10 m/s（`/carla/ego_vehicle/twist`，30Hz） |
| 自车实际峰值速度 | 14.6 m/s |
| 加减速行为 | 平滑，无跳变（油门/刹车指令速率限制 4/8 m/s²） |
| 急停与退出 | 零 Twist → CARLA 侧全力刹停 |

## 按键说明

| 按键 | 作用 |
| --- | --- |
| W / ↑ | 油门加速（速度指令上限 10 m/s） |
| S / ↓ | 前进时刹车，静止时倒车（倒车指令上限 5 m/s） |
| A / ← | 左转 |
| D / → | 右转 |
| 空格 | 急停（发布零速度指令） |
| ESC / 关闭窗口 / Ctrl+C | 发布零速度后退出 |

## 话题接口

| 方向 | 话题 | 消息类型 | 说明 |
| --- | --- | --- | --- |
| 发布 | `/carla/ego_vehicle/twist`（默认，参数 `~twist_topic` 可修改） | `geometry_msgs/Twist` | `linear.x` 速度指令（m/s），`angular.z` 转向指令（rad），发布频率 30Hz |

## 与已有「手动控制」示例的区别

| 对比项 | 已有「手动控制」示例 | 本模块 `vehicle_pygame_teleop` |
| --- | --- | --- |
| 控制入口 | ros-bridge 内置手动驾驶（在 CARLA 窗口按 `B`） | 独立 ROS 节点，键盘窗口操作 |
| 指令接口 | 无对外接口 | 发布标准 `/carla/ego_vehicle/twist`，可编程、可复用 |
| 指令平滑 | 无 | 加减速与转向均平滑，车辆不跳变 |
| 退出行为 | - | ESC / 关闭窗口 / Ctrl+C 均自动发布零速度 |
| 适用场景 | 日常体验 CARLA | 作为感知、规划、端到端任务的遥控与调试基座 |
