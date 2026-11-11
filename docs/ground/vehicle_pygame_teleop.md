# 车辆键盘遥操作与二维模拟

`vehicle_pygame_teleop` 是一个基于 pygame 的车辆手动遥操作功能包：通过键盘控制车辆的纵向速度与转向，以 30Hz 向 `/cmd_vel` 话题发布 `geometry_msgs/Twist` 指令；同时内置俯视二维驾驶模拟，车辆会在窗口中随控制指令实时运动，无需启动 Carla 等外部仿真器即可快速验证手动控制链路。

功能包源码位于仓库的 `src/vehicle_pygame_teleop` 目录。

![](../img/ground/vehicle_pygame_teleop.png)

## 功能特点

- W/↑ 加速、S/↓ 刹车/倒车、A/← 与 D/→ 控制左右转向，加减速过程平滑
- 空格键手刹急停；ESC、关闭窗口或 Ctrl+C 退出时自动发布零速度，避免车辆保持最后速度失控
- 二维模拟窗口：网格地图、车身与车头朝向随动、实时线速度/角速度 HUD 显示
- 标准 `/cmd_vel` 接口，可直接对接支持 Twist 指令的仿真器或真实底盘节点
- 车辆物理参数（限速、加减速度、转向响应、发布频率、话题名）全部可通过配置文件与 launch 参数调整，无需改代码
- 按键解释与运动学积分与 pygame 渲染解耦，核心逻辑可脱离 ROS 与显示器做离线回归测试

## 环境准备

- **推荐**：Ubuntu 20.04 + ROS Noetic（Python 3）
- **也可运行**：Ubuntu 18.04 + ROS Melodic（Python 2.7）
- 安装 pygame 依赖：

```bash
sudo apt update
# Ubuntu 20.04 + ROS Noetic：
sudo apt install -y python3-pygame
# Ubuntu 18.04 + ROS Melodic（其 rospy 是 Python 2.7）：
sudo apt install -y python-pygame
```

> **关于 Melodic。** 本模块只使用 Python 2.7 与 3.x 都支持的语法 —— 不用 f-string，也不引入 3.7+ 才有的 `from __future__ import annotations`，因此源码在 Noetic（Python 3）与 Melodic（Python 2.7）下都能运行。这一点不是小事：REP 151《Python 3 Support》规定 **Noetic 才是第一个完全支持 Python 3 的 ROS 1 发行版**，Melodic 及更早版本的 `rospy` 只有 Python 2.7 版本，含 3.7+ 语法的脚本会直接 `SyntaxError`。
>
> 但要注意区分两件事：**源码兼容 Python 2.7，不等于脚本会被 Python 2.7 启动**。本模块脚本的 shebang 按 Noetic 约定写作 `#!/usr/bin/env python3`；在 Melodic 上需要让脚本由 `python2` 启动 —— 正常途径是 `catkin_install_python` 按发行版重写 shebang（REP 151 的多发行版做法）。若直接执行源码脚本，请显式指定解释器：`python2 "$(rospack find vehicle_pygame_teleop)/scripts/main.py"`。

> 文档推荐的 apt 源 pygame 为 1.9.6，本模块的绘制接口（`draw.line` / `draw.circle` / `Surface` / `SRCALPHA` 等）在该版本下均可正常工作；代码中刻意不使用 `rect(..., border_radius=)` 这类 1.9.6 不支持的参数。

## 构建与环境加载

将功能包放入 ROS 工作空间的 `src` 目录（例如 `~/catkin_ws/src`），然后编译并加载环境：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> 提示：若希望每个终端自动加载工作空间环境，可执行 `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`。

## 启动运行

统一入口为 `launch/main.launch`，节点入口为 `scripts/main.py`：

```bash
roslaunch vehicle_pygame_teleop main.launch
```

也可以只运行节点（前台）：

```bash
rosrun vehicle_pygame_teleop main.py
```

启动后弹出二维模拟窗口，点击窗口获取键盘焦点即可控制车辆，画面中的车辆会随按键实时加速、转向和移动。

## 按键说明

| 按键  | 作用  |
| --- | --- |
| W / ↑ | 油门加速 |
| S / ↓ | 前进时刹车，静止时倒车 |
| A / ← | 左转  |
| D / → | 右转  |
| 空格  | 手刹急停 |
| ESC / 关闭窗口 | 发布零速度后退出 |

按键与 `/cmd_vel` 语义的对应关系：左转时 `angular.z` 为正，右转时为负；前进时 `linear.x` 为正，倒车时为负。倒车过程中左右转向的效果会整体反向（差速底盘特性），该行为可用参数 `vehicle/reverse_steer_invert` 关闭。

## 话题接口

| 方向  | 话题  | 消息类型 | 说明  |
| --- | --- | --- | --- |
| 发布  | `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` 纵向速度（m/s），`angular.z` 偏航角速度（rad/s），发布频率 30Hz |

话题名与发布频率分别由 `topic/cmd_vel`、`rate/publish_hz` 参数决定，下表为默认值。

## 参数配置

参数默认值集中在 `config/vehicle_pygame_teleop.yaml`，由 `main.launch` 加载，并由 launch `<arg>` 覆盖，因此在命令行即可临时调整，不必修改代码或配置文件。

### 车辆物理与运行参数

| 参数名 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- |
| `vehicle/max_speed` | `1.5` | m/s | 最大前进速度 |
| `vehicle/max_reverse` | `0.8` | m/s | 最大倒车速度 |
| `vehicle/accel` | `2.2` | m/s² | 油门与倒车加速度 |
| `vehicle/brake` | `4.5` | m/s² | 刹车减速度（倒车中踩油门时的制动也用它） |
| `vehicle/handbrake` | `8.0` | m/s² | 空格手刹减速度 |
| `vehicle/drag` | `1.2` | m/s² | 无按键时的滑行阻力减速度 |
| `vehicle/max_yaw_rate` | `1.4` | rad/s | 最大偏航角速度（左转为正） |
| `vehicle/yaw_response` | `8.0` | rad/s² | 转向响应角加速度，越大越"跟手" |
| `vehicle/reverse_steer_invert` | `true` | — | 倒车时转向效果反向 |
| `rate/publish_hz` | `30.0` | Hz | `/cmd_vel` 发布频率 |
| `topic/cmd_vel` | `/cmd_vel` | — | 速度指令话题 |

### 命令行覆盖示例

```bash
# 把最高速度提到 3 m/s、把发布频率改成 50Hz
roslaunch vehicle_pygame_teleop main.launch max_speed:=3.0 publish_hz:=50.0

# 换话题名，接到 Carla 的车辆控制话题
roslaunch vehicle_pygame_teleop main.launch cmd_vel_topic:=/carla/ego_vehicle/twist

# 关闭"倒车转向反向"，方便对比行为差异
roslaunch vehicle_pygame_teleop main.launch reverse_steer_invert:=false
```

参数分层与优先级：`config` 里的 yaml 提供默认值 → 同名 launch `<arg>` 覆盖 → 代码内的常量是最后的兜底值，三者数值一致。节点用 `rospy.get_param("vehicle/max_speed")` 这类相对名读取，与 launch 中位于顶层的 `<param>`/`<rosparam>` 对应，可用 `rosparam get /vehicle/max_speed` 在运行时核对。

## 运动学模型与实现要点

车辆按差速底盘（非完整约束）模型积分，每一帧执行三步：

**1. 纵向速度。** 手刹优先，按下空格时以 `handbrake` 减速到 0 且不响应油门/刹车；否则按油门、刹车、滑行三种情况更新：

```
油门：v ← min(max_speed,     v + accel·dt)        （若 v < 0 则先以 brake 制动到 0）
刹车：v ← max(-max_reverse,  v - accel·dt)        （若 v > 0 则先以 brake 制动到 0）
滑行：v ← v ∓ drag·dt                             （向 0 收敛）
```

**2. 转向角速度。** 左转给目标值 `+max_yaw_rate`，右转给 `-max_yaw_rate`，再按 `yaw_response` 限幅逼近，避免角速度跳变：

```
ω ← ω + sign(ω* − ω) · min(|ω* − ω|, yaw_response·dt)
```

若本帧更新后的速度低于 `-0.02 m/s`（正在倒车）且 `reverse_steer_invert` 为真，则把目标角速度取反 —— 这正是倒车时"方向打反"的物理来源。

**3. 运动学积分。** 用欧拉法推进航向与位置（世界坐标，x 向东、y 向北，逆时针为正）：

```
θ ← θ + ω·dt
x ← x + v·cos θ·dt
y ← y + v·sin θ·dt
```

随后把 `(v, ω)` 直接映射为 `Twist.linear.x` 与 `Twist.angular.z` 发布，其余分量恒为 0。俯视画面把世界坐标映射到屏幕上车辆固定在窗口中心：每米对应 22 像素，y 轴方向取反，车体按 `yaw` 旋转绘制，因此画面中的运动方向与积分的世界坐标严格一致。

其他实现细节：

- 单帧 `dt` 上限截断为 0.1s：拖动窗口或系统卡顿时，不会因为一帧积分过大而让车辆"瞬移穿墙"。
- 退出路径有三条（窗口关闭、ESC、Ctrl+C），都会被兜底为零速度；Ctrl+C 走 `rospy.on_shutdown` 回调，此阶段不能再使用 `rospy.sleep`，因此回调里以 `wait=False` 连发两条零速度。
- 字体枚举失败时回退到 pygame 内置默认字体，保证节点在缺字体的环境里仍能启动。

## 目录结构

```
vehicle_pygame_teleop/
├── CMakeLists.txt                     # 安装三个脚本与 launch/config
├── package.xml                        # 依赖：rospy / geometry_msgs / python3-pygame
├── README.md
├── config/
│   └── vehicle_pygame_teleop.yaml     # 全部可调参数（默认值）
├── launch/
│   └── main.launch                    # 统一入口：加载 yaml、暴露 <arg>、启动 main.py
└── scripts/
    ├── main.py                        # 入口（main.* 约定）
    ├── teleop_core.py                 # 纯逻辑：参数、控制意图、运动学（不依赖 pygame/ROS）
    └── vehicle_pygame_teleop.py       # 节点：pygame 渲染 + rospy 发布
```

## 离线测试

`teleop_core.py` 不导入 pygame 与 rospy，因此核心逻辑可以在没有 ROS、没有仿真器、没有显示器的机器上直接跑离线回归（不需要 `roscore`，也不需要 pygame）：

```bash
cd <仓库根目录>
python3 tests/test_vehicle_pygame_teleop_local.py   # Noetic / 任意 Python 3
python2 tests/test_vehicle_pygame_teleop_local.py   # Melodic / Python 2.7
```

测试覆盖参数默认值与覆盖、按键到控制意图的映射、油门/刹车/倒车/手刹/滑行的纵向行为、限速点、转向平滑与倒车反向、位置积分，以及工程文件约定（入口为 `main.py`、launch 加载 yaml 并暴露参数等）。全部通过时打印 `全部通过` 并以 0 退出；有失败项时打印失败清单并以 1 退出。

## 常见问题

- **窗口里的车不动**：pygame 窗口需要键盘焦点，用鼠标点一下窗口再按键。
- **`TypeError: rect() takes no keyword arguments`**：使用了 pygame 1.9.6 不支持的 `border_radius` 参数。本模块已避免该写法；若自行修改绘制代码请勿引入。
- **Ctrl+C 后车辆还在跑**：确认最后一条 `/cmd_vel` 为零速度。本模块已把停车注册到 `rospy.on_shutdown`，可用
  `rostopic echo /cmd_vel` 后按 Ctrl+C 验证最后一条 `linear.x` 是否为 0。
- **改了参数没生效**：确认是通过 `roslaunch` 启动并传的是 launch `<arg>` 名（不带 `vehicle/` 前缀），例如 `max_speed:=3.0`；也可以直接编辑 `config/vehicle_pygame_teleop.yaml`。
- **找不到 `main.py`**：先在 `~/catkin_ws` 重新 `catkin_make` 并 `source devel/setup.bash`。

## 参考

- [pygame 官方文档](https://www.pygame.org/docs/)
- [geometry_msgs/Twist 消息定义](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)
- [ROS 节点编写入门（rospy）](https://wiki.ros.org/rospy/Tutorials)

---

**大模型使用声明**：本页文档与对应代码在整理参数化改造、离线测试与文档说明时使用了大模型辅助，提交内容由提交者负责。
