# vehicle_pygame_teleop

基于 pygame 的 ROS 车辆键盘遥操作功能包，**内置俯视二维驾驶模拟**：键盘输入实时驱动画面中的车辆运动，同时以默认 30Hz 向 `/cmd_vel` 发布 `geometry_msgs/Twist` 速度指令，无需启动外部仿真器即可验证手动控制效果。

## 功能

- 方向键 / WASD 控制车辆油门、刹车/倒车与左右转向，加减速过程平滑
- 空格键手刹急停；ESC、关闭窗口或 Ctrl+C 退出时自动发布零速度，车辆可靠停车
- 内置俯视二维模拟：网格地图、车身与车头朝向随动、实时线速度/角速度显示
- 默认 30Hz 稳定输出 `/cmd_vel`，接口格式与 Carla 等仿真器的遥控接口一致
- 车辆物理参数（限速、加减速度、转向响应、发布频率、话题名）全部由配置文件与 launch 参数提供，无需改代码
- 按键解释与运动学积分与 pygame 渲染解耦，核心逻辑可脱离 ROS 与显示器做离线回归测试

## 运行环境

- **推荐**：Ubuntu 20.04 + ROS Noetic（Python 3）
- **也可运行**：Ubuntu 18.04 + ROS Melodic（Python 2.7）
- pygame：Noetic 上装 `python3-pygame`，Melodic 上装 `python-pygame`

> 代码只使用 Python 2.7 与 3.x 都支持的语法（不用 f-string，也不引入 3.7+ 才有的
> `from __future__ import annotations`），因此**源码**在两代发行版下都能运行。之所以要做到这点，
> 是因为 Melodic 的 rospy 仍是 Python 2.7 —— REP 151 规定 Noetic 才是第一个完全支持
> Python 3 的 ROS 1 发行版。
>
> 注意区分：**源码兼容 Python 2.7 ≠ 脚本会被 Python 2.7 启动**。脚本 shebang 按 Noetic 约定
> 写作 `python3`；在 Melodic 上需让脚本由 `python2` 启动（`catkin_install_python` 会按发行版
> 重写 shebang），或直接执行时显式指定：
> `python2 "$(rospack find vehicle_pygame_teleop)/scripts/main.py"`

## 1. 安装依赖

```bash
sudo apt update
# Ubuntu 20.04 + ROS Noetic：
sudo apt install -y python3-pygame
# Ubuntu 18.04 + ROS Melodic（其 rospy 是 Python 2.7）：
sudo apt install -y python-pygame
```

## 2. 放入工作空间并构建

将整个 `vehicle_pygame_teleop` 文件夹放入 ROS 工作空间的 `src` 目录（例如 `~/catkin_ws/src`），然后编译：

```bash
cd ~/catkin_ws
catkin_make
```

## 3. 加载工作空间环境

每个新终端使用本功能包前，先 source 工作空间环境（只需执行一次，建议写入 `~/.bashrc`）：

```bash
source ~/catkin_ws/devel/setup.bash
```

## 4. 运行

统一入口为 `launch/main.launch`，节点入口为 `scripts/main.py`：

```bash
roslaunch vehicle_pygame_teleop main.launch
```

也可以只运行节点（前台）：

```bash
rosrun vehicle_pygame_teleop main.py
```

启动后弹出二维模拟窗口，**点击窗口使其获得键盘焦点**，即可开始控制车辆。

## 按键说明

| 按键 | 功能 |
| --- | --- |
| W 或 ↑ | 油门加速 |
| S 或 ↓ | 前进时刹车，静止时倒车 |
| A 或 ← | 左转（`angular.z` 为正） |
| D 或 → | 右转（`angular.z` 为负） |
| 空格 | 手刹急停 |
| ESC / 关闭窗口 | 发布零速度后退出 |

## 话题接口

- 发布话题：`/cmd_vel`（`geometry_msgs/Twist`）
  - `linear.x`：纵向线速度，前进为正、倒车为负（m/s）
  - `angular.z`：偏航角速度，左转为正、右转为负（rad/s）
- 默认发布频率：30Hz

## 参数配置

默认值见 `config/vehicle_pygame_teleop.yaml`，由 `main.launch` 加载，并由 launch `<arg>` 覆盖。

| 参数名 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- |
| `vehicle/max_speed` | `1.5` | m/s | 最大前进速度 |
| `vehicle/max_reverse` | `0.8` | m/s | 最大倒车速度 |
| `vehicle/accel` | `2.2` | m/s² | 油门与倒车加速度 |
| `vehicle/brake` | `4.5` | m/s² | 刹车减速度 |
| `vehicle/handbrake` | `8.0` | m/s² | 手刹减速度 |
| `vehicle/drag` | `1.2` | m/s² | 无按键时的滑行阻力减速度 |
| `vehicle/max_yaw_rate` | `1.4` | rad/s | 最大偏航角速度 |
| `vehicle/yaw_response` | `8.0` | rad/s² | 转向响应角加速度 |
| `vehicle/reverse_steer_invert` | `true` | — | 倒车时转向效果反向 |
| `rate/publish_hz` | `30.0` | Hz | `/cmd_vel` 发布频率 |
| `topic/cmd_vel` | `/cmd_vel` | — | 速度指令话题 |

命令行覆盖示例（用 launch 参数名，不带 `vehicle/` 前缀）：

```bash
roslaunch vehicle_pygame_teleop main.launch max_speed:=3.0 publish_hz:=50.0
roslaunch vehicle_pygame_teleop main.launch cmd_vel_topic:=/carla/ego_vehicle/twist
roslaunch vehicle_pygame_teleop main.launch reverse_steer_invert:=false
```

## 目录结构

```
vehicle_pygame_teleop/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── vehicle_pygame_teleop.yaml     # 全部可调参数（默认值）
├── launch/
│   └── main.launch                    # 加载 yaml、暴露 <arg>、启动 main.py
└── scripts/
    ├── main.py                        # 入口（main.* 约定）
    ├── teleop_core.py                 # 纯逻辑：参数、控制意图、运动学（不依赖 pygame/ROS）
    └── vehicle_pygame_teleop.py       # 节点：pygame 渲染 + rospy 发布
```

## 离线测试

`teleop_core.py` 不导入 pygame 与 rospy，因此核心逻辑可在没有 ROS、没有仿真器、没有显示器的机器上做离线回归：

```bash
cd <仓库根目录>
python3 tests/test_vehicle_pygame_teleop_local.py   # Noetic / 任意 Python 3
python2 tests/test_vehicle_pygame_teleop_local.py   # Melodic / Python 2.7
```

覆盖参数默认值与覆盖、按键到控制意图的映射、油门/刹车/倒车/手刹/滑行的纵向行为、限速点、转向平滑与倒车反向、位置积分，以及工程文件约定。全部通过时打印 `全部通过` 并以 0 退出。
