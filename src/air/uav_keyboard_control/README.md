# uav_keyboard_control

任务①：**无人机仿真 + 键盘控制**。把终端按键转换为机体系速度指令，发布到 `/uav/cmd_vel`，
由 `carlair_ros_bridge` 负责连仿真器、换算坐标并执行。

## 按键映射（机体系）

| 按键 | 动作 | 速度分量 |
|---|---|---|
| `W` / `S` | 前进 / 后退 | `linear.x = ±horiz_speed` |
| `A` / `D` | 左移 / 右移 | `linear.y = ±horiz_speed` |
| `R` / `F` | 上升 / 下降 | `linear.z = ±vert_speed` |
| `Q` / `E` | 右转 / 左转 | `angular.z = ±yaw_rate` |
| 组合键（如 `W`+`A`、`W`+`Q`） | 斜向 / 边走边转 | 各轴分量叠加 |
| 松开 / 其它键 | 悬停 | 速度按加速度上限平滑回零 |
| `ESC` / `Ctrl-C` | 退出 | 先补发一条零速 |

> `Q`/`E` 的方向遵循 AirSim 机体系 `yaw_rate` 约定：`Q` 为 `+yaw_rate`（机头右转）。
> 若实际飞行中觉得方向反了，交换 `keyboard_control.py` 里 `KEY_MAP` 的 `q`/`e` 符号即可（一行）。

> **多键组合**：终端只送"按下"的字节、没有"抬起"事件，所以"是否按住"用
> "最近 `key_hold_timeout`（默认 0.15 s）内是否还收到过这个键"来判断——按住时终端
> 自动重复会持续送字节。多个键同时按住时各轴分量相加，`W`+`S` 这类互斥组合自然抵消。
> **平滑**：下发的速度按 `accel_*` 斜坡逼近目标（水平默认 4 m/s²，约 0.5 s 到 2 m/s），
> 松开后同样按斜坡回零，桥接层收到的是一条连续变化的指令序列而不是阶跃。
> **方向键不再误退出**：`ESC [ A` 这类转义序列会被整段吃掉，按方向键既不退出也不会被当成运动键。

## 运行

### 方式 A：roslaunch（会弹一个 xterm 窗口，作业要求的 launch 启动）

```bash
# 一次性安装 xterm（键盘节点需要独立终端窗口来读键）
sudo apt install -y xterm

# 终端 1：先启动桥接（连仿真、执行指令、自动起飞悬停）
source ~/bridge_ws/devel/setup.bash
roslaunch carlair_ros_bridge main.launch

# 终端 2：键盘控制（会弹出一个 xterm 小窗口）
source ~/bridge_ws/devel/setup.bash
roslaunch uav_keyboard_control main.launch
```

把焦点放在**弹出来的 xterm 窗口**上，按 `W/A/S/D/R/F/Q/E` 操控（可同时按住多个键斜飞）；松开后平滑减速到悬停，`ESC` 退出。

### 方式 B：rosrun 前台运行（不装 xterm 也能用）

```bash
source ~/bridge_ws/devel/setup.bash
rosrun uav_keyboard_control main.py
```

直接在**当前终端**里按 WASD 即可（`rosrun` 会把终端 stdin 接给节点）。

## 参数（config/keyboard.yaml，可用 launch 参数覆盖）

```bash
roslaunch uav_keyboard_control main.launch horiz_speed:=3.0 vert_speed:=1.5 yaw_rate:=0.8
# 想要"老版本"的瞬时阶跃手感（不做加速度平滑）：
roslaunch uav_keyboard_control main.launch accel_horiz:=0 accel_vert:=0 accel_yaw:=0
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `control/horiz_speed` | 2.0 | 水平速度 m/s |
| `control/vert_speed` | 1.0 | 垂直速度 m/s |
| `control/yaw_rate` | 1.0 | 偏航角速度 rad/s |
| `control/key_hold_timeout` | 0.15 | 按键保持判定 s（终端无抬起事件，靠自动重复续期） |
| `control/accel_horiz` | 4.0 | 水平加速度上限 m/s²，≤0 关闭平滑（瞬时到位） |
| `control/accel_vert` | 2.0 | 垂直加速度上限 m/s² |
| `control/accel_yaw` | 4.0 | 偏航角加速度上限 rad/s² |
| `rate/publish_hz` | 30.0 | 指令发布频率 |

## 说明

- 本模块**只发指令**，不直连仿真器；仿真/坐标换算/安全限幅都在桥接层。
  这样键盘、规划器、神经网络控制器可以共用同一条 `/uav/cmd_vel` 通道，互不耦合。
- **退出必停**：`ESC`/`Ctrl-C` 退出前会补发一条全零 `Twist`，不用等桥接层 0.5 s 超时保护。
- 平滑只改"速度怎么变"，不改"速度去哪"：目标速度仍由 `control/*_speed` 决定，
  桥接层的限幅（`max_horiz_speed` 等）与超时悬停也照旧生效。
- **roslaunch 的 stdin 坑**：`roslaunch` 启动的节点拿不到终端 stdin（节点被放进新的会话，
  连 `/dev/tty` 都打不开），所以键盘输入读不到——这也是 `teleop_twist_keyboard` 通常用
  `rosrun` 跑的原因。本模块的 launch 文件用 `launch-prefix="xterm -e"` 给键盘节点单独弹一个
  xterm 终端来读键；节点内部同时优先读 `/dev/tty`、失败回退 `sys.stdin`，两种跑法都兼容。
- CarlaAir 视口**内置** WASD 键盘操控（任务①的"仿真"部分）；本模块提供的是
  **ROS 侧**的键盘控制（任务①的"键盘控制"部分），两者可对比验证。

## 本地测试（无需 ROS / 仿真器 / 终端）

```bash
python3 tests/test_keyboard_local.py   # 79 项：单键映射 / 组合键 / 按键保持 / 加速度斜坡 / 转义序列 / 工程文件
python3 -m unittest discover -s src/air/uav_keyboard_control/tests -v   # 退出键 + 30 Hz 主循环集成（假终端/假时钟）
```

`key_to_cmd` 是纯函数，`merge_target`/`KeyTracker`/`VelocitySmoother` 都可以脱离
ROS/终端单独测试，`step_once` 把一次按键推进一个控制周期，均可在无 ROS 环境下回归。
