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
| 松开 / 其它键 | 悬停 | 全零 |
| `ESC` / `Ctrl-C` | 退出 | — |

> `Q`/`E` 的方向遵循 AirSim 机体系 `yaw_rate` 约定：`Q` 为 `+yaw_rate`（机头右转）。
> 若实际飞行中觉得方向反了，交换 `keyboard_control.py` 里 `KEY_MAP` 的 `q`/`e` 符号即可（一行）。

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

把焦点放在**弹出来的 xterm 窗口**上，按 `W/A/S/D/R/F/Q/E` 操控；松开即悬停，`ESC` 退出。

### 方式 B：rosrun 前台运行（不装 xterm 也能用）

```bash
source ~/bridge_ws/devel/setup.bash
rosrun uav_keyboard_control main.py
```

直接在**当前终端**里按 WASD 即可（`rosrun` 会把终端 stdin 接给节点）。

## 参数（config/keyboard.yaml，可用 launch 参数覆盖）

```bash
roslaunch uav_keyboard_control main.launch horiz_speed:=3.0 vert_speed:=1.5 yaw_rate:=0.8
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `control/horiz_speed` | 2.0 | 水平速度 m/s |
| `control/vert_speed` | 1.0 | 垂直速度 m/s |
| `control/yaw_rate` | 1.0 | 偏航角速度 rad/s |
| `rate/publish_hz` | 30.0 | 指令发布频率 |

## 说明

- 本模块**只发指令**，不直连仿真器；仿真/坐标换算/安全限幅都在桥接层。
  这样键盘、规划器、神经网络控制器可以共用同一条 `/uav/cmd_vel` 通道，互不耦合。
- **roslaunch 的 stdin 坑**：`roslaunch` 启动的节点拿不到终端 stdin（节点被放进新的会话，
  连 `/dev/tty` 都打不开），所以键盘输入读不到——这也是 `teleop_twist_keyboard` 通常用
  `rosrun` 跑的原因。本模块的 launch 文件用 `launch-prefix="xterm -e"` 给键盘节点单独弹一个
  xterm 终端来读键；节点内部同时优先读 `/dev/tty`、失败回退 `sys.stdin`，两种跑法都兼容。
- CarlaAir 视口**内置** WASD 键盘操控（任务①的"仿真"部分）；本模块提供的是
  **ROS 侧**的键盘控制（任务①的"键盘控制"部分），两者可对比验证。

## 本地测试（无需 ROS / 仿真器 / 终端）

```bash
python3 tests/test_keyboard_local.py   # 35 项：按键映射 / 速度配置 / 字段映射 / 工程文件
```

`key_to_cmd` 是纯函数，`step_once` 把一次按键转成一条 Twist 消息，均可在无 ROS 环境下回归。
