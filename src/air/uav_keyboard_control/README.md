# uav_keyboard_control

任务①：**无人机仿真 + 键盘控制**。把终端按键转换为机体系速度指令，发布到 `/uav/cmd_vel`，
由 `carlair_ros_bridge` 负责连仿真器、换算坐标并执行。本模块同时作为任务②数据集采集时的人工遥控入口。

## 按键映射（机体系 x 前 / y 左 / z 上）

| 按键 | 动作 | 速度分量 | 默认幅值 |
|---|---|---|---|
| `W` / `S` | 前进 / 后退 | `linear.x = ±forward_speed` | ±2.0 m/s |
| `A` / `D` | 左移 / 右移 | `linear.y = ±side_speed` | ±0.5 m/s |
| `R` / `F` | 上升 / 下降 | `linear.z = ±vert_speed` | ±1.0 m/s |
| `Q` / `E` | 右转 / 左转 | `angular.z = ±yaw_rate` | ±0.4 rad/s |
| `空格` | 立即悬停 | 全零 | — |
| `ESC` | 退出 | — | — |

> `Q` 为 `+yaw_rate`（机头右转），`E` 为 `-yaw_rate`（机头左转）。各轴幅值已对齐部署安全限幅
> （`max_vx=2.0`、`max_vy=0.5`、`max_yaw=0.4`），保证采集标签尺度与模型输出一致。
> 若实际飞行方向反了，交换 `keyboard_control.py` 中 `KEY_MAP` 的 `q`/`e` 符号即可（一行）。

## 运行

前置：先在另一个终端启动 `carlair_ros_bridge`（连仿真、执行指令、自动起飞悬停）。

```bash
source ~/catkin_ws/devel/setup.bash

# 方式 A：roslaunch（作业要求的 launch 启动）
roslaunch uav_keyboard_control main.launch

# 方式 B：rosrun 前台运行
rosrun uav_keyboard_control main.py
```

把焦点放在键盘终端，按 `W/A/S/D/R/F/Q/E` 操控，`ESC` 退出。

## 参数（`config/keyboard.yaml`）

| 参数 | 默认 | 说明 |
|---|---:|---|
| `control/forward_speed` | 2.0 m/s | `W/S` 前后速度，对齐 `max_vx` |
| `control/side_speed` | 0.5 m/s | `A/D` 左右速度，对齐 `max_vy` |
| `control/vert_speed` | 1.0 m/s | `R/F` 升降速度 |
| `control/yaw_rate` | 0.4 rad/s | `Q/E` 偏航角速度，对齐 `max_yaw` |
| `control/hold_timeout` | 0.3 s | 松开按键后保持指令的最长时长，超时自动悬停 |
| `rate/publish_hz` | 30.0 Hz | 指令发布频率 |
| `topic/cmd_vel` | `/uav/cmd_vel` | 速度指令话题，与桥接包一致 |

## 实现说明

- **状态锁存 + 超时悬停**：按住按键持续保持指令；松开或焦点丢失后超过 `hold_timeout` 秒无新按键，
  自动清零悬停，避免一直保持上一条指令把无人机带飞；空格 / 未知键立即清零。
- 节点**只发指令**，不直连仿真器；仿真、坐标换算、安全限幅都在桥接层，键盘、规划器、
  神经网络控制器共用同一条 `/uav/cmd_vel` 通道，互不耦合。
- `key_to_cmd` 是纯函数，`step_once` 把一次按键转成一条 `Twist`，均可在无 ROS 环境下回归测试。
- CarlaAir 视口**内置** WASD 操控（任务①的「仿真」部分）；本模块提供 **ROS 侧**键盘控制
  （任务①的「键盘控制」部分），两者可对比验证。

## 文件

- `scripts/keyboard_control.py`：按键映射、锁存 + 超时悬停、节点主循环
- `scripts/main.py`：统一入口，复用 `keyboard_control.main()`
- `config/keyboard.yaml`：速度、超时、发布频率与话题参数
- `launch/main.launch`：启动入口，加载 `keyboard.yaml`

## 本地测试（无需 ROS / 仿真器 / 终端）

```bash
python3 tests/test_keyboard_local.py
```

## 声明

本项目使用大模型辅助撰写，作者对全部内容负责。
