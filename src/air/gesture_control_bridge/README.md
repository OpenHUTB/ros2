# gesture_control_bridge

**手势控制无人机（桥接层版）**：订阅 `gesture_control_drone` 发布的 `/gesture_command`，
按 `carlair_ros_bridge` 的契约翻译成 `/uav/action`、`/uav/cmd_vel`、`/uav/goal`，
由桥接层统一执行（桥接层是仓库里唯一连接仿真器的地方）。

```
手势感知（gesture_control_drone）      本模块（translate only）            桥接层（唯一连仿真器）
/mediapipe → /gesture_command ──► GestureBridgeNode ──► /uav/action ─┐
                                    ├─ /uav/cmd_vel  ─────────────────┤──► carlair_ros_bridge ──► AirSim
                                    └─ /uav/goal     ─────────────────┘        ├─ 限速/限幅
                                                                              ├─ cmd_timeout 超时悬停
                                                                              └─ /uav/odom
```

## 1. 为什么单独做一个"桥接层版"

`gesture_control_drone`（已合并）是**直连 AirSim** 的实现，好处是不依赖 CarlaAir 栈，
但绕开了桥接层，也就绕开了桥接层提供的安全逻辑。本模块把"执行"换了个底：

| | `gesture_control_drone` | `gesture_control_bridge`（本模块） |
|---|---|---|
| 连接仿真器 | 自己持有 `MultirotorClient` | **不持有**，全部走桥接层 |
| 限速/限幅 | 模块自己实现 | 桥接层 `control/*` 统一 |
| 失控保护 | 模块自己的 `command_timeout` | 桥接层 `rate/cmd_timeout` 超时悬停 |
| 位姿 | 自己发 `/drone/odom` | 直接用桥接层 `/uav/odom` |
| 换仿真器/多机 | 要改模块 | 只改桥接层参数 |

> 桥接层原本只有 `/uav/cmd_vel` 与 `/uav/goal` 两个通道，**没有 takeoff/land**
> （靠 `control/auto_takeoff` 启动即起飞）。本模块顺手给桥接层补了一条最小动作通道
> `/uav/action`（`takeoff` / `land` / `hover`），这样手势指令集才能在桥接层完整表达。

## 2. 目录结构

```
gesture_control_bridge/
├── CMakeLists.txt                 catkin_install_python 只装入口，辅助模块走 install(FILES)
├── package.xml                    依赖 rospy/std_msgs/geometry_msgs/nav_msgs
├── README.md
├── config/gesture_bridge.yaml     翻译层参数（话题、速度、指令流频率、急停语义）
├── launch/main.launch             统一启动入口（可顺带拉起桥接层与手势感知）
└── scripts/
    ├── main.py                    模块入口（转发到节点）
    ├── gesture_bridge_node.py     订阅 /gesture_command -> 发桥接层消息
    └── bridge_cmd.py              ★ 纯逻辑翻译（不 import rospy，可离线单测）
```

## 3. 运行

```bash
# 1) 编译
cd ~/uav_ws && catkin_make && source devel/setup.bash

# 2) 宿主机启动 AirSim（或 CarlaAir）

# 3) 一条命令拉起：桥接层 + 手势感知 + 本模块
roslaunch gesture_control_bridge main.launch host:=192.168.49.1

# 或分两终端（推荐，方便看桥接层日志）：
roslaunch carlair_ros_bridge main.launch host:=192.168.49.1 \
    auto_takeoff:=false publish_image:=false publish_lidar:=false
roslaunch gesture_control_bridge main.launch start_bridge:=false
```

**两个必须注意的点**

1. 桥接层 `auto_takeoff` 必须为 `false`：否则桥接层启动即起飞，手势再发 `takeoff`
   会被 AirSim 拒绝（`Cannot perform takeoff because vehicle is already moving`）。
2. 不用桥接层的相机/雷达时（纯 AirSim Blocks 没有 `front_rgb`/`lidar1`），
   用 `publish_image:=false publish_lidar:=false`，否则那两个节点会报错退出。

绕开手势、直接验证执行层：

```bash
rostopic pub -1 /gesture_command std_msgs/String "data: 'takeoff'"
rostopic pub -1 /gesture_command std_msgs/String "data: 'forward'"
rostopic pub -1 /gesture_command std_msgs/String "data: 'land'"
rostopic echo /gesture_bridge/state      # 本模块的执行结果
rostopic echo /uav/status                # 桥接层的状态（ACTION:takeoff / VELOCITY / HOVER …）
```

## 4. 指令 → 桥接层消息

| 手势指令 | 通道 | 消息 | 说明 |
|---|---|---|---|
| `takeoff` / `land` / `hover` | `/uav/action` | `std_msgs/String` | 一条消息；桥接层在工作线程里执行 |
| `forward` | `/uav/cmd_vel` | `geometry_msgs/Twist` | `x = +speed`，机体系前 |
| `up` / `down` | `/uav/cmd_vel` | `Twist` | `z = ±speed`，机体系上/下 |
| `yaw_left` / `yaw_right` | `/uav/cmd_vel` | `Twist` | `angular.z = ±yaw_rate`（rad/s） |
| `home` | `/uav/goal` | `geometry_msgs/Point` | `(0, 0, home_altitude)`，ENU |
| `estop` | `/uav/cmd_vel` + `/uav/action` | `Twist` ×N + `String` | 先发 `estop_zeros` 条零速度，再 `hover`（或 `land`） |

**为什么速度指令要发 N 条**：桥接层的 `cmd_duration`（默认 0.1 s）是**单条**指令的
作用时长，而 `cmd_timeout`（0.5 s）内没有新指令就自动悬停。所以一次 1 s 的位移必须
拆成 `duration × stream_hz`（默认 20 条、间隔 0.05 s）连续下发，否则只会动 0.1 s。

## 5. 参数

见 `config/gesture_bridge.yaml`：`speed`、`yaw_rate`、`action_duration`、`stream_hz`、
`home_altitude`、`estop_action`、`estop_zeros`、`command_cooldown` 与全部话题名。
**限速/限幅**属于桥接层（`carlair_ros_bridge/config/bridge.yaml` 的 `control/*`），
两边不要重复配置。

## 6. 测试

```bash
python3 tests/test_gesture_bridge_local.py
```

不需要 ROS、摄像头、仿真器：ROS 用替身模块、桥接层参数直接读 `bridge.yaml` 校验。
覆盖：指令分类与词表契约（与 `gesture_control_drone/scripts/gesture_map.py` 逐项比对）、
`Twist`/`Point`/`String` 映射与符号、指令流条数与节拍、`cmd_timeout` 兼容性、
急停（零速 + 动作）、home 目标点、节点行为（入队/冷却/退出安全）、工程文件约定。

## 附：大模型使用声明

本模块代码与文档在编写过程中使用了 AI 大模型辅助（需求分析、代码与文档起草）。
逻辑部分由 `tests/test_gesture_bridge_local.py` 离线回归覆盖；真机动作序列以现场日志为准。
作者对提交内容的正确性与完整性负全部责任。
