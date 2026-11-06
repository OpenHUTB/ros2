# twist_to_control — 把 Twist 速度指令转换成车辆控制量

把 `/cmd_vel` 上的 `geometry_msgs/Twist`（线速度 `v`、角速度 `ω`）换算成车辆真正能执行的量：
前轮转向角、纵向速度，以及差速底盘的左右轮速。

`Twist` 表达的是"想让车怎么走"，转向角和轮速才是"车要打多少方向盘"。
中间缺的这一层换算正是本包做的事：

```
曲率  k = ω / v
转角  δ = atan(k · L)        L 为轴距
```

## 1. 功能

| 功能 | 说明 |
|---|---|
| 三种底盘模型 | `bicycle`（整车单车模型，输出一个前轮转角）、`ackermann`（左右前轮分别求角）、`diff_drive`（输出左右轮线速度，`control` 里的转角字段仅作等效参考） |
| 低速奇点处理 | `v → 0` 时 `k = ω / v` 发散，`|v|` 小于 `min_speed` 时锁存上一帧曲率，并在诊断话题里标记 |
| 倒车方向 | `v < 0` 时曲率自动反号，倒车打方向的方向正确 |
| 双级限幅 | 先按 `max_curvature` 截断曲率（路径本身太弯），再按 `max_steering` 截断转角（方向盘转不到那么大） |
| 加速度限制 | 速度按 `max_accel` 逐帧逼近目标，避免速度阶跃 |
| 转角速率限制 | 转角按 `max_steering_rate` 逐帧逼近目标，避免方向盘瞬移、指令抖动 |
| 指令超时保护 | `timeout` 秒收不到 `/cmd_vel` 时按零速度处理 |
| 内置运动学模拟 | `use_mock_odom:=true` 时自己积分车辆位姿并发 `/odom`，**没有 CARLA 也能跑通闭环** |
| 退出清零 | `rospy.on_shutdown` 与 `main()` 结尾各发一次零控制量 |

## 2. 环境与依赖

- 系统：Ubuntu 18.04 + ROS Melodic（Python 2.7 或 3.x 均可）
- 或：Ubuntu 20.04 + ROS Noetic（Python 3）
- 依赖：`rospy`、`geometry_msgs`、`std_msgs`、`nav_msgs`，均随 ROS 安装，**不需要额外装包**
- **不需要 CARLA、不需要 GPU、不需要显示器**

## 3. 安装与构建

```bash
mkdir -p ~/catkin_ws/src
cp -r src/ground/twist_to_control ~/catkin_ws/src/

cd ~/catkin_ws
source /opt/ros/melodic/setup.bash     # Noetic 换成 /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

纯 Python 包：`make` 阶段没有编译输出是正常的，`devel/lib/twist_to_control/` 为空也是正常的，
`roslaunch` 会直接从源码目录执行脚本。

Melodic 上如果提示找不到 `rospy`，说明脚本 shebang 指向了 `python3`：

```bash
sed -i '1s|.*|#!/usr/bin/env python2|' ~/catkin_ws/src/twist_to_control/scripts/*.py
chmod +x ~/catkin_ws/src/twist_to_control/scripts/*.py
```

## 4. 运行

每个终端都先执行：

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

```bash
# 终端 1：启动节点
roslaunch twist_to_control main.launch

# 终端 2：看控制量
rostopic echo /vehicle/control        # [前轮转角, 速度, 曲率]

# 终端 3：发速度指令
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'
```

命令行覆盖参数：

```bash
# 换成差速底盘，50Hz 发布
roslaunch twist_to_control main.launch model:=diff_drive track_width:=1.6 publish_hz:=50.0

# 开启内置运动学模拟：无需 CARLA 也能看到车按指令跑圆
roslaunch twist_to_control main.launch use_mock_odom:=true
```

## 5. 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `model` | `bicycle` | 底盘模型：`bicycle` / `ackermann` / `diff_drive` |
| `wheelbase` | `2.7` | 轴距 L（米） |
| `track_width` | `1.5` | 轮距 b（米） |
| `max_steering` | `0.55` | 前轮最大转角（弧度，约 31.5°） |
| `max_curvature` | `1.0` | 最大曲率（1/米），对应最小转弯半径 1 米 |
| `max_speed` | `5.0` | 最大速度（米/秒），正反向都限 |
| `max_accel` | `2.0` | 纵向加速度上限（米/秒²） |
| `max_steering_rate` | `2.0` | 转角变化率上限（弧度/秒） |
| `min_speed` | `0.05` | 曲率死区（米/秒） |
| `timeout` | `0.5` | `/cmd_vel` 超时（秒） |
| `publish_hz` | `20.0` | 控制量发布频率（赫兹） |
| `cmd_vel_topic` | `/cmd_vel` | 输入速度指令 |
| `control_topic` | `/vehicle/control` | 输出控制量 |
| `wheel_topic` | `/vehicle/wheel_speed` | 输出左右轮速（差速模型） |
| `info_topic` | `/vehicle/control_info` | 输出诊断量 |
| `odom_topic` | `/odom` | 内置模拟的里程计 |
| `use_mock_odom` | `false` | 是否启用内置模拟 |

## 6. 话题

| 话题 | 类型 | 方向 | 内容 |
|---|---|---|---|
| `cmd_vel_topic` | `geometry_msgs/Twist` | 订阅 | `linear.x` 线速度、`angular.z` 角速度 |
| `control_topic` | `std_msgs/Float64MultiArray` | 发布 | `[前轮转角(rad), 速度(m/s), 曲率(1/m)]` |
| `wheel_topic` | `std_msgs/Float64MultiArray` | 发布 | `[左轮速(m/s), 右轮速(m/s)]`，仅差速模型 |
| `info_topic` | `std_msgs/Float64MultiArray` | 发布 | `[请求转角, 实发转角, 是否饱和, 是否锁存, 是否速率受限]`，后三项 1 表示是 |
| `odom_topic` | `nav_msgs/Odometry` | 发布 | 内置模拟的位姿，仅 `use_mock_odom:=true` |

## 7. 三种底盘模型

| 模型 | 输出 | 适用 |
|---|---|---|
| `bicycle` | 一个前轮转角 | 四轮乘用车、Carla 默认车辆 |
| `ackermann` | 左右前轮各自的转角 | 需要真实转向梯形几何时；内侧轮转角必然大于外侧轮 |
| `diff_drive` | 左右轮线速度 | 差速小车、TurtleBot |

阿克曼几何（`k` 为曲率，`L` 轴距，`b` 轮距）：

```
δ_inner = atan(L·k / (1 - b·k/2))
δ_outer = atan(L·k / (1 + b·k/2))
```

差速分配（`ω = k·v`）：

```
v_left  = v - ω·b/2
v_right = v + ω·b/2
```

两者都满足"两轮平均等于车体速度、两轮之差等于 `ω·b`"。

## 8. 离线测试

不需要 ROS、不需要 CARLA、不需要显示器：

```bash
cd <仓库根目录>
python3 tests/test_twist_to_control_local.py
```

期望输出：

```
通过 230 项，失败 0 项
全部通过
```

## 9. 目录结构

```
twist_to_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── twist_to_control.yaml      # 全部默认参数
├── launch/
│   └── main.launch                # 17 个带 doc 的 <arg>，加载 yaml 后逐项覆盖
└── scripts/
    ├── main.py                    # 入口（仓库约定的 main.*）
    ├── twist_core.py              # 纯逻辑：换算、限幅、积分，不 import rospy
    └── twist_node.py              # ROS 收发与日志
```

## 10. 常见问题

**Q：`rostopic echo /vehicle/control` 一直是 0？**
A：节点默认只在收到 `/cmd_vel` 后才推进，且 `timeout`（默认 0.5 秒）后自动归零。
用 `rostopic pub -r 10` 持续发布即可，`-r 1` 会被超时判定为零。

**Q：转角一直是 0，但速度不为 0？**
A：说明 `angular.z` 为 0。曲率只由 `angular.z / linear.x` 决定。

**Q：低速时转角突然不动了？**
A：速度低于 `min_speed` 时曲率无意义，节点锁存上一帧曲率，`control_info` 的第 4 位为 1。
调小 `min_speed` 可以让死区更窄，但速度接近 0 时转角会变得非常敏感。

**Q：`roslaunch` 报 `is neither a launch file in package`？**
A：没 `source` 工作空间。新终端先 `source /opt/ros/melodic/setup.bash` 再 `source ~/catkin_ws/devel/setup.bash`。

## 11. 许可证

MIT。
