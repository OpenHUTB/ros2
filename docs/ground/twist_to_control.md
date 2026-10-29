# 扭转控制

## 参考

* [Carla 扭转控制](https://openhutb.github.io/doc/carla_twist_to_control/)

## 这一页解决什么问题

`/cmd_vel` 上的 `geometry_msgs/Twist` 是**上层规划**给出的意图：想让车以多快前进、
以多大角速度转弯。但车本身并不认识"角速度"这个量——方向盘只能转到某个角度，
油门只能给到某个速度，差速底盘只能控制左右轮各转多快。

于是中间缺一层换算：

```
规划层            Twists（v, ω）        "我想怎么走"
   │
   ├── 本页实现的转换 ──┐
   │                    │
执行层            转向角 δ、轮速 v_l / v_r   "车实际怎么动"
```

缺了这一层会发生什么：把 `Twist` 直接发给一辆阿克曼底盘的车，`angular.z` 会被忽略，
车只会直着开；即使某些控制器接受 `Twist`，它在低速时也会因为 `ω / v` 发散而输出
离谱的转角。本页给出的实现就是把这一层补齐。

## 自主实现：`twist_to_control` 包

对应源码：[`src/ground/twist_to_control`](https://github.com/OpenHUTB/ros2/tree/master/src/ground/twist_to_control)。

### 运动学公式

把车沿后轴看成一个质点，前后轴之间距离为轴距 `L`，前轮转角为 `δ`，
则车体角速度与速度的关系是：

```
ω = v · tan(δ) / L
```

反过来，已知规划层给出的 `(v, ω)`，先求曲率 `k`，再求转角：

```
k = ω / v
δ = atan(k · L)
```

曲率 `k` 的倒数就是转弯半径：`k = 0.5` 表示半径 2 米。这个量比转角更本质——
它把"速度多快、转多急"两个量合并成了一个几何量，所以本实现内部始终以曲率为中心，
不同的底盘模型只是在曲率之后分岔：

| 底盘模型 | 由曲率到执行量的换算 |
|---|---|
| `bicycle` | `δ = atan(k · L)`，输出一个前轮转角 |
| `ackermann` | `δ_inner = atan(L·k/(1 - b·k/2))`、`δ_outer = atan(L·k/(1 + b·k/2))` |
| `diff_drive` | `v_left = v - ω·b/2`、`v_right = v + ω·b/2` |

`diff_drive` 没有方向盘，`/vehicle/control` 里的转角字段仍按等效单车模型给出（方便与另
两种模型对照），真正执行请看 `/vehicle/wheel_speed`。

阿克曼几何中内侧轮必然比外侧轮转得更多（转弯半径更小），差速分配中转弯外侧轮
必然比内侧轮转得更快。这两个关系都写进了回归测试。

### 四个容易踩的坑，以及本实现的处理

**1. 低速奇点。** `k = ω / v` 在 `v → 0` 时发散。速度 0.01 m/s、角速度 0.5 rad/s
就解出曲率 50（半径 2 厘米），任何实车都做不到。

处理：设死区 `min_speed`（默认 0.05 m/s），`|v|` 低于它时**锁存上一帧曲率**，
并在诊断话题里把"锁存"标志置 1。这比直接输出 0 更合理——车在缓慢挪动时，
方向盘应该保持刚才的角度，而不是突然回正。

**2. 倒车方向。** `v` 取负值时 `k = ω / v` 自动反号，因此转角方向自动正确。
这一点不需要特殊分支，但很容易被"先取绝对值再算"的写法破坏，所以测试里专门
校验了倒车时转角为负。

**3. 双级限幅。** 一次超限可能是两种完全不同的原因：路径本身太弯（曲率超限），
或者方向盘转不到那么大（转角超限）。本实现分两级截断——先按 `max_curvature`
截断曲率，再按 `max_steering` 截断转角，`control_info` 的"饱和"位合并上报两者。

**4. 指令抖动。** 相邻两帧 `Twist` 稍有差异，转角就会跳变，实车表现为方向盘来回摆。
本实现同时限制加速度（`max_accel`）与转角变化率（`max_steering_rate`），
让执行量逐帧逼近目标，`control_info` 的"速率受限"位标记当前是否还在追赶。

### 参数

所有参数都在 `config/twist_to_control.yaml` 里给出默认值，并由 `launch/main.launch`
用 17 个带 `doc` 的 `<arg>` 暴露，可以命令行覆盖。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `model` | `bicycle` | 底盘模型：`bicycle` / `ackermann` / `diff_drive` |
| `wheelbase` | `2.7` | 轴距 L（米） |
| `track_width` | `1.5` | 轮距 b（米） |
| `max_steering` | `0.55` | 前轮最大转角（弧度，约 31.5°） |
| `max_curvature` | `1.0` | 最大曲率（1/m），对应最小转弯半径 1 米 |
| `max_speed` | `5.0` | 最大速度（m/s），正反向都限 |
| `max_accel` | `2.0` | 纵向加速度上限（m/s²） |
| `max_steering_rate` | `2.0` | 转角变化率上限（rad/s） |
| `min_speed` | `0.05` | 曲率死区（m/s） |
| `timeout` | `0.5` | `/cmd_vel` 超时（秒） |
| `publish_hz` | `20.0` | 发布频率（Hz） |
| `use_mock_odom` | `false` | 是否启用内置运动学模拟 |

### 话题

| 话题 | 类型 | 方向 | 内容 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅 | `linear.x`、`angular.z` |
| `/vehicle/control` | `std_msgs/Float64MultiArray` | 发布 | `[转角, 速度, 曲率]` |
| `/vehicle/wheel_speed` | `std_msgs/Float64MultiArray` | 发布 | `[左轮速, 右轮速]`，仅差速模型 |
| `/vehicle/control_info` | `std_msgs/Float64MultiArray` | 发布 | `[请求转角, 实发转角, 饱和, 锁存, 速率受限]` |
| `/odom` | `nav_msgs/Odometry` | 发布 | 内置模拟位姿，仅 `use_mock_odom:=true` |

### 运行步骤

```bash
# 构建
cp -r src/ground/twist_to_control ~/catkin_ws/src/
cd ~/catkin_ws && source /opt/ros/melodic/setup.bash && catkin_make
source devel/setup.bash

# 终端 1
roslaunch twist_to_control main.launch

# 终端 2：发指令（持续发布，低于 timeout 的频率会被判为零速度）
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'

# 终端 3：看输出
rostopic echo /vehicle/control
```

速度 1.0 m/s、角速度 0.5 rad/s、轴距 2.7 m 时，稳态曲率为 0.5、转角约为 0.933 rad
（超过默认的 `max_steering` = 0.55，因此会被限幅并上报饱和）。把角速度降到 0.2
即可得到约 0.495 rad 的转角，落在限幅之内。

想看车真的按指令跑起来，不需要 CARLA：

```bash
roslaunch twist_to_control main.launch use_mock_odom:=true
```

节点会自己按运动学积分位姿并发布 `/odom`，`rostopic echo /odom` 能看到
位置沿圆弧推进。

### 离线回归测试

```bash
cd <仓库根目录>
python3 tests/test_twist_to_control_local.py
```

测试只用标准库，覆盖参数默认值与读取、曲率换算（含倒车与死区）、转角与双级限幅、
阿克曼内外轮关系、差速轮速守恒、逐帧推进（加速度限制、转角速率限制、锁存、饱和）、
内置模拟（直线、定圆、倒车、`dt = 0`）、四元数往返，以及工程文件约定
（launch 入口、yaml 键名、核心逻辑不依赖 rospy 等）。

### 与已有模块的关系

本包与 `vehicle_pygame_teleop`、`waypoint_target_switcher` 是同一条链路的相邻环节：

```
vehicle_pygame_teleop   键盘 ──→ /cmd_vel ┐
waypoint_target_switcher 路径点 ──→ /cmd_vel ┤
                                                 ├──→ twist_to_control ──→ 转角 / 轮速
```

前两个模块负责**产生** `Twist`，本包负责把 `Twist` **翻译**成执行量，三者可以串起来跑，
也可以单独使用。本包不修改上述任一模块。

## 常见问题

**Q：`/vehicle/control` 一直是 0？**
A：`/cmd_vel` 需要持续发布。节点在 `timeout`（默认 0.5 s）收不到指令时按零速度处理，
所以 `rostopic pub -r 1` 会被判为零速。

**Q：转角一直是 0，但速度不为 0？**
A：`angular.z` 为 0。曲率只由 `angular.z / linear.x` 决定。

**Q：低速时转角不动？**
A：`|v|` 低于 `min_speed` 时进入死区锁存，`control_info` 第 4 位为 1。

**Q：`is neither a launch file in package`？**
A：新终端没 source 工作空间，先 `/opt/ros/melodic/setup.bash` 再 `devel/setup.bash`。

## 说明

本模块使用大模型辅助编写代码、测试与文档，提交内容由本人负责。
