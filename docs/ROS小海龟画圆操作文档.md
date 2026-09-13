# ROS 小海龟画圆操作文档

## 一、实验目的

1. 熟悉 ROS 的基本操作，掌握 `roscore`、`rosrun`、`rostopic` 等常用命令；
2. 理解 ROS 话题（Topic）的发布与订阅机制；
3. 通过向 `/turtle1/cmd_vel` 话题发布速度控制消息，控制小海龟（turtlesim）画出一个圆形轨迹。

## 二、实验环境

| 项目 | 配置 |
| --- | --- |
| 操作系统 | Ubuntu（Linux） |
| ROS 版本 | ROS Noetic（ros_comm 1.17.4） |
| 仿真工具 | turtlesim 小海龟仿真器 |
| 主机名 | ubuntu（ROS_MASTER_URI=http://ubuntu:11311/） |
| 终端工具 | 系统终端（需打开 3 个终端窗口） |

## 三、实验原理

### 3.1 话题通信机制

`turtlesim_node` 节点启动后会订阅 `/turtle1/cmd_vel` 话题，该话题的消息类型为 `geometry_msgs/Twist`，用于接收小海龟的运动速度指令。我们通过 `rostopic pub` 命令周期性地向该话题发布速度消息，即可控制小海龟运动。

`geometry_msgs/Twist` 消息包含线速度和角速度两部分：

```yaml
linear:    # 线速度 (m/s)
  x: 0.0
  y: 0.0
  z: 0.0
angular:   # 角速度 (rad/s)
  x: 0.0
  y: 0.0
  z: 0.0
```

小海龟在平面内运动，只需使用 `linear.x`（前进线速度 v）和 `angular.z`（绕 z 轴旋转角速度 ω），其余分量保持为 0。

### 3.2 画圆原理

当小海龟同时具有恒定的前进线速度 v 和恒定的转向角速度 ω 时，其运动轨迹为一个圆。圆周运动满足：

$$
v = \omega r \quad\Longrightarrow\quad r = \frac{v}{\omega}
$$

其中：

- v 为线速度 `linear.x`，本实验取 **2.0 m/s**；
- ω 为角速度 `angular.z`，本实验取 **1.8 rad/s**；
- r 为所画圆的半径。

因此本实验画出的圆半径为：

$$
r = \frac{2.0}{1.8} \approx 1.11 \ \text{m}
$$

走完一整圈所需时间（周期）为：

$$
T = \frac{2\pi}{\omega} = \frac{2\pi}{1.8} \approx 3.49 \ \text{s}
$$

> 说明：线速度决定圆的大小和运动快慢，角速度决定转向快慢；两者比值恒定时轨迹为圆，改变 v 或 ω 的数值即可改变圆的半径。

## 四、操作步骤

### 步骤 1：启动 ROS 主节点 roscore（终端 1）

打开第一个终端，输入：

```bash
roscore
```

正常启动后终端会显示如下关键信息，表示 ROS Master 已在 11311 端口运行：

```text
started roslaunch server http://ubuntu:46293/
ros_comm version 1.17.4

SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.17.4

NODES

auto-starting new master
process[master]: started with pid [2660]
ROS_MASTER_URI=http://ubuntu:11311/
...
started core service [/rosout]
```

> 注意：该终端在实验过程中需保持运行，不能关闭。

### 步骤 2：启动小海龟仿真节点（终端 2）

新开第二个终端，输入：

```bash
rosrun turtlesim turtlesim_node
```

执行后会弹出蓝色背景的 **TurtleSim** 仿真窗口，终端显示小海龟的初始位姿（初始位置约为 x=5.544445, y=5.544445，朝向 theta=0）：

```text
[INFO]: Starting turtlesim with node name /turtlesim
[INFO]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
```

### 步骤 3：发布速度指令控制小海龟画圆（终端 3）

再新开第三个终端，输入以下命令，以 1Hz 的频率持续向 `/turtle1/cmd_vel` 话题发布速度消息：

```bash
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- 'linear: {x: 2.0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 1.8}'
```

命令参数说明：

| 参数 | 含义 |
| --- | --- |
| `pub` | 表示发布话题消息 |
| `/turtle1/cmd_vel` | 小海龟速度控制话题名 |
| `geometry_msgs/Twist` | 速度消息类型 |
| `-r 1` | 以 1Hz 的频率（每秒 1 次）循环发布 |
| `--` | 其后为 YAML 格式的消息内容 |
| `linear.x: 2.0` | 前进线速度 2.0 m/s |
| `angular.z: 1.8` | 逆时针角速度 1.8 rad/s |

命令执行后，即可在 TurtleSim 窗口中看到小海龟一边前进一边匀速转向，约 3.5 秒画出一个圆，并持续沿圆形轨迹运动。

### 步骤 4：停止运动

在终端 3 中按 `Ctrl + C` 停止发布速度消息，小海龟随即停止运动；如需结束实验，依次在终端 2、终端 1 中按 `Ctrl + C` 关闭仿真节点和 roscore 即可。

## 五、实验结果

小海龟在 TurtleSim 窗口中以初始位置为参考，画出了一个半径约为 1.11m 的光滑圆形轨迹，实验现象与理论分析一致，验证了以下结论：

1. 通过 `rostopic pub` 可以向指定话题发布消息，实现对节点的控制；
2. 当线速度与角速度均为恒定值且不为零时，小海龟做匀速圆周运动；
3. 圆的半径由线速度与角速度的比值决定，即 r = v / ω。

（此处插入实验运行截图）

![运行效果](截图.png)

## 六、拓展：调整圆的大小

修改 `linear.x` 与 `angular.z` 的比值即可改变圆的半径，例如：

```bash
# 半径更小的圆：r = 1.0 / 2.0 = 0.5 m
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- 'linear: {x: 1.0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 2.0}'

# 半径更大的圆：r = 3.0 / 1.0 = 3.0 m
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- 'linear: {x: 3.0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 1.0}'
```

将 `angular.z` 改为负值（如 -1.8），小海龟将沿**顺时针**方向画圆。

## 七、常见问题

1. **执行 `rosrun` 提示找不到 turtlesim 包**：先执行 `sudo apt install ros-noetic-turtlesim` 安装，并确认已执行 `source /opt/ros/noetic/setup.bash`。
2. **小海龟不动**：检查终端 1 的 `roscore` 是否仍在运行，话题名和消息类型是否拼写正确。
3. **轨迹不是闭合的圆**：确认发布时使用了 `-r` 参数持续发布（而非只发布一次），并保持线速度、角速度恒定。
4. **海龟走出窗口**：说明圆半径过大，适当减小 `linear.x` 或增大 `angular.z` 即可。
