# 小海龟画正方形实验

## 一、实验目的与环境说明

### 1.1 实验目的

1. 掌握 ROS 话题通信机制：节点作为**发布者**向 `/turtle1/cmd_vel` 话题发布 `geometry_msgs/Twist` 速度消息；
2. 理解**时间开环控制**的原理：不依赖传感器反馈，利用 \(t = s / v\) 与 \(t = \theta / \omega\) 计算运动时长；
3. 通过 turtlesim 仿真器，让小海龟自主绘制一个边长 2 m 的正方形并回到起点。

### 1.2 实验环境

| 项目 | 配置 |
| ---- | ---- |
| 操作系统 | Ubuntu 20.04（VMware 虚拟机） |
| ROS 发行版 | ROS Noetic |
| 仿真器 | turtlesim |
| 编程语言 | Python 3（rospy） |
| 功能包 | turtle_motion |

功能包目录结构如下：

```text
turtle_motion/
├── package.xml        # 包清单，声明依赖 rospy 和 geometry_msgs
├── CMakeLists.txt     # 编译配置，把脚本安装到 bin 目录
└── scripts/
    └── square_draw.py # 主程序：控制小海龟画正方形的节点
```

## 二、核心控制原理与算法解析

### 2.1 话题通信机制

turtlesim 仿真器启动后，小海龟会一直订阅名为 `/turtle1/cmd_vel` 的话题。我们编写的节点 `turtle_square_node` 作为发布者，只要向该话题发布一条 `Twist` 消息，小海龟就会按照消息中指定的速度运动。

`Twist` 消息中本实验只用到两个分量：

- `linear.x`：线速度，控制海龟前进的快慢（单位 m/s）；
- `angular.z`：角速度，控制海龟转头的快慢（单位 rad/s），取正值绕 z 轴逆时针旋转，即**左转**。

**注意**：turtlesim 内置 0.5 秒看门狗——只要超过 0.5 秒没有收到新的 `cmd_vel` 消息，它就会自动把海龟速度清零（强制刹车）。所以速度指令不能只发一条就完事，必须**高频持续发布**：本实验用 `rospy.Rate(10)` 以 10 Hz 的频率不断重发当前指令，直到动作时间结束。

### 2.2 时间开环控制

本实验不订阅海龟的位姿话题（如 `/turtle1/pose`），属于**开环控制**：根据目标位移直接计算需要运动多长时间，时间一到就发零速度停机。

- 直行一条边：位移 \(s\)、线速度 \(v\)，则运动时间

$$t = \frac{s}{v}$$

  例如边长 \(s = 2\ \text{m}\)、速度 \(v = 1\ \text{m/s}\)，走 2 秒即可停下。
- 左转 90°：转角 \(\theta = \pi / 2\)、角速度 \(\omega\)，则运动时间

$$t = \frac{\theta}{\omega}$$

  例如角速度 \(\omega = 1\ \text{rad/s}\)，转 1.57 秒即可停下。

### 2.3 防惯性滑动的停机处理

海龟具有惯性，速度指令撤销后仍会滑行一段距离，导致边长不准、转角不是直角。因此：

1. 每次动作结束后，以 10 Hz 持续发布**全零速度**消息 `Twist()`，持续 0.5 秒，让海龟停下；
2. 这 0.5 秒同时充当两个动作之间的停顿，把残余惯性完全消掉，再进入下一个动作。

### 2.4 算法流程

1. 初始化节点 `turtle_square_node`，创建指向 `/turtle1/cmd_vel` 的发布者和 `rospy.Rate(10)` 频率控制对象；
2. 等待 1 秒，保证发布者与 turtlesim 订阅端建立连接，避免第一条消息丢失；
3. 循环 4 次，每轮执行：以 10 Hz 持续发布直行指令 2 秒（1.0 m/s × 2 s = 2 m）→ 持续发布空速度 0.5 秒 → 以 10 Hz 持续发布转弯指令 1.57 秒（1.0 rad/s × 1.57 s ≈ 90°）→ 持续发布空速度 0.5 秒；
4. 循环内外均通过 `rospy.is_shutdown()` 和 `try/except/finally` 做保护，节点被 `Ctrl+C` 中断时海龟也能安全停下。

## 三、完整源码展示

### 3.1 主程序 `scripts/square_draw.py`

> 最新源码同步保存在本书仓库根目录 `square_draw.py`，可直接查看与下载。

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 小海龟画正方形节点：turtle_square_node
# 思路：直走一条边 -> 停 0.5 秒 -> 左转 90 度 -> 停 0.5 秒，循环 4 次就是正方形
#
# 重点：turtlesim 内置 0.5 秒看门狗，一旦超过 0.5 秒没收到新的 cmd_vel 消息
# 就会强制把海龟刹停。如果只在动作开始时发一条速度指令，海龟会被中途刹车，
# 导致边长不足、转角不足 90 度。
# 因此本节点用 rospy.Rate(10) 以 10 Hz 高频持续发布速度指令，直到动作结束。

import rospy
from geometry_msgs.msg import Twist


class SquareDrawer:
    """控制小海龟画边长 2 m 的正方形"""

    def __init__(self):
        # 初始化节点
        rospy.init_node('turtle_square_node')
        # 发布者：把速度指令发到 /turtle1/cmd_vel 话题
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        # 10 Hz 高频持续发布，防止 turtlesim 0.5 秒超时强制刹车
        self.rate = rospy.Rate(10)

    def publish_for(self, twist, duration):
        """在 duration 秒内以 10 Hz 持续发布 twist，时间一到就结束"""
        end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.pub.publish(twist)
            self.rate.sleep()

    def stop(self, duration=0.5):
        """持续发布空速度 0.5 秒：让海龟停下，并把残余惯性消掉"""
        self.publish_for(Twist(), duration)

    def run(self):
        # 直行指令：线速度 1.0 m/s
        straight = Twist()
        straight.linear.x = 1.0
        # 转弯指令：角速度 1.0 rad/s，绕 z 轴逆时针旋转即左转
        turn = Twist()
        turn.angular.z = 1.0

        # 先等 1 秒，让发布者和 turtlesim 的订阅端连好，避免第一条消息丢失
        rospy.sleep(1.0)

        rospy.loginfo("开始画正方形，边长 2 米")
        for i in range(4):
            # 直行：1.0 m/s x 2 s = 2 m
            rospy.loginfo("正在画第 %d 条边", i + 1)
            self.publish_for(straight, 2.0)
            self.stop()

            # 左转 90 度：1.0 rad/s x 1.57 s ≈ π/2
            rospy.loginfo("正在转第 %d 个角", i + 1)
            self.publish_for(turn, 1.57)
            self.stop()

        rospy.loginfo("正方形画完啦！")


if __name__ == '__main__':
    node = SquareDrawer()
    try:
        node.run()
    except rospy.ROSInterruptException:
        # 按 Ctrl+C 或者节点被关掉的时候走到这里
        rospy.loginfo("节点被中断，让海龟停下来")
    finally:
        # 不管正常画完还是中间出错，最后都再发一次 0 速度，保险一点
        if not rospy.is_shutdown():
            node.pub.publish(Twist())
```

### 3.2 构建文件 `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(turtle_motion)

## 查找catkin和我们用到的依赖（rospy和geometry_msgs）
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

## 声明这个catkin包，因为包里面只有python代码，所以不用导出库
catkin_package()

## 把square_draw.py装到bin目录下，装好之后就可以用
## rosrun turtle_motion square_draw.py 直接运行了
catkin_install_python(PROGRAMS
  scripts/square_draw.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### 3.3 包清单 `package.xml`

```xml
<?xml version="1.0"?>
<!-- turtle_motion 功能包清单：小海龟自主绘制正方形轨迹 -->
<package format="2">
  <name>turtle_motion</name>
  <version>0.0.0</version>
  <description>turtle_motion: 小海龟自主绘制正方形轨迹功能包</description>
  <maintainer email="tianyutu981@todo.todo">tianyutu981</maintainer>
  <license>TODO</license>

  <!-- 构建工具依赖 -->
  <buildtool_depend>catkin</buildtool_depend>

  <!-- 构建依赖 -->
  <build_depend>rospy</build_depend>
  <build_depend>geometry_msgs</build_depend>

  <!-- 运行依赖 -->
  <exec_depend>rospy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>

  <export>
  </export>
</package>
```

## 四、运行与验证

### 4.1 编译功能包

把 `turtle_motion` 放到工作空间 `src` 目录下（不要放在共享目录里编译），然后：

```bash
cd ~/catkin_ws              # 进入工作空间根目录
catkin_make                 # 编译
source devel/setup.bash     # 刷新环境
```

### 4.2 启动节点

一共需要三个终端：

```bash
# 终端1：启动 ROS 主节点
roscore

# 终端2：启动小海龟仿真器
rosrun turtlesim turtlesim_node

# 终端3：运行画正方形节点
rosrun turtle_motion square_draw.py
```

### 4.3 话题订阅/发布验证

在节点运行的同时，另开终端依次执行：

```bash
rostopic list                        # 查看所有话题，确认 /turtle1/cmd_vel 存在
rostopic info /turtle1/cmd_vel       # 查看话题类型，应为 geometry_msgs/Twist
rostopic echo /turtle1/cmd_vel       # 实时打印发布者发出的速度消息
rosnode list                         # 确认 turtle_square_node 已注册
```

节点运行时 `rostopic echo` 的输出节选如下：直行阶段只有 `linear.x` 为 1.0，转弯阶段只有 `angular.z` 为 1.0，动作之间为全零停机消息。由于指令以 10 Hz 持续发布，同一动作期间会连续刷出内容相同的消息，直到动作切换。

```text
linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.0
---
```

### 4.4 预期结果

节点运行后，在 turtlesim 窗口中可以看到小海龟沿直线行走 2 m、左转 90°，重复 4 次后回到起点，绘制出一个边长 2 m 的正方形，终端同步打印 `正在画第 N 条边`、`正在转第 N 个角` 等日志，最后输出 `正方形画完啦！`。

## 五、总结

本实验通过编写发布者节点向 `/turtle1/cmd_vel` 话题发布 `Twist` 消息，实现了小海龟自主绘制正方形的任务。实验中掌握了 ROS 话题通信的基本用法，理解了时间开环控制的原理及其局限（海龟惯性带来的误差通过停机等待来抑制，turtlesim 0.5 秒看门狗的强制刹车则通过 10 Hz 高频持续发布来规避），并练习了 `rostopic` 等命令行工具的验证方法。后续可以把控制律替换为基于 `/turtle1/pose` 位姿反馈的闭环控制，使轨迹更加精确。
