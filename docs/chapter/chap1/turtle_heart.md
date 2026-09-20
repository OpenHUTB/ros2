\# 小海龟画爱心实验



\## 一、实验目的与环境说明



\### 1.1 实验目的

1\. 掌握 ROS 话题通信机制。节点作为发布者（Publisher）发布 `/turtle1/cmd\_vel` 话题，发送 `geometry\_msgs/Twist` 消息控制海龟运动；同时作为订阅者（Subscriber）订阅 `/turtle1/pose` 话题，接收海龟的位姿消息。

2\. 掌握利用参数方程生成轨迹点的方法。利用爱心参数方程生成一系列密集的坐标点，控制小海龟依次跟踪这些点，从而绘制出爱心图案。

3\. 通过 turtlesim 仿真器，体会闭环反馈控制中距离与角度误差的计算与修正。



\### 1.2 实验环境

| 配置项 | 版本/说明 |

| :--- | :--- |

| 操作系统 | Ubuntu 20.04 (VMware虚拟机) |

| ROS 版本 | ROS Noetic |

| 仿真器 | turtlesim |

| 编程语言 | Python 3.8 |

| 功能包 | turtle\_heart |



\*\*功能包目录结构如下：\*\*

```text

turtle\_heart/

├── package.xml             # 包清单

├── CMakeLists.txt          # 构建文件

├── scripts/

│   └── main.py             # 主程序：控制小海龟绘制爱心

└── README.md               # 运行环境和步骤说明

```



注意：文档的 Markdown 文件保存在 docs/chap1/turtle\_heart.md 中。



二、核心控制原理与算法解析



2.1 话题通信机制



turtlesim 仿真器启动后，小海龟会自动订阅 /turtle1/cmd\_vel 话题。我们编写的节点脚本作为发布者向该话题发送 Twist 消息控制海龟运动，同时通过 /turtle1/pose 话题接收自身位姿（包含 x, y 坐标和 theta 朝向角）。本节点订阅该话题，实时读取当前位置。



Twist 消息中本实验用到的两个分量：



· linear.x：线速度，控制海龟前进的快慢（单位：m/s）

· angular.z：角速度，控制海龟转动的快慢（单位：rad/s）



Pose 消息中用到三个分量：



· x, y：海龟在平面上的坐标（单位：m）

· theta：海龟的朝向角（单位：rad）



2.2 爱心参数方程与轨迹跟踪



与画正方形走直线和转固定角度不同，爱心是一个复杂的曲线。本实验采用心形参数方程生成轨迹点：

x = 16 \* sin^3(t)

y = 13 \* cos(t) - 5 \* cos(2t) - 2 \* cos(3t) - cos(4t)



其中 t 的取值范围是 \[0, 2\\pi]。代码中均匀采样 60 个点。为了适配 turtlesim 的 11.08×11.08 画布边界，对计算出的坐标进行了缩放（乘 0.25）和偏移（加 5.5），确保爱心能居中显示且不撞墙。



闭环跟踪控制：

对于每个目标点 (x\_{target}, y\_{target})，计算当前点 (x\_{cur}, y\_{cur}) 的距离：

d = \\sqrt{(x\_{target} - x\_{cur})^2 + (y\_{target} - y\_{cur})^2}

当 d < 0.15 时，认为到达目标点，发布 0 速度停止海龟，然后转向下一个点。



角度控制：

计算目标点相对于当前点的期望角度 \\theta\_{target} = \\text{atan2}(y\_{target} - y\_{cur}, x\_{target} - x\_{cur})。

通过将角速度 angular.z 设为 4.0 \\times (\\theta\_{target} - \\theta\_{cur})，使得海龟平滑地转向目标点。



2.3 动作间隙的稳定处理



由于小海龟具有物理惯性，每次到达一个目标点后，需要发布一次零速度指令，并 rospy.sleep(0.05) 短暂停歇，让海龟的动能消耗完毕，消除运动惯性带来的位置偏差，然后再向下一目标点移动。



2.4 算法流程



1\. 初始化节点，创建发布者发布 /turtle1/cmd\_vel，创建订阅者订阅 /turtle1/pose，设置控制频率为 10Hz。

2\. 等待第一帧位姿数据，确保连接建立。

3\. 根据爱心参数方程，计算并存储 61 个目标坐标点。

4\. 遍历目标点列表：

&#x20;  · 计算当前位置与目标点的距离 d。

&#x20;  · 如果 d < 0.15，停止并等待 0.05s，切换到下一个目标点。

&#x20;  · 计算角度差，将角度归一化到 \[-\\pi, \\pi]，发布线速度与角速度指令。

5\. 循环结束后，发布停止指令，绘制完成。



三、完整源码展示



3.1 主程序 scripts/main.py



```python

\#!/usr/bin/env python3

import rospy

from geometry\_msgs.msg import Twist

from turtlesim.msg import Pose

import math

import time



current\_pose = None



def pose\_callback(msg):

&#x20;   global current\_pose

&#x20;   current\_pose = msg



class TurtleDrawer:

&#x20;   def \_\_init\_\_(self):

&#x20;       rospy.init\_node('draw\_heart', anonymous=True)

&#x20;       self.pub = rospy.Publisher('/turtle1/cmd\_vel', Twist, queue\_size=10)

&#x20;       rospy.Subscriber('/turtle1/pose', Pose, pose\_callback)

&#x20;       self.rate = rospy.Rate(10) # 10Hz

&#x20;       time.sleep(2)



&#x20;   def move\_to(self, target\_x, target\_y):

&#x20;       """让海龟平滑移动到目标点"""

&#x20;       global current\_pose

&#x20;       while not rospy.is\_shutdown():

&#x20;           if current\_pose is None:

&#x20;               continue

&#x20;           

&#x20;           dx = target\_x - current\_pose.x

&#x20;           dy = target\_y - current\_pose.y

&#x20;           distance = math.sqrt(dx\*\*2 + dy\*\*2)



&#x20;           # 距离小于 0.15 就算到达

&#x20;           if distance < 0.15:

&#x20;               self.stop()

&#x20;               break



&#x20;           target\_theta = math.atan2(dy, dx)

&#x20;           angle\_diff = target\_theta - current\_pose.theta

&#x20;           

&#x20;           # 角度归一化到 \[-pi, pi]

&#x20;           while angle\_diff > math.pi: angle\_diff -= 2 \* math.pi

&#x20;           while angle\_diff < -math.pi: angle\_diff += 2 \* math.pi



&#x20;           move\_cmd = Twist()

&#x20;           # 降低线速度，防止冲过头

&#x20;           move\_cmd.linear.x = min(1.0, 0.8 \* distance)

&#x20;           # 提高转向灵敏度

&#x20;           move\_cmd.angular.z = 4.0 \* angle\_diff 



&#x20;           self.pub.publish(move\_cmd)

&#x20;           self.rate.sleep()



&#x20;   def stop(self):

&#x20;       """停止海龟"""

&#x20;       move\_cmd = Twist()

&#x20;       move\_cmd.linear.x = 0.0

&#x20;       move\_cmd.angular.z = 0.0

&#x20;       self.pub.publish(move\_cmd)

&#x20;       time.sleep(0.05)



&#x20;   def draw\_heart(self):

&#x20;       rospy.loginfo("开始绘制爱心...")

&#x20;       

&#x20;       points = \[]

&#x20;       for i in range(61):

&#x20;           t = i \* (2 \* math.pi / 60)

&#x20;           

&#x20;           # 爱心参数方程

&#x20;           x = 16 \* math.sin(t)\*\*3

&#x20;           y = 13 \* math.cos(t) - 5 \* math.cos(2\*t) - 2 \* math.cos(3\*t) - math.cos(4\*t)

&#x20;           

&#x20;           # 缩放和平移，以适应 turtlesim 边界

&#x20;           scaled\_x = max(0.5, min(10.5, x \* 0.25 + 5.5))

&#x20;           scaled\_y = max(0.5, min(10.5, y \* 0.25 + 5.5))

&#x20;           

&#x20;           points.append((scaled\_x, scaled\_y))



&#x20;       for point in points:

&#x20;           rospy.loginfo(f"移动到: {point}")

&#x20;           self.move\_to(point\[0], point\[1])

&#x20;           time.sleep(0.05)



&#x20;       rospy.loginfo("爱心绘制完成！")



if \_\_name\_\_ == '\_\_main\_\_':

&#x20;   try:

&#x20;       drawer = TurtleDrawer()

&#x20;       drawer.draw\_heart()

&#x20;   except rospy.ROSInterruptException:

&#x20;       pass

```



3.2 构建文件 CMakeLists.txt



```cmake

cmake\_minimum\_required(VERSION 3.0.2)

project(turtle\_heart)



find\_package(catkin REQUIRED COMPONENTS

&#x20; rospy

&#x20; geometry\_msgs

&#x20; turtlesim

)



catkin\_package()



catkin\_install\_python(PROGRAMS

&#x20; scripts/main.py

&#x20; DESTINATION ${CATKIN\_PACKAGE\_BIN\_DESTINATION}

)

```



3.3 包清单 package.xml



```xml

<?xml version="1.0"?>

<package format="2">

&#x20; <name>turtle\_heart</name>

&#x20; <version>0.0.0</version>

&#x20; <description>The turtle\_heart package</description>

&#x20; <maintainer email="liming@todo.todo">liming</maintainer>

&#x20; <license>TODO</license>



&#x20; <buildtool\_depend>catkin</buildtool\_depend>

&#x20; <build\_depend>rospy</build\_depend>

&#x20; <build\_depend>geometry\_msgs</build\_depend>

&#x20; <build\_depend>turtlesim</build\_depend>



&#x20; <exec\_depend>rospy</exec\_depend>

&#x20; <exec\_depend>geometry\_msgs</exec\_depend>

&#x20; <exec\_depend>turtlesim</exec\_depend>

</package>

```



四、运行与验证



4.1 编译功能包



在工作空间根目录下运行：



```bash

cd \~/catkin\_ws

catkin\_make

source devel/setup.bash

```



4.2 启动节点



需要打开三个终端，分别运行：



1\. 终端 1 (ROS 核心)：

&#x20;  ```bash

&#x20;  roscore

&#x20;  ```

2\. 终端 2 (仿真器)：

&#x20;  ```bash

&#x20;  rosrun turtlesim turtlesim\_node

&#x20;  ```

3\. 终端 3 (运行画爱心脚本)：

&#x20;  ```bash

&#x20;  rosrun turtle\_heart main.py

&#x20;  ```



4.3 话题订阅与验证



在节点运行时，另开终端依次执行：



```bash

rostopic list

rostopic info /turtle1/cmd\_vel

rostopic echo /turtle1/pose

```



预期可以看到 Twist 线速度 linear.x 在 0.0-1.0 之间不断变化，角速度 angular.z 频繁调整，表明程序正在实时计算并发布速度指令。



4.4 预期结果



节点启动后，在 turtle\_win 窗口中可以看到小海龟先移动至起始点，然后平滑地画出一个完整的爱心图案。最后海龟停在爱心上方。



![小海龟画爱心运行效果](images/turtle_heart.png)



五、总结



本实验通过发布 /turtle1/cmd\_vel 话题实现了对 turtlesim 小海龟的速度控制，并通过订阅 /turtle1/pose 位姿话题实现了闭环反馈。相较于走直线的几何图形，画爱心需要处理复杂的参数方程轨迹点。通过计算距离与角度差，结合速度比例控制，成功完成了爱心图案的绘制。实验深化了对 ROS 话题通信机制和闭环控制算法的理解。

