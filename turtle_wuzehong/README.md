# git-ros-schoolwork
# ROS1 Kinetic — 小海龟 turtlesim 作业

功能：仿真小海龟自动转圈

## 操作步骤
一、虚拟机：安装 turtlesim

# 安装小海龟包
sudo apt update
sudo apt install ros-humble-turtlesim

验证是否装好
bash
运行
ros2 pkg list | grep turtlesim

二、创建工作空间 & 功能包
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 创建python功能包 turtle_circle
ros2 pkg create --build-type ament_python turtle_circle --dependencies rclpy geometry_msgs

三、写画圆代码
bash
运行
cd turtle_circle/turtle_circle
gedit circle.py

完整circle.py复制进去
python
运行
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleNode(Node):
    def __init__(self):
        super().__init__("circle_node")
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5     # 线速度
        msg.angular.z = 0.5    # 角速度，一起不为0就转圈
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CircleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

四、注册可执行程序（修改 setup.py）
打开setup.py，找到entry_points改成下面：
python
运行
entry_points={
    'console_scripts': [
        'circle = turtle_circle.circle:main',
    ],
},

五、编译 + 环境生效
cd ~/catkin_ws
colcon build
source install/setup.bash

六、运行测试（两个终端）
终端 1，启动海龟窗口
roscore
运行
ros2 run turtlesim turtlesim_node

终端 2，启动自己写的转圈程序

运行
ros2 run turtle_circle circle

此时小海龟不停转圈，在这里 Ubuntu 截图（虚拟机截图，不要手机拍屏幕）保存图片。
Ctrl+C 关闭程序。
