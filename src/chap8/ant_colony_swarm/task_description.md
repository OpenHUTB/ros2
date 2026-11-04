# ant_colony_swarm 功能包：基于机器视觉的多机器人蚁群编队
## 概述
ant_colony_swarm是ROS2 Python功能包，用于多机器人领航跟随蚁群编队，通过机载相机视觉识别领航机器人标记，实现编队协同运动。

## 目录结构
src/ant_colony_swarm

├── ant_colony_swarm/

│   ├── __init__.py

│   ├── vision_detect_node.py

│   └── swarm_control_node.py

├── launch/

│   └── swarm_bringup.launch.py

├── resource/

│   └── ant_colony_swarm

├── package.xml

├── setup.py

└── task_description.md

## 环境依赖
- ROS2 Humble
- opencv-python
- cv_bridge
- numpy

---

## 1. vision_detect_node.py 
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionDetectNode(Node):
    def __init__(self):
        super().__init__('vision_detect_node')
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(
            Image, '/camera/image_raw', self.img_callback, 10
        )
        self.pub_target = self.create_publisher(Pose2D, '/leader_pose', 10)
        self.get_logger().info("视觉检测节点启动")

    def img_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        # 红色标记阈值，领航机器人贴红色标识
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 1e-6:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]
                msg_pose = Pose2D()
                msg_pose.x = cx
                msg_pose.y = cy
                msg_pose.theta = 0.0
                self.pub_target.publish(msg_pose)

def main(args=None):
    rclpy.init(args=args)
    node = VisionDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
## 2. swarm_control_node.py
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
import math

class SwarmControlNode(Node):
    def __init__(self):
        super().__init__('swarm_control_node')
        self.sub_target = self.create_subscription(
            Pose2D, '/leader_pose', self.target_callback, 10
        )
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.img_w = 640
        self.kp_ang = 0.002
        self.kp_lin = 0.001
        self.get_logger().info("蚁群编队控制节点启动")

    def target_callback(self, msg: Pose2D):
        twist = Twist()
        err_x = msg.x - self.img_w / 2
        # 角度偏差控制
        twist.angular.z = self.kp_ang * err_x
        # 距离控制，越靠近领航者速度越小
        dist = math.hypot(err_x, msg.y - 320)
        twist.linear.x = self.kp_lin * dist
        self.pub_cmd.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SwarmControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
## 3. swarm_bringup.launch.py
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vision_node = Node(
        package="ant_colony_swarm",
        executable="vision_detector",
        name="vision_detector"
    )
    control_node = Node(
        package="ant_colony_swarm",
        executable="swarm_control",
        name="swarm_control"
    )
    return LaunchDescription([
        vision_node,
        control_node
    ])
```
## 4. package.xml
```python
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ant_colony_swarm</name>
  <version>0.0.0</version>
  <description>基于机器视觉的多机器人蚁群编队仿真</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>opencv-python</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```
## 5. setup.py
```python
import os
from setuptools import find_packages, setup

package_name = 'ant_colony_swarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            [os.path.join('launch', 'swarm_bringup.launch.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='基于机器视觉的多机器人蚁群编队仿真',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_detector = ant_colony_swarm.vision_detect_node:main',
            'swarm_control = ant_colony_swarm.swarm_control_node:main',
        ],
    },
)
```
### 编译与运行命令
# 编译
colcon build --packages-select ant_colony_swarm
# 刷新环境
source install/setup.bash
# 一键启动
ros2 launch ant_colony_swarm swarm_bringup.launch.py

## 运行效果图
![运行效果图]Vision Detection Node Startup Log.png
https://github.com/Xavier-101666/ros2/blob/ant/src/chap8/Vision%20Detection%20Node%20Startup%20Log.png
![运行效果图]Visual Perception for Multi-Robot Ant Colony Formation.png
https://github.com/Xavier-101666/ros2/blob/ant/src/chap8/Visual%20Perception%20for%20Multi-Robot%20Ant%20Colony%20Formation.png
