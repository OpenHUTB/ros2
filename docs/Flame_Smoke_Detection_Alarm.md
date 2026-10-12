# 火焰烟雾识别告警 — ROS2视觉目标追踪项目
## 项目简介
本项目基于ROS2 Humble与OpenCV实现火焰目标识别。视觉节点订阅相机图像话题，利用HSV颜色空间阈值分割识别火焰，检测到目标后在终端输出告警信息，完成视觉目标识别追踪任务。

## 实验环境
VMware虚拟机 Ubuntu22.04，ROS2 Humble，Python3，OpenCV，cv_bridge，numpy

## 项目文件结构
holoocean_static_camera_yolo_water_fire_detect/

├── holoocean_static_camera_yolo_water_fire_detect/

│   └── fire_detector_node.py

├── launch/
│   └── fire_detect.launch.py

├── package.xml

└── setup.py

## 核心代码
### 1. fire_detector_node.py
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class FireDetectorNode(Node):
    def __init__(self):
        super().__init__('fire_detector_node')
        self.bridge = CvBridge()
        # 订阅相机图像话题
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("火焰识别告警节点已启动，等待图像输入")

    def image_callback(self, msg):
        # ROS图像转为OpenCV图像
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # 火焰HSV阈值范围
        lower_red = np.array([0, 120, 120])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            self.get_logger().info("【告警】检测到火焰目标！")
        else:
            self.get_logger().info("未检测到火焰")

def main():
    rclpy.init()
    node = FireDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
### 2. fire_detect.launch.py
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='holoocean_static_camera_yolo_water_fire_detect',
            executable='fire_detector',
            name='fire_detector_node',
            output='screen'
        )
    ])
```
### 3. setup.py
```python
from setuptools import find_packages, setup

package_name = 'holoocean_static_camera_yolo_water_fire_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fire_detect.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='ROS2 fire detection vision node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fire_detector = holoocean_static_camera_yolo_water_fire_detect.fire_detector_node:main',
        ],
    },
)
```
### 编译与运行命令
```
# 进入工作空间
cd ~/fire_ws
# 编译
colcon build
# 加载环境变量
source install/setup.bash
# 启动项目
ros2 launch holoocean_static_camera_yolo_water_fire_detect fire_detect.launch.py
```
![运行效果图](https://raw.githubusercontent.com/Xavier-101666/ros2/11c73f8cc920d1bff009321f6b15126be377b854/你的图片文件名.png)
