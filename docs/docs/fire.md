# ROS2 火焰视觉检测实验
## 实验简介
本项目基于 ROS2 Humble + OpenCV，使用 HSV 颜色阈值方法完成火焰目标识别。订阅相机图像话题，识别到火焰后输出告警日志，实现简单的视觉目标检测功能。

## 实验环境
- 操作系统：Ubuntu 22.04
- ROS版本：ROS2 Humble
- Python版本：Python3.10
- 依赖库：OpenCV、cv_bridge、rclpy

## 项目文件结构
fire_detect/

├── fire_detect/

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
        self.get_logger().info("火焰识别告警节点已启动，等待图像输入")
        # 图像转换工具
        self.bridge = CvBridge()
        # 订阅相机图像话题
        self.sub_img = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )
        # 发布检测后标记图像
        self.pub_img = self.create_publisher(
            Image,
            "/fire_detect/image_result",
            10
        )

    def image_callback(self, msg):
        # ros图像转opencv图像
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # 火焰红色HSV两组阈值
        lower_red1 = np.array([0, 120, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 80])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # 轮廓查找
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        fire_detected = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100:  # 过滤小噪点
                fire_detected = True
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(cv_img, (x,y), (x+w,y+h), (0,0,255), 2)

        if fire_detected:
            self.get_logger().warn("【告警】检测到火焰目标！")

        # 转回ros图像发布
        out_msg = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
        self.pub_img.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FireDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("程序终止")
    finally:
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
    fire_node = Node(
        package="fire_detect",
        executable="fire_detector_node",
        output="screen"
    )
    return LaunchDescription([fire_node])

```
### 3. setup.py
```python
from setuptools import setup

package_name = 'fire_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fire_detect.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2火焰视觉检测实验',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fire_detector_node = fire_detect.fire_detector_node:main',
        ],
    },
)

```
### 4·package.xml
```python
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>fire_detect</name>
  <version>0.0.0</version>
  <description>火焰视觉检测实验 ROS2 功能包</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>opencv-python</depend>

  <test_depend>ament_pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

### 编译与运行命令
```
# 进入工作空间
cd ~/ros2_ws
# 编译
colcon build
# 加载环境变量
source install/setup.bash

# 方式1：launch文件启动（推荐）
ros2 launch fire_detect fire_detect.launch.py

# 方式2：直接运行节点
ros2 run fire_detect fire_detector_node
```
![运行效果图](https://raw.githubusercontent.com/Xavier-101666/ros2/11c73f8cc920d1bff009321f6b15126be377b854/你的图片文件名.png)
