# water 功能包：水下ROV声呐图像CNN目标识别
## 概述
water是ROS2 Python功能包，用于水下ROV侧扫声呐图像目标识别，基于CNN卷积神经网络，对Gazebo仿真相机输出图像进行推理分类。

## 目录结构
src/water

├── water/

│   ├── __init__.py

│   └── sonar_infer_node.py

├── launch/

│   └── rov.launch.py

├── resource/

│   └── water

├── package.xml

└── setup.py

## 环境依赖
- ROS2 Humble
- opencv-python
- pytorch
- cv_bridge

### 核心代码
## 1. sonar_infer_node.py
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import torch.nn as nn
import torch.nn.functional as F

# ---------------------- CNN模型定义 ----------------------
class SonarCNN(nn.Module):
    def __init__(self, num_classes=3):
        super().__init__()
        self.conv1 = nn.Conv2d(1, 16, 3, padding=1)
        self.conv2 = nn.Conv2d(16, 32, 3, padding=1)
        self.pool = nn.MaxPool2d(2,2)
        self.fc1 = nn.Linear(32 * 32 * 32, 128)
        self.fc2 = nn.Linear(128, num_classes)

    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = torch.flatten(x, 1)
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

# 类别名称，可根据你的数据集修改
CLASS_NAMES = ["rock", "fish", "wreck"]

# ---------------------- ROS2节点 ----------------------
class SonarInferNode(Node):
    def __init__(self):
        super().__init__('sonar_infer_node')
        self.bridge = CvBridge()

        # 加载CNN模型
        self.model = SonarCNN(num_classes=3)
        self.model.eval()
        self.get_logger().info("Sonar CNN model loaded")

        # 订阅Gazebo相机图像话题
        self.sub_img = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("Sonar inference node started, waiting for image...")

    def image_callback(self, msg):
        try:
            # ROS图像转OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # 转为灰度图（声呐图像是单通道灰度）
            gray_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            # 缩放到模型输入尺寸
            resized_img = cv2.resize(gray_img, (128,128))

            # 图像转tensor，适配网络输入 [batch, channel, H, W]
            tensor_img = torch.from_numpy(resized_img).float().unsqueeze(0).unsqueeze(0) / 255.0

            # CNN推理
            with torch.no_grad():
                out = self.model(tensor_img)
                pred_idx = torch.argmax(out, dim=1).item()
                pred_class = CLASS_NAMES[pred_idx]

            # 在ROS日志打印识别结果
            self.get_logger().info(f"【识别结果】目标类别: {pred_class}")

            # 可视化窗口（可选）
            cv2.putText(cv_img, pred_class, (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
            cv2.imshow("Sonar Image", cv_img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Image process error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = SonarInferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```
## 2. rov.launch.py
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='water',
            executable='sonar_infer_node',
            name='sonar_infer_node',
            output='screen'
        )
    ])
```
## 3. package.xml
```python
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>water</name>
  <version>0.0.0</version>
  <description>Water ROV package with CNN sonar target recognition</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>opencv_python</depend>
  <depend>torch</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```
## 4. setup.py
```python
from setuptools import find_packages, setup

package_name = 'water'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Water ROV package with CNN sonar target recognition',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_infer_node = water.sonar_infer_node:main',
        ],
    },
)
```
## 编译命令
```bash
colcon build --packages-select water
source install/setup.bash
```
## 运行命令
ros2 launch water rov.launch.py

## 运行效果图
![运行效果图] Camera_Image_Raw_Data.jpg

![运行效果图] Underwater Target Recognition Demo.jpg



    
