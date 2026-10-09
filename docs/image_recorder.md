# 地面载具 - 图像录制 ROS 2 封装模块

## 一、作业概述与背景
本模块属于 ROS 2 与仿真环境之间的桥接功能。基于 ROS 2 实现了一个图像录制节点，用于订阅 Carla 仿真器发布的摄像头话题，实现按下回车键将当前图像保存至本地的功能，为后续的视觉数据处理提供辅助。

## 二、运行环境
* 操作系统：Ubuntu 22.04（虚拟机）
* ROS 2 版本：Humble
* 依赖库：rclpy, sensor_msgs, cv_bridge, opencv-python

## 三、核心代码文件清单
本模块包含以下核心文件，均已提交至仓库：
* `docs/image_recorder.md`：模块说明文档（即本文档）。
* `image_recorder/main.py`：图像订阅与录制主程序。
* `image_recorder/setup.py`：ROS 2 功能包配置文件。
* `image_recorder/launch/main.launch.py`：一键启动文件。

## 四、核心代码内容

### 1. main.py（主程序）
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageRecorder(Node):
    def __init__(self):
        super().__init__('image_recorder_node')
        self.subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/camera/rgb/front/image', 
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('Node started, waiting for images...')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Image Recorder", cv_image)
            key = cv2.waitKey(1)
            if key == 13:
                cv2.imwrite('capture.jpg', cv_image)
                self.get_logger().info('Saved as capture.jpg')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_recorder = ImageRecorder()
    rclpy.spin(image_recorder)
    image_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. setup.py（配置文件）

```python
from setuptools import setup
import os
from glob import glob

package_name = 'image_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = image_recorder.main:main',
        ],
    },
)
```

3. main.launch.py（启动文件）

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_recorder',
            executable='main',
            name='image_recorder_node',
            output='screen'
        )
    ])
```

五、运行步骤

在终端中执行以下命令进行编译和运行：

```bash
cd ~/my_ws
colcon build --packages-select image_recorder
source install/setup.bash
ros2 launch image_recorder main.launch.py
```

六、运行效果与验证

节点成功启动，终端输出 [INFO] [image_recorder_node]: Node started, waiting for images...，表示 ROS 2 节点已进入监听状态，底层通信机制运行正常。

验证结果说明：由于本机虚拟机环境未安装 Carla 仿真器，未能实时获取真实的图像数据，但图像订阅与节点启动逻辑已通过编译与运行验证。

https://github.com/Rafaye-71-star/ros2/raw/master/run_result.png

七、提交信息（Commit Message / PR 描述）

```text
feat(ground): 新增 Carla 图像捕获 ROS 2 封装模块

基于 ROS 2 实现了一个图像录制节点，用于订阅仿真器发布的摄像头话题，并实现按下回车键单帧保存图片的功能。

主要文件：
* docs/image_recorder.md: 模块说明文档
* image_recorder/main.py: 订阅图像话题并保存图片
* image_recorder/setup.py: ROS 2 包定义
* image_recorder/launch/main.launch.py: launch 启动入口

已验证：
1. colcon build 编译通过
2. ros2 launch image_recorder main.launch.py 正常运行
3. 节点启动并进入等待图像状态

备注：因本机环境未安装 Carla，无法实时获取仿真图像，但节点逻辑已通过运行验证。
```
