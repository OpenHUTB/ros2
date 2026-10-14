# 肌肉骨骼人形机器人 ROS2 自主避障仿真项目
项目存放路径：docs/musculoskeletal

## 1、项目简介
本项目针对肌肉骨骼人形机器人，设计ROS2自主避障控制算法。
为降低虚拟机仿真压力、快速验证ROS节点、话题发布订阅与避障逻辑，采用turtlesim轻量化仿真载体完成算法验证。
算法实时获取机器人本体位姿，判断与障碍物边界的安全距离。当靠近边界障碍物，机器人停止前进并原地转向；无障碍物时保持匀速直行。本套控制框架可迁移到肌肉骨骼人形机器人Gazebo模型。

## 2、项目目录结构
musculoskeletal
├── README.md
└── src
├── avoid_node.py     # 避障主代码
├── setup.py          # 功能包编译配置
└── package.xml       # 功能包描述文件

## 3、项目实现过程

步骤1：搭建ROS2工作空间

在Ubuntu虚拟机终端创建ROS2工作空间，进入src目录，创建musculoskeletal_avoid功能包，导入rclpy、消息接口等依赖。
```bash
mkdir -p ~/musculoskeletal_ws/src
cd ~/musculoskeletal_ws/src
ros2 pkg create --build-type ament_python musculoskeletal_avoid
###
```
## 4、核心代码

1、src/avoid_node.py
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MusculoskeletalRobotAvoidNode(Node):
    def __init__(self):
        super().__init__("musculoskeletal_avoid_node")
        # 订阅机器人位姿话题
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        # 发布速度控制指令话题
        self.vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel",10)
        self.get_logger().info("✅肌肉骨骼人形机器人避障节点启动成功")

    def pose_callback(self, msg:Pose):
        vel_msg = Twist()
        safe_threshold = 1.0
        # 边界障碍物检测逻辑
        if msg.x < safe_threshold or msg.x > (11-safe_threshold) or msg.y < safe_threshold or msg.y > (11-safe_threshold):
            # 靠近障碍物：停止前进，原地旋转避开障碍
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.4
        else:
            # 处于安全区域，保持匀速直行
            vel_msg.linear.x = 0.3
            vel_msg.angular.z = 0.0
        self.vel_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MusculoskeletalRobotAvoidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```
2、src/setup.py
```python
from setuptools import setup

package_name = 'musculoskeletal_avoid'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='musculoskeletal humanoid robot obstacle avoidance simulation based on ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avoid_node = musculoskeletal_avoid.avoid_node:main'
        ],
    },
)
```
3、src/package.xml
```python

<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>musculoskeletal_avoid</name>
  <version>0.0.0</version>
  <description>Musculoskeletal humanoid robot obstacle avoidance simulation based on ROS2</description>
  <maintainer email="student@todo.todo">student</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <build_depend>rclpy</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>turtlesim</build_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>turtlesim</exec_depend> 

  <test_depend>ament_pytest</test_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```
## 5、操作说明
> 运行环境：Ubuntu22.04 + ROS2 Humble 虚拟机
### 5.1 前置准备
1. 虚拟机系统内已经成功安装ROS2 Humble；
2. 确保turtlesim功能包已经安装，若未安装，在终端执行：
```bash
sudo apt update
sudo apt install ros-humble-turtlesim
```
### 5.2 编译项目代码
进入工作空间根目录
cd ~/musculoskeletal_ws
编译功能包
colcon build
加载ROS2环境（新开终端都需要执行这条）
source install/setup.bash
### 5.3 项目启动操作
终端 1（启动仿真环境）
source install/setup.bash
ros2 run turtlesim turtlesim_node
终端 2（启动避障控制节点）
source install/setup.bash
ros2 run musculoskeletal_avoid avoid_node
### 5.4 仿真观测

观察仿真窗口：

1. 机器人在安全区域会持续匀速向前直行；
2. 当机器人靠近窗口边界（障碍物），自动停止前进，原地旋转；
3. 旋转离开危险边界后，继续向前直行，自动完成避障。

### 5.5 停止程序操作

1. 在对应终端按下 `Ctrl + C`，即可停止当前运行的节点；
2. 关闭仿真窗口，结束本次仿真实验。

运行效果：https://github.com/Toeting/ros2/commit/49f5fa73e54302fdf24ff3662df000979609b6f3

