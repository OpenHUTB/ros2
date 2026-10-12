
# 空域载具 - 无人机飞行轨迹类型识别与 ROS2 自动任务日志

## 作业概述与背景
本模块基于 ROS 2 实现了一套无人机飞行轨迹类型识别系统。系统通过发布节点模拟无人机飞行指令，接收节点实时订阅数据，并自动识别当前的飞行状态（直线、旋转、悬停），同时实时绘制飞行轨迹图并生成自动任务日志。

## 运行环境
* 操作系统：Ubuntu 22.04（虚拟机）
* ROS 2 版本：Humble
* 依赖库：rclpy, geometry_msgs, matplotlib

## 核心代码文件清单
本模块包含以下核心文件：
* `trajectory_publisher.py`：模拟无人机飞行轨迹发布节点。
* `trajectory_analyzer.py`：轨迹识别、绘图与日志记录节点。

## 核心代码内容

### 1.trajectory_publisher.py（发布节点）
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(Twist, '/drone/pose', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.state = 0
        self.get_logger().info('轨迹发布节点已启动。')

    def timer_callback(self):
        msg = Twist()
        self.state += 1
        if self.state % 3 == 1:
            msg.linear.x = 1.0
            self.get_logger().info('发布: 前进指令')
        elif self.state % 3 == 2:
            msg.angular.z = 1.0
            self.get_logger().info('发布: 旋转指令')
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('发布: 悬停指令')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. trajectory_analyzer.py（识别与绘图节点）

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import math

class TrajectoryAnalyzer(Node):
    def __init__(self):
        super().__init__('trajectory_analyzer')
        self.subscription = self.create_subscription(Twist, '/drone/pose', self.listener_callback, 10)
        self.get_logger().info('轨迹识别节点已启动，等待飞行数据...')
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-', label='Drone Path')
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_title('UAV Flight Trajectory')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.legend()
        self.ax.grid(True)
        self.xs, self.ys = [], []

    def listener_callback(self, msg):
        if msg.linear.x > 0:
            self.get_logger().info('【轨迹识别】当前飞行状态：直线前进')
            self.x += 0.5 * math.cos(self.theta)
            self.y += 0.5 * math.sin(self.theta)
        elif msg.angular.z > 0:
            self.get_logger().info('【轨迹识别】当前状态：原地旋转')
            self.theta += 0.2
        else:
            self.get_logger().info('【轨迹识别】当前状态：悬停等待')

        self.xs.append(self.x)
        self.ys.append(self.y)
        self.line.set_data(self.xs, self.ys)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 运行步骤

在终端中依次执行以下命令进行编译和运行：

```bash
cd ~/my_ws/src/drone_trajectory_logger
python3 trajectory_publisher.py
```

（在另一个终端中运行）

```bash
cd ~/my_ws/src/drone_trajectory_logger
python3 trajectory_analyzer.py
```

## 运行效果与验证（截图）

https://github.com/Rafaye-71-star/ros2/blob/feature/drone-trajectory-logger/capture_2.png

效果说明：发布节点成功发送不同的飞行指令，识别节点成功解析并打印出对应的飞行状态日志，同时基于实时数据绘制了飞行轨迹图（椭圆），验证了轨迹识别与自动日志记录功能的正确性。

## 提交信息（Commit Message / PR 描述）

```text
feat(air): 新增无人机飞行轨迹类型识别与自动任务日志模块

基于 ROS 2 实现了无人机飞行轨迹的自动识别与实时绘图日志记录功能。

主要文件：
* drone_trajectory_logger/trajectory_publisher.py: 发布节点
* drone_trajectory_logger/trajectory_analyzer.py: 识别与绘图节点

已验证：
1. 两个节点均正常启动
2. 飞行指令成功发布与接收
3. 轨迹识别日志输出正确，飞行轨迹图绘制成功

备注：本模块使用模拟的飞行指令流进行验证，后续可接入真实的无人机仿真数据。
```
