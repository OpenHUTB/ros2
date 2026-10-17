# ROS2 移动小车超声波避障节点开发（含传感器数据模拟）

## 1. 项目概述
本项目在 ROS 2 环境下实现了一个移动小车的超声波避障节点。由于脱离真实硬件，节点内置了传感器数据模拟功能，按周期生成动态的 `sensor_msgs/msg/Range` 数据，通过闭环逻辑控制小车前进与转向。

## 2. 环境要求
*   **操作系统**：Ubuntu 20.04 / 22.04（虚拟机或物理机均可）
*   **ROS 2 版本**：Foxy / Humble
*   **依赖包**：`rclpy`, `sensor_msgs`, `geometry_msgs`

## 3. 目录结构
确保本功能包放置在仓库的 `src/` 目录下，结构如下：
```text
src/ultra_avoid/
├── launch/
│   └── car.launch.py       # 启动文件
├── resource/
│   └── ultra_avoid         # 资源标记文件
├── ultra_avoid/
│   ├── __init__.py
│   └── ultra_node.py       # 核心避障节点代码
├── package.xml             # 包依赖配置
├── setup.cfg               # 脚本安装配置
└── setup.py                # Python 编译配置

```
## 4. 核心代码实现
```
本项目的核心业务逻辑位于 `src/ultra_avoid/ultra_avoid/ultra_node.py`。该节点同时充当**传感器模拟器**和**避障决策大脑**：
1. 每隔 0.1 秒（10Hz）生成一个 `sensor_msgs/msg/Range` 类型的模拟超声波数据并发布。
2. 订阅该数据，通过状态机判断：当距离小于 `0.5m` 时，下发停止并转向的指令；当距离大于 `0.8m` 时，恢复前进。
3. 将控制指令以 `geometry_msgs/msg/Twist` 格式发布到 `/cmd_vel` 话题。

### 4.1 核心节点代码 (`ultra_node.py`)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class UltraAvoidNode(Node):
    def __init__(self):
        super().__init__('ultra_avoid_node')
        # 核心参数设置
        self.stop_dist = 0.5      # 距离小于 0.5m 触发避障
        self.resume_dist = 0.8    # 距离大于 0.8m 恢复前进
        self.fwd_speed = 0.22     # 前进速度 (m/s)
        self.turn_speed = 0.8     # 转向速度 (rad/s)
        
        self.current_distance = float('inf')
        self.state = 'FORWARD'
        self.sim_counter = 0

        # 1. 发布传感器数据 (sensor_msgs/Range)
        self.range_pub = self.create_publisher(Range, '/ultrasonic/range', 10)
        
        # 2. 订阅传感器数据进行决策
        self.range_sub = self.create_subscription(
            Range, '/ultrasonic/range', self.range_callback, 10)
            
        # 3. 发布小车控制指令 (geometry_msgs/Twist)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 10Hz 循环定时器
        self.create_timer(0.1, self.timer_callback)

    def generate_simulated_range(self):
        """核心逻辑：模拟传感器从远到近，再恢复的过程"""
        self.sim_counter += 1
        phase = (self.sim_counter % 80) / 80.0
        if phase < 0.5:
            return 2.0 - (2.0 - 0.2) * (phase / 0.5) # 靠近障碍物
        else:
            return 0.2 + (2.0 - 0.2) * ((phase - 0.5) / 0.5) # 远离障碍物

    def timer_callback(self):
        """生成并发布 Range 消息"""
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = float(self.generate_simulated_range())
        self.range_pub.publish(msg)

    def range_callback(self, msg):
        """核心逻辑：接收距离 -> 状态机判断 -> 发布速度"""
        self.current_distance = msg.range
        
        # 迟滞区间状态机
        if self.state == 'FORWARD' and self.current_distance < self.stop_dist:
            self.state = 'TURN'
        elif self.state == 'TURN' and self.current_distance > self.resume_dist:
            self.state = 'FORWARD'

        cmd = Twist()
        if self.state == 'FORWARD':
            cmd.linear.x = self.fwd_speed  # 前进
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0             # 停车
            cmd.angular.z = self.turn_speed # 转向
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = UltraAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5. 操作与运行说明

在确保虚拟机或物理机已经安装了 ROS 2 环境（如 Humble 版本）的前提下，请按照以下步骤在工作空间中编译并运行本项目。

### 5.1 编译工作空间
打开终端，进入你的 ROS 2 工作空间根目录（例如 `~/ros2_ws`），执行以下命令：

```bash
cd ~/ros2_ws

# 1. 加载系统 ROS 2 环境变量
source /opt/ros/humble/setup.bash

# 2. 使用 colcon 编译 ultra_avoid 包
# （注：--parallel-workers 1 用于限制单线程编译，防止虚拟机因资源不足卡死）
colcon build --packages-select ultra_avoid --parallel-workers 1
```
### 5.2 启动避障节点
编译成功后，刷新当前工作空间的环境变量，并启动节点
```bash
# 刷新工作空间环境
source install/setup.bash

# 启动 launch 文件
ros2 launch ultra_avoid car.launch.py
```
### 5.3 验证运行结果
终端 1：查看超声波传感器模拟数据
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /ultrasonic/range
```
终端 2：查看小车控制指令
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /cmd_vel
```
### 5.4 进阶：动态调整参数
项目支持通过命令行传参，动态修改避障阈值和速度，无需修改代码：
```bash
ros2 run ultra_avoid ultra_node --ros-args \
  -p stop_distance:=0.8 \
  -p resume_distance:=1.2 \
  -p forward_speed:=0.15
```
![运行效果]<img width="2358" height="1360" alt="ROS Noetic Humble Gazebo 11-2026-10-17-23-00-27" src="https://github.com/user-attachments/assets/cbae1707-f5a0-40e1-ac20-72cca2c4e9ff" />
