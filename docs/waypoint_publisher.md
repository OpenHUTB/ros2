# ROS2 Humble 路径点发布器
## 1 项目简介
本项目基于 ROS2 Humble，编写路径点发布节点，向 Nav2 发布路径（Path）消息，可在 RViz2 可视化观察规划路径，用于机器人定点导航。
### 使用工具
1. 虚拟机：VMware Workstation
2. 操作系统：Ubuntu 22.04
3. ROS平台：ROS 2 Humble
4. 可视化工具：RViz2
5. 编程语言：Python3
6. 版本管理：Git + GitHub
## 2 环境准备
- 系统：Ubuntu 22.04
- ROS版本：ROS2 Humble
- 依赖包：`rclpy`、`nav_msgs`、`geometry_msgs`

安装依赖（如果缺少）：
```bash
sudo apt update
sudo apt install ros-humble-nav-msgs ros-humble-geometry-msgs
```
## 3. 创建 ROS2 功能包
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python waypoint_publisher
cd waypoint_publisher/waypoint_publisher
新建waypoint_pub.py文件，写入路径发布代码
## 4.核心代码
```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(Path, '/plan', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # 路径点
        poses = [
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (2.0, 1.0)
        ]
        for x,y in poses:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)
        self.get_logger().info("发布路径点成功")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
## 5.编译与运行
cd ~/ros2_ws
colcon build --packages-select waypoint_publisher
source install/setup.bash
ros2 run waypoint_publisher waypoint_pub
## 6.RViz2配置
新开终端，启动rviz
rviz2
全局坐标系 `Fixed Frame` 设置为 `map`
点击 Add，添加 `Path` 可视化插件
Path 插件的 Topic 填写 `/plan`
此时即可看到发布出来的路径折线
![节点运行截图](image/node_run.jpg)
![话题数据截图](image/echo_topic.jpg)

## 7.验证消息
新开终端查看话题消息
ros2 topic echo /plan
打印出路径消息，代表节点正常发布
## 8.总结
本节点可以自定义waypoint数组，修改坐标即可更改机器人目标路径，发布的Path消息可直接供给Nav2导航栈使用，在RViz2中实现可视化路径,运用了豆包大模型。
