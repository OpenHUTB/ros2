# ROS2 Humble 路径点发布器
## 1 项目简介
本项目基于 ROS2 Humble，编写路径点发布节点，向 Nav2 发布路径（Path）消息，可在 RViz2 可视化观察规划路径，用于机器人定点导航。

## 2 环境准备
- 系统：Ubuntu 22.04
- ROS版本：ROS2 Humble
- 依赖包：`rclpy`、`nav_msgs`、`geometry_msgs`

安装依赖（如果缺少）：
```bash
sudo apt update
sudo apt install ros-humble-nav-msgs ros-humble-geometry-msgs
3.创建Ros2功能包
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python waypoint_publisher
cd waypoint_publisher/waypoint_publisher
新建waypoint_pub.py文件，写入路径发布代码
4.核心代码
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        # 创建Path发布器，话题名 /plan
        self.publisher_ = self.create_publisher(Path, '/plan', 10)
        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        # 定义一系列路径点
        waypoints = [
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0)
        ]

        for x,y in waypoints:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
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
修改setup.py，在entry_pionts里添加节点入口：
entry_points={
    'console_scripts': [
        'waypoint_pub = waypoint_publisher.waypoint_pub:main',
    ],
},
5.编译与运行
cd ~/ros2_ws
colcon build --packages-select waypoint_publisher
source install/setup.bash
ros2 run waypoint_publisher waypoint_pub
6.RViz2配置
新开终端，启动rviz
rviz2
全局坐标系 `Fixed Frame` 设置为 `map`
点击 Add，添加 `Path` 可视化插件
Path 插件的 Topic 填写 `/plan`
此时即可看到发布出来的路径折线
![rviz截图1](image/Screenshot%20from%2026-10-10%2022-33-14.png)
![rviz截图2](image/Screenshot%20from%2026-10-10%2022-42-53.png)
7.验证消息
新开终端查看话题消息
ros2 topic echo /plan
打印出路径消息，代表节点正常发布
8.总结
本节点可以自定义waypoint数组，修改坐标即可更改机器人目标路径，发布的Path消息可直接供给Nav2导航栈使用，在RViz2中实现可视化路径
