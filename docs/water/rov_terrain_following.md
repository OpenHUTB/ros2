# 水下 ROV 定高巡航与海底地形跟随仿真模块

## 1. 功能介绍 (Overview)

在真实的海底测绘、光缆巡检或沉船打捞任务中，水下机器人（ROV）通常需要与海底保持固定的安全距离（如 3 米）进行定高巡航。然而，由于海底地形起伏不平，单纯依赖操作员使用键盘进行垂直方向（Z轴）的调整不仅费时费力，还极易发生碰撞。

本模块引入**“定高模式 (Altitude Hold)”**，基于水下高度计（Altimeter/DVL）的数据，结合 PID 控制算法，实现 ROV 对起伏海底地形的自动跟随。开启该模块后，操作员只需控制 ROV 的前进方向（X轴）和航向（Yaw），ROV 将自动调节垂直推力，保持设定的对地高度。

## 2. 核心逻辑 (Core Logic)

*   **高度数据获取**：订阅仿真环境提供的 `/rov/altitude` (对地高度) 话题。
*   **地形跟随 PID**：计算 `目标设定高度` 与 `当前实际高度` 的误差，通过 PID 控制器输出补偿向上的推力。
*   **平滑滤波**：由于海底地形可能存在突变的岩石，加入了一阶低通滤波，防止 ROV 在跟随地形时出现剧烈震荡。

## 3. 运行与测试 (How to Run)

1. 启动包含起伏海底地形的仿真环境：
   ```bash
   ros2 launch rov_sim rov_terrain_world.launch.py
   
## 4. 核心控制逻辑 (Core Logic Code)

以下为该模块的核心 PID 控制代码实现（基于 ROS 2 Python 接口）：

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class TerrainFollowingNode(Node):
    def __init__(self):
        super().__init__('terrain_following_node')
        
        # 声明参数：目标高度默认 2.0 米
        self.declare_parameter('target_altitude', 2.0)
        self.target_alt = self.get_parameter('target_altitude').value
        
        # 简单的 PID 参数
        self.kp = 1.5
        self.kd = 0.5
        self.prev_error = 0.0
        
        # 订阅当前对地高度
        self.sub_alt = self.create_subscription(
            Float32, '/rov/altitude', self.altitude_callback, 10)
        
        # 发布垂直推力指令
        self.pub_cmd = self.create_publisher(Twist, '/rov/cmd_vel', 10)

    def altitude_callback(self, msg):
        current_alt = msg.data
        
        # 计算误差
        error = self.target_alt - current_alt
        d_error = error - self.prev_error
        self.prev_error = error
        
        # PID 计算推力
        z_thrust = (self.kp * error) + (self.kd * d_error)
        
        # 发布控制指令 (限制最大推力为 1.0)
        cmd = Twist()
        cmd.linear.z = max(min(z_thrust, 1.0), -1.0) 
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TerrainFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
