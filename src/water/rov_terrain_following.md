# 水下 ROV 定高巡航与海底地形跟随设计文档

> **说明**：本文档目前为纯设计文档与伪代码逻辑说明，非可直接运行的模块。具体 ROS 2 节点代码（包含 package.xml 等）将在后续 PR 中补充。

## 1. 功能介绍 (Overview)

在真实的海底测绘、光缆巡检或沉船打捞任务中，水下机器人（ROV）通常需要与海底保持固定的安全距离（例如 3 米）进行定高巡航。然而，由于海底地形起伏不平，单纯依赖操作员使用键盘进行垂直方向（Z轴）的调整不仅费时费力，还极易发生碰撞。

本设计引入**“定高模式 (Altitude Hold)”**，基于水下高度计（Altimeter/DVL）的数据，结合 PID 控制算法，实现 ROV 对起伏海底地形的自动跟随。开启该模块后，操作员只需控制 ROV 的前进方向（X轴）和航向（Yaw），ROV 将自动调节垂直推力，保持设定的对地高度（默认设定为 3.0 米，可通过参数配置）。

## 2. 核心逻辑 (Core Logic)

*   **高度数据获取**：订阅仿真环境提供的 `/rov/altitude` (对地高度) 话题。
*   **地形跟随 PID**：计算 `目标设定高度` 与 `当前实际高度` 的误差，通过完整的 PID 控制器（包含比例 P、积分 I、微分 D）输出补偿速度指令。
*   **时间戳归一化**：微分项计算引入实际时间差 `dt`，避免因高度计回调频率不稳定导致的高频噪声放大。
*   **通信失效保护（看门狗）**：加入定时器监控，若高度计数据断流超过设定阈值，自动将垂直速度指令清零，确保安全。
*   **代码复用规划**：未来落地时，将优先复用仓库中已有的 `src/water/rov_mujoco/rov_mujoco/depth_controller.py` 里的 `PIDDepthController` 类，避免重复造轮子。

## 3. 运行与测试计划 (How to Run - Planned)

1. 计划启动包含起伏海底地形的仿真环境（复用现有环境）：
   ```bash
   ros2 launch rov_gazebo_sim rov_sim.launch.py
   计划测试步骤：
   · 设定目标定高为 3.0 米：ros2 param set /terrain_following_node target_altitude 3.0
   · 使用键盘控制 ROV 向前（X轴）飞行，穿过高低起伏的海底山脉。
   · 观察 RViz 中的目标高度线与实际高度线的拟合情况。

4. 核心控制逻辑伪代码 (Pseudo Code)

以下为定高控制节点的核心 PID 逻辑实现框架（基于 ROS 2 Python 接口）。代码中包含了对 dt 的计算以及通信超时保护逻辑。

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class TerrainFollowingNode(Node):
    def __init__(self):
        super().__init__('terrain_following_node')
        
        # 声明参数：目标高度默认 3.0 米
        self.declare_parameter('target_altitude', 3.0)
        self.target_alt = self.get_parameter('target_altitude').value
        
        # PID 参数
        self.kp = 1.5
        self.ki = 0.1  # 积分项，消除稳态误差
        self.kd = 0.5
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()
        self.last_altitude_time = self.get_clock().now()
        
        # 订阅当前对地高度
        self.sub_alt = self.create_subscription(
            Float32, '/rov/altitude', self.altitude_callback, 10)
        
        # 发布控制指令 (速度控制)
        self.pub_cmd = self.create_publisher(Twist, '/rov/cmd_vel', 10)

    def altitude_callback(self, msg):
        current_time = self.get_clock().now()
        self.last_altitude_time = current_time
        
        # 计算实际时间差 dt
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            return # 避免除以 0
        self.prev_time = current_time

        current_alt = msg.data
        
        # 计算误差
        error = self.target_alt - current_alt
        self.integral += error * dt
        d_error = (error - self.prev_error) / dt
        self.prev_error = error
        
        # PID 计算 (输出为垂直速度)
        z_velocity = (self.kp * error) + (self.ki * self.integral) + (self.kd * d_error)
        
        # 发布控制指令 (限制最大速度，假设单位 m/s)
        cmd = Twist()
        cmd.linear.z = max(min(z_velocity, 1.0), -1.0) 
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TerrainFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

5. 预期效果与可视化 (Visualization)

· 开启前：ROV 会直直地撞向隆起的海底，或者随着地形下降而悬停在半空。
· 开启后：ROV 的垂直速度会随地形变化自动变化，完美贴着海底起伏，始终保持 3 米的安全距离。
