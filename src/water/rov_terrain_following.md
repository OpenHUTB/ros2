# 水下 ROV 定高巡航与海底地形跟随设计文档

> **说明**：本文档目前为纯设计文档与逻辑说明，非可运行模块。具体 ROS 2 节点代码及配置将在后续 PR 中补充。

## 1. 功能介绍 (Overview)
在真实的海底测绘、光缆巡检等任务中，ROV 通常需要与海底保持固定安全距离（例如 3 米）进行定高巡航。由于海底地形起伏不平，单纯依赖操作员使用键盘进行垂直方向（Z轴）调整极易发生碰撞。

本设计引入**“定高模式 (Altitude Hold)”**，基于水下高度计（Altimeter/DVL）数据，结合 PID 控制算法，实现 ROV 对起伏海底地形的自动跟随。开启该模块后，操作员只需控制 ROV 的前进方向（X轴）和航向（Yaw），节点将自动调节垂直速度，保持设定的对地高度（默认 3.0 米，可通过参数配置）。

## 2. 核心逻辑与架构 (Core Logic & Architecture)

*   **复用现有 PID 控制器**：未来落地时，将直接复用仓库中已有的 `src/water/rov_mujoco/rov_mujoco/depth_controller.py` 里的 `PIDDepthController` 类（该类已实现 dt 归一化微分、积分限幅 anti-windup 和洋流前馈），避免重复造轮子。本节点仅负责高度数据获取、指令分发与失效保护。
*   **速度指令合并策略 (Mux)**：当前 `rov_teleop` 和本定高节点均会向 `/rov/cmd_vel` 发布指令，为避免互相清零，本节点设计为**订阅** `teleop` 输出的速度指令，仅**覆盖** `linear.z` 后，再统一发布给底层桥接节点。
*   **通信失效保护（看门狗）**：基于 `rclpy` 的 `create_timer` 实现。若 `/rov/altitude` 数据断流超过阈值（如 0.5 秒），定时器会强制将垂直速度指令清零，确保安全。

## 3. 运行与测试计划 (Planned Dependencies)

**⚠️ 前置依赖（尚未实现）**：
1. 仿真侧需先新增高度传感器（如 Gazebo Sonar/Altimeter 插件或 DVL），并发布 `/rov/altitude` 话题。
2. 需解决现有 `rov_bridge` 每次收到 `cmd_vel` 时重置模型位姿的问题，否则高度数据无意义。

**计划测试步骤**：
1. 启动包含起伏海底地形的仿真环境及键盘控制。
2. 运行定高测试节点。
3. 使用键盘控制 ROV 向前飞行，观察 RViz 中目标高度线与实际高度线的拟合情况。

**计划验证指标**：
* 高度跟踪的 RMSE 和最大偏差。
* 断流保护触发时间。
* 阶跃恢复时间。

## 4. 核心控制逻辑伪代码 (Pseudo Code Framework)

以下为节点组织架构伪代码。核心 PID 计算直接调用 `PIDDepthController`，本节点仅处理订阅、发布与看门狗逻辑。

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rov_mujoco.depth_controller import PIDDepthController

class TerrainFollowingNode(Node):
    def __init__(self):
        super().__init__('terrain_following_node')
        
        self.declare_parameter('target_altitude', 3.0)
        self.declare_parameter('timeout_threshold', 0.5)
        
        # 假设复用现有的 PID 控制器实例
        self.pid = PIDDepthController() 
        
        self.last_altitude_time = self.get_clock().now()
        self.current_alt = 0.0
        
        self.sub_alt = self.create_subscription(Float32, '/rov/altitude', self.altitude_callback, 10)
        self.sub_cmd = self.create_subscription(Twist, '/rov/cmd_vel_teleop', self.cmd_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/rov/cmd_vel', 10)
        
        # 看门狗定时器 (10Hz)
        self.timer = self.create_timer(0.1, self.watchdog_check)

    def altitude_callback(self, msg):
        self.current_alt = msg.data
        self.last_altitude_time = self.get_clock().now()

    def cmd_callback(self, teleop_msg):
        # 混合指令：保留操作员的 X/Y 平移，只覆盖 Z 轴
        cmd = teleop_msg
        target = self.get_parameter('target_altitude').value
        # 调用现有 PID 类计算推力/速度
        cmd.linear.z = self.pid.compute(target, self.current_alt) 
        self.pub_cmd.publish(cmd)

    def watchdog_check(self):
        # 超时保护：如果 0.5s 没有收到高度数据，下发 0 速度
        elapsed = (self.get_clock().now() - self.last_altitude_time).nanoseconds / 1e9
        if elapsed > self.get_parameter('timeout_threshold').value:
            self.get_logger().warn("高度计数据断流，触发失效保护！")
            stop_cmd = Twist()
            stop_cmd.linear.z = 0.0
            self.pub_cmd.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TerrainFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
