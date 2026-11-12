#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import mujoco
import os
from ament_index_python.packages import get_package_share_directory

class MujocoSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_sim_node')
        
        # 动态获取模型路径
        package_path = get_package_share_directory('rov_mujoco_slam')
        model_path = os.path.join(package_path, 'models', 'rov_underwater.xml')
        
        try:
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
            self.get_logger().info('MuJoCo 模型加载成功!')
        except Exception as e:
            self.get_logger().error(f'模型加载失败: {e}')
            raise e

        # 创建发布者
        self.pose_pub = self.create_publisher(PoseStamped, '/rov/pose', 10)
        
        # 定时器 (100Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.step_count = 0

    def timer_callback(self):
        mujoco.mj_step(self.model, self.data)
        
        if self.step_count % 100 == 0:
            pos = self.data.qpos[0:3] 
            self.get_logger().info(f'仿真时间: {self.data.time:.2f}s | 位置: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}')
            
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.pose.position.x = pos[0]
            msg.pose.position.y = pos[1]
            msg.pose.position.z = pos[2]
            self.pose_pub.publish(msg)
            
        self.step_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()