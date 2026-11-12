#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
import math

class SonarFilterNode(Node):
    def __init__(self):
        super().__init__('sonar_filter_node')
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.true_distance = 5.0 # 真实的障碍物距离为 5 米
        self.noise_level = 1.5   # 模拟水下环境的高斯噪声标准差
        self.window_size = 5     # 滑动平均滤波窗口大小
        self.history = []        # 用于存储历史数据的队列
        
        self.get_logger().info('🌊 水下声呐数据滤波节点已启动！')
        self.get_logger().info(f'>>> 设定真实障碍物距离: {self.true_distance} m (带噪声)')
        self.get_logger().info(f'>>> 启用滑动平均滤波 (窗口大小: {self.window_size})')
        self.get_logger().info('--------------------------------------------------')

    def timer_callback(self):
        # 1. 模拟传感器产生带噪声的数据
        raw_data = self.true_distance + random.gauss(0, self.noise_level)
        
        # 2. 滑动平均滤波
        self.history.append(raw_data)
        if len(self.history) > self.window_size:
            self.history.pop(0) # 移除最旧的数据
            
        filtered_data = sum(self.history) / len(self.history)
        
        # 3. 计算误差并打印结果
        raw_error = abs(raw_data - self.true_distance)
        filtered_error = abs(filtered_data - self.true_distance)
        
        log_msg = (
            f"📡 原始数据: {raw_data:.2f}m (误差 {raw_error:.2f}m) | "
            f"✨ 滤波后: {filtered_data:.2f}m (误差 {filtered_error:.2f}m)"
        )
        
        # 如果滤波后的数据比原始数据更接近真实值，用绿字提示（这里用 info 代替）
        self.get_logger().info(log_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SonarFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()