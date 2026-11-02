#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wind_drift_sim/main.py - ROS2 节点入口

以 50Hz 仿真悬停无人机在定常风场中的位置漂移, 对比三种策略并以
PoseStamped 发布: /drift/no_compensation 与 /drift/pd_compensation,
运行结束后打印漂移指标对比。

运行（需 ROS2 Humble 环境）:
    python3 main.py           # 运行 30 秒仿真后输出对比并退出
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import core


class WindDriftNode(Node):

    def __init__(self):
        super().__init__('wind_drift_sim')
        self.pub_none = self.create_publisher(
            PoseStamped, 'drift/no_compensation', 10)
        self.pub_pd = self.create_publisher(
            PoseStamped, 'drift/pd_compensation', 10)

        self.dt = 0.02
        self.plant_none = core.WindDriftPlant(wind=(3.0, 0.0))
        self.plant_pd = core.WindDriftPlant(wind=(3.0, 0.0))
        self.policy_pd = core.pd_compensation(kp=0.8, kd=1.2)
        self.records_none, self.records_pd = [], []
        self.steps = 0
        self.max_steps = int(30.0 / self.dt)

        self.get_logger().info('风场漂移仿真节点启动: 3m/s 东风, 50Hz, 30s')
        self.timer = self.create_timer(self.dt, self._tick)

    def _publish(self, pub, pos):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x, msg.pose.position.y = pos
        msg.pose.position.z = 10.0  # 悬停高度
        msg.pose.orientation.w = 1.0
        pub.publish(msg)

    def _tick(self):
        p_none = self.plant_none.step((0.0, 0.0), self.dt)
        ctrl = self.policy_pd(self.plant_pd.pos, self.plant_pd.vel, 0.0)
        p_pd = self.plant_pd.step(ctrl, self.dt)
        self.records_none.append(p_none)
        self.records_pd.append(p_pd)
        self._publish(self.pub_none, p_none)
        self._publish(self.pub_pd, p_pd)

        self.steps += 1
        if self.steps >= self.max_steps:
            m_none = core.drift_metrics(self.records_none)
            m_pd = core.drift_metrics(self.records_pd)
            self.get_logger().info('===== 漂移对比 (3m/s 定常风, 30s) =====')
            self.get_logger().info(
                f"  无补偿:   最大漂移 {m_none['max_drift']:.2f} m, "
                f"终态 {m_none['final_drift']:.2f} m")
            self.get_logger().info(
                f"  PD 补偿:  最大漂移 {m_pd['max_drift']:.2f} m, "
                f"终态 {m_pd['final_drift']:.2f} m")
            raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = WindDriftNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
