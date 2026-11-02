#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pid_altitude_controller/main.py - ROS2 节点入口

以 50Hz 运行高度阶跃仿真, 发布:
    /drone/altitude_setpoint  (std_msgs/Float32) 目标高度
    /drone/altitude_actual    (std_msgs/Float32) 实际高度
运行结束后打印超调量、调节时间、稳态误差三项指标。

运行（需 ROS2 Humble 环境）:
    python3 main.py           # 20 秒阶跃仿真后输出指标并退出
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import core


class PidAltitudeNode(Node):

    def __init__(self):
        super().__init__('pid_altitude_controller')
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.6)
        self.declare_parameter('kd', 0.8)
        self.declare_parameter('target_alt', 10.0)

        kp = float(self.get_parameter('kp').value)
        ki = float(self.get_parameter('ki').value)
        kd = float(self.get_parameter('kd').value)
        self.target = float(self.get_parameter('target_alt').value)

        self.pub_sp = self.create_publisher(Float32, 'drone/altitude_setpoint', 10)
        self.pub_ac = self.create_publisher(Float32, 'drone/altitude_actual', 10)

        self.sim = core.simulate_altitude_step(kp, ki, kd, self.target)
        self.ts, self.zs, self.us, self.metrics = self.sim
        self.idx = 0
        self.dt = 0.02

        self.get_logger().info(
            f'PID 高度仿真启动: kp={kp}, ki={ki}, kd={kd}, 目标 {self.target}m')
        self.timer = self.create_timer(self.dt, self._tick)

    def _tick(self):
        if self.idx >= len(self.ts):
            m = self.metrics
            self.get_logger().info('===== 阶跃响应指标 =====')
            self.get_logger().info(f"  峰值高度:   {m['peak']:.2f} m")
            self.get_logger().info(f"  超调量:     {m['overshoot_pct']:.1f} %")
            self.get_logger().info(f"  调节时间:   {m['settling_time']:.1f} s")
            self.get_logger().info(f"  稳态误差:   {m['steady_error']:.3f} m")
            raise SystemExit(0)

        t = self.ts[self.idx]
        z = self.zs[self.idx]
        msg = Float32()
        msg.data = self.target
        self.pub_sp.publish(msg)
        msg.data = z
        self.pub_ac.publish(msg)
        self.idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = PidAltitudeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
