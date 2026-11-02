#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
battery_monitor_rth/main.py - ROS2 节点入口

订阅模拟的电池状态话题, 执行三级低电量告警 (INFO/RTH/LAND),
并以 1Hz 发布 /drone/battery_status (无真实电池硬件时用内部模型仿真)。

运行（需 ROS2 Humble 环境）:
    python3 main.py            # 仿真模式, 持续放电直到 RTH 触发后退出
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

import core


class BatteryMonitorNode(Node):

    def __init__(self):
        super().__init__('battery_monitor_rth')
        self.declare_parameter('rth_soc', 0.25)
        self.declare_parameter('land_soc', 0.10)
        self.declare_parameter('hover_current', 15.0)
        self.declare_parameter('time_scale', 60.0)  # 仿真时间倍率 (60 = 每秒模拟 1 分钟)

        self.sim = core.BatterySim(soc0=1.0)
        self.monitor = core.RthMonitor()
        self.pub_soc = self.create_publisher(Float32, 'drone/battery_soc', 10)
        self.pub_state = self.create_publisher(String, 'drone/battery_event', 10)
        self.time_scale = float(self.get_parameter('time_scale').value)

        self.get_logger().info(
            f'电池监控节点启动 (仿真模式, 1Hz, 时间倍率 {self.time_scale}x)')
        self.timer = self.create_timer(1.0, self._tick)
        self.t = 0.0

    def _tick(self):
        current = float(self.get_parameter('hover_current').value)
        dt = self.time_scale  # 每次tick模拟 time_scale 秒
        soc = self.sim.step(current, dt)
        self.t += dt

        soc_msg = Float32()
        soc_msg.data = soc
        self.pub_soc.publish(soc_msg)

        event = self.monitor.check(soc)
        if event:
            msg = String()
            msg.data = event
            self.pub_state.publish(msg)
            self.get_logger().warn(
                f'[t={self.t:.0f}s] SOC={soc * 100:.1f}% 触发事件: {event}')
            if event == 'LAND':
                self.get_logger().info('强制降落事件已发布, 节点退出')
                raise SystemExit(0)
        elif int(self.t) % 60 < self.time_scale:
            self.get_logger().info(
                f'[t={self.t:.0f}s] SOC={soc * 100:.1f}% '
                f'电压={sim_voltage(self.sim, current):.2f}V 正常')


def sim_voltage(sim, current):
    return sim.voltage(current)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
