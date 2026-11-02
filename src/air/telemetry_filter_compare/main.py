#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
telemetry_filter_compare/main.py - ROS2 节点入口

以 10Hz 生成含噪高度遥测, 分别经滑动平均与卡尔曼滤波后发布:
    /drone/altitude_raw      (std_msgs/Float32) 原始含噪高度
    /drone/altitude_ma       滑动平均滤波结果
    /drone/altitude_kalman   卡尔曼滤波结果
运行结束后在终端打印三种数据的 RMSE 对比表。

运行（需 ROS2 Humble 环境）:
    python3 main.py           # 运行 30 秒后自动输出对比并退出
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import core


def true_alt_fn(t):
    return 10.0 + 0.05 * t  # 缓慢爬升


class TelemetryFilterNode(Node):

    def __init__(self):
        super().__init__('telemetry_filter_compare')
        self.declare_parameter('run_seconds', 30.0)

        self.pub_raw = self.create_publisher(Float32, 'drone/altitude_raw', 10)
        self.pub_ma = self.create_publisher(Float32, 'drone/altitude_ma', 10)
        self.pub_kf = self.create_publisher(Float32, 'drone/altitude_kalman', 10)

        self.kalman = core.ScalarKalman()
        self.window = []
        self.t = 0.0
        self.records = {'raw': [], 'ma': [], 'kf': []}
        self.truth = []

        run_s = float(self.get_parameter('run_seconds').value)
        self.timer = self.create_timer(0.1, self._tick)
        self.end_timer = self.create_timer(run_s, self._finish)
        self.get_logger().info('遥测滤波对比节点启动 (10Hz), 运行 30 秒...')

    def _tick(self):
        raw = true_alt_fn(self.t) + core.random_noise()
        ma_val = core.moving_average(self.window + [raw], 9)[-1] \
            if self.window else raw
        self.window = (self.window + [raw])[-9:]
        kf_val = self.kalman.update(raw)

        msg = Float32()
        msg.data = raw
        self.pub_raw.publish(msg)
        msg.data = ma_val
        self.pub_ma.publish(msg)
        msg.data = kf_val
        self.pub_kf.publish(msg)

        self.records['raw'].append(raw)
        self.records['ma'].append(ma_val)
        self.records['kf'].append(kf_val)
        self.truth.append(true_alt_fn(self.t))
        self.t += 0.1

    def _finish(self):
        truth = [true_alt_fn(i * 0.1) for i in range(len(self.records['raw']))]
        self.get_logger().info('===== 滤波效果对比 (RMSE, 越小越好) =====')
        self.get_logger().info(
            f"  原始数据:   {core.rmse(self.records['raw'], truth):.3f} m")
        self.get_logger().info(
            f"  滑动平均:   {core.rmse(self.records['ma'], truth):.3f} m")
        self.get_logger().info(
            f"  卡尔曼滤波: {core.rmse(self.records['kf'], truth):.3f} m")
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
