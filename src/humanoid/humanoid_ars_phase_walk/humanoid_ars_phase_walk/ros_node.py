# -*- coding: utf-8 -*-
"""ROS 2 节点：把 CPG / 学习策略的关节目标按相位发出来。

话题
----
* ``~/joint_target`` ``std_msgs/Float64MultiArray`` = [hip_l, knee_l, ankle_l, hip_r, knee_r, ankle_r]
* ``~/phase``        ``std_msgs/Float32``          当前相位 (rad)
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray

import humanoid_ars_phase_walk.config as config
import humanoid_ars_phase_walk.env as env


class ArsWalkNode(Node):
    def __init__(self):
        super().__init__("ars_walk_node")
        self.declare_parameter("period", 0.02)
        self.period = max(0.005, float(self.get_parameter("period").value))
        self.t = 0.0
        self.pub_j = self.create_publisher(Float64MultiArray, "~/joint_target", 10)
        self.pub_p = self.create_publisher(Float32, "~/phase", 10)
        self.create_timer(self.period, self._tick)
        self.get_logger().info(f"CPG 基线，周期 {self.period*1000:.0f} ms")

    def _tick(self):
        self.t += self.period
        ph = 2.0 * math.pi * config.FREQ * self.t
        ctrl = env.cpg(ph)
        m = Float64MultiArray()
        m.data = [float(x) for x in ctrl]
        self.pub_j.publish(m)
        p = Float32()
        p.data = float(ph % (2.0 * math.pi))
        self.pub_p.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = ArsWalkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()