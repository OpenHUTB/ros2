# -*- coding: utf-8 -*-
"""ROS 2 节点：对一段肌肉激活流做在线协同分解。

订阅 ``/emg/activation``（``std_msgs/Float64MultiArray``，每次 n 块肌肉的激活值），
攒满 ``window`` 个采样点后做一次 NMF，发布：

* ``/emg/synergy_vaf``（``std_msgs/Float64``）：当前窗的重建率 VAF；
* ``/emg/synergy_matrix``（``std_msgs/Float64MultiArray``）：k×n 的协同权重（行优先展开）。

用途：把"这段动作需要几个协同"变成可以在线观察的量，而不是离线跑一次脚本。
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from .core import nmf, vaf


class SynergyNode(Node):
    def __init__(self):
        super().__init__("muscle_synergy_node")

        self.declare_parameter("k", 3)
        self.declare_parameter("window", 200)
        self.declare_parameter("iters", 200)
        self.declare_parameter("rate", 0.2)

        gp = self.get_parameter
        self.k = int(gp("k").value)
        self.window = int(gp("window").value)
        self.iters = int(gp("iters").value)
        self.buf = deque(maxlen=self.window)

        self.sub = self.create_subscription(
            Float64MultiArray, "/emg/activation", self.on_sample, 10)
        self.pub_vaf = self.create_publisher(Float64, "/emg/synergy_vaf", 10)
        self.pub_w = self.create_publisher(Float64MultiArray, "/emg/synergy_matrix", 10)
        self.create_timer(1.0 / float(gp("rate").value), self.on_timer)

        self.last = None
        self.get_logger().info(
            "muscle_synergy 启动：k=%d，窗口 %d 采样，NMF 迭代 %d"
            % (self.k, self.window, self.iters))

    def on_sample(self, msg: Float64MultiArray):
        v = np.asarray(msg.data, dtype=float)
        if v.size == 0:
            return
        if self.last is not None and v.size != self.last:
            self.get_logger().warn("肌肉数变化（%d → %d），清空缓冲" % (self.last, v.size))
            self.buf.clear()
        self.last = v.size
        self.buf.append(v)

    def on_timer(self):
        if len(self.buf) < max(10, self.window // 4):
            return
        E = np.vstack(self.buf).T          # n_muscles × T
        k = min(self.k, E.shape[0])
        W, H = nmf(E, k, iters=self.iters)
        v = vaf(E, W, H)

        m = Float64()
        m.data = float(v)
        self.pub_vaf.publish(m)

        out = Float64MultiArray()
        out.data = [float(x) for x in W.T.ravel()]
        self.pub_w.publish(out)
        self.get_logger().debug("VAF=%.4f（%d 个采样，k=%d）" % (v, E.shape[1], k))


def main(args=None):
    rclpy.init(args=args)
    node = SynergyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
