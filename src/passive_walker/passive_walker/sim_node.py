#!/usr/bin/env python3
"""无动力被动行走器的 ROS 2 节点。

节点把 MuJoCo 仿真推进结果里的“已走步数”发布到 ``/passive_walker/steps``（std_msgs/Int32），
每一步 = 轮子转过一根辐条（12 辐条时为 30°）。运行方式：

    ros2 run passive_walker main.py --node
    ros2 launch passive_walker main.launch.py

本文件只在需要 ROS 2 时导入 rclpy，离线验证请用 ``python -m passive_walker.sim``。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from passive_walker.sim import PassiveWalker, format_summary

TOPIC_NAME = "passive_walker/steps"


class PassiveWalkerNode(Node):
    """按固定周期推进仿真，并发布累计步数。"""

    def __init__(self, seconds=20.0, push=0.3, period=0.1):
        super().__init__("passive_walker")
        self.declare_parameter("seconds", float(seconds))   # 仿真总时长（秒）
        self.declare_parameter("push", float(push))         # 初始推力（m/s）
        self.declare_parameter("period", float(period))     # 每个回调推进的仿真时间（秒）

        self.total = float(self.get_parameter("seconds").value)
        self.period = float(self.get_parameter("period").value)
        self.sim = PassiveWalker(push_speed=float(self.get_parameter("push").value))

        self.pub = self.create_publisher(Int32, TOPIC_NAME, 10)
        self.timer = self.create_timer(self.period, self.on_timer)
        self.finished = False
        self.get_logger().info(
            "被动行走器已启动：%d 根辐条，无执行器（nu=%d），总时长 %.1f s"
            % (self.sim.n_spoke, self.sim.model.nu, self.total))

    def on_timer(self):
        if self.finished:
            return
        self.sim.run(self.period)
        msg = Int32()
        msg.data = int(self.sim.steps)
        self.pub.publish(msg)
        if self.sim.data.time >= self.total:
            self.finished = True
            self.timer.cancel()
            self.get_logger().info(format_summary(self.sim.summary()))


def main(args=None, seconds=20.0, push=0.3):
    rclpy.init(args=args)
    node = PassiveWalkerNode(seconds=seconds, push=push)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
