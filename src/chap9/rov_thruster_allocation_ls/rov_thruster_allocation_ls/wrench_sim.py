"""按预设时序发布期望力/力矩，驱动分配节点。"""

import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node

# (持续时间 s, force xyz, torque xyz)
SCRIPT = [
    (6.0, (0.0, 0.0, 4.0), (0.0, 0.0, 0.0)),
    (6.0, (2.0, 0.0, 2.0), (0.0, 0.0, 3.0)),
    (6.0, (6.0, 0.0, 1.0), (2.0, 0.0, 0.0)),
    (6.0, (0.0, 0.0, 60.0), (0.0, 0.0, 0.0)),
]


class WrenchSim(Node):
    def __init__(self):
        super().__init__("wrench_sim")
        self.pub = self.create_publisher(Wrench, "/rov/wrench", 10)
        self.create_timer(0.1, self.step)
        self.index = 0
        self.elapsed = 0.0
        self.get_logger().info("wrench_sim: %d phases" % len(SCRIPT))

    def step(self):
        if self.index >= len(SCRIPT):
            return
        duration, force, torque = SCRIPT[self.index]
        if self.elapsed >= duration:
            self.index += 1
            self.elapsed = 0.0
            if self.index >= len(SCRIPT):
                self.get_logger().info("wrench_sim finished")
                return
            duration, force, torque = SCRIPT[self.index]
            self.get_logger().info("phase %d: force=%s torque=%s" % (self.index, force, torque))
        msg = Wrench()
        msg.force.x, msg.force.y, msg.force.z = force
        msg.torque.x, msg.torque.y, msg.torque.z = torque
        self.pub.publish(msg)
        self.elapsed += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = WrenchSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
