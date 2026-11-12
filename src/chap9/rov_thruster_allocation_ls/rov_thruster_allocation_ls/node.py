"""ROS 2 节点：订阅期望六维力/力矩，发布 8 个推力器指令。"""

import csv

import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from rov_thruster_allocation_ls import core


class AllocationNode(Node):
    def __init__(self):
        super().__init__("rov_thruster_allocation_ls")
        self.declare_parameter("limit", 10.0)
        self.declare_parameter("failed", "")
        self.declare_parameter("csv_path", "/tmp/rov_thruster_allocation_ls.csv")
        self.limit = float(self.get_parameter("limit").value)
        failed = str(self.get_parameter("failed").value).strip()
        self.tam = core.build_tam(core.THRUSTERS)
        if failed:
            idx = [int(v) for v in failed.split(",") if v.strip()]
            self.tam = core.fault_mask(self.tam, idx)
            self.get_logger().info("fault mask applied to thrusters %s" % idx)

        self.pub = self.create_publisher(Float64MultiArray, "/rov/thruster_cmd", 10)
        self.status = self.create_publisher(String, "/rov/allocation_state", 10)
        self.create_subscription(Wrench, "/rov/wrench", self.on_wrench, 10)

        self.csv_path = self.get_parameter("csv_path").value
        self.f = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.f)
        self.writer.writerow(["t"] + ["w%d" % i for i in range(6)] +
                             ["t%d" % i for i in range(len(core.THRUSTERS))] +
                             ["utilization", "saturated"])
        self.t = 0.0
        self.count = 0
        self.get_logger().info(
            "thruster allocation ready: %d thrusters limit=%.1fN csv=%s"
            % (len(core.THRUSTERS), self.limit, self.csv_path))

    def on_wrench(self, msg):
        wrench = [msg.force.x, msg.force.y, msg.force.z,
                  msg.torque.x, msg.torque.y, msg.torque.z]
        thrust, achieved, saturated, util = core.allocate(wrench, self.tam, self.limit)

        out = Float64MultiArray()
        out.data = thrust
        self.pub.publish(out)
        self.status.publish(String(data="%.3f,%s,%.3f" % (util, saturated, achieved[2])))

        self.t += 0.05
        self.writer.writerow([round(self.t, 3)] + [round(v, 3) for v in wrench] +
                             [round(v, 3) for v in thrust] + [round(util, 3), int(saturated)])
        if self.count % 20 == 0:
            self.f.flush()
        self.count += 1
        if self.count % 20 == 1:
            self.get_logger().info(
                "wrench=[%s] util=%.2f sat=%s thrust=[%s]"
                % (" ".join("%.2f" % v for v in wrench), util, saturated,
                   " ".join("%.2f" % v for v in thrust)))


def main(args=None):
    rclpy.init(args=args)
    node = AllocationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()
