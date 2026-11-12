# -*- coding: utf-8 -*-
"""ROS 2 节点：把失效包线做成可订阅的诊断信息。

参数给定机型（quad/hexa/octo）与"失效桨编号"后，启动时算一次，发布：

* ``/uav/allocation_margin``（``std_msgs/Float64MultiArray``）：
  ``[正常余量, 单桨失效·自由偏航余量, 单桨失效·零偏航余量, 失效后载重上限 kg]``
* ``/uav/allocation_report``（``std_msgs/String``）：一行结论（要不要自旋）

用途：飞控在掉桨时可以直接比对"当前载荷是否还在包线内"。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from .core import hexa, max_hover, max_hover_zero_yaw, octo, quad_x


class AllocationNode(Node):
    def __init__(self):
        super().__init__("rotor_failure_env_node")

        self.declare_parameter("config", "quad")
        self.declare_parameter("failed_rotor", 0)
        self.declare_parameter("thrust_max", 10.0)
        self.declare_parameter("mass", 2.0)
        self.declare_parameter("rate", 1.0)

        gp = self.get_parameter
        name = str(gp("config").value).lower()
        maker = {"quad": quad_x, "quad-x": quad_x, "hexa": hexa, "octo": octo}.get(name)
        if maker is None:
            self.get_logger().warn("未知机型 %r，回退 quad" % name)
            maker = quad_x

        self.cfg = maker(thrust_max=float(gp("thrust_max").value),
                         mass=float(gp("mass").value))
        self.failed = int(gp("failed_rotor").value)
        if not 0 <= self.failed < self.cfg.n():
            self.get_logger().warn("失效桨编号越界，改为 0")
            self.failed = 0

        self.pub_margin = self.create_publisher(
            Float64MultiArray, "/uav/allocation_margin", 10)
        self.pub_report = self.create_publisher(
            String, "/uav/allocation_report", 10)

        self.normal = max_hover(self.cfg)
        self.free = max_hover(self.cfg, {self.failed})
        self.zero = max_hover_zero_yaw(self.cfg, {self.failed})

        self.get_logger().info(
            "%s（%d 桨）失效 #%d：正常 %.0f%% → 自旋 %.0f%% / 零偏航 %.0f%%，"
            "载重上限 %.2f kg"
            % (self.cfg.name, self.cfg.n(), self.failed,
               100 * self.normal["margin"], 100 * self.free["margin"],
               100 * self.zero["margin"], self.free["mass_limit"]))

        self.create_timer(1.0 / float(gp("rate").value), self.on_timer)

    def on_timer(self):
        m = Float64MultiArray()
        m.data = [float(self.normal["margin"]), float(self.free["margin"]),
                  float(self.zero["margin"]), float(self.free["mass_limit"])]
        self.pub_margin.publish(m)

        if self.zero["margin"] <= 1e-6:
            text = "必须自旋悬停（零偏航不可行）"
        elif self.zero["margin"] > 0.99 * self.free["margin"]:
            text = "可不自旋悬停"
        else:
            text = "自旋可多 %.1f 个百分点" % (100 * (self.free["margin"] - self.zero["margin"]))
        s = String()
        s.data = "%s 失效#%d：%s；载重上限 %.2f kg" % (
            self.cfg.name, self.failed, text, self.free["mass_limit"])
        self.pub_report.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = AllocationNode()
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
