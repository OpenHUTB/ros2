# -*- coding: utf-8 -*-
"""ROS 2 节点：按 ZMP 规划出的轨迹发布关节目标与 ZMP 遥测。

话题
----
* ``~/joint_target`` ``std_msgs/Float64MultiArray`` = [hip_l, knee_l, ankle_l, hip_r, knee_r, ankle_r]
* ``~/zmp_state``    ``std_msgs/Float64MultiArray`` = [com_x, zmp_x, zmp_ref_x, torso_z]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import humanoid_stair_zmp_preview.config as config
import humanoid_stair_zmp_preview.gait as gait


class StairZmpNode(Node):
    def __init__(self):
        super().__init__("stair_zmp_node")
        self.declare_parameter("direction", "up")
        self.declare_parameter("step_height", config.STEP_HEIGHT)
        self.declare_parameter("step_depth", config.STEP_DEPTH)
        self.declare_parameter("period", 0.05)

        self.plan = gait.plan(float(self.get_parameter("step_height").value),
                              float(self.get_parameter("step_depth").value),
                              config.STEP_COUNT, str(self.get_parameter("direction").value))
        self.i = 0
        self.pub_j = self.create_publisher(Float64MultiArray, "~/joint_target", 10)
        self.pub_z = self.create_publisher(Float64MultiArray, "~/zmp_state", 10)
        self.create_timer(max(0.01, float(self.get_parameter("period").value)), self._tick)
        self.get_logger().info(f"规划完成：{len(self.plan['com'])} 帧，方向 {self.get_parameter('direction').value}")

    def _tick(self):
        p = self.plan
        n = len(p["com"])
        if self.i >= n:
            self.i = 0
        k = self.i
        jl, jr = p["joints"]["l"], p["joints"]["r"]
        m = Float64MultiArray()
        m.data = [float(jl["hip"][k]), float(jl["knee"][k]), float(jl["ankle"][k]),
                  float(jr["hip"][k]), float(jr["knee"][k]), float(jr["ankle"][k])]
        self.pub_j.publish(m)
        z = Float64MultiArray()
        fz = float(min(p["feet"]["l"][k, 1], p["feet"]["r"][k, 1]))
        z.data = [float(p["com"][k]), float(p["zmp"][k]), float(p["zmp_ref"][k]),
                  float(config.HIP_Z - config.CROUCH + fz)]
        self.pub_z.publish(z)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = StairZmpNode()
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