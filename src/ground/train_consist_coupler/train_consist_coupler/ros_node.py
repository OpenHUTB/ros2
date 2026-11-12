"""ROS 2 回放节点：把离线算好的编组状态按帧发出来。

没有 ROS 也能跑：导入失败时本文件不会被 main 使用，离线模式只依赖标准库。
"""

import sys

from .scenarios import SCENARIOS
from .sim import run_scenario

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, String
    from visualization_msgs.msg import Marker, MarkerArray
    ROS_AVAILABLE = True
except ImportError:                                     # pragma: no cover
    ROS_AVAILABLE = False


if ROS_AVAILABLE:

    class ConsistNode(Node):
        def __init__(self):
            super().__init__("train_consist_coupler")
            self.declare_parameter("scenario", "consist_head_first")
            self.declare_parameter("playback_rate", 1.0)
            self.declare_parameter("frame_step", 1)
            # >=0 时定格在该帧反复发布，便于截图与排障；-1 表示正常回放
            self.declare_parameter("hold_frame", -1)

            name = self.get_parameter("scenario").value
            if name not in SCENARIOS:
                self.get_logger().warn("未知场景 %s，回退到 consist_head_first" % name)
                name = "consist_head_first"
            self.rate = float(self.get_parameter("playback_rate").value)
            self.step = max(1, int(self.get_parameter("frame_step").value))

            self.get_logger().info("计算场景 %s ..." % name)
            self.result = run_scenario(dict(SCENARIOS[name]))
            self.history = self.result["history"]
            self.length = self.result["train_length"] / self.result["n_cars"]
            self.hold = int(self.get_parameter("hold_frame").value)
            self.frame = 0
            if 0 <= self.hold < len(self.history["t"]):
                self.frame = self.hold

            self.marker_pub = self.create_publisher(MarkerArray, "/train_consist/markers", 10)
            self.force_pub = self.create_publisher(Float64MultiArray, "/train_consist/coupler_force", 10)
            self.status_pub = self.create_publisher(String, "/train_consist/status", 10)
            period = 0.1 if self.hold >= 0 else max(0.01, 0.02 / self.rate)
            self.timer = self.create_timer(period, self.on_timer)
            if self.hold >= 0:
                self.get_logger().info("定格模式：持续发布第 %d 帧（t=%.2f s）"
                                       % (self.frame, self.history["t"][self.frame]))
            else:
                self.get_logger().info("回放开始，共 %d 帧" % len(self.history["t"]))

        def on_timer(self):
            index = self.frame
            if index >= len(self.history["t"]):
                self.timer.cancel()
                self.get_logger().info("回放结束：停车误差 %+0.3f m，最大压钩 %.1f kN，最大拉钩 %.1f kN"
                                       % (self.result["stop_error"],
                                          self.result["peak_buff"] / 1e3,
                                          self.result["peak_draft"] / 1e3))
                return

            positions = self.history["x"][index]
            couplers = self.history["couplers"][index]
            stamp = self.get_clock().now().to_msg()

            markers = MarkerArray()
            for car, centre in enumerate(positions):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = stamp
                marker.ns = "cars"
                marker.id = car
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = float(centre)
                marker.pose.position.z = 2.1          # 车体中心离地高度
                marker.pose.orientation.w = 1.0
                # 车体画短 1 m，留出车钩间隙，否则四节车会连成一条看不出编组
                marker.scale.x = self.length - 1.0
                marker.scale.y = 2.8                  # 车宽
                marker.scale.z = 3.6                  # 车高
                marker.color.a = 0.95
                if couplers:
                    local = min(car, len(couplers) - 1)
                    force = couplers[local]
                    level = min(1.0, abs(force) / 60e3)     # 60 kN 达到最深配色
                    if force >= 0:                          # 压钩：红
                        marker.color.r = 0.85 + 0.15 * level
                        marker.color.g = 0.34 - 0.14 * level
                        marker.color.b = 0.30 - 0.12 * level
                    else:                                   # 拉钩：蓝
                        marker.color.r = 0.22 + 0.10 * level
                        marker.color.g = 0.46 + 0.18 * level
                        marker.color.b = 0.82 + 0.18 * level
                else:
                    marker.color.r, marker.color.g, marker.color.b = 0.32, 0.38, 0.46
                markers.markers.append(marker)

            # 每个车钩上方标注力值，方便在 RViz 里直接读数
            for j, value in enumerate(couplers):
                label = Marker()
                label.header.frame_id = "map"
                label.header.stamp = stamp
                label.ns = "labels"
                label.id = j
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = float(0.5 * (positions[j] + positions[j + 1]))
                label.pose.position.z = 7.0
                label.pose.orientation.w = 1.0
                label.scale.z = 3.0
                label.color.a = 1.0
                if value > 0:                         # 压钩
                    label.color.r, label.color.g, label.color.b = 1.0, 0.45, 0.4
                else:                                 # 拉钩
                    label.color.r, label.color.g, label.color.b = 0.45, 0.7, 1.0
                label.text = "%+.1f kN" % (value / 1e3)
                markers.markers.append(label)

            forces = Float64MultiArray()
            forces.data = [float(value) / 1e3 for value in couplers]
            status = String()
            status.data = ("t=%.2f s  v=%.2f km/h  buff=%.1f kN  draft=%.1f kN"
                           % (self.history["t"][index],
                              self.history["v_head"][index] * 3.6,
                              self.result["peak_buff"] / 1e3,
                              self.result["peak_draft"] / 1e3))

            self.marker_pub.publish(markers)
            self.force_pub.publish(forces)
            self.status_pub.publish(status)
            if self.hold < 0:
                self.frame += self.step


def main(argv=None):
    if not ROS_AVAILABLE:
        print("没有检测到 rclpy，请用离线模式：python3 -m train_consist_coupler.main --offline")
        return 1
    rclpy.init(args=argv)
    node = ConsistNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
