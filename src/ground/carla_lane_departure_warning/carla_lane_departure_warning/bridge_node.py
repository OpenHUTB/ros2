#!/usr/bin/env python3
"""CARLA↔ROS 桥接节点（LDW 版）。

职责：
  * 连接 CARLA（同步模式 20 Hz），清理残留 actor，生成自车；
  * 每 tick 从地图 API 取自车所在车道中心线路点，计算带符号横向偏移
    与航向偏差，发布 /ldw/lateral_offset、/ldw/heading_error、
    /ldw/lane_width；
  * 主循环 world.tick()，退出恢复异步并清理。

与 BSD/AEB/CSL 一致的设计：预警语义零控车——本节点不订阅控制话题，
车辆的驱动由演示场景或外部驾驶模块负责。
"""

import math
import os
import sys

import carla
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# 让 colcon install / 源码直跑两种方式都能找到本包
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lane_departure_core import heading_error, lateral_offset  # noqa: E402
import carla_common as cc  # noqa: E402


class LDWBridgeNode(Node):

    def __init__(self):
        super().__init__("ldw_bridge_node")
        self.declare_parameter("host", cc.DEFAULT_HOST)
        self.declare_parameter("port", cc.DEFAULT_PORT)
        self.declare_parameter("town", cc.DEFAULT_MAP)
        self.declare_parameter("blueprint", cc.EGO_BLUEPRINT)

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.town = self.get_parameter("town").value
        self.blueprint = self.get_parameter("blueprint").value

        self.pub_offset = self.create_publisher(Float32, "ldw/lateral_offset", 10)
        self.pub_heading = self.create_publisher(Float32, "ldw/heading_error", 10)
        self.pub_width = self.create_publisher(Float32, "ldw/lane_width", 10)

        self.client, self.world = cc.connect(self.host, self.port, self.town)
        cc.cleanup_actors(self.world)
        self.vehicle, tf = cc.spawn_vehicle(self.world, self.blueprint)
        self.get_logger().info(
            f"自车已生成 ({tf.location.x:.1f}, {tf.location.y:.1f})，"
            "LDW 纯地图感知（无传感器依赖）")

    def measure(self, ego_xy, ego_yaw, wp):
        """几何测量（纯函数，可离线测试）→ (offset, heading_err, lane_width)。

        wp: CARLA waypoint（车道中心线），其 transform 为中心线位姿。
        """
        road_yaw = math.radians(wp.transform.rotation.yaw)
        wxy = (wp.transform.location.x, wp.transform.location.y)
        offset = lateral_offset(ego_xy, wxy, road_yaw)
        herr = heading_error(math.radians(ego_yaw), road_yaw)
        return offset, herr, float(wp.lane_width)

    def spin_once(self):
        tf = self.vehicle.get_transform()
        ego_xy = (tf.location.x, tf.location.y)
        # 仅投影到可行驶车道：偏离过大时避免投影到人行道伪造数据
        wp = self.world.get_map().get_waypoint(
            tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
        offset, herr, width = self.measure(ego_xy, tf.rotation.yaw, wp)
        self.pub_offset.publish(Float32(data=float(offset)))
        self.pub_heading.publish(Float32(data=float(herr)))
        self.pub_width.publish(Float32(data=float(width)))
        self.world.tick()
        return True

    def shutdown(self):
        try:
            self.vehicle.destroy()
        except Exception:  # noqa: BLE001
            pass
        cc.restore_async(self.world)

    def main(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0)
                self.spin_once()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LDWBridgeNode()
    try:
        node.main()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
