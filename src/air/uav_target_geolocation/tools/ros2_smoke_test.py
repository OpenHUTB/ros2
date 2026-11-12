#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 链路 smoke test：验证 geo_node 的话题收发。

前提：geo_node 已通过 launch 启动（见 CI workflow 的 ros2 job，或手动
``ros2 launch uav_target_geolocation main.launch.py``）。

流程：发一帧 /uav/odom 位姿 → 发一帧 /gimbal/los 视线 → 发一次
/target/geo/new_station 触发收尾 → 断言 /target/ground 收到单站估计。
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


def main() -> int:
    rclpy.init()
    node = Node("geo_smoke_test")
    got = {"ground": False, "point": None}

    def on_ground(msg: PointStamped) -> None:
        got["ground"] = True
        got["point"] = (msg.point.x, msg.point.y)
        node.get_logger().info(
            f"received ground estimate ({msg.point.x:.2f}, {msg.point.y:.2f})")

    node.create_subscription(PointStamped, "/target/ground", on_ground, 10)
    pub_odom = node.create_publisher(Odometry, "/uav/odom", 10)
    pub_los = node.create_publisher(Vector3Stamped, "/gimbal/los", 10)
    pub_new = node.create_publisher(Empty, "/target/geo/new_station", 10)

    # 等 geo_node 完成订阅建立
    time.sleep(2.0)

    odom = Odometry()
    odom.header.stamp.sec = 10
    odom.header.stamp.nanosec = 0
    odom.header.frame_id = "map"
    odom.pose.pose.position.x = 60.0
    odom.pose.pose.position.y = 0.0
    odom.pose.pose.position.z = 80.0
    odom.pose.pose.orientation.w = 1.0
    pub_odom.publish(odom)

    los = Vector3Stamped()
    los.header.stamp.sec = 10
    los.header.stamp.nanosec = 100_000_000  # 与 odom 差 0.1 s，在限值内
    los.header.frame_id = "body"
    los.vector.x = 0.0
    los.vector.y = 0.0
    los.vector.z = -1.0
    pub_los.publish(los)

    pub_new.publish(Empty())

    deadline = time.time() + 5.0
    while time.time() < deadline and not got["ground"]:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if not got["ground"]:
        raise SystemExit("FAIL: /target/ground 未收到输出")
    # 位姿 (60,0,80)、视线正下方 → 单站打点应落在 (60,0) 附近
    x, y = got["point"]
    if abs(x - 60.0) > 1.0 or abs(y - 0.0) > 1.0:
        raise SystemExit(f"FAIL: 打点偏离预期，得到 ({x:.2f}, {y:.2f})")
    print("PASS: /target/ground 收到单站估计，打点与预期一致")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
