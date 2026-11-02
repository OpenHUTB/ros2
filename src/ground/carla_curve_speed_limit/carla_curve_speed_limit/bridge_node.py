#!/usr/bin/env python3
"""CARLA↔ROS 桥接节点（CSL 版）。

职责：
  * 连接 CARLA（同步模式 20 Hz），清理残留 actor，生成自车；
  * 每 tick 从地图 API 取自车前方 look_ahead 范围的路点，经曲率预瞄
    （取最急弯）计算目标速度，发布 /csl/target_speed、/csl/ego_speed、
    /csl/curve_radius；
  * 订阅 /csl/control（[throttle, brake]）施加到车辆；
  * 主循环 world.tick()，退出恢复异步并清理。

与 BSD/AEB 的关键区别：CSL 不依赖任何传感器——限速决策的数据源是
地图路点，前瞻性与确定性由地图拓扑保证。
"""

import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

# 让 colcon install / 源码直跑两种方式都能找到本包
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from curve_speed_core import CSLParams, target_speed_along, waypoint_curvatures  # noqa: E402
import carla_common as cc  # noqa: E402


class CSLBridgeNode(Node):

    def __init__(self):
        super().__init__("csl_bridge_node")
        self.declare_parameter("host", cc.DEFAULT_HOST)
        self.declare_parameter("port", cc.DEFAULT_PORT)
        self.declare_parameter("town", cc.DEFAULT_MAP)
        self.declare_parameter("blueprint", cc.EGO_BLUEPRINT)
        self.declare_parameter("look_ahead", 30.0)
        self.declare_parameter("a_lat_max", 2.5)
        self.declare_parameter("v_min", 1.5)
        self.declare_parameter("valid_max", 30.0)

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.town = self.get_parameter("town").value
        self.blueprint = self.get_parameter("blueprint").value
        self.p = CSLParams(
            look_ahead=float(self.get_parameter("look_ahead").value),
            a_lat_max=float(self.get_parameter("a_lat_max").value),
            v_min=float(self.get_parameter("v_min").value),
            valid_max=float(self.get_parameter("valid_max").value),
        )

        self.pub_target = self.create_publisher(Float32, "csl/target_speed", 10)
        self.pub_speed = self.create_publisher(Float32, "csl/ego_speed", 10)
        self.pub_radius = self.create_publisher(Float32, "csl/curve_radius", 10)
        self.sub_control = self.create_subscription(
            Float32MultiArray, "csl/control", self._on_control, 10)
        self._control = [0.0, 0.0]

        self.client, self.world = cc.connect(self.host, self.port, self.town)
        cc.cleanup_actors(self.world)
        self.vehicle, tf = cc.spawn_vehicle(self.world, self.blueprint)
        self.get_logger().info(
            f"自车已生成 ({tf.location.x:.1f}, {tf.location.y:.1f})，"
            f"前瞻 {self.p.look_ahead:.0f} m，a_lat_max={self.p.a_lat_max}")

    def _on_control(self, msg):
        self._control = list(msg.data)[:2]

    def _front_points(self):
        """自车当前位置 + 前方 look_ahead 范围路点 (N,2)，按行驶方向排序。"""
        loc = self.vehicle.get_location()
        wp = self.world.get_map().get_waypoint(loc)
        pts = [(loc.x, loc.y)]
        wp_next = wp
        dist, step = 0.0, 2.0
        while dist < self.p.look_ahead:
            nxt = wp_next.next(step)
            if not nxt:
                break
            wp_next = nxt[0]
            t = wp_next.transform.location
            pts.append((t.x, t.y))
            dist += step
        return pts

    def _target_speed(self, pts):
        """前瞻路点 → 目标速度 = min(巡航速度, 预瞄限速)，再受 v_min 兜底。"""
        v = target_speed_along(pts, self.p.a_lat_max, self.p.valid_max)
        return float(max(min(v, self.p.cruise_speed), self.p.v_min))

    def _curve_radius(self, pts):
        """前瞻路点 → 最急弯半径（m）；直道返回 999 哨兵。"""
        if len(pts) < 3:
            return 999.0
        k = float(np.max(np.abs(waypoint_curvatures(np.asarray(pts)))))
        return 1.0 / k if k > 1e-6 else 999.0

    def spin_once(self):
        pts = self._front_points()
        target = self._target_speed(pts)
        speed = cc.get_speed(self.vehicle)
        self.pub_target.publish(Float32(data=float(target)))
        self.pub_speed.publish(Float32(data=float(speed)))
        self.pub_radius.publish(Float32(data=float(self._curve_radius(pts))))
        cc.apply_control(self.vehicle, throttle=self._control[0],
                         brake=self._control[1])
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
    node = CSLBridgeNode()
    try:
        node.main()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
