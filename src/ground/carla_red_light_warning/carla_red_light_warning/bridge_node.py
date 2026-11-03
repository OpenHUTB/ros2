#!/usr/bin/env python3
"""CARLA↔ROS 桥接节点（RLW 版）。

职责：
  * 连接 CARLA（同步模式 20 Hz），清理残留 actor，生成自车；
  * 每 tick 查询自车当前受控交通灯（vehicle.get_traffic_light()）
    及其停止线位置，发布 /rlw/light_state、/rlw/dist_to_stop、
    /rlw/ego_speed；
  * 主循环 world.tick()，退出恢复异步并清理。

与 LDW/AEB/BSD/CSL 一致的设计：预警语义零控车——本节点不订阅
控制话题，车辆驱动由演示场景或外部驾驶模块负责。
"""

import math
import os
import sys

import carla
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

# 让 colcon install / 源码直跑两种方式都能找到本包
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from red_light_core import VALID_LIGHTS  # noqa: E402
import carla_common as cc  # noqa: E402

# 自车不受任何交通灯影响时发布的占位距离（m），保证下游状态机走 normal
FALLBACK_DIST = 1.0e4

_LIGHT_NAME = {"Red": "red", "Yellow": "yellow", "Green": "green",
               "Off": "off", "Unknown": "off"}


def light_state_name(carla_state):
    """carla.TrafficLightState 枚举 → 小写字符串（纯函数，可桩测）。"""
    name = getattr(carla_state, "name", None)
    return _LIGHT_NAME.get(name, "off")


def stop_points_xy(locations):
    """get_stop_locations 返回元素归一化为 (x, y) 列表。

    兼容两种元素形态：carla.Location（自带 .x/.y）与
    carla.Waypoint（.location 为 Location）。
    """
    pts = []
    for e in locations:
        loc = getattr(e, "location", e)
        pts.append((float(loc.x), float(loc.y)))
    return pts


def dist_to_stop(ego_xy, stop_xy_list):
    """自车到最近停车点距离（m）；空列表返回 None（纯函数，可桩测）。"""
    if not stop_xy_list:
        return None
    return min(math.hypot(sx - ego_xy[0], sy - ego_xy[1])
               for sx, sy in stop_xy_list)


class RLWBridgeNode(Node):

    def __init__(self):
        super().__init__("rlw_bridge_node")
        self.declare_parameter("host", cc.DEFAULT_HOST)
        self.declare_parameter("port", cc.DEFAULT_PORT)
        self.declare_parameter("town", cc.DEFAULT_MAP)
        self.declare_parameter("blueprint", cc.EGO_BLUEPRINT)

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.town = self.get_parameter("town").value
        self.blueprint = self.get_parameter("blueprint").value

        self.pub_light = self.create_publisher(String, "rlw/light_state", 10)
        self.pub_dist = self.create_publisher(Float32, "rlw/dist_to_stop", 10)
        self.pub_speed = self.create_publisher(Float32, "rlw/ego_speed", 10)

        self.client, self.world = cc.connect(self.host, self.port, self.town)
        cc.cleanup_actors(self.world)
        self.vehicle, tf = cc.spawn_vehicle(self.world, self.blueprint)
        self.get_logger().info(
            f"自车已生成 ({tf.location.x:.1f}, {tf.location.y:.1f})，"
            "RLW 交通灯感知（零传感器依赖）")

    def sense(self, ego, ego_xy):
        """交通灯感知（可桩测的纯查询封装）→ (light_state, dist_to_stop)。

        优先 vehicle.get_traffic_light()（自车受灯控制时由仿真器
        直接给出当前灯），停止线用 get_stop_locations()（0.9.14+）；
        不可用时回退到灯自身位置距离（较粗，标注为近似值）。
        """
        tl = ego.get_traffic_light()
        if tl is None:
            return "off", FALLBACK_DIST
        state = light_state_name(ego.get_traffic_light_state())
        getter = getattr(tl, "get_stop_locations", None)
        dist = None
        if getter is not None:
            try:
                dist = dist_to_stop(ego_xy, stop_points_xy(getter()))
            except Exception:  # noqa: BLE001  API 行为异常时回退
                dist = None
        if dist is None:
            t = tl.get_transform()
            dist = math.hypot(t.location.x - ego_xy[0],
                              t.location.y - ego_xy[1])
        return state, dist

    def spin_once(self):
        tf = self.vehicle.get_transform()
        ego_xy = (tf.location.x, tf.location.y)
        state, dist = self.sense(self.vehicle, ego_xy)
        self.pub_light.publish(String(data=state))
        self.pub_dist.publish(Float32(data=float(dist)))
        self.pub_speed.publish(Float32(data=float(cc.get_speed(self.vehicle))))
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
    node = RLWBridgeNode()
    try:
        node.main()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
