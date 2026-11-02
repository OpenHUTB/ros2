"""ROS2 节点：规划垂直车位泊车路径并发布到 RViz2 可视化。

发布话题
--------
* ``/parking_path``     ``nav_msgs/Path``           泊车轨迹（可直接交给控制器跟踪）
* ``/parking_markers``  ``visualization_msgs/MarkerArray`` 车位框、目标位姿、双色轨迹

服务
----
* ``/plan_parking``     ``std_srvs/Trigger``        重新规划
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .planner import (PlannerConfig, SlotParams, VehicleParams,
                      count_gear_shifts, densify, goal_pose_from_slot,
                      plan_parking, slot_corners_world)


def yaw_to_quaternion(yaw):
    """偏航角转四元数（只绕 Z 轴旋转）。"""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class ParkingPathPlanner(Node):
    """泊车路径规划节点。"""

    def __init__(self):
        super().__init__('parking_path_planner')

        # ---------------- 参数 ----------------
        self.frame_id = self.declare_parameter('frame_id', 'map').value

        self.slot = SlotParams(
            width=self.declare_parameter('slot_width', 2.70).value,
            depth=self.declare_parameter('slot_depth', 5.50).value,
            center_x=self.declare_parameter('slot_center_x', 0.0).value,
            center_y=self.declare_parameter('slot_center_y', 2.75).value,
            heading_deg=self.declare_parameter('slot_heading_deg', 0.0).value,
        )
        self.vehicle = VehicleParams(
            wheelbase=self.declare_parameter('wheelbase', 2.70).value,
            width=self.declare_parameter('vehicle_width', 1.80).value,
            length=self.declare_parameter('vehicle_length', 4.50).value,
            max_steer_deg=self.declare_parameter('max_steer_deg', 35.0).value,
        )
        self.start = (
            self.declare_parameter('start_x', -7.0).value,
            self.declare_parameter('start_y', -4.20).value,
            math.radians(self.declare_parameter('start_yaw_deg', 0.0).value),
        )
        self.config = PlannerConfig(
            step=self.declare_parameter('step', 0.40).value,
            goal_xy_tolerance=self.declare_parameter('goal_xy_tolerance', 0.35).value,
            neighbors_parked=self.declare_parameter('neighbors_parked', True).value,
        )
        auto_plan = self.declare_parameter('auto_plan', True).value

        # ---------------- 通信 ----------------
        self.result = None
        self.path_pub = self.create_publisher(Path, '/parking_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/parking_markers', 10)
        self.create_service(Trigger, '/plan_parking', self.on_plan)
        # RViz2 启动晚于节点时也能看到结果
        self.create_timer(1.0, self.publish_all)

        if auto_plan:
            self.plan()

        self.get_logger().info(
            '泊车路径规划节点已启动：调用 `ros2 service call /plan_parking '
            'std_srvs/srv/Trigger {}` 重新规划')

    # ---------------- 规划 ----------------
    def plan(self):
        goal = goal_pose_from_slot(self.slot, self.vehicle)
        self.result = plan_parking(self.start, goal, self.vehicle, self.slot,
                                   self.config)
        if self.result.success:
            self.get_logger().info(
                f'规划成功：里程 {self.result.cost:.2f} m，'
                f'换挡 {count_gear_shifts(self.result.poses)} 次，'
                f'扩展 {self.result.iterations} 次')
        else:
            self.get_logger().warn(f'规划失败：{self.result.message}')
        self.publish_all()
        return self.result

    def on_plan(self, request, response):
        result = self.plan()
        response.success = bool(result.success)
        response.message = (
            f'里程 {result.cost:.2f} m，换挡 {count_gear_shifts(result.poses)} 次'
            if result.success else result.message)
        return response

    # ---------------- 发布 ----------------
    def publish_all(self):
        stamp = self.get_clock().now().to_msg()

        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp
        if self.result is not None and self.result.success:
            for x, y, yaw, _gear in densify(self.result.poses):
                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.header.stamp = stamp
                pose.pose.position.x = x
                pose.pose.position.y = y
                qz, qw = yaw_to_quaternion(yaw)
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                path.poses.append(pose)
        self.path_pub.publish(path)
        self.marker_pub.publish(self.build_markers(stamp))

    def _line_marker(self, stamp, namespace, marker_id, points, rgba, width=0.06):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        marker.pose.orientation.w = 1.0
        for x, y in points:
            p = Point()
            p.x, p.y, p.z = x, y, 0.02
            marker.points.append(p)
        return marker

    def build_markers(self, stamp):
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        # 车位框
        slot_points = slot_corners_world(self.slot)
        markers.markers.append(
            self._line_marker(stamp, 'slot', 0, slot_points + slot_points[:1],
                              (0.10, 0.80, 0.30, 1.0), width=0.05))

        # 目标位姿箭头
        gx, gy, gyaw = goal_pose_from_slot(self.slot, self.vehicle)
        goal = Marker()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = stamp
        goal.ns = 'goal'
        goal.id = 1
        goal.type = Marker.ARROW
        goal.action = Marker.ADD
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = 0.05
        qz, qw = yaw_to_quaternion(gyaw)
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        goal.scale.x, goal.scale.y, goal.scale.z = 1.2, 0.16, 0.16
        goal.color.r, goal.color.g, goal.color.b, goal.color.a = 0.90, 0.20, 0.20, 1.0
        markers.markers.append(goal)

        # 前进段（蓝）与倒车段（橙）
        if self.result is not None and self.result.success:
            dense = densify(self.result.poses)
            for gear, namespace, marker_id, rgba in (
                    (1, 'path_forward', 10, (0.10, 0.45, 0.85, 1.0)),
                    (-1, 'path_reverse', 11, (0.90, 0.42, 0.15, 1.0))):
                segment = [(x, y) for x, y, _yaw, g in dense if g == gear]
                if len(segment) >= 2:
                    markers.markers.append(
                        self._line_marker(stamp, namespace, marker_id,
                                          segment, rgba, width=0.09))
        return markers


def main(args=None):
    rclpy.init(args=args)
    node = ParkingPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
