# Copyright 2026 Toeting
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""禁飞区检测与安全预警节点."""

from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def _point_on_segment(px, py, ax, ay, bx, by, eps=1e-9):
    """判断点 (px,py) 是否在线段 (ax,ay)-(bx,by) 上（含端点）."""
    seg_len2 = (bx - ax) ** 2 + (by - ay) ** 2
    if seg_len2 <= eps:
        # 退化边（重复顶点）不构成边界，否则任意点都会被判为"在边上"，
        # 进而让整个围栏恒返回 False（永远 SAFE）
        return False
    cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
    if abs(cross) > eps:
        return False
    dot = (px - ax) * (bx - ax) + (py - ay) * (by - ay)
    if dot < -eps:
        return False
    if dot > seg_len2 + eps:
        return False
    return True


def is_point_in_polygon(x, y, polygon):
    """
    使用射线法判断点是否在凸/简单多边形内部.

    * 顶点顺序不影响结果（顺时针/逆时针均可）
    * 重复顶点（零长度边）会被忽略，不影响判定
    * 边界（落在边上或顶点）一律返回 False，仅内部返回 True
    * 空多边形、退化多边形（< 3 点）返回 False
    """
    if not polygon or len(polygon) < 3:
        return False

    # 先显式排除边界点，保证语义一致
    n = len(polygon)
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]
        if _point_on_segment(x, y, ax, ay, bx, by):
            return False

    # 射线法：向右发射，统计穿过次数
    inside = False
    p1x, p1y = polygon[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
        p1x, p1y = p2x, p2y
    return inside


class GeofenceNode(Node):
    """禁飞区检测核心节点."""

    def __init__(self):
        """初始化订阅者和发布者."""
        super().__init__('geofence_node')

        # 可通过参数指定期望的坐标系 frame_id，为空则不校验
        self.declare_parameter('expected_frame_id', 'map')
        self.expected_frame_id = self.get_parameter(
            'expected_frame_id').get_parameter_value().string_value

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.geofence_sub = self.create_subscription(
            PolygonStamped, '/geofence', self.geofence_callback, 10)
        self.status_pub = self.create_publisher(
            String, '/geofence_status',
            # 状态是"当前值"语义：用 TRANSIENT_LOCAL 做 latched，
            # 后启动的订阅者也能立即拿到最近一次状态
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.current_x = 0.0
        self.current_y = 0.0
        self.odom_frame_id = None
        self.polygon_points = []
        self.geofence_frame_id = None
        self.last_status = None
        self.get_logger().info('禁飞区检测节点已启动，等待数据...')

    def geofence_callback(self, msg):
        """接收围栏多边形顶点并记录 frame_id."""
        self.geofence_frame_id = msg.header.frame_id
        self.polygon_points = [(p.x, p.y) for p in msg.polygon.points]
        if self.polygon_points:
            # 围栏更新后重发一次当前状态，避免新旧围栏状态相同时不重发
            self.last_status = None
            self.get_logger().info(
                f'收到新的禁飞区配置，共 {len(self.polygon_points)} 个顶点，'
                f'frame_id={self.geofence_frame_id}')

    def odom_callback(self, msg):
        """接收载具位置并进行判断."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.odom_frame_id = msg.header.frame_id

        # 坐标系一致性校验：要求 odom 与 geofence 在同一 frame_id 下
        if (self.expected_frame_id and
                self.odom_frame_id is not None and
                self.geofence_frame_id is not None):
            if (self.odom_frame_id != self.geofence_frame_id or
                    self.odom_frame_id != self.expected_frame_id):
                self.get_logger().warn(
                    f'坐标系不一致！odom={self.odom_frame_id}, '
                    f'geofence={self.geofence_frame_id}, '
                    f'expected={self.expected_frame_id}；'
                    f'请先用 tf2 转换到同一坐标系再检测。')
                return

        if not self.polygon_points:
            return

        inside = is_point_in_polygon(
            self.current_x, self.current_y, self.polygon_points)
        current_status = 'CRITICAL' if inside else 'SAFE'

        if current_status != self.last_status:
            # 消息体只放枚举字符串，坐标写进日志，便于下游直接比较
            status_msg = String()
            status_msg.data = current_status
            self.status_pub.publish(status_msg)
            self.last_status = current_status

            if inside:
                self.get_logger().error(
                    f'闯入禁飞区！坐标: '
                    f'({self.current_x:.1f}, {self.current_y:.1f})')
            else:
                self.get_logger().info(
                    f'安全。坐标: '
                    f'({self.current_x:.1f}, {self.current_y:.1f})')


def main(args=None):
    """节点主入口."""
    rclpy.init(args=args)
    node = GeofenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
