"""ROS2 节点：把会遇避碰仿真按帧回放成 RViz2 里能看的动画。

节点启动时先离线程跑完整个场景（几百毫秒），然后按固定频率把逐帧状态
发成 MarkerArray，同时把主船的完整轨迹发成 nav_msgs/Path。
规划本身不依赖 ROS，所以这里只做"算完之后怎么显示"。
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from . import scenarios, sim
from .colreg import ColregConfig

SHIP_RGB = [(0.11, 0.32, 0.57), (0.78, 0.33, 0.17), (0.27, 0.52, 0.24)]
SAFE_RGB = (0.57, 0.69, 0.82)
TEXT_RGB = (0.12, 0.14, 0.16)


def _quat(yaw_deg: float) -> tuple[float, float, float, float]:
    """航向角（0=正北、顺时针）换成 RViz 用的四元数（yaw 从 +x 轴逆时针）。"""
    yaw = math.radians(90.0 - yaw_deg)
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class UsvColregNode(Node):
    def __init__(self) -> None:
        super().__init__('usv_colreg_avoidance')

        self.declare_parameter('scenario', 'crossing')
        self.declare_parameter('d_safe', 400.0)
        self.declare_parameter('playback_rate', 8.0)
        self.declare_parameter('frame_step', 4)

        name = self.get_parameter('scenario').value
        self.config = ColregConfig(d_safe=float(self.get_parameter('d_safe').value))
        self.step = max(1, int(self.get_parameter('frame_step').value))

        self.result = sim.run(scenarios.by_name(name), config=self.config)
        self.index = 0

        self.marker_pub = self.create_publisher(MarkerArray, '/usv_colreg/markers', 10)
        self.path_pub = self.create_publisher(Path, '/usv_colreg/track_a', 10)
        self.status_pub = self.create_publisher(String, '/usv_colreg/status', 10)

        self.create_service(Trigger, '/usv_colreg/replay', self._on_replay)

        self._publish_path()
        rate = max(1.0, float(self.get_parameter('playback_rate').value))
        period = 1.0 / rate
        self.timer = self.create_timer(period, self._tick)

        metrics = self.result.metrics
        self.get_logger().info(
            f'场景 {name}：最小会遇距离 {metrics["min_separation"]:.1f} m，'
            f'碰撞 {metrics["collision"]}，共 {len(self.result.frames)} 帧')

    # ------------------------------------------------------------ 发布
    def _publish_path(self) -> None:
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for frame in self.result.frames[::4]:
            pose = frame.ships[0]
            point = Point(x=float(pose[0]), y=float(pose[1]), z=0.0)
            from geometry_msgs.msg import PoseStamped
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position = point
            qx, qy, qz, qw = _quat(pose[2])
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        self.path_pub.publish(path)

    def _tick(self) -> None:
        frame = self.result.frames[self.index]
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        stamp = self.get_clock().now().to_msg()

        # 已经走过的轨迹
        for i in range(len(frame.ships)):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = f'track_{i}'
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 1.2
            marker.color.r, marker.color.g, marker.color.b = SHIP_RGB[i % 3]
            marker.color.a = 0.85
            marker.points = [
                Point(x=float(f.ships[i][0]), y=float(f.ships[i][1]), z=0.0)
                for f in self.result.frames[:self.index + 1:self.step * 2]
            ]
            markers.markers.append(marker)

        # 船位
        for i, (x, y, heading) in enumerate(frame.ships):
            body = Marker()
            body.header.frame_id = 'map'
            body.header.stamp = stamp
            body.ns = f'ship_{i}'
            body.id = i
            body.type = Marker.CUBE
            body.action = Marker.ADD
            body.pose.position = Point(x=float(x), y=float(y), z=0.0)
            qx, qy, qz, qw = _quat(heading)
            body.pose.orientation.x = qx
            body.pose.orientation.y = qy
            body.pose.orientation.z = qz
            body.pose.orientation.w = qw
            body.scale.x = 14.0
            body.scale.y = 5.0
            body.scale.z = 1.5
            body.color.r, body.color.g, body.color.b = SHIP_RGB[i % 3]
            body.color.a = 1.0
            markers.markers.append(body)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = stamp
            label.ns = f'label_{i}'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(x=float(x), y=float(y), z=8.0)
            label.scale.z = 7.0
            label.color.r, label.color.g, label.color.b = TEXT_RGB
            label.color.a = 1.0
            role = frame.roles[i]
            tag = {'give-way': '让路', 'stand-on': '直航', 'free': '自由'}.get(role, role)
            label.text = f'{"ABC"[i % 3]} {tag}'
            markers.markers.append(label)

        # 主船的安全会遇距离圈
        ring = Marker()
        ring.header.frame_id = 'map'
        ring.header.stamp = stamp
        ring.ns = 'safe_ring'
        ring.id = 0
        ring.type = Marker.LINE_STRIP
        ring.action = Marker.ADD
        ring.scale.x = 0.8
        ring.color.r, ring.color.g, ring.color.b = SAFE_RGB
        ring.color.a = 1.0
        cx, cy, _ = frame.ships[0]
        for k in range(73):
            ang = 2.0 * math.pi * k / 72.0
            ring.points.append(Point(x=float(cx) + self.config.d_safe * math.cos(ang),
                                     y=float(cy) + self.config.d_safe * math.sin(ang),
                                     z=0.0))
        markers.markers.append(ring)

        self.marker_pub.publish(markers)

        status = String()
        status.data = (f't={frame.t:.1f}s '
                       + ' '.join(f'{"ABC"[i]}={frame.roles[i]}/{frame.situations[i]}'
                                  for i in range(len(frame.ships))))
        self.status_pub.publish(status)

        self.index = (self.index + self.step) % len(self.result.frames)

    def _on_replay(self, request, response):
        self.index = 0
        response.success = True
        response.message = '已从头回放'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UsvColregNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
