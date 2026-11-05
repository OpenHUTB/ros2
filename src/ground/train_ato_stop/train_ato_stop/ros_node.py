"""ROS 2 节点：把一次 ATO 运行回放到 RViz2。

可视化思路：横轴是线路里程，纵轴不再是高度，而是"速度"——限速、规划包络、
实际速度三条曲线都按 speed_scale 折算成高度画在线路上方，列车本体贴在线路
上走。这样一张 RViz 图里能同时看到"车在哪"和"速度曲线长什么样"。

话题：
  /train_ato/markers  visualization_msgs/MarkerArray  线路、限速段、曲线、列车
  /train_ato/status   std_msgs/String                 当前状态文本
  /train_ato/path     nav_msgs/Path                   已经走过的线路区段
服务：
  /train_ato/replay   std_srvs/Trigger                从头再放一遍
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from . import scenarios, sim
from .ato import AtoConfig
from .train import TrainSpec

FRAME = 'map'
TRACK_Y = 0.0
GRADE_Y = -60.0


def _marker(ns: str, mid: int, kind: int, frame: str = FRAME) -> Marker:
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = kind
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    return m


def _color(m: Marker, r: float, g: float, b: float, a: float = 1.0) -> None:
    m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a


class TrainAtoNode(Node):
    def __init__(self) -> None:
        super().__init__('train_ato_stop')
        self.declare_parameter('scenario', 'flat')
        self.declare_parameter('speed_scale', 4.0)
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('frame_step', 10)

        name = self.get_parameter('scenario').value
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.rate = float(self.get_parameter('playback_rate').value)
        self.step = int(self.get_parameter('frame_step').value)

        self.config = AtoConfig()
        self.spec = TrainSpec()
        self.result = None
        self.idx = 0
        self._load(name)

        self.pub_markers = self.create_publisher(MarkerArray, '/train_ato/markers', 10)
        self.pub_status = self.create_publisher(String, '/train_ato/status', 10)
        self.pub_path = self.create_publisher(Path, '/train_ato/path', 10)
        self.create_service(Trigger, '/train_ato/replay', self._on_replay)
        self.timer = self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"场景 {self.result.title}：{len(self.result.frames)} 帧，"
            f"停车误差 {self.result.metrics['stop_error']:+.2f} m")

    # ------------------------------------------------------------ 场景
    def _load(self, name: str) -> None:
        table = {sc.name: sc for sc in scenarios.all_scenarios()}
        scenario = table.get(name, scenarios.flat())
        self.result = sim.run(scenario, spec=self.spec, config=self.config)
        self.idx = 0
        self.static_markers = self._build_static()

    def _on_replay(self, request, response):
        self.idx = 0
        response.success = True
        response.message = '已重新开始'
        return response

    # ------------------------------------------------------------ 静态部分
    def _build_static(self) -> MarkerArray:
        arr = MarkerArray()
        res = self.result
        ss = self.speed_scale

        track = _marker('track', 0, Marker.LINE_STRIP)
        track.scale.x = 6.0
        _color(track, 0.25, 0.27, 0.30)
        track.points = [Point(x=0.0, y=TRACK_Y, z=1.0),
                        Point(x=res.stop_x, y=TRACK_Y, z=1.0)]
        arr.markers.append(track)

        # 限速区段：颜色区分 160 / 80
        for i, seg in enumerate(res.segments):
            m = _marker('limit_zone', i + 1, Marker.LINE_STRIP)
            m.scale.x = 14.0
            top = max(s.limit_kph for s in res.segments)
            if seg.limit_kph < top - 1e-6:
                _color(m, 0.78, 0.42, 0.12)
            else:
                _color(m, 0.22, 0.45, 0.65)
            m.points = [Point(x=seg.start, y=TRACK_Y, z=1.5),
                        Point(x=seg.end, y=TRACK_Y, z=1.5)]
            arr.markers.append(m)

        # 坡度色带
        for i, seg in enumerate(res.segments):
            if abs(seg.grade) < 0.01:
                continue
            m = _marker('grade', 20 + i, Marker.LINE_STRIP)
            m.scale.x = 10.0
            if seg.grade > 0:
                _color(m, 0.07, 0.47, 0.43)
            else:
                _color(m, 0.60, 0.35, 0.10)
            m.points = [Point(x=seg.start, y=GRADE_Y, z=1.5),
                        Point(x=seg.end, y=GRADE_Y, z=1.5)]
            arr.markers.append(m)

        # 起终点站台
        for i, (x, label) in enumerate(((0.0, 'A'), (res.stop_x, 'B'))):
            post = _marker('station', 40 + i, Marker.CUBE)
            post.scale.x, post.scale.y, post.scale.z = 14.0, 90.0, 4.0
            post.pose.position.x = x
            post.pose.position.y = TRACK_Y
            post.pose.position.z = 2.0
            _color(post, 0.75, 0.23, 0.19)
            arr.markers.append(post)

            text = _marker('station_text', 50 + i, Marker.TEXT_VIEW_FACING)
            text.pose.position.x = x
            text.pose.position.y = TRACK_Y - 90.0
            text.pose.position.z = 5.0
            text.scale.z = 45.0
            text.text = f'STATION {label}'
            _color(text, 0.75, 0.23, 0.19)
            arr.markers.append(text)

        # 限速包络与 ATP 允许速度（画在速度空间里）
        for mid, (profile, rgb) in enumerate((
                (res.profile, (0.12, 0.32, 0.57)),
                (res.atp_profile, (0.60, 0.41, 0.12)))):
            m = _marker('speed_curve', 70 + mid, Marker.LINE_STRIP)
            m.scale.x = 6.0
            _color(m, *rgb)
            m.points = [Point(x=x, y=TRACK_Y + v * 3.6 * ss, z=2.0)
                        for x, v in zip(profile.x, profile.v_envelope)]
            arr.markers.append(m)

        return arr

    # ------------------------------------------------------------ 每帧
    def _tick(self) -> None:
        res = self.result
        total = len(res.frames)
        frame = res.frames[self.idx]
        ss = self.speed_scale

        arr = MarkerArray()
        arr.markers.extend(self.static_markers.markers)

        # 已经跑过的实际速度曲线
        passed = _marker('actual', 90, Marker.LINE_STRIP)
        passed.scale.x = 6.0
        _color(passed, 0.78, 0.33, 0.17)
        end = min(self.idx + 1, total)
        step = max(1, len(res.frames) // 600)
        pts = [Point(x=res.frames[i].x, y=TRACK_Y + res.frames[i].v * 3.6 * ss, z=3.0)
               for i in range(0, end, step)]
        if end - 1 >= 0:
            pts.append(Point(x=frame.x, y=TRACK_Y + frame.v * 3.6 * ss, z=3.0))
        passed.points = pts
        arr.markers.append(passed)

        # 列车本体
        train = _marker('train', 100, Marker.CUBE)
        train.scale.x, train.scale.y, train.scale.z = 34.0, 16.0, 6.0
        train.pose.position.x = frame.x
        train.pose.position.y = TRACK_Y
        train.pose.position.z = 4.0
        _color(train, 0.12, 0.13, 0.15)
        arr.markers.append(train)

        # 状态文字：截图时不用另外去 echo 话题也能看懂当前在干什么
        label = _marker('status_text', 103, Marker.TEXT_VIEW_FACING)
        label.pose.position.x = 320.0
        label.pose.position.y = 820.0
        label.pose.position.z = 6.0
        label.scale.z = 60.0
        label.text = (f'T={frame.t:.0f}S  V={frame.v * 3.6:.0f}KM/H  '
                      f'REMAIN={res.stop_x - frame.x:.0f}M')
        _color(label, 0.15, 0.18, 0.22)
        arr.markers.append(label)

        # 列车到速度曲线的竖直连线
        link = _marker('link', 101, Marker.LINE_STRIP)
        link.scale.x = 3.0
        _color(link, 0.70, 0.74, 0.78)
        link.points = [Point(x=frame.x, y=TRACK_Y, z=3.0),
                       Point(x=frame.x, y=TRACK_Y + frame.v * 3.6 * ss, z=3.0)]
        arr.markers.append(link)

        # 当前速度点
        dot = _marker('speed_dot', 102, Marker.SPHERE)
        dot.scale.x = dot.scale.y = dot.scale.z = 22.0
        dot.pose.position.x = frame.x
        dot.pose.position.y = TRACK_Y + frame.v * 3.6 * ss
        dot.pose.position.z = 3.0
        if frame.atp != 'normal':
            _color(dot, 0.75, 0.23, 0.19)
        else:
            _color(dot, 0.27, 0.52, 0.23)
        arr.markers.append(dot)

        self.pub_markers.publish(arr)

        status = String()
        status.data = (
            f'T={frame.t:6.1f}s  V={frame.v * 3.6:6.1f}km/h  '
            f'REMAIN={res.stop_x - frame.x:7.1f}m  {frame.note}')
        self.pub_status.publish(status)

        path = Path()
        path.header.frame_id = FRAME
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = []
        for i in range(0, end, max(1, step * 4)):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = res.frames[i].x
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_path.publish(path)

        self.idx += max(1, int(self.step * self.rate))
        if self.idx >= total:
            self.idx = 0


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = TrainAtoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
