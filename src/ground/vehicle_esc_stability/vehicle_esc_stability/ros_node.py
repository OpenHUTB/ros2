"""ROS 2 节点：把一次 ESC 仿真回放到 RViz2。

可视化布局：同一条路上并排两条车道，上车道跑「ESC 关」，下车道跑
「ESC 开」，两辆车同时出发，各自留下轨迹。这样一眼能看出无 ESC 的车
在哪一刻甩出去、有 ESC 的车跟参考线贴得有多紧。

话题：
  /esc_stability/markers  visualization_msgs/MarkerArray  车道、轨迹、车辆、文字
  /esc_stability/status   std_msgs/String                 当前状态文本
服务：
  /esc_stability/replay   std_srvs/Trigger                从头再放一遍
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from . import scenarios, sim

FRAME = 'map'
LANE_GAP = 18.0
LANE_WIDTH = 3.5


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


def _yaw_quaternion(m: Marker, yaw: float) -> None:
    m.pose.orientation.z = math.sin(yaw / 2.0)
    m.pose.orientation.w = math.cos(yaw / 2.0)


class EscStabilityNode(Node):
    """把 ESC / 无 ESC 两次运行并排回放。"""

    def __init__(self) -> None:
        super().__init__('vehicle_esc_stability')
        self.declare_parameter('scenario', 'lane_change')
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('frame_step', 4)
        self.declare_parameter('lane_gap', LANE_GAP)

        name = str(self.get_parameter('scenario').value)
        self.rate = float(self.get_parameter('playback_rate').value)
        self.step = max(1, int(self.get_parameter('frame_step').value))
        self.gap = float(self.get_parameter('lane_gap').value)

        table = {sc.name: sc for sc in scenarios.all_scenarios()}
        scenario = table.get(name, scenarios.lane_change())
        self.off = sim.run(scenario, esc_enabled=False)
        self.on = sim.run(scenario, esc_enabled=True)
        self.total = min(len(self.off.frames), len(self.on.frames))
        # 游标用浮点累加，回放倍速可以是 0.5 这种小数；取帧时再取整
        self.cursor = 0.0

        self.pub_markers = self.create_publisher(MarkerArray, '/esc_stability/markers', 10)
        self.pub_status = self.create_publisher(String, '/esc_stability/status', 10)
        self.create_service(Trigger, '/esc_stability/replay', self._on_replay)
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f'{scenario.title}：无 ESC 最大侧偏角 '
            f'{self.off.metrics["max_beta_deg"]:.1f} deg，有 ESC '
            f'{self.on.metrics["max_beta_deg"]:.1f} deg')

    # ------------------------------------------------------------ 服务
    def _on_replay(self, request, response):
        self.cursor = 0.0
        response.success = True
        response.message = '已重新开始'
        return response

    # ------------------------------------------------------------ 静态元素
    def _road(self, arr: MarkerArray) -> None:
        for row, base_y in ((0, self.gap / 2.0), (1, -self.gap / 2.0)):
            for lane in (-LANE_WIDTH, 0.0, LANE_WIDTH):
                m = _marker('lane', row * 10 + int((lane + 5) * 2), Marker.LINE_STRIP)
                m.scale.x = 1.2
                _color(m, 0.72, 0.78, 0.85)
                m.points = [Point(x=0.0, y=base_y + lane, z=0.0),
                            Point(x=self._xmax(), y=base_y + lane, z=0.0)]
                arr.markers.append(m)

            text = _marker('lane_text', row, Marker.TEXT_VIEW_FACING)
            text.pose.position.x = 20.0
            text.pose.position.y = base_y + LANE_WIDTH * 1.9
            text.pose.position.z = 0.5
            text.scale.z = 2.2
            text.text = 'ESC OFF' if row == 0 else 'ESC ON'
            _color(text, 0.78, 0.33, 0.17) if row == 0 else _color(text, 0.10, 0.36, 0.62)
            arr.markers.append(text)

    def _xmax(self) -> float:
        return max(f.state.x for f in self.on.frames) * 1.05

    def _reference(self, arr: MarkerArray) -> None:
        path = self.on.scenario.path
        if path is None:
            return
        for row, base_y in ((0, self.gap / 2.0), (1, -self.gap / 2.0)):
            m = _marker('reference', row, Marker.LINE_STRIP)
            m.scale.x = 3.0
            _color(m, 0.25, 0.50, 0.29, 0.9)
            m.points = [Point(x=px, y=base_y + py, z=0.2) for px, py in path.points[::4]]
            arr.markers.append(m)

    # ------------------------------------------------------------ 每帧
    def _tick(self) -> None:
        index = min(int(self.cursor), self.total - 1)
        frame_off = self.off.frames[index]
        frame_on = self.on.frames[index]

        arr = MarkerArray()
        self._road(arr)
        self._reference(arr)

        for row, result, base_y, rgb, frame in (
                (0, self.off, self.gap / 2.0, (0.78, 0.33, 0.17), frame_off),
                (1, self.on, -self.gap / 2.0, (0.10, 0.36, 0.62), frame_on)):
            end = min(index + 1, self.total)
            step = max(1, self.total // 400)
            trail = _marker('trail', row, Marker.LINE_STRIP)
            trail.scale.x = 2.4
            _color(trail, *rgb)
            trail.points = [Point(x=result.frames[i].state.x,
                                  y=base_y + result.frames[i].state.y, z=0.3)
                            for i in range(0, end, step)]
            arr.markers.append(trail)

            car = _marker('car', row, Marker.CUBE)
            car.scale.x, car.scale.y, car.scale.z = 4.4, 1.82, 1.2
            car.pose.position.x = frame.state.x
            car.pose.position.y = base_y + frame.state.y
            car.pose.position.z = 0.8
            _yaw_quaternion(car, frame.state.yaw)
            _color(car, *rgb)
            arr.markers.append(car)

            label = _marker('car_text', row, Marker.TEXT_VIEW_FACING)
            label.pose.position.x = frame.state.x
            label.pose.position.y = base_y + frame.state.y + 3.2
            label.pose.position.z = 1.5
            label.scale.z = 2.0
            label.text = (f'beta {frame.state.sideslip * 57.3:.1f} deg  '
                          f'r {frame.state.r * 57.3:.0f} deg/s')
            _color(label, *rgb)
            arr.markers.append(label)

        self.pub_markers.publish(arr)

        status = String()
        status.data = (
            f't={frame_on.t:.1f}s  '
            f'ESC off beta {frame_off.state.sideslip * 57.3:.1f} / '
            f'ESC on beta {frame_on.state.sideslip * 57.3:.1f} deg  '
            f'ESC {frame_on.esc.intensity * 100:.0f}%')
        self.pub_status.publish(status)

        self.cursor += self.step * self.rate
        if self.cursor >= self.total:
            self.cursor = 0.0


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = EscStabilityNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C 与 SIGTERM 走同一分支。SIGTERM 时 rclpy 的信号处理器已经把上下文
        # 关掉了，spin 会抛 ExternalShutdownException；不接住就会打一屏 traceback。
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            # 上下文已经被信号处理器关过就不要重复关，否则 RCLError
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
