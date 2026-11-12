"""ROS 2 节点：把一次半主动悬架仿真回放到 RViz2。

可视化布局：两排并排摆开，上排是被动减振器（阻尼固定 2400 / 2100 N·s/m），
下排是半主动（阻尼在 500～6500 N·s/m 之间实时调节）。每排都画车身参考面、
四个减振器、四个车轮、以及脚下那条正在往后滚的路面；最前面一排竖条是四个
减振器此刻的阻尼系数，颜色从蓝（最软）到橙（最硬）。

车身始终保持在世界原点，路面按行驶里程向后滚动，所以看的是"车压过什么"，
而不是"车跑到哪了"。

话题：
  /suspension/markers  visualization_msgs/MarkerArray  路面、车身、减振器、车轮、阻尼条
  /suspension/status   std_msgs/String                 当前时刻的姿态与轮荷
服务：
  /suspension/replay   std_srvs/Trigger                从头再放一遍
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
from .params import DamperConfig, VehicleSpec

FRAME = 'map'

# 静态几何，单位 m。这些量只影响画面比例，不参与动力学
WHEEL_RADIUS = 0.32          # 车轮中心离地高度
BODY_PLANE = 0.42            # 车身参考面（悬架上安装点）高度
WHEEL_WIDTH = 0.22

ROAD_HALF = 3.2              # 路面线半长，m
ROAD_POINTS = 161
BAR_X = (-1.5, -0.5, 0.5, 1.5)
BAR_PLANE = -0.26            # 阻尼条底面的高度，m
BAR_MAX = 0.42               # 阻尼条满量程高度，m
BAR_MIN = 0.012

C_SOFT = (0.16, 0.47, 0.71)  # 最软
C_HARD = (0.78, 0.33, 0.17)  # 最硬
C_PASSIVE = (0.35, 0.38, 0.42)
C_ROAD = (0.55, 0.58, 0.62)
C_WHEEL = (0.22, 0.24, 0.27)


def _marker(ns: str, mid: int, kind: int, frame: str = FRAME) -> Marker:
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = kind
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    return m


def _at(m: Marker, x: float, y: float, z: float) -> None:
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z


def _color(m: Marker, rgb, alpha: float = 1.0) -> None:
    m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], alpha


def _mix(low, high, t: float):
    t = min(1.0, max(0.0, t))
    return tuple(low[i] + (high[i] - low[i]) * t for i in range(3))


def _tilt(m: Marker, pitch: float, roll: float) -> None:
    """按俯仰 + 侧倾拼四元数。小角度下先滚后俯与先俯后滚差别可以忽略。"""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    m.pose.orientation.x = sr * cp
    m.pose.orientation.y = -cr * sp
    m.pose.orientation.z = sr * sp
    m.pose.orientation.w = cr * cp


class SuspensionNode(Node):
    """被动 / 半主动两套减振器同场回放。"""

    def __init__(self) -> None:
        super().__init__('semi_active_suspension')
        self.declare_parameter('scenario', 'speed_bump')
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('frame_step', 15)
        self.declare_parameter('height_gain', 4.0)
        self.declare_parameter('row_gap', 1.5)

        name = str(self.get_parameter('scenario').value)
        self.rate = float(self.get_parameter('playback_rate').value)
        self.step = max(1, int(self.get_parameter('frame_step').value))
        self.gain = max(0.2, float(self.get_parameter('height_gain').value))
        self.gap = float(self.get_parameter('row_gap').value)

        table = {sc.name: sc for sc in scenarios.all_scenarios()}
        scenario = table.get(name, scenarios.speed_bump())
        self.spec = VehicleSpec()
        self.damper = DamperConfig()
        self.rows = (
            # (标签, 结果, 横向位置, 颜色)
            ('PASSIVE', sim.run(scenario, self.spec, semi_active=False), self.gap / 2.0,
             C_PASSIVE),
            ('SEMI-ACTIVE', sim.run(scenario, self.spec, semi_active=True), -self.gap / 2.0,
             C_HARD),
        )
        self.scenario = scenario
        self.total = min(len(self.rows[0][1].frames), len(self.rows[1][1].frames))
        # 游标用浮点累加，回放倍速可以是 0.5 这种小数；取帧时再取整
        self.cursor = 0.0

        self.pub_markers = self.create_publisher(MarkerArray, '/suspension/markers', 10)
        self.pub_status = self.create_publisher(String, '/suspension/status', 10)
        self.create_service(Trigger, '/suspension/replay', self._on_replay)
        self.create_timer(0.03, self._tick)

        passive, active = self.rows[0][1], self.rows[1][1]
        self.get_logger().info(
            f'{scenario.title}：被动峰值 {passive.metrics["accel_peak"]:.2f} m/s^2，'
            f'半主动 {active.metrics["accel_peak"]:.2f} m/s^2')

    # ------------------------------------------------------------ 服务
    def _on_replay(self, request, response):
        self.cursor = 0.0
        response.success = True
        response.message = '已重新开始'
        return response

    # ------------------------------------------------------------ 坐标换算
    def _mount_z(self, frame, corner: int) -> float:
        """车身侧安装点的垂向位置（放大后）。"""
        state = frame.state
        shift = (state.z_s
                 - self.spec.axle_position(corner) * state.pitch
                 + self.spec.lateral_position(corner) * state.roll)
        return BODY_PLANE + shift * self.gain

    def _wheel_z(self, frame, corner: int) -> float:
        return WHEEL_RADIUS + frame.state.z_u[corner] * self.gain

    # ------------------------------------------------------------ 每帧元素
    def _road(self, arr: MarkerArray, row: int, base_y: float, t: float) -> None:
        travel = self.scenario.travel(t)
        points = []
        for k in range(ROAD_POINTS):
            x = -ROAD_HALF + 2.0 * ROAD_HALF * k / (ROAD_POINTS - 1)
            height = self.scenario.road.height(travel + x)
            points.append(Point(x=x, y=base_y, z=height * self.gain))
        line = _marker('road', row, Marker.LINE_STRIP)
        line.scale.x = 0.02
        _color(line, C_ROAD)
        line.points = points
        arr.markers.append(line)

        # 每个轮子正下方的接地点，单独点一个短横，看得出轮胎压缩量
        for corner in range(4):
            ground = self.scenario.road.height(
                travel + self.spec.axle_position(corner))
            pad = _marker('patch', row * 10 + corner, Marker.CUBE)
            pad.scale.x, pad.scale.y, pad.scale.z = 0.26, 0.20, 0.006
            _at(pad, self.spec.axle_position(corner),
                base_y + self.spec.lateral_position(corner),
                ground * self.gain + 0.004)
            _color(pad, C_ROAD, 0.85)
            arr.markers.append(pad)

    def _body(self, arr: MarkerArray, row: int, base_y: float, frame) -> None:
        cube = _marker('body', row, Marker.CUBE)
        cube.scale.x, cube.scale.y, cube.scale.z = 2.62, 1.58, 0.09
        _at(cube, (self.spec.cg_to_front - self.spec.cg_to_rear) / 2.0, base_y,
            BODY_PLANE + frame.state.z_s * self.gain)
        _tilt(cube, frame.state.pitch, frame.state.roll)
        _color(cube, self.rows[row][3], 0.55)
        arr.markers.append(cube)

    def _corner(self, arr: MarkerArray, row: int, base_y: float, frame,
                corner: int) -> None:
        x = self.spec.axle_position(corner)
        y = base_y + self.spec.lateral_position(corner)
        top_z = self._mount_z(frame, corner)
        wheel_z = self._wheel_z(frame, corner)

        strut = _marker('strut', row * 10 + corner, Marker.LINE_STRIP)
        ratio = ((frame.damping[corner] - self.spec.damper_min)
                 / max(1.0, self.spec.damper_max - self.spec.damper_min))
        # 阻尼越大画得越粗，颜色也从蓝走到橙，一眼能看出阀门开度
        strut.scale.x = 0.012 + 0.030 * ratio
        _color(strut, _mix(C_SOFT, C_HARD, ratio))
        strut.points = [Point(x=x, y=y, z=top_z), Point(x=x, y=y, z=wheel_z)]
        arr.markers.append(strut)

        tyre = _marker('wheel', row * 10 + corner, Marker.CYLINDER)
        tyre.scale.x, tyre.scale.y, tyre.scale.z = WHEEL_RADIUS * 2.0, WHEEL_RADIUS * 2.0, WHEEL_WIDTH
        _at(tyre, x, y, wheel_z)
        # 圆柱默认沿 z 轴，绕 x 转 90 度让轮轴指向 y
        tyre.pose.orientation.x = math.sqrt(0.5)
        tyre.pose.orientation.w = math.sqrt(0.5)
        _color(tyre, C_WHEEL)
        arr.markers.append(tyre)

    def _bars(self, arr: MarkerArray, row: int, base_y: float, frame) -> None:
        for index, corner in enumerate((0, 1, 2, 3)):
            ratio = ((frame.damping[corner] - self.spec.damper_min)
                     / max(1.0, self.spec.damper_max - self.spec.damper_min))
            height = BAR_MIN + (BAR_MAX - BAR_MIN) * max(0.0, ratio)
            bar = _marker('bar', row * 10 + corner, Marker.CUBE)
            bar.scale.x, bar.scale.y, bar.scale.z = 0.22, 0.22, height
            _at(bar, BAR_X[index], base_y, BAR_PLANE + height / 2.0)
            _color(bar, _mix(C_SOFT, C_HARD, ratio))
            arr.markers.append(bar)

            label = _marker('bar_text', row * 10 + corner, Marker.TEXT_VIEW_FACING)
            _at(label, BAR_X[index], base_y, BAR_PLANE + BAR_MAX + 0.09)
            label.scale.z = 0.075
            label.text = '%s %4.0f' % (('FL', 'FR', 'RL', 'RR')[index],
                                       frame.damping[corner])
            _color(label, _mix(C_SOFT, C_HARD, ratio))
            arr.markers.append(label)

    def _hud(self, arr: MarkerArray, row: int, base_y: float, frame,
             result) -> None:
        title = _marker('title', row, Marker.TEXT_VIEW_FACING)
        _at(title, -1.35, base_y, BODY_PLANE + 0.34)
        title.scale.z = 0.11
        title.text = self.rows[row][0]
        _color(title, self.rows[row][3])
        arr.markers.append(title)

        text = _marker('hud', row, Marker.TEXT_VIEW_FACING)
        _at(text, 1.05, base_y, BODY_PLANE + 0.34)
        text.scale.z = 0.082
        text.text = ('PEAK %.2f M/S2  PITCH %.2f  ROLL %.2f DEG'
                     % (result.metrics['accel_peak'], frame.pitch_deg, frame.roll_deg))
        _color(text, C_ROAD)
        arr.markers.append(text)

    # ------------------------------------------------------------ 主循环
    def _tick(self) -> None:
        index = min(int(self.cursor), self.total - 1)
        arr = MarkerArray()
        for row, (_, result, base_y, _) in enumerate(self.rows):
            frame = result.frames[index]
            self._road(arr, row, base_y, frame.t)
            self._body(arr, row, base_y, frame)
            for corner in range(4):
                self._corner(arr, row, base_y, frame, corner)
            self._bars(arr, row, base_y, frame)
            self._hud(arr, row, base_y, frame, result)
        self.pub_markers.publish(arr)

        passive, active = self.rows[0][1], self.rows[1][1]
        index = min(index, min(len(passive.frames), len(active.frames)) - 1)
        frame_p = passive.frames[index]
        frame_a = active.frames[index]
        status = String()
        status.data = (
            't=%.2fs  %s  pitch %.2f -> %.2f deg  roll %.2f -> %.2f deg  '
            'min load %.0f -> %.0f N'
            % (frame_a.t, self.scenario.name,
               frame_p.pitch_deg, frame_a.pitch_deg,
               frame_p.roll_deg, frame_a.roll_deg,
               min(frame_p.tire_load), min(frame_a.tire_load)))
        self.pub_status.publish(status)

        self.cursor += self.step * self.rate
        if self.cursor >= self.total:
            self.cursor = 0.0


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = SuspensionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C 与 SIGTERM 走同一分支。SIGTERM 时 rclpy 的信号处理器已经把上下文
        # 关掉了，这里再抛一个 rclpy 自己的 ExternalShutdownException 出来，不接住
        # 就会打一屏 traceback（rcl_shutdown already called）。
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
