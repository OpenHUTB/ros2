"""ROS 2 节点: 按参数算一次风包线, 周期性把结果发布成 JSON.

计算全部在 core 里完成（纯 math, 有单测）；这里只负责参数读取、消息发布和日志。
需要在 Ubuntu 22.04 + Humble 上运行：

    ros2 launch uav_hover_wind_limit main.launch.py
    ros2 topic echo /uav/wind_limit
"""
import json

from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String

from .core import build_limits

# (参数名, 默认值); available_thrust_to_weight=0 表示不提供该可选参数.
PARAMETERS = (
    ('mass', 2.0),
    ('drag_area', 0.15),
    ('max_tilt_deg', 25.0),
    ('altitude_m', 0.0),
    ('available_thrust_to_weight', 0.0),
    ('period_s', 5.0),
)


def report_payload(report):
    """把 report 序列化成发布用的 JSON 文本（单独提出来便于无 ROS 测试）."""
    return json.dumps(report, ensure_ascii=False)


def build_from_parameters(values):
    """参数字典 → HoverWindLimit; available_thrust_to_weight 为 0/None 时按未提供处理."""
    tw = values.get('available_thrust_to_weight') or None
    return build_limits(values['mass'], values['drag_area'], values['max_tilt_deg'],
                        values.get('altitude_m', 0.0), tw)


class WindLimitNode(Node):
    """节点: 启动时算一次, 之后按 period_s 重新发布（transient local, 晚加入也能拿到）."""

    def __init__(self):
        super().__init__('uav_hover_wind_limit')
        for name, default in PARAMETERS:
            self.declare_parameter(name, default)
        values = {name: self.get_parameter(name).value for name, _ in PARAMETERS}
        self.limits = build_from_parameters(values)
        self.report = self.limits.report()
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.publisher = self.create_publisher(String, 'uav/wind_limit', qos)
        self.period_s = max(0.5, float(values['period_s']))
        self.timer = self.create_timer(self.period_s, self.publish_once)
        self.publish_once()

    def publish_once(self):
        """发一次 JSON 结果并打一条摘要日志."""
        self.publisher.publish(String(data=report_payload(self.report)))
        budget = self.report['tilt_budget']
        self.get_logger().info(
            'max wind %.2f m/s (Beaufort %d), effective tilt %.2f deg, limited by %s'
            % (self.report['max_wind_m_s'], self.report['max_wind_beaufort'],
               budget['effective_tilt_deg'], budget['limited_by']))


def main(args=None):
    """ROS 2 入口（console_scripts: wind_limit_node）."""
    import rclpy

    rclpy.init(args=args)
    node = WindLimitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover - Ctrl-C 退出
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
