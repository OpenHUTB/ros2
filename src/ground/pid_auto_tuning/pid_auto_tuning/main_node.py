"""RViz 可视化节点：闭环跟踪目标航向，发布当前/目标朝向箭头。

启动后先离线做一次整定，再用整定参数驱动闭环，目标航向每隔
switch_period 秒在 ±target_deg 之间切换，便于截图或录 GIF。
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray

from .autotune import select_best_gains
from .pid_controller import PIDController
from .plant import HeadingPlant
from .relay_tune import run_relay_feedback


class PidAutoTuningNode(Node):
    def __init__(self):
        super().__init__('pid_auto_tuning')

        self.declare_parameter('target_deg', 90.0)
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('switch_period', 6.0)
        self.declare_parameter('u_limit', 2.5)

        target_deg = float(self.get_parameter('target_deg').value)
        self.dt = float(self.get_parameter('dt').value)
        self.switch_period = float(self.get_parameter('switch_period').value)
        self.u_limit = float(self.get_parameter('u_limit').value)
        self.target = math.radians(target_deg)

        self.heading_pub = self.create_publisher(Float64, '~/heading', 10)
        self.target_pub = self.create_publisher(Float64, '~/target', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', 10)

        relay = run_relay_feedback(dt=self.dt, setpoint=self.target, u_max=self.u_limit)
        self.gains, _, info = select_best_gains(relay['ku'], relay['tu'],
                                                setpoint=self.target,
                                                dt=self.dt, u_limit=self.u_limit)
        self.get_logger().info(
            f'整定结果 Ku={relay["ku"]:.3f} Tu={relay["tu"]:.3f}s '
            f'Kp={self.gains[0]:.3f} Ki={self.gains[1]:.3f} Kd={self.gains[2]:.4f}'
            f'（网格边界命中: {info["hit_boundary"]}）')

        self.plant = HeadingPlant(dt=self.dt, u_max=self.u_limit)
        self.pid = PIDController(*self.gains, self.dt, u_limit=self.u_limit)
        self.sign = 1.0
        self.elapsed = 0.0
        self.last_switch = 0.0

        self.timer = self.create_timer(self.dt, self.on_timer)
        self.get_logger().info('发布 ~/markers（实际话题 /pid_auto_tuning/markers），RViz 中 Fixed Frame 设为 map')

    def on_timer(self):
        self.elapsed += self.dt
        if self.elapsed - self.last_switch >= self.switch_period:
            self.sign = -self.sign
            self.last_switch = self.elapsed
            self.pid.reset()

        goal = self.target * self.sign
        error = goal - self.plant.theta
        control = self.pid.update(error)
        self.plant.step(control)

        heading_msg = Float64()
        heading_msg.data = self.plant.theta
        self.heading_pub.publish(heading_msg)

        target_msg = Float64()
        target_msg.data = goal
        self.target_pub.publish(target_msg)

        self.marker_pub.publish(self._build_markers(goal))

    def _build_markers(self, goal):
        markers = MarkerArray()

        current = Marker()
        current.header.frame_id = 'map'
        current.header.stamp = self.get_clock().now().to_msg()
        current.ns = 'heading'
        current.id = 0
        current.type = Marker.ARROW
        current.action = Marker.ADD
        current.pose.orientation.z = math.sin(self.plant.theta / 2.0)
        current.pose.orientation.w = math.cos(self.plant.theta / 2.0)
        current.scale.x = 1.2
        current.scale.y = 0.08
        current.scale.z = 0.08
        current.color.g = 1.0
        current.color.a = 1.0
        markers.markers.append(current)

        target = Marker()
        target.header.frame_id = 'map'
        target.header.stamp = current.header.stamp
        target.ns = 'target'
        target.id = 1
        target.type = Marker.ARROW
        target.action = Marker.ADD
        target.pose.orientation.z = math.sin(goal / 2.0)
        target.pose.orientation.w = math.cos(goal / 2.0)
        target.scale.x = 1.5
        target.scale.y = 0.04
        target.scale.z = 0.04
        target.color.r = 1.0
        target.color.a = 0.6
        markers.markers.append(target)

        return markers


def main(args=None):
    rclpy.init(args=args)
    node = PidAutoTuningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
