#!/usr/bin/env python3
"""PX4 Offboard 控制：订阅 /vision/target，输出速度设定点实现视觉追踪。

时序：持续发布 OffboardControlMode（10 Hz）→ t=2.0 s 切 Offboard（DO_SET_MODE）→
t=2.5 s 解锁（ARM），随后按目标偏移做水平居中、按目标面积保持距离。
目标丢失（面积小于 MIN_AREA_DETECT）时输出零速度，保持当前位置不继续前飞。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class OffboardControl(Node):
    """把图像偏差转成 PX4 速度指令的 Offboard 控制节点。"""

    K_Y = 0.12          # 水平居中比例增益
    K_Z = 0.08          # 垂直居中比例增益
    K_D = 0.02          # 距离（像素面积）比例增益
    DESIRED_AREA = 15000.0   # 期望目标面积
    MAX_V = 1.5         # 速度限幅 m/s
    TAKEOFF_ALT = -2.0  # PX4 坐标系向下为正，-2.0 即离地 2 m
    MIN_AREA_DETECT = 300.0  # 与 color_tracker 的有效面积阈值一致

    def __init__(self):
        super().__init__('offboard_control')
        self.off_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.create_subscription(
            PointStamped, '/vision/target', self.target_cb, 10)
        self.dx = self.dy = self.area = 0.0
        self.start = self.get_clock().now()
        self._armed = False
        self.create_timer(0.1, self.control)

    def target_cb(self, msg):
        self.dx = msg.point.x
        self.dy = msg.point.y
        self.area = msg.point.z

    def send_cmd(self, cmd, p1, p2):
        vc = VehicleCommand()
        vc.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        vc.command = cmd
        vc.param1 = p1
        vc.param2 = p2
        vc.target_system = 1
        vc.target_component = 1
        vc.source_system = 1
        vc.source_component = 1
        self.cmd_pub.publish(vc)

    def control(self):
        now = self.get_clock().now()
        t = (now - self.start).nanoseconds * 1e-9

        om = OffboardControlMode()
        om.timestamp = int(now.nanoseconds / 1000)
        om.position = True
        om.velocity = True
        self.off_pub.publish(om)

        ts = TrajectorySetpoint()
        ts.timestamp = int(now.nanoseconds / 1000)
        ts.position = [float('nan'), float('nan'), self.TAKEOFF_ALT]
        if self.area > self.MIN_AREA_DETECT:
            vx = self.K_D * (self.DESIRED_AREA - self.area)
            vy = -self.K_Y * self.dx
            vz = -self.K_Z * self.dy
        else:
            # 未检出目标：保持位置，避免按面积误差持续前飞
            vx = vy = vz = 0.0
        vx = max(-self.MAX_V, min(self.MAX_V, vx))
        vy = max(-self.MAX_V, min(self.MAX_V, vy))
        vz = max(-self.MAX_V, min(self.MAX_V, vz))
        ts.velocity = [vx, vy, vz]
        ts.yaw = 0.0
        self.traj_pub.publish(ts)

        if 2.0 < t < 2.2:
            self.send_cmd(176, 1.0, 6.0)
            self.get_logger().info('Offboard mode set')
        if t > 2.5 and not self._armed:
            self.send_cmd(400, 1.0, 0.0)
            self._armed = True
            self.get_logger().info('ARM sent')


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
