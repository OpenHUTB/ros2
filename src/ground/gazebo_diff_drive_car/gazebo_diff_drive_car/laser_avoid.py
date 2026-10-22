"""基于 2D 激光雷达的自主避障节点（ROS 2）。

感知：把 360° 扫描划分为五个扇区，各取最小距离；
      并用邻域连续性校验剔除车体自遮挡产生的孤立近距回波。
决策：三态有限状态机（巡航 / 避障 / 紧急后退），进入与退出避障
      使用不同阈值形成滞回，抑制状态抖动。
控制：由左右两侧可通行度之差决定转向（势场法），并施加最小转向力度
      约束以规避局部极小值；线速度随前方障碍接近程度线性衰减。

话题：
    订阅 ``scan``        sensor_msgs/LaserScan
    发布 ``cmd_vel``     geometry_msgs/Twist
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


#: 五个扇区的角度边界（弧度）
SECTORS = {
    'front': (-math.radians(30.0), math.radians(30.0)),
    'left': (math.radians(30.0), math.radians(90.0)),
    'right': (-math.radians(90.0), -math.radians(30.0)),
    'left_bk': (math.radians(90.0), math.pi),
    'right_bk': (-math.pi, -math.radians(90.0)),
}


class State:
    """有限状态机状态常量。"""

    CRUISE = 'cruise'
    AVOID = 'avoid'
    EMERGENCY = 'emergency'


class LaserAvoidNode(Node):
    """激光雷达避障节点。"""

    def __init__(self):
        """声明参数、创建发布者与订阅者。"""
        super().__init__('laser_avoid')

        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('safe_distance', 1.0)
        self.declare_parameter('clear_distance', 1.4)
        self.declare_parameter('danger_distance', 0.45)
        self.declare_parameter('cruise_speed', 0.35)
        self.declare_parameter('avoid_speed', 0.18)
        self.declare_parameter('back_speed', -0.12)
        self.declare_parameter('min_turn', 0.45)
        self.declare_parameter('max_turn', 1.30)
        self.declare_parameter('turn_gain', 1.8)
        self.declare_parameter('emergency_hold', 0.8)
        self.declare_parameter('self_occlusion_range', 0.30)
        self.declare_parameter('min_close_hits', 5)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)
        self.danger_distance = float(
            self.get_parameter('danger_distance').value)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.avoid_speed = float(self.get_parameter('avoid_speed').value)
        self.back_speed = float(self.get_parameter('back_speed').value)
        self.min_turn = float(self.get_parameter('min_turn').value)
        self.max_turn = float(self.get_parameter('max_turn').value)
        self.turn_gain = float(self.get_parameter('turn_gain').value)
        self.emergency_hold = float(self.get_parameter('emergency_hold').value)
        self.self_occlusion_range = float(
            self.get_parameter('self_occlusion_range').value)
        self.min_close_hits = int(self.get_parameter('min_close_hits').value)

        self.state = State.CRUISE
        self.emergency_started = None
        self.stats = {'scans': 0, 'avoid': 0, 'emergency': 0,
                      'min_front': float('inf')}

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10)

        self.get_logger().info(
            '激光避障已启动：scan=%s, cmd_vel=%s, safe=%.2fm, danger=%.2fm'
            % (self.scan_topic, self.cmd_vel_topic,
               self.safe_distance, self.danger_distance))

    # ------------------------------------------------------------------
    # 感知
    # ------------------------------------------------------------------

    def valid_beams(self, scan):
        """过滤无效量测，返回 ``(角度, 距离)`` 列表。"""
        beams = []
        for index, distance in enumerate(scan.ranges):
            if not (distance == distance):          # nan
                continue
            if distance == float('inf') or distance < scan.range_min:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            while angle >= math.pi:
                angle -= 2.0 * math.pi
            while angle < -math.pi:
                angle += 2.0 * math.pi
            beams.append((angle, distance))
        return beams

    def filter_self_occlusion(self, beams):
        """剔除孤立的近距离回波（车体自遮挡）。

        真实障碍会连续遮挡多束激光，自遮挡通常只影响少数几束，
        因此用邻域内近距回波数量区分二者。
        """
        filtered = []
        total = len(beams)
        window = self.min_close_hits

        for idx, (angle, distance) in enumerate(beams):
            if distance >= self.self_occlusion_range:
                filtered.append((angle, distance))
                continue

            hits = 0
            for offset in range(-window, window + 1):
                j = idx + offset
                if 0 <= j < total and beams[j][1] < self.self_occlusion_range:
                    hits += 1

            if hits >= self.min_close_hits:
                filtered.append((angle, distance))
            # 否则丢弃（自遮挡噪声）

        return filtered

    def sector_distances(self, scan):
        """返回各扇区的最小距离。"""
        beams = self.filter_self_occlusion(self.valid_beams(scan))
        result = {name: scan.range_max for name in SECTORS}

        for angle, distance in beams:
            for name, (low, high) in SECTORS.items():
                if low <= angle < high:
                    if distance < result[name]:
                        result[name] = distance
                    break

        return result

    # ------------------------------------------------------------------
    # 决策与控制
    # ------------------------------------------------------------------

    def scan_callback(self, scan):
        """收到一帧激光数据后决策并发布速度指令。"""
        sectors = self.sector_distances(scan)
        self.stats['scans'] += 1
        front = sectors['front']
        if front < self.stats['min_front']:
            self.stats['min_front'] = front

        state = self.update_state(front)
        twist = Twist()

        if state == State.CRUISE:
            twist.linear.x = self.cruise_speed
            twist.angular.z = 0.0

        elif state == State.AVOID:
            twist.linear.x = self.avoid_linear_speed(front)
            twist.angular.z = self.avoid_angular_speed(
                min(sectors['left'], sectors['left_bk']),
                min(sectors['right'], sectors['right_bk']),
                )

        else:  # EMERGENCY
            left = min(sectors['left'], sectors['left_bk'])
            right = min(sectors['right'], sectors['right_bk'])
            twist.linear.x = self.back_speed
            twist.angular.z = self.max_turn if left >= right else -self.max_turn

        self.cmd_pub.publish(twist)

    def update_state(self, front):
        """按前方距离更新有限状态机状态。"""
        now = self.get_clock().now()

        if self.state == State.EMERGENCY:
            elapsed = (now - self.emergency_started).nanoseconds / 1e9
            if elapsed < self.emergency_hold:
                return State.EMERGENCY
            if front > self.clear_distance:
                self.switch_state(State.CRUISE, front)
                return State.CRUISE
            if front < self.danger_distance:
                # 后退到期但仍过近：继续后退并重置计时，
                # 避免与 AVOID 状态反复切换
                self.emergency_started = now
                return State.EMERGENCY
            self.switch_state(State.AVOID, front)
            return State.AVOID

        if front < self.danger_distance:
            self.emergency_started = now
            self.stats['emergency'] += 1
            self.switch_state(State.EMERGENCY, front)
            return State.EMERGENCY

        if self.state == State.CRUISE:
            if front < self.safe_distance:
                self.stats['avoid'] += 1
                self.switch_state(State.AVOID, front)
                return State.AVOID
        else:
            if front > self.clear_distance:
                self.switch_state(State.CRUISE, front)
                return State.CRUISE

        return self.state

    def switch_state(self, new_state, front):
        """切换状态并记录日志。"""
        if new_state == self.state:
            return
        self.get_logger().info(
            '状态切换 %s -> %s（前方 %.2f m）'
            % (self.state, new_state, front))
        self.state = new_state

    def avoid_linear_speed(self, front):
        """避障线速度：随前方距离接近危险阈值而线性衰减。"""
        span = self.safe_distance - self.danger_distance
        if span <= 1e-6:
            return self.avoid_speed
        ratio = (front - self.danger_distance) / span
        ratio = max(0.0, min(1.0, ratio))
        return self.avoid_speed * ratio

    def avoid_angular_speed(self, left, right):
        """避障角速度：朝较空旷一侧转，并保证最小转向力度。"""
        delta = left - right
        normalized = max(-1.0, min(1.0, delta / 2.0))
        omega = self.turn_gain * normalized

        if abs(omega) < self.min_turn:
            omega = self.min_turn if delta >= 0 else -self.min_turn

        if max(left, right) < self.safe_distance:
            omega = self.max_turn if left >= right else -self.max_turn

        return max(-self.max_turn, min(omega, self.max_turn))


def main(args=None):
    """节点入口。"""
    rclpy.init(args=args)
    node = LaserAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
