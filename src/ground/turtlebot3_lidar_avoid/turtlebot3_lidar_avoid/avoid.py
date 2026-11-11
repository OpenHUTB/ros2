"""TurtleBot3 激光雷达自主避障节点。

订阅激光雷达话题获取扫描数据，按扇区划分统计前方/左侧/右侧最近距离，
通过有限状态机（巡航 / 避障 / 紧急后退）与势场法选择转向方向，
最终发布 Twists 速度指令至 ``/cmd_vel`` 控制 TurtleBot3 运动。

算法要点
--------
1. 感知：把 360° 扫描按角度划分为三个扇区，各取最小距离。
   取最小值而非均值，是因为均值会被远处空旷区域稀释，
   导致对近处障碍不敏感。
2. 决策：三态有限状态机。进入避障与退出避障使用不同阈值（滞回），
   避免距离在阈值附近波动时状态高频切换。
3. 控制：转向方向由左右两侧可通行度之差决定；线速度随前方障碍
   接近程度线性衰减，越接近障碍开得越慢。

运行示例
--------
    ros2 run turtlebot3_lidar_avoid avoid_node
    ros2 run turtlebot3_lidar_avoid avoid_node --ros-args -p scan_topic:=/out/scan
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


#: 各扇区的角度边界（弧度）
SECTOR_FRONT = (-math.radians(30.0), math.radians(30.0))
SECTOR_LEFT = (math.radians(30.0), math.radians(90.0))
SECTOR_RIGHT = (-math.radians(90.0), -math.radians(30.0))


class State:
    """有限状态机的状态常量。"""

    CRUISE = 'cruise'
    AVOID = 'avoid'
    EMERGENCY = 'emergency'


class TurtleBot3LidarAvoid(Node):
    """TurtleBot3 激光雷达避障节点。"""

    def __init__(self):
        """创建参数、订阅者与发布者。"""
        super().__init__('turtlebot3_lidar_avoid')

        # ---------------- 参数 ----------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('safe_distance', 0.60)
        self.declare_parameter('clear_distance', 0.90)
        self.declare_parameter('danger_distance', 0.28)
        self.declare_parameter('cruise_speed', 0.15)
        self.declare_parameter('avoid_speed', 0.08)
        self.declare_parameter('back_speed', -0.05)
        self.declare_parameter('min_turn', 0.50)
        self.declare_parameter('max_turn', 1.20)
        self.declare_parameter('turn_gain', 1.50)
        self.declare_parameter('emergency_hold', 0.8)

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

        if self.clear_distance <= self.safe_distance:
            self.get_logger().warn(
                'clear_distance(%.2f) 应大于 safe_distance(%.2f)，'
                '否则滞回失效' % (self.clear_distance, self.safe_distance))

        # ---------------- 状态 ----------------
        self.state = State.CRUISE
        self.emergency_started = None

        # ---------------- 通信接口 ----------------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10)

        self.get_logger().info(
            '避障节点已启动：scan=%s, cmd_vel=%s, safe=%.2fm, danger=%.2fm'
            % (self.scan_topic, self.cmd_vel_topic,
               self.safe_distance, self.danger_distance))

    # ------------------------------------------------------------------
    # 感知
    # ------------------------------------------------------------------

    def sector_distance(self, scan, sector):
        """计算指定角度扇区内的最小有效距离。

        :param scan: ``sensor_msgs/LaserScan`` 消息
        :param sector: ``(起始角, 结束角)`` 元组，单位弧度
        :return: 该扇区最小距离（米）；若扇区内无有效回波则返回量程上限
        """
        low, high = sector
        best = scan.range_max

        for index, distance in enumerate(scan.ranges):
            # 剔除 inf / nan 与小于最小量程的噪声
            if not (distance == distance):
                continue
            if distance == float('inf') or distance < scan.range_min:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            # 规范化到 [-pi, pi)
            while angle >= math.pi:
                angle -= 2.0 * math.pi
            while angle < -math.pi:
                angle += 2.0 * math.pi

            if low <= angle < high and distance < best:
                best = distance

        return best

    # ------------------------------------------------------------------
    # 决策 + 控制
    # ------------------------------------------------------------------

    def scan_callback(self, scan):
        """收到一帧激光数据后决策并发布速度指令。"""
        front = self.sector_distance(scan, SECTOR_FRONT)
        left = self.sector_distance(scan, SECTOR_LEFT)
        right = self.sector_distance(scan, SECTOR_RIGHT)

        state = self.update_state(front)
        twist = Twist()

        if state == State.CRUISE:
            twist.linear.x = self.cruise_speed
            twist.angular.z = 0.0

        elif state == State.AVOID:
            twist.linear.x = self.avoid_linear_speed(front)
            twist.angular.z = self.avoid_angular_speed(left, right)

        else:  # EMERGENCY
            twist.linear.x = self.back_speed
            twist.angular.z = self.max_turn if left >= right else -self.max_turn

        self.cmd_pub.publish(twist)

    def update_state(self, front):
        """按前方距离更新有限状态机状态并返回新状态。

        :param front: 正前方最小距离（米）
        :return: 当前状态字符串
        """
        now = self.get_clock().now()

        if self.state == State.EMERGENCY:
            elapsed = (now - self.emergency_started).nanoseconds / 1e9
            if elapsed < self.emergency_hold:
                return State.EMERGENCY
            if front > self.clear_distance:
                self.switch_state(State.CRUISE, front)
                return State.CRUISE
            if front < self.danger_distance:
                # 后退时长已满但仍过近：继续后退并重置计时。
                # 若此时转入 AVOID，会因距离仍小于 danger_distance 立刻
                # 再进入 EMERGENCY，造成 emergency/avoid 反复切换。
                self.emergency_started = now
                return State.EMERGENCY
            self.switch_state(State.AVOID, front)
            return State.AVOID

        if front < self.danger_distance:
            self.emergency_started = now
            self.switch_state(State.EMERGENCY, front)
            return State.EMERGENCY

        if self.state == State.CRUISE:
            if front < self.safe_distance:
                self.switch_state(State.AVOID, front)
                return State.AVOID
        else:  # AVOID
            if front > self.clear_distance:
                self.switch_state(State.CRUISE, front)
                return State.CRUISE

        return self.state

    def switch_state(self, new_state, front):
        """切换状态并记录日志。

        :param new_state: 目标状态
        :param front: 触发切换时的前方距离，用于日志
        """
        if new_state == self.state:
            return
        self.get_logger().info(
            '状态切换 %s -> %s（前方 %.2f m）'
            % (self.state, new_state, front))
        self.state = new_state

    def avoid_linear_speed(self, front):
        """避障时的线速度：随前方距离接近危险阈值而线性衰减。

        :param front: 正前方最小距离（米）
        :return: 线速度（米/秒）
        """
        span = self.safe_distance - self.danger_distance
        if span <= 1e-6:
            return self.avoid_speed
        ratio = (front - self.danger_distance) / span
        ratio = max(0.0, min(1.0, ratio))
        return self.avoid_speed * ratio

    def avoid_angular_speed(self, left, right):
        """避障时的角速度：朝较空旷一侧转向，并保证最小转向力度。

        :param left: 左侧最小距离（米）
        :param right: 右侧最小距离（米）
        :return: 角速度（弧度/秒）
        """
        delta = left - right
        # 归一化，避免远距离处增益过大
        normalized = max(-1.0, min(1.0, delta / 2.0))
        omega = self.turn_gain * normalized

        # 最小转向力度：规避势场法的局部极小值问题，
        # 避免车辆贴着障碍面平移而不真正转开
        if abs(omega) < self.min_turn:
            omega = self.min_turn if delta >= 0 else -self.min_turn

        # 两侧都被堵住时原地大角度转向
        if max(left, right) < self.safe_distance:
            omega = self.max_turn if left >= right else -self.max_turn

        return max(-self.max_turn, min(omega, self.max_turn))


def main(args=None):
    """节点入口。"""
    rclpy.init(args=args)
    node = TurtleBot3LidarAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前发一次零速度让车停下；若 context 已被 launch 销毁
        # （SIGTERM / 关闭仿真时），publish 会抛 RCLInvalid，直接忽略即可
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
