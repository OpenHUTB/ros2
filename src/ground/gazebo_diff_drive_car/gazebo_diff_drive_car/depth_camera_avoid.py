"""基于 RGB-D 深度相机的视觉避障节点（ROS 2）。

感知：把深度图水平划分为左/中/右三个竖直条带，竖直方向只取中间
      25%~75% 行（避开顶部天空与底部地面），对每个条带取低百分位数
      作为该方向的通近程度。取百分位而非最小值/均值，兼顾抗噪与敏感性。
      另外用直方图峰值检测剔除 Gazebo 深度相机对无回波区域输出的
      固定「地板值」——该值占比可达 60% 以上，不清除会锁死统计结果。
决策：与激光版本同构的三态有限状态机。
控制：势场法转向 + 速度随距离衰减 + 原地点转超时保护。

话题：
    订阅 ``camera/depth/image_raw``   sensor_msgs/Image（32FC1）
    发布 ``cmd_vel``                  geometry_msgs/Twist
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class State:
    """有限状态机状态常量。"""

    CRUISE = 'cruise'
    AVOID = 'avoid'
    EMERGENCY = 'emergency'


class DepthCameraAvoidNode(Node):
    """深度相机视觉避障节点。"""

    def __init__(self):
        """声明参数、创建发布者与订阅者。"""
        super().__init__('depth_camera_avoid')

        self.declare_parameter('depth_topic', 'camera/depth/image_raw')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('safe_distance', 1.2)
        self.declare_parameter('clear_distance', 1.6)
        self.declare_parameter('danger_distance', 0.5)
        self.declare_parameter('cruise_speed', 0.30)
        self.declare_parameter('avoid_speed', 0.15)
        self.declare_parameter('back_speed', -0.10)
        self.declare_parameter('min_turn', 0.40)
        self.declare_parameter('max_turn', 1.20)
        self.declare_parameter('turn_gain', 1.6)
        self.declare_parameter('emergency_hold', 0.8)
        self.declare_parameter('max_spin_duration', 1.5)
        self.declare_parameter('depth_percentile', 25.0)
        self.declare_parameter('floor_value_ratio', 0.30)
        self.declare_parameter('floor_bin_width', 0.01)
        self.declare_parameter('roi_row_start', 0.25)
        self.declare_parameter('roi_row_end', 0.75)

        self.depth_topic = self.get_parameter('depth_topic').value
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
        self.max_spin_duration = float(
            self.get_parameter('max_spin_duration').value)
        self.depth_percentile = float(
            self.get_parameter('depth_percentile').value)
        self.floor_value_ratio = float(
            self.get_parameter('floor_value_ratio').value)
        self.floor_bin_width = float(
            self.get_parameter('floor_bin_width').value)
        self.roi_row_start = float(self.get_parameter('roi_row_start').value)
        self.roi_row_end = float(self.get_parameter('roi_row_end').value)

        self.state = State.CRUISE
        self.emergency_started = None
        self.spin_started = None
        self.stats = {'frames': 0, 'avoid': 0, 'emergency': 0,
                      'min_mid': float('inf')}

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10)

        self.get_logger().info(
            '视觉避障已启动：depth=%s, cmd_vel=%s, safe=%.2fm, 百分位=%.0f%%'
            % (self.depth_topic, self.cmd_vel_topic,
               self.safe_distance, self.depth_percentile))

    # ------------------------------------------------------------------
    # 感知
    # ------------------------------------------------------------------

    def image_to_array(self, msg):
        """把深度图消息转换为 ``(H, W)`` 的浮点数组（单位米）。"""
        if msg.encoding == '32FC1':
            dtype = np.dtype(np.float32)
            scale = 1.0
        elif msg.encoding == '16UC1':
            dtype = np.dtype(np.uint16)
            scale = 0.001
        else:
            self.get_logger().warn(
                '不支持的深度图编码：%s' % msg.encoding, once=True)
            return None

        try:
            data = np.frombuffer(msg.data, dtype=dtype)
            image = data.reshape((msg.height, msg.width)).astype(np.float32)
        except (ValueError, TypeError) as exc:
            self.get_logger().warn('深度图解析失败：%s' % exc, once=True)
            return None

        if scale != 1.0:
            image = image * scale
        return image

    @staticmethod
    def detect_floor_value(values, bin_width, ratio_threshold):
        """检测深度图中的固定「地板值」（无效标记）。

        Gazebo 深度相机对无回波区域并非输出 0 或 inf，而是输出一个
        固定常数。该区域可能占据画面很大比例，若不剔除，
        百分位统计会被完全锁死。

        :param values: 一维有效深度数组
        :param bin_width: 直方图分箱宽度（米）
        :param ratio_threshold: 判定为地板值所需的最小像素占比
        :return: 地板值；未检出时返回 None
        """
        if values.size < 100:
            return None

        vmin = float(values.min())
        vmax = float(values.max())
        if vmax - vmin < bin_width:
            return None

        n_bins = int((vmax - vmin) / bin_width) + 1
        n_bins = min(n_bins, 4096)
        hist, edges = np.histogram(values, bins=n_bins)

        peak = int(np.argmax(hist))
        peak_ratio = float(hist[peak]) / float(values.size)
        if peak_ratio >= ratio_threshold:
            return float((edges[peak] + edges[peak + 1]) / 2.0)
        return None

    def roi_percentile(self, image, col_start, col_end):
        """计算指定竖直条带的低百分位深度。

        :return: 该条带的深度百分位数（米）；无有效像素时返回 inf
        """
        height, width = image.shape
        c0 = int(col_start * width)
        c1 = int(col_end * width)
        r0 = int(self.roi_row_start * height)
        r1 = int(self.roi_row_end * height)

        patch = image[r0:r1, c0:c1]
        valid = patch[np.isfinite(patch) & (patch > 0.01)]
        if valid.size == 0:
            return float('inf')

        floor_value = self.detect_floor_value(
            valid, self.floor_bin_width, self.floor_value_ratio)
        if floor_value is not None:
            keep = np.abs(valid - floor_value) > (self.floor_bin_width * 0.5)
            filtered = valid[keep]
            if filtered.size >= 30:
                valid = filtered

        if valid.size == 0:
            return float('inf')

        return float(np.percentile(valid, self.depth_percentile))

    # ------------------------------------------------------------------
    # 决策与控制
    # ------------------------------------------------------------------

    def depth_callback(self, msg):
        """收到一帧深度图后决策并发布速度指令。"""
        image = self.image_to_array(msg)
        if image is None:
            return

        self.stats['frames'] += 1

        d_left = self.roi_percentile(image, 0.00, 0.25)
        d_mid = self.roi_percentile(image, 0.37, 0.62)
        d_right = self.roi_percentile(image, 0.75, 1.00)

        if not np.isfinite(d_mid):
            d_mid = self.clear_distance * 2.0
        if d_mid < self.stats['min_mid']:
            self.stats['min_mid'] = d_mid

        state = self.update_state(d_mid)
        twist = Twist()
        left = d_left if np.isfinite(d_left) else self.clear_distance
        right = d_right if np.isfinite(d_right) else self.clear_distance

        if state == State.CRUISE:
            twist.linear.x = self.cruise_speed
            twist.angular.z = 0.0

        elif state == State.AVOID:
            span = self.safe_distance - self.danger_distance
            ratio = 0.0 if span <= 1e-6 else (d_mid - self.danger_distance) / span
            ratio = max(0.0, min(1.0, ratio))
            twist.linear.x = self.avoid_speed * ratio

            delta = left - right
            normalized = max(-1.0, min(1.0, delta / 2.0))
            omega = self.turn_gain * normalized
            if abs(omega) < self.min_turn:
                omega = self.min_turn if delta >= 0 else -self.min_turn

            if max(left, right) < self.safe_distance:
                omega = self.max_turn if left >= right else -self.max_turn
                twist.linear.x = 0.0
                if self.spin_timed_out():
                    return
            else:
                self.spin_started = None

            twist.angular.z = max(-self.max_turn, min(omega, self.max_turn))

        else:  # EMERGENCY
            twist.linear.x = self.back_speed
            twist.angular.z = self.max_turn if left >= right else -self.max_turn

        self.cmd_pub.publish(twist)

    def spin_timed_out(self):
        """原地点转超时保护：避免传感器异常时无限打转。"""
        now = self.get_clock().now()
        if self.spin_started is None:
            self.spin_started = now
            return False

        elapsed = (now - self.spin_started).nanoseconds / 1e9
        if elapsed <= self.max_spin_duration:
            return False

        self.get_logger().warn(
            '原地转向已持续 %.1fs 仍未脱困，恢复正常行驶' % self.max_spin_duration)
        self.spin_started = None
        self.state = State.CRUISE
        self.cmd_pub.publish(Twist())
        return True

    def update_state(self, d_mid):
        """按中区深度更新有限状态机状态。"""
        now = self.get_clock().now()

        if self.state == State.EMERGENCY:
            elapsed = (now - self.emergency_started).nanoseconds / 1e9
            if elapsed < self.emergency_hold:
                return State.EMERGENCY
            if d_mid > self.clear_distance:
                self.switch_state(State.CRUISE, d_mid)
                return State.CRUISE
            if d_mid < self.danger_distance:
                self.emergency_started = now
                return State.EMERGENCY
            self.switch_state(State.AVOID, d_mid)
            return State.AVOID

        if d_mid < self.danger_distance:
            self.emergency_started = now
            self.stats['emergency'] += 1
            self.switch_state(State.EMERGENCY, d_mid)
            return State.EMERGENCY

        if self.state == State.CRUISE:
            if d_mid < self.safe_distance:
                self.stats['avoid'] += 1
                self.switch_state(State.AVOID, d_mid)
                return State.AVOID
        else:
            if d_mid > self.clear_distance:
                self.switch_state(State.CRUISE, d_mid)
                return State.CRUISE

        return self.state

    def switch_state(self, new_state, distance):
        """切换状态并记录日志。"""
        if new_state == self.state:
            return
        self.get_logger().info(
            '状态切换 %s -> %s（中区 %.2f m）'
            % (self.state, new_state, distance))
        self.state = new_state


def main(args=None):
    """节点入口。"""
    rclpy.init(args=args)
    node = DepthCameraAvoidNode()
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


# 保留 math 导入以便外部按需复用（当前未直接使用）
_ = math
