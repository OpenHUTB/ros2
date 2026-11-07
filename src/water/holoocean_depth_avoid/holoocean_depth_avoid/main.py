#!/usr/bin/env python3
"""基于深度图的水下机器人前视避障 + 海底定高巡航节点。

订阅深度相机图像 (sensor_msgs/Image, 32FC1, 单位米)，发布 /auv/cmd_vel (Twist)。
- 前向区域最近深度 < danger_dist  -> 触发脱困链：直退 -> 原地转向 -> 向前脱离
- 持续用底部区域中位深度做定高，避免贴底或上浮到水面
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class AuvDepthAvoid(Node):
    def __init__(self):
        super().__init__('auv_depth_avoid')

        # ---- 可参数化的话题与行为参数 ----
        self.declare_parameter('depth_topic', '/auv/depth/depth/image_raw')
        self.declare_parameter('cmd_topic', '/auv/cmd_vel')
        self.declare_parameter('danger_dist', 1.0)      # 前方低于此距离(m) 触发脱困
        self.declare_parameter('warn_dist', 2.0)        # 前方低于此距离(m) 减速绕开
        self.declare_parameter('cruise_speed', 0.8)     # 巡航前进速度
        self.declare_parameter('back_speed', -0.6)      # 直退速度
        self.declare_parameter('turn_rate', 0.6)        # 原地转向角速度
        self.declare_parameter('target_altitude', 1.5)  # 期望离底高度(m)
        self.declare_parameter('alt_deadband', 0.3)     # 定高死区(m)
        self.declare_parameter('heave_gain', 0.4)       # 定高垂向速度增益
        # 状态计时（单位：图像回调次数，约 0.1s/次）
        self.declare_parameter('t_back', 12)
        self.declare_parameter('t_turn', 18)
        self.declare_parameter('t_after', 12)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.danger_dist = float(self.get_parameter('danger_dist').value)
        self.warn_dist = float(self.get_parameter('warn_dist').value)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.back_speed = float(self.get_parameter('back_speed').value)
        self.turn_rate = float(self.get_parameter('turn_rate').value)
        self.target_alt = float(self.get_parameter('target_altitude').value)
        self.alt_band = float(self.get_parameter('alt_deadband').value)
        self.heave_gain = float(self.get_parameter('heave_gain').value)
        self.t_back = int(self.get_parameter('t_back').value)
        self.t_turn = int(self.get_parameter('t_turn').value)
        self.t_after = int(self.get_parameter('t_after').value)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub = self.create_subscription(
            Image, self.depth_topic, self.depth_cb, 10)

        # ---- 状态机 ----
        # GO / AVOID / ESCAPE_BACK / ESCAPE_TURN / AFTER_ESCAPE
        self.state = 'GO'
        self.timer = 0
        self.turn_dir = 1.0   # 1.0 右转, -1.0 左转

        self.get_logger().info(
            f'auv_depth_avoid 已启动: 订阅 {self.depth_topic} -> 发布 {self.cmd_topic}')

    def depth_cb(self, msg):
        try:
            # 32FC1: float32, 单位米; 远处为 inf
            depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except Exception as e:
            self.get_logger().warn(f'深度图转换失败: {e}')
            return

        h, w = depth.shape[:2]
        # 只保留有限值
        valid = depth[np.isfinite(depth)]

        # 前向 ROI：画面中央矩形区域（前方障碍物）
        fx0, fx1 = int(w * 0.30), int(w * 0.70)
        fy0, fy1 = int(h * 0.30), int(h * 0.70)
        fwd = depth[fy0:fy1, fx0:fx1]
        fwd_valid = fwd[np.isfinite(fwd)]
        fwd_min = float(np.min(fwd_valid)) if fwd_valid.size else 99.0

        # 底部 ROI：画面下方中央（到海底的距离 -> 定高）
        bx0, bx1 = int(w * 0.35), int(w * 0.65)
        by0, by1 = int(h * 0.75), int(h * 0.95)
        bot = depth[by0:by1, bx0:bx1]
        bot_valid = bot[np.isfinite(bot)]
        bottom_alt = float(np.median(bot_valid)) if bot_valid.size else 99.0

        vel = Twist()

        # ---- 脱困计时中，不做新决策 ----
        if self.timer > 0:
            self.timer -= 1
            if self.state == 'ESCAPE_BACK':
                vel.linear.x = self.back_speed
                self.get_logger().warn(f'[直退] 剩余 {self.timer}')
            elif self.state == 'ESCAPE_TURN':
                vel.linear.x = 0.0
                vel.angular.z = self.turn_dir * self.turn_rate
                self.get_logger().warn(f'[原地转向] 剩余 {self.timer}')
            elif self.state == 'AFTER_ESCAPE':
                vel.linear.x = self.cruise_speed
                vel.angular.z = self.turn_dir * 0.2
                self.get_logger().info(f'[脱困后脱离] 剩余 {self.timer}')
            self._apply_heave(vel, bottom_alt)
            self.pub.publish(vel)
            return

        # ---- 计时归零：状态迁移 ----
        if self.state == 'ESCAPE_BACK':
            self.state = 'ESCAPE_TURN'
            self.timer = self.t_turn
            return
        if self.state == 'ESCAPE_TURN':
            self.state = 'AFTER_ESCAPE'
            self.timer = self.t_after
            return
        if self.state == 'AFTER_ESCAPE':
            self.state = 'GO'

        # ---- 正常决策 ----
        if fwd_min < self.danger_dist:
            # 真危险：左/右哪边开阔往哪转
            left = depth[int(h*0.3):int(h*0.7), 0:int(w*0.25)]
            right = depth[int(h*0.3):int(h*0.7), int(w*0.75):w]
            lv = left[np.isfinite(left)]
            rv = right[np.isfinite(right)]
            l_mean = float(np.mean(lv)) if lv.size else 0.0
            r_mean = float(np.mean(rv)) if rv.size else 0.0
            self.turn_dir = 1.0 if r_mean >= l_mean else -1.0
            self.state = 'ESCAPE_BACK'
            self.timer = self.t_back
            self.get_logger().warn(
                f'==== 前方 {fwd_min:.2f}m 太近，触发脱困链 ====')
        elif fwd_min < self.warn_dist:
            self.state = 'AVOID'
            # 减速，向开阔侧转
            left = depth[int(h*0.3):int(h*0.7), 0:int(w*0.25)]
            right = depth[int(h*0.3):int(h*0.7), int(w*0.75):w]
            lv = left[np.isfinite(left)]
            rv = right[np.isfinite(right)]
            l_mean = float(np.mean(lv)) if lv.size else 0.0
            r_mean = float(np.mean(rv)) if rv.size else 0.0
            self.turn_dir = 1.0 if r_mean >= l_mean else -1.0
            vel.linear.x = self.cruise_speed * 0.4
            vel.angular.z = self.turn_dir * self.turn_rate * 0.6
            self.get_logger().info(f'前方 {fwd_min:.2f}m, 减速绕开')
        else:
            self.state = 'GO'
            vel.linear.x = self.cruise_speed
            vel.angular.z = 0.0

        self._apply_heave(vel, bottom_alt)
        self.pub.publish(vel)

    def _apply_heave(self, vel, bottom_alt):
        """根据到海底的距离做简单定高：太高就下潜，太低就上浮。"""
        if bottom_alt > 50.0:      # 深度图无效（看不到底），不动
            vel.linear.z = 0.0
            return
        err = self.target_alt - bottom_alt
        if abs(err) < self.alt_band:
            vel.linear.z = 0.0
        else:
            # bottom_alt < target -> 离底太近 -> 需要上浮 (linear.z 正)
            vel.linear.z = float(np.clip(self.heave_gain * err, -0.5, 0.5))


def main():
    rclpy.init()
    node = AuvDepthAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
