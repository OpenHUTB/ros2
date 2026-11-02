#!/usr/bin/env python3
"""Mbot 激光雷达避障节点（ROS 1 Noetic + Gazebo）。

订阅 /scan，按前 / 左 / 右三个扇区的最小距离做阈值避障，
向 /cmd_vel 发布 geometry_msgs/Twist。

本节点相对于已有模块 turtlebot3_lidar_avoid（ROS 2 Humble + TurtleBot3 Waffle）
的差异：
- 运行时：ROS 1 Noetic + catkin，适用于 Ubuntu 20.04 / ROS 1 环境；
- 机器人：面向 Mbot（与 turtlebot3_lidar_avoid 面向 TurtleBot3 Waffle 不同）；
- 实现：单文件实现，去除 ROS 2 参数服务器等特性，便于 ROS 1 初学者入门；
- 安全：增加 NaN/Inf 过滤、异常长度检测、/scan 超时保护与 shutdown 停车。
"""

import math
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# —— 参数（硬编码，单文件示例；实际项目可改为 rosparam） ——
SAFE_DISTANCE = 0.8       # 进入避障阈值 (m)
DANGER_DISTANCE = 0.25    # 紧急停车阈值 (m)
CRUISE_SPEED = 0.30       # 正常直行速度 (m/s)
AVOID_SPEED = 0.05        # 避障时线速度 (m/s)
TURN_RATE = 1.0           # 避障角速度 (rad/s)
SCAN_TIMEOUT_SEC = 1.5    # 无扫描数据超时阈值 (s)
RANGE_MAX_CLIP = 3.5      # 距离上限裁剪 (m)

# 正常 360° 激光雷达每个扇区大致对应：
# front:   ranges[  0:15] + ranges[345:360]  → 正前方 ±15°
# left:    ranges[ 15:60]                    → 左前方  15°~ 60°
# right:   ranges[300:345]                  → 右前方 -60°~-15°
FRONT_SLICE = list(range(0, 15)) + list(range(345, 360))
LEFT_SLICE = list(range(15, 60))
RIGHT_SLICE = list(range(300, 345))

EXPECTED_SCAN_LEN = 360  # TurtleBot3 / Mbot 常见 360 束


class LidarAvoidNode:
    """ROS 1 激光雷达避障节点。"""

    def __init__(self):
        rospy.init_node('mbot_lidar_avoid_node', anonymous=False)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)

        # 安全相关状态
        self._last_scan_time = None   # 最近一次收到 /scan 的 wall time
        self._scan_ok = False         # 上一帧扫描是否通过合法性检查

        # 注册退出回调，保证 Ctrl+C / rosnode kill 时停车
        rospy.on_shutdown(self._on_shutdown)

    # ------------------------------------------------------------------
    # 工具函数
    # ------------------------------------------------------------------
    @staticmethod
    def _valid_min(values):
        """在 values 中筛选有限值后返回最小距离；全部非法返回 None。"""
        filtered = [v for v in values if v is not None and math.isfinite(v)]
        if not filtered:
            return None
        return min(min(filtered), RANGE_MAX_CLIP)

    @staticmethod
    def _zero_twist():
        t = Twist()
        t.linear.x = 0.0
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = 0.0
        return t

    def _publish_zero(self):
        self.pub.publish(self._zero_twist())

    # ------------------------------------------------------------------
    # 主逻辑
    # ------------------------------------------------------------------
    def _scan_callback(self, msg):
        now = time.time()
        self._last_scan_time = now

        # —— 1. 数据合法性检查 ——
        # LaserScan 的 ranges 长度应与 angle_increment 覆盖角度一致；
        # TurtleBot3/Mbot 标准 360° 雷达应为 360 束。
        if msg.ranges is None or len(msg.ranges) != EXPECTED_SCAN_LEN:
            rospy.logwarn_throttle(2.0,
                '[mbot_lidar_avoid] 收到异常长度的扫描数据 len=%d，期望 %d，停车保护',
                0 if msg.ranges is None else len(msg.ranges), EXPECTED_SCAN_LEN)
            self._scan_ok = False
            self._publish_zero()
            return

        # —— 2. 三个扇区分别取最小有限值 ——
        front = self._valid_min(msg.ranges[i] for i in FRONT_SLICE)
        left = self._valid_min(msg.ranges[i] for i in LEFT_SLICE)
        right = self._valid_min(msg.ranges[i] for i in RIGHT_SLICE)

        # 若所有扇区都没有有限距离（全部 NaN / Inf），视为异常 → 停车
        if front is None and left is None and right is None:
            rospy.logwarn_throttle(2.0,
                '[mbot_lidar_avoid] 扫描数据全部为 NaN/Inf，停车保护')
            self._scan_ok = False
            self._publish_zero()
            return

        # 某一扇区缺数据时保守处理：取 RANGE_MAX_CLIP（视为安全空旷）
        if front is None:
            front = RANGE_MAX_CLIP
        if left is None:
            left = RANGE_MAX_CLIP
        if right is None:
            right = RANGE_MAX_CLIP

        self._scan_ok = True

        # —— 3. 决策 ——
        twist = self._decide(front, left, right)
        self.pub.publish(twist)

    def _decide(self, front, left, right):
        """基于三个扇区距离输出 Twist。"""
        twist = Twist()

        # 紧急停车：前方过近
        if front < DANGER_DISTANCE:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            return twist

        # 正常避障：前方小于安全阈值
        if front < SAFE_DISTANCE:
            twist.linear.x = AVOID_SPEED
            # 朝两侧空间更宽的方向转
            twist.angular.z = TURN_RATE if left > right else -TURN_RATE
            return twist

        # 直行
        twist.linear.x = CRUISE_SPEED
        twist.angular.z = 0.0
        return twist

    # ------------------------------------------------------------------
    # 超时 & 退出保护
    # ------------------------------------------------------------------
    def _watchdog_tick(self):
        """由外部定时调用；若长时间没收到 /scan 则停车。"""
        if self._last_scan_time is None:
            # 节点刚启动还没收到过数据 → 停车更安全
            self._publish_zero()
            return
        elapsed = time.time() - self._last_scan_time
        if elapsed > SCAN_TIMEOUT_SEC:
            rospy.logwarn_throttle(2.0,
                '[mbot_lidar_avoid] %.1fs 未收到 /scan，停车保护', elapsed)
            self._publish_zero()

    def _on_shutdown(self):
        rospy.loginfo('[mbot_lidar_avoid] 节点退出，发布零速度')
        try:
            self._publish_zero()
        except Exception:
            pass  # 退出时 Publisher 可能已失效，忽略


def main():
    node = LidarAvoidNode()
    # 用 Timer 10Hz 做 watchdog；既保证超时停车，也保证退出前最后发一次 0
    rate = rospy.Rate(10.0)
    try:
        while not rospy.is_shutdown():
            node._watchdog_tick()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
