#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""差速小车沿墙行驶 ROS 节点（模块入口 main.py）。

订阅 /scan（sensor_msgs/LaserScan），从激光数据中取正前、正右、右前、右后、
正左五路距离，交给 wall_follower.py 的控制器，发布 /cmd_vel（Twist）。

运行：
    rosrun diff_drive_wall_following main.py
    roslaunch diff_drive_wall_following main.launch
"""
from __future__ import print_function

import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from wall_follower import STATE_FOLLOW, STATE_NAMES, STATE_TURN, WallFollower

# 五路测距对应的机器人坐标系角度（rad，0 为正前方，逆时针为正）
BEAM_ANGLES = {
    'front': 0.0,
    'side': -math.pi / 2,
    'front_right': -math.pi / 4,
    'back_right': -3 * math.pi / 4,
    'left': math.pi / 2,
}
WINDOW = math.radians(12)   # 每路取 ±12° 内的最小距离，抗噪


class WallFollowingNode(object):

    def __init__(self):
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.cmd_topic = rospy.get_param('~cmd_topic', '/cmd_vel')
        self.scan_timeout = rospy.get_param('~scan_timeout', 0.5)

        self.follower = WallFollower(
            target_distance=rospy.get_param('~target_distance', 0.5),
            k_distance=rospy.get_param('~k_distance', 1.6),
            k_align=rospy.get_param('~k_align', 1.2),
            max_angular=rospy.get_param('~max_angular', 0.9),
            base_speed=rospy.get_param('~base_speed', 0.35),
            turn_speed=rospy.get_param('~turn_speed', 0.8),
            front_block_distance=rospy.get_param('~front_block_distance', 0.75),
            front_clear_distance=rospy.get_param('~front_clear_distance', 1.1),
        )

        self.distances = None
        self.last_scan_time = None
        self.last_time = None
        self.last_state = None

        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        rospy.Subscriber(self.scan_topic, LaserScan, self.on_scan)
        rospy.loginfo('沿墙行驶节点已启动：scan=%s -> cmd=%s',
                      self.scan_topic, self.cmd_topic)

    def _beam_distance(self, scan, angle):
        """在 angle 附近 [-WINDOW, +WINDOW] 内取最小有效距离。"""
        best = float('inf')
        start = scan.angle_min
        inc = scan.angle_increment
        count = len(scan.ranges)
        if count == 0 or inc == 0.0:
            return best

        for i in range(count):
            a = start + i * inc
            # 把角度差归一化到 [-pi, pi]
            diff = math.atan2(math.sin(a - angle), math.cos(a - angle))
            if abs(diff) > WINDOW:
                continue
            r = scan.ranges[i]
            if r is None or r != r:  # None / NaN
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            if r < best:
                best = r
        return best

    def on_scan(self, msg):
        self.distances = dict((name, self._beam_distance(msg, angle))
                              for name, angle in BEAM_ANGLES.items())
        self.last_scan_time = rospy.Time.now()

    def publish(self, v, w):
        # 控制器已对缺失测距做兜底；这里再挡一层，保证 cmd_vel 不会出现 NaN/inf
        if not math.isfinite(v):
            v = 0.0
        if not math.isfinite(w):
            w = 0.0
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(rospy.get_param('~control_rate', 10.0))
        self.last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            self.last_time = now

            if self.last_scan_time is None:
                rate.sleep()
                continue

            if (now - self.last_scan_time).to_sec() > self.scan_timeout:
                self.publish(0.0, 0.0)
                rospy.logwarn_throttle(2.0, '超过 %.1f 秒没有收到激光数据，已停车',
                                       self.scan_timeout)
                rate.sleep()
                continue

            d = self.distances
            v, w, state = self.follower.update(
                d['front'], d['side'], d['front_right'], d['back_right'],
                d['left'], dt)
            self.publish(v, w)

            if state != self.last_state:
                rospy.loginfo('状态切换：%s -> %s',
                              STATE_NAMES.get(self.last_state, '无'),
                              STATE_NAMES.get(state, state))
                self.last_state = state

            rate.sleep()

        self.publish(0.0, 0.0)


def main():
    rospy.init_node('diff_drive_wall_following')
    node = WallFollowingNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.publish(0.0, 0.0)
        rospy.loginfo('节点退出，已发布零速度')


if __name__ == '__main__':
    main()
