#!/usr/bin/env python3
import rospy
import random
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class NoStuckWander:
    """基于激光雷达的防卡死漫游节点。

    订阅 /scan，发布 /cmd_vel。正常情况下向前漫游，
    遇到障碍时按距离进入「绕障 / 脱困」状态机，避免机器人撞墙或卡死。
    """

    def __init__(self):
        rospy.init_node("no_stuck_wander")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        self.VX_FAST = 0.09
        self.VX_SLOW = 0.04
        self.T_ENTER = 0.65
        self.T_EXIT = 0.78
        self.T_DANGER = 0.40
        # 时间 tick = 0.1s
        self.T_BACK = 14        # 直退1.4s
        self.T_TURN = 16        # 原地转1.6s
        self.T_AVOID = 16
        self.T_AFTER_ESCAPE = 10  # 脱困后强制前进一小段（救命！防卡死）
        # 状态
        self.state = "GO"
        self.timer = 0
        self.cmd_ang = 0.0
        self.fl = 99.0
        self.fm = 99.0
        self.fr = 99.0
        self.rate = rospy.Rate(10)

    def scan_cb(self, msg):
        n = len(msg.ranges)
        half = n // 2
        seg = half // 3

        def min_valid(arr):
            good = []
            for x in arr:
                if msg.range_min < x < msg.range_max:
                    good.append(x)
            return min(good) if good else 99.0

        self.fl = min_valid(msg.ranges[0:seg])
        self.fm = min_valid(msg.ranges[seg:2 * seg])
        self.fr = min_valid(msg.ranges[2 * seg:half])

    def run(self):
        vel = Twist()
        while not rospy.is_shutdown():
            fl, fm, fr = self.fl, self.fm, self.fr
            minf = min(fl, fm, fr)
            if self.timer > 0:
                self.timer -= 1
                if self.state == "ESCAPE_BACK":
                    vel.linear.x = -0.22
                    vel.angular.z = 0.0
                    rospy.logwarn("【直退】剩余:%d", self.timer)
                elif self.state == "ESCAPE_TURN":
                    vel.linear.x = 0.0
                    vel.angular.z = self.cmd_ang
                    rospy.logwarn("【原地转开】剩余:%d", self.timer)
                elif self.state == "AFTER_ESCAPE":
                    # 关键：脱困之后强制往前冲一小段，脱离墙角引力区
                    vel.linear.x = self.VX_FAST
                    vel.angular.z = self.cmd_ang * 0.3 + random.uniform(-0.15, 0.15)
                    rospy.loginfo("【脱困后向前脱离】剩余:%d", self.timer)
                elif self.state == "AVOID":
                    vel.linear.x = self.VX_SLOW
                    vel.angular.z = self.cmd_ang
                    rospy.loginfo("【绕障】剩余:%d", self.timer)
                else:  # GO
                    vel.linear.x = self.VX_FAST
                    vel.angular.z = random.uniform(-0.08, 0.08)
                self.pub.publish(vel)
                self.rate.sleep()
                continue
            # === 计时器0，才允许新状态 ===
            if self.state == "ESCAPE_BACK":
                # 直退完 → 进入原地转
                self.state = "ESCAPE_TURN"
                self.timer = self.T_TURN
            elif self.state == "ESCAPE_TURN":
                # 原地转完 → 强制向前走一段，不管墙近不近！
                self.state = "AFTER_ESCAPE"
                self.timer = self.T_AFTER_ESCAPE
            elif self.state == "AFTER_ESCAPE":
                # 这段走完 → 回到正常GO
                self.state = "GO"
                vel.linear.x = self.VX_FAST
                vel.angular.z = 0.0
            elif minf < self.T_DANGER:
                # 真危险：启动完整脱困链
                if fl < fr:
                    self.cmd_ang = -1.2
                else:
                    self.cmd_ang = 1.2
                self.state = "ESCAPE_BACK"
                self.timer = self.T_BACK
                rospy.logwarn("====触发完整脱困链====")
            elif minf < self.T_ENTER:
                self.state = "AVOID"
                self.timer = self.T_AVOID
                diff = abs(fl - fr)
                if diff < 0.12:
                    self.cmd_ang = 0.8
                elif fl < fr:
                    self.cmd_ang = -0.8
                else:
                    self.cmd_ang = 0.8
                rospy.loginfo("进入绕障")
            elif minf > self.T_EXIT:
                self.state = "GO"
                vel.linear.x = self.VX_FAST
                vel.angular.z = 0.0
            self.pub.publish(vel)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = NoStuckWander()
        node.run()
    except rospy.ROSInterruptException:
        pass
