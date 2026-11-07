#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class AutoParking:
    """基于 /scan 的侧方自动泊车：沿右侧巡航找车位，检测到有效车位空隙后执行
    越过车位末端 → 倒回对准 → 向左摆尾入位 → 回正倒深 → 停稳。"""

    def __init__(self):
        rospy.init_node("auto_parking_node")
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        # ---- 车位检测（右侧找位，index 0 为车头正前，index 270 为右侧）----
        self.right_side = 999.0          # 右侧最近障碍距离
        self.gap_far = 0.9               # 右侧距离大于此 → 出现“车位空隙”
        self.gap_near = 0.7              # 右侧距离小于此 → 空隙结束（又是墙/车）
        self.slot_min = 0.6              # 有效车位最小长度（米）
        self.slot_max = 1.8              # 有效车位最大长度（米）

        # ---- 侧方停车机动参数（可按你的仿真微调）----
        self.entry_pass = 0.30           # 车头越过车位末端的余量
        self.rear_offset = 0.10          # 倒回时停在车位中心前一点的余量
        self.back_in_dist = 0.55         # 摆尾入位阶段倒车距离
        self.straighten_dist = 0.40      # 回正后继续倒深距离
        self.cruise_v = 0.12             # 巡航速度
        self.reverse_v = -0.12           # 倒车速度
        self.turn_w = 0.45               # 摆尾转向角速度

        # ---- 状态与里程计 ----
        self.pos = 0.0                   # 带符号的里程计累计位移（前进 +，倒车 -）
        self.last_odom = None
        self.state = "CRUISE"
        self.slot_start_pos = None
        self.slot_len = None
        self.maneuver_start_pos = None
        self.cruise_start_time = rospy.Time.now().to_sec()
        self.cruise_timeout = 60.0       # 巡航最多找位多久（秒），找不到就停

        self.rate = rospy.Rate(10)

    # ---------------- 回调 ----------------
    def odom_callback(self, msg):
        v = msg.twist.twist.linear.x
        if self.last_odom is not None:
            dt = (msg.header.stamp - self.last_odom).to_sec()
            if 0.0 < dt < 1.0:
                self.pos += v * dt
        self.last_odom = msg.header.stamp

    def laser_callback(self, msg):
        if len(msg.ranges) < 300:
            return
        right = msg.ranges[260:280]
        valid = [d for d in right if 0.1 < d < 12.0]
        self.right_side = min(valid) if valid else 999.0

    # ---------------- 工具 ----------------
    def publish(self, vx, wz):
        m = Twist()
        m.linear.x = vx
        m.angular.z = wz
        self.vel_pub.publish(m)

    # ---------------- 状态机 ----------------
    def cruise(self):
        # 找位：右侧距离从近→远→近 的一段空隙视为车位
        if self.right_side > self.gap_far:
            if self.slot_start_pos is None:
                self.slot_start_pos = self.pos
                rospy.loginfo("检测到车位空隙起点")
        else:
            if self.slot_start_pos is not None:
                self.slot_len = self.pos - self.slot_start_pos
                if self.slot_min <= self.slot_len <= self.slot_max:
                    rospy.loginfo("有效车位长度 %.2fm，开始泊车", self.slot_len)
                    self.state = "ALIGN"
                    self.maneuver_start_pos = self.pos
                    return
                rospy.loginfo("空隙 %.2fm 无效，继续找位", self.slot_len)
                self.slot_start_pos = None
        # 空隙过长视为连续空场，放弃
        if self.slot_start_pos is not None and (self.pos - self.slot_start_pos) > self.slot_max:
            self.slot_start_pos = None
        self.publish(self.cruise_v, 0.0)

    def align(self):
        # 车头越过车位末端 entry_pass，为倒车留出空间
        target = self.slot_start_pos + self.slot_len + self.entry_pass
        if self.pos >= target:
            rospy.loginfo("已越过车位末端，开始倒回对准")
            self.state = "BACK_TO_ENTRY"
            self.maneuver_start_pos = self.pos
            return
        self.publish(self.cruise_v, 0.0)

    def back_to_entry(self):
        # 倒车到车位中心附近（车尾对准入口）
        target = self.slot_start_pos + self.slot_len / 2 - self.rear_offset
        if self.pos <= target:
            rospy.loginfo("已对准车位入口，摆尾入位")
            self.state = "BACK_IN"
            self.maneuver_start_pos = self.pos
            return
        self.publish(self.reverse_v, 0.0)

    def back_in(self):
        # 倒车 + 向左打满方向，车尾摆入车位
        if (self.maneuver_start_pos - self.pos) >= self.back_in_dist:
            rospy.loginfo("车尾已摆入，回正方向盘")
            self.state = "STRAIGHTEN"
            self.maneuver_start_pos = self.pos
            return
        self.publish(self.reverse_v, self.turn_w)

    def straighten(self):
        # 回正，继续倒车到目标深度
        if (self.maneuver_start_pos - self.pos) >= self.straighten_dist:
            rospy.loginfo("泊车入位完成")
            self.state = "DONE"
            return
        self.publish(self.reverse_v, 0.0)

    # ---------------- 主循环 ----------------
    def run(self):
        rospy.loginfo("自动泊车开始：沿右侧找车位 → 侧方停车")
        while not rospy.is_shutdown():
            if self.state == "CRUISE":
                # 长时间找不到有效车位则停车退出，避免原地空转
                if rospy.Time.now().to_sec() - self.cruise_start_time > self.cruise_timeout:
                    rospy.logwarn("巡航超时未找到有效车位，停车")
                    self.state = "DONE"
                    continue
                self.cruise()
            elif self.state == "ALIGN":
                self.align()
            elif self.state == "BACK_TO_ENTRY":
                self.back_to_entry()
            elif self.state == "BACK_IN":
                self.back_in()
            elif self.state == "STRAIGHTEN":
                self.straighten()
            elif self.state == "DONE":
                rospy.loginfo("泊车完成，停车！")
                self.publish(0.0, 0.0)
                break
            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = AutoParking()
        node.run()
    except rospy.ROSInterruptException:
        pass
