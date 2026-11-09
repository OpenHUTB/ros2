#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hutb_grid_slam_nav: 导航目标发布与性能统计节点
==================================
向 move_base 发布 2D 导航目标点, 并统计:
  - 到达时间
  - 实际行驶路径长度
  - 终点误差
用于"导航与 SLAM 同步仿真"的性能评价。

用法:
  roslaunch hutb_grid_slam_nav main.launch phase:=nav   # 目标 (2.0, 1.0) 自动发布
"""
import sys
import math

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class NavGoal:
    def __init__(self, gx, gy):
        self.gx, self.gy = gx, gy
        self.start_pose = None
        self.start_time = None
        self.travelled = 0.0
        self.last_odom = None
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('等待 move_base action server...')
        self.client.wait_for_server()
        rospy.loginfo('move_base 已连接')

    def odom_cb(self, msg):
        if self.last_odom is not None and self.start_time is not None:
            dx = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
            dy = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y
            self.travelled += math.hypot(dx, dy)
        self.last_odom = msg

    def run(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.gx
        goal.target_pose.pose.position.y = self.gy
        goal.target_pose.pose.orientation.w = 1.0
        self.start_time = rospy.Time.now()
        self.travelled = 0.0
        rospy.loginfo('发布导航目标: (%.2f, %.2f)' % (self.gx, self.gy))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(120))
        state = self.client.get_state()
        dur = (rospy.Time.now() - self.start_time).to_sec()
        err = 0.0
        if self.last_odom is not None:
            err = math.hypot(self.last_odom.pose.pose.position.x - self.gx,
                             self.last_odom.pose.pose.position.y - self.gy)
        rospy.loginfo('导航结果: state=%d 耗时=%.2fs 路程=%.2fm 终点误差=%.2fm'
                      % (state, dur, self.travelled, err))
        return state


def main():
    rospy.init_node('tb3_nav_goal')
    # roslaunch 会注入 __name:= 等 remap 参数, 需过滤
    args = [a for a in sys.argv[1:] if not a.startswith('__')]
    # 支持 launch 传参 (rosparam ~goal_x/~goal_y) 或命令行参数
    gx = rospy.get_param('~goal_x', float(args[0]) if len(args) > 0 else 2.0)
    gy = rospy.get_param('~goal_y', float(args[1]) if len(args) > 1 else 1.0)
    NavGoal(gx, gy).run()


if __name__ == '__main__':
    main()
