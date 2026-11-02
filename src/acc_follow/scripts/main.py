#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py —— ACC 自适应巡航 ROS Noetic 节点

功能：
    * 订阅前车状态 /lead_state（std_msgs/Float32MultiArray: [x_lead, v_lead]）
    * 计算 ACC 目标速度，发布 /cmd_vel（geometry_msgs/Twist，linear.x）
    * 未收到前车数据时使用内置场景（前车 15 m/s 巡航 -> 减速停车 -> 起步）

用法（配合 launch/main.launch）：
    roslaunch acc_follow main.launch
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from vehicle_model import VehicleModel, LeadVehicle
from acc_controller import ACCController

# 内置场景（与仿真一致）：前车 15 m/s 巡航 -> 8s 减速停车 -> 停 7s -> 起步
T_STOP_BEGIN = 8.0
T_STOP_END = 13.0
T_GO_BEGIN = 20.0
DT = 0.05


class ACCNode:
    def __init__(self):
        rospy.init_node("acc_follow", anonymous=False)

        # 参数（launch 文件可覆盖）
        self.v_cruise = rospy.get_param("~v_cruise", 15.0)   # 巡航速度 m/s
        self.d0 = rospy.get_param("~d0", 3.0)                # 停车安全间距 m
        self.t_gap = rospy.get_param("~t_gap", 1.0)          # 跟车时距 s
        self.use_external = rospy.get_param("~use_external", False)

        self.ctrl = ACCController(v_cruise=self.v_cruise,
                                  d0=self.d0, t_gap=self.t_gap)
        self.host = VehicleModel(x=0.0, v=0.0, v_max=self.v_cruise, dt=DT)

        # 前车状态（外部输入 or 内置场景）
        self.lead_x = None
        self.lead_v = None
        if self.use_external:
            self.sub = rospy.Subscriber("/lead_state", Float32MultiArray,
                                        self.on_lead_state)
        else:
            self.lead = LeadVehicle(x=50.0, v=self.v_cruise,
                                    v_cruise=self.v_cruise, dt=DT)
            self.lead.set_phase_times(T_STOP_BEGIN, T_STOP_END, T_GO_BEGIN)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.loginfo("ACC 节点启动：v_cruise=%.1f m/s, d0=%.1f m, t_gap=%.1f s"
                      % (self.v_cruise, self.d0, self.t_gap))

    def on_lead_state(self, msg):
        if len(msg.data) >= 2:
            self.lead_x = msg.data[0]
            self.lead_v = msg.data[1]

    def step(self):
        """推进一个仿真步并发布 /cmd_vel。"""
        if self.use_external:
            if self.lead_x is None:
                rospy.logwarn_throttle(2.0, "等待 /lead_state 数据...")
                return
            lead_v = self.lead_v
            d_center = self.lead_x - self.host.x
        else:
            lead_v = self.lead.step()
            d_center = self.lead.x - self.host.x

        v_cmd, d_des = self.ctrl.compute(d_center, lead_v, self.host.v)
        self.host.step(v_cmd)

        d = d_center - self.ctrl.car_len   # 保险杠间距
        twist = Twist()
        twist.linear.x = v_cmd
        self.pub.publish(twist)

        rospy.loginfo_throttle(1.0,
            "车距=%.1f m 期望=%.1f m 前车=%.1f m/s 本车=%.1f m/s 目标=%.1f m/s"
            % (d, d_des, lead_v, self.host.v, v_cmd))

    def run(self):
        rate = rospy.Rate(int(1.0 / DT))
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    try:
        ACCNode().run()
    except rospy.ROSInterruptException:
        pass
