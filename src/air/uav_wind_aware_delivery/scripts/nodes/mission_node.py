#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""空投任务节点：发布 setpoint / release / mission 状态。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _bootstrap  # noqa: E402

import rospy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from std_msgs.msg import Bool, Float64MultiArray, String  # noqa: E402

from uav_wind_aware_delivery.mission import DeliveryMission, MissionConfig  # noqa: E402
from uav_wind_aware_delivery.state import State  # noqa: E402


class MissionNode:
    def __init__(self):
        rospy.init_node("uav_mission")
        target = [
            rospy.get_param("~target_x", 5.0),
            rospy.get_param("~target_y", 2.0),
            0.0,
        ]
        self.mission = DeliveryMission(
            MissionConfig(
                target=target,
                release_alt=rospy.get_param("~release_alt", 3.0),
                hover_time=rospy.get_param("~hover_time", 0.8),
                compensate=bool(rospy.get_param("~compensate", True)),
            ),
            model_path=os.path.join(_bootstrap.PKG_ROOT, "models", "impact_mlp.npz"),
        )
        self.state = None
        self.last_phase = None
        self.sp_pub = rospy.Publisher("/uav/setpoint", PoseStamped, queue_size=1)
        self.rel_pub = rospy.Publisher("/uav/release", Bool, queue_size=1)
        self.phase_pub = rospy.Publisher("/uav/mission", String, queue_size=1)
        rospy.Subscriber("/uav/state", Float64MultiArray, self.on_state, queue_size=1)
        self.rate = rospy.Rate(50)

    def on_state(self, msg):
        try:
            self.state = State.from_vector(msg.data)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "bad state: %s", exc)

    def spin(self):
        while not rospy.is_shutdown():
            if self.state is None:
                self.rate.sleep()
                continue
            out = self.mission.update(self.state)
            sp = PoseStamped()
            sp.header.stamp = rospy.Time.now()
            sp.header.frame_id = "map"
            sp.pose.position.x = float(out["setpoint"][0])
            sp.pose.position.y = float(out["setpoint"][1])
            sp.pose.position.z = float(out["setpoint"][2])
            sp.pose.orientation.w = 1.0
            self.sp_pub.publish(sp)
            self.rel_pub.publish(Bool(data=out["release"]))
            self.phase_pub.publish(String(data=out["phase"]))
            if out["phase"] != self.last_phase:
                rospy.loginfo("mission phase -> %s (t=%.2f)", out["phase"], self.state.t)
                self.last_phase = out["phase"]
            self.rate.sleep()


if __name__ == "__main__":
    try:
        MissionNode().spin()
    except rospy.ROSInterruptException:
        pass
