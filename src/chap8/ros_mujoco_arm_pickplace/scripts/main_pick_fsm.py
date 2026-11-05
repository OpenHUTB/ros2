#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main_pick_fsm.py —— 5 状态抓放状态机（ROS Noetic 节点）

状态：复位 -> 靠近 -> 下降 -> 抓取 -> 抬升放置 -> 循环
  RESET     : 回到 HOME 位姿；同时把方块复位回抓取点（发布 /cube_reset）
  APPROACH  : 三次多项式轨迹到方块正上方
  DESCEND   : 缓慢下降到抓取位姿
  GRASP     : 发布 /grasp_signal True（桥端做距离校验 + 运动学锁定）
  LIFT_PLACE: 抬升 -> 搬运 -> 下降放置 -> 发布 /grasp_signal False 释放

设计要点：
  - 本节点只做"决策"，物理由 main_mujoco_bridge.py 执行：
       /joint_target  (Float64MultiArray)  目标关节角
       /grasp_signal  (Bool)               抓取/释放
       /cube_reset    (Bool)               方块复位
  - 关节角一律按名字从 /joint_states 读取（不受 freejoint 顺序影响）
  - 复用 pickplace_core 的 IK/三次插值/超时判定，保证与无界面测试行为一致
"""
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState

from pickplace_core import PickPlaceCore, find_model_path


class PickPlaceFSM:
    CTRL_HZ = 50.0

    def __init__(self):
        model_path = rospy.get_param("~model_path", find_model_path())
        # 决策用核心：只做 IK/轨迹/状态机，不步进物理
        self.core = PickPlaceCore(xml_path=model_path, verbose=False)

        self.pub_target = rospy.Publisher("/joint_target", Float64MultiArray, queue_size=10)
        self.pub_grasp = rospy.Publisher("/grasp_signal", Bool, queue_size=10)
        self.pub_reset = rospy.Publisher("/cube_reset", Bool, queue_size=10)
        self.sub_js = rospy.Subscriber("/joint_states", JointState, self.cb_js, queue_size=10)

        self.current_q = np.zeros(self.core.nj)
        self.has_cube = False
        self._got_js = False
        rospy.loginfo("FSM Started")

    def cb_js(self, msg):
        # 按名字取角（顺序无关，健壮）
        q = np.zeros(self.core.nj)
        for i, name in enumerate(self.core.joint_names):
            if name in msg.name:
                q[i] = msg.position[msg.name.index(name)]
        self.current_q = q
        self._got_js = True

    def send_target(self, q):
        self.pub_target.publish(Float64MultiArray(data=q.tolist()))

    def run(self):
        rate = rospy.Rate(self.CTRL_HZ)
        while not rospy.is_shutdown():
            if not self._got_js:
                rate.sleep()
                continue
            dt = 1.0 / self.CTRL_HZ
            ns, nhc, cmd = self.core.decide(self.current_q, self.has_cube, dt)
            self.core.state, self.has_cube = ns, nhc

            if cmd is not None:
                for c in (cmd if isinstance(cmd, list) else [cmd]):
                    if c[0] == "target":
                        self.send_target(c[1])
                    elif c[0] == "grasp":
                        self.pub_grasp.publish(Bool(data=c[1]))
                    elif c[0] == "reset":
                        self.pub_reset.publish(Bool(data=True))
            rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("pick_fsm", anonymous=False)
        PickPlaceFSM().run()
    except Exception as e:
        import traceback
        rospy.logerr(f"FSM error: {e}\n{traceback.format_exc()}")
