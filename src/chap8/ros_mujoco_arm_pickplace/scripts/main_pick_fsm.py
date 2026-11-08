#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main_pick_fsm.py —— 5 状态抓放状态机（ROS Noetic 节点，纯决策）

状态：复位 -> 靠近 -> 下降 -> 抓取 -> 抬升放置 -> 循环
  RESET     : 回到 HOME 位姿；同时发布 /cube_reset 把方块复位回抓取点
  APPROACH  : 三次多项式轨迹到方块正上方
  DESCEND   : 缓慢下降到抓取位姿
  GRASP     : 发布 /grasp_signal True（桥端做距离校验 + 运动学锁定）
  LIFT_PLACE: 抬升 -> 搬运 -> 下降放置 -> 发布 /grasp_signal False 释放

设计要点（含评审意见 #10 的整改）：
  - 本节点**不加载 MuJoCo 模型**：IK / 轨迹 / 状态机全部来自
    pickplace_planner.py（纯 numpy，无 mujoco 依赖），只为算关节目标和
    时间轴，不需要整份 XML，也不会与 bridge 的物理状态产生第二份副本。
  - 本节点只做决策，物理由 main_mujoco_bridge.py 执行：
       /joint_target  (Float64MultiArray)  目标关节角
       /grasp_signal  (Bool)               抓取/释放
       /cube_reset    (Bool)               方块复位
  - 关节角一律按名字从 /joint_states 读取（不受 freejoint 顺序影响）
"""
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState

from pickplace_planner import PickPlacePlanner, JOINT_NAMES, CTRL_HZ


class PickPlaceFSM:
    CTRL_HZ = CTRL_HZ

    def __init__(self):
        # 决策器：纯运动学，不加载 models/ur5e.xml
        self.planner = PickPlacePlanner(verbose=False)
        self.joint_names = list(JOINT_NAMES)
        self.nj = len(self.joint_names)

        self.pub_target = rospy.Publisher("/joint_target", Float64MultiArray, queue_size=10)
        self.pub_grasp = rospy.Publisher("/grasp_signal", Bool, queue_size=10)
        self.pub_reset = rospy.Publisher("/cube_reset", Bool, queue_size=10)
        self.sub_js = rospy.Subscriber("/joint_states", JointState, self.cb_js, queue_size=10)

        self.current_q = np.zeros(self.nj)
        self._got_js = False
        rospy.loginfo(f"FSM Started (decision-only, {self.nj} joints, no MuJoCo model loaded)")

    def cb_js(self, msg):
        # 按名字取角（顺序无关，健壮）
        q = np.zeros(self.nj)
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                q[i] = msg.position[msg.name.index(name)]
        self.current_q = q
        self._got_js = True

    def send_target(self, q):
        self.pub_target.publish(Float64MultiArray(data=[float(v) for v in q]))

    def run(self):
        rate = rospy.Rate(self.CTRL_HZ)
        while not rospy.is_shutdown():
            if not self._got_js:
                rate.sleep()
                continue
            dt = 1.0 / self.CTRL_HZ
            ns, nhc, cmd = self.planner.decide(self.current_q, self.planner.has_cube, dt)
            self.planner.state, self.planner.has_cube = ns, nhc

            if cmd is not None:
                for c in (cmd if isinstance(cmd, list) else [cmd]):
                    if c[0] == "target":
                        self.send_target(c[1])
                    elif c[0] == "grasp":
                        self.pub_grasp.publish(Bool(data=bool(c[1])))
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
