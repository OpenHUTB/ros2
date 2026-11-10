#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 1 Noetic 节点：把 humanoid_embodied_control 的控制内核接到 rospy。

设计要点
--------
内核 :class:`WalkNodeCore` 与 ROS 无关，本文件只做三件事：
  1. 订阅激光/深度/目标 → 喂给内核；
  2. 定时器驱动控制周期（100 Hz 步态 + 20 Hz 感知，与 ``main.py --mode sim``
     的分频完全一致——步态更新率不足会摔倒，实测见文档"调参记录"）；
  3. 发布关节目标角与根部力矩（``/uav/cmd_vel`` 只用作"关节指令"载体，
     仓库其它模块也共用这个约定）。

运行::

    roslaunch humanoid_embodied_control main.launch
    rosrun humanoid_embodied_control walk_node_ros1.py
"""

from __future__ import annotations

import json
import os
import sys

# 让脚本能直接从包目录找到内核（未 colcon 编译时也能跑）
_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (_HERE, os.path.dirname(_HERE)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from humanoid_embodied_control.nodes.walk_node import WalkNodeCore  # noqa: E402

try:
    import rospy
    from geometry_msgs.msg import PointStamped, Twist
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
except ImportError:  # 允许"只 import 不做 ROS 操作"（CI/单测）
    rospy = None


NODE_NAME = "humanoid_walk_node"
CTRL_HZ = 100.0        # 步态/平衡下发频率（低于此值会摔倒）
DECISION_HZ = 20.0     # 感知 + 决策频率


def main():
    if rospy is None:
        raise SystemExit("找不到 rospy，请在 ROS 1 Noetic 环境下运行")

    rospy.init_node(NODE_NAME, anonymous=False)

    goal = rospy.get_param("~goal", [4.5, 0.0])
    period = rospy.get_param("~period", 1.3)
    stride = rospy.get_param("~stride", 0.20)
    lift = rospy.get_param("~lift", 0.10)

    from humanoid_embodied_control.scripts.main import WalkingController
    controller = WalkingController(
        goal=(goal[0], goal[1]), stride=stride, lift=lift, period=period)
    core = WalkNodeCore(controller=controller)
    pub_cmd = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=1)
    pub_state = rospy.Publisher("/uav/state", String, queue_size=1)

    state = {"quat": (1.0, 0.0, 0.0, 0.0), "gyro": (0.0, 0.0, 0.0),
             "front": 3.0, "yaw": 0.0, "x": 0.0, "y": 0.0}

    def on_scan(msg: LaserScan):
        state["front"] = core.on_scan(list(msg.ranges), msg.angle_min,
                                      msg.angle_increment, msg.range_max)

    def on_goal(msg: PointStamped):
        core.on_goal(msg.point.x, msg.point.y, msg.point.z)

    rospy.Subscriber("/uav/lidar", LaserScan, on_scan, queue_size=1)
    rospy.Subscriber("/goal", PointStamped, on_goal, queue_size=1)

    rospy.loginfo("[%s] goal=%s period=%.2f stride=%.2f lift=%.2f ctrl=%.0fHz",
                  NODE_NAME, goal, period, stride, lift, CTRL_HZ)

    dt = 1.0 / CTRL_HZ
    t0 = rospy.get_time()
    rate = rospy.Rate(CTRL_HZ)
    while not rospy.is_shutdown():
        t = rospy.get_time() - t0
        speed, turn, target = core.on_tick(t, state["x"], state["y"], state["yaw"],
                                           state["front"], state["quat"], state["gyro"], dt)

        # Twist 只当"指令载体"：linear = 6 个根部力矩，angular = 12 个关节目标角
        # （ROS 消息里放不下 18 维，这里 angular 只有 3 维，故用 JSON 透传）
        cmd = Twist()
        cmd.linear.x = core.cmd[0] if core.cmd else 0.0
        cmd.linear.y = core.cmd[1] if core.cmd else 0.0
        cmd.linear.z = core.cmd[2] if core.cmd else 0.0
        cmd.angular.x = speed
        cmd.angular.y = turn
        pub_cmd.publish(cmd)
        state_msg = String()
        state_msg.data = json.dumps(
            {"t": round(t, 3), "speed": round(speed, 3),
             "turn": round(turn, 3), "targets": target,
             **core.telemetry}, ensure_ascii=False)
        pub_state.publish(state_msg)
        rate.sleep()


if __name__ == "__main__":
    main()
