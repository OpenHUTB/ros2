#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""指令下发节点：把 ROS 话题指令转成 CarlaAir/AirSim 的飞行指令.

订阅：
    /uav/cmd_vel     geometry_msgs/Twist   速度指令（默认机体系：x 前, y 左, z 上）
    /uav/goal        geometry_msgs/Point   目标点指令（ENU 世界系，飞到该点）
    /uav/action      std_msgs/String       动作指令（takeoff / land / hover）

发布：
    /uav/status      std_msgs/String       运行状态（HOVER / GOAL / VELOCITY / GOAL_DONE /
                                           ACTION:x / ACTION_DONE x / ACTION_FAILED x ...）

安全设计：
    1) 速度指令按 control/* 参数限幅（水平、垂直、偏航角速度）
    2) 超过 rate/cmd_timeout 没有新指令 -> 自动悬停，避免失控
    3) 目标点任务在独立线程中执行，期间速度指令让位（避免两种指令互相打断）
    4) 动作指令（takeoff/land）也是阻塞调用，放独立线程；执行期间速度通道同样让位，
       并且与目标点任务互斥，避免"边降落边飞目标点"
"""
from __future__ import annotations

import os
import sys
import threading

import rospy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_client import SimClient  # noqa: E402


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class CmdVelSubscriber(object):
    def __init__(self):
        host = rospy.get_param("sim/host", "127.0.0.1")
        port = int(rospy.get_param("sim/airsim_port", 41451))
        self.vehicle = rospy.get_param("sim/vehicle_name", "")
        self.cmd_topic = rospy.get_param("topic/cmd_vel", "/uav/cmd_vel")
        self.goal_topic = rospy.get_param("topic/goal", "/uav/goal")
        self.action_topic = rospy.get_param("topic/action", "/uav/action")
        self.status_topic = rospy.get_param("topic/status", "/uav/status")
        self.hz = float(rospy.get_param("rate/odom_hz", 20.0))
        self.timeout = float(rospy.get_param("rate/cmd_timeout", 0.5))
        self.max_h = float(rospy.get_param("control/max_horiz_speed", 5.0))
        self.max_v = float(rospy.get_param("control/max_vert_speed", 3.0))
        self.max_yaw = float(rospy.get_param("control/max_yaw_rate", 1.0))
        self.body_frame = bool(rospy.get_param("control/body_frame", True))
        # 单条速度指令的持续时长（秒）。应与上游策略的控制周期一致：
        # 训练环境的 dt 是 0.1 s，所以默认 0.1，避免"指令作用时间比训练时长"导致的过冲震荡。
        self.cmd_duration = float(rospy.get_param("control/cmd_duration", 0.1))
        self.goal_speed = float(rospy.get_param("control/goal_speed", 2.0))
        auto_takeoff = bool(rospy.get_param("control/auto_takeoff", True))

        self.sim = SimClient(host=host, port=port, vehicle_name=self.vehicle)
        self.sim.connect()
        rospy.loginfo("cmd_vel_sub: 已连接仿真器 %s:%d", host, port)

        if auto_takeoff:
            rospy.loginfo("cmd_vel_sub: 自动起飞 ...")
            self.sim.takeoff()
            self.sim.hover()
            rospy.loginfo("cmd_vel_sub: 已起飞并悬停")

        self.pub_status = rospy.Publisher(self.status_topic, String, queue_size=10)
        rospy.Subscriber(self.cmd_topic, Twist, self.cb_cmd_vel, queue_size=1)
        rospy.Subscriber(self.goal_topic, Point, self.cb_goal, queue_size=1)
        rospy.Subscriber(self.action_topic, String, self.cb_action, queue_size=1)

        self.last_cmd = None
        self.last_cmd_time = 0.0
        self.is_hovering = True
        self.goal_active = False
        self.action_active = False
        self.action_timeout = float(rospy.get_param("control/action_timeout", 30.0))
        self.rate = rospy.Rate(self.hz)
        self._publish_status("READY")

    # ------------------------------------------------------------------ 回调
    def cb_cmd_vel(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = rospy.get_time()

    def cb_goal(self, msg: Point):
        if self.goal_active or self.action_active:
            rospy.logwarn("cmd_vel_sub: 已有目标点/动作任务在执行，忽略新目标 (%.2f, %.2f, %.2f)",
                          msg.x, msg.y, msg.z)
            return
        target = (float(msg.x), float(msg.y), float(msg.z))
        rospy.loginfo("cmd_vel_sub: 收到目标点 ENU (%.2f, %.2f, %.2f)", *target)
        th = threading.Thread(target=self._goal_worker, args=(target,))
        th.daemon = True
        th.start()

    def _goal_worker(self, target):
        self.goal_active = True
        self._publish_status("GOAL %.2f %.2f %.2f" % target)
        try:
            self.sim.move_to_position_enu(target, speed=self.goal_speed)
            self._publish_status("GOAL_DONE %.2f %.2f %.2f" % target)
            rospy.loginfo("cmd_vel_sub: 已到达目标点 (%.2f, %.2f, %.2f)", *target)
        except Exception as exc:  # noqa: BLE001
            self._publish_status("GOAL_FAILED %s" % exc)
            rospy.logwarn("cmd_vel_sub: 目标点任务失败: %s", exc)
        finally:
            self.goal_active = False
            self.last_cmd = None
            self.sim.hover()
            self.is_hovering = True

    # ------------------------------------------------------------------ 动作通道
    def cb_action(self, msg: String):
        """``/uav/action``：takeoff / land / hover（手势模块等上层用）."""
        action = (msg.data or "").strip().lower()
        if action not in ("takeoff", "land", "hover"):
            rospy.logwarn("cmd_vel_sub: 未知动作 %r（支持 takeoff/land/hover）", action)
            return
        if self.goal_active or self.action_active:
            rospy.logwarn("cmd_vel_sub: 已有目标点/动作任务在执行，忽略 %s", action)
            return
        rospy.loginfo("cmd_vel_sub: 收到动作指令 %s", action)
        th = threading.Thread(target=self._action_worker, args=(action,))
        th.daemon = True
        th.start()

    def _action_worker(self, action):
        """同步的 takeoff/land 会阻塞到动作完成，必须放线程里跑。"""
        self.action_active = True
        self.last_cmd = None            # 动作期间速度通道让位
        self._publish_status("ACTION:%s" % action)
        try:
            if action == "takeoff":
                self.sim.takeoff(timeout=self.action_timeout)
            elif action == "land":
                self.sim.land(timeout=self.action_timeout)
            else:
                self.sim.hover()
            self.is_hovering = True
            self._publish_status("ACTION_DONE %s" % action)
            rospy.loginfo("cmd_vel_sub: 动作 %s 完成", action)
        except Exception as exc:  # noqa: BLE001
            self._publish_status("ACTION_FAILED %s %s" % (action, exc))
            rospy.logwarn("cmd_vel_sub: 动作 %s 失败: %s", action, exc)
        finally:
            self.action_active = False

    # ------------------------------------------------------------------ 主循环
    def step_once(self):
        """执行一次控制循环（独立成函数，便于不依赖 ROS 的单元测试）."""
        if self.goal_active or self.action_active:
            return  # 目标点 / 动作任务优先，速度通道让位

        fresh_cmd = (self.last_cmd is not None and
                     (rospy.get_time() - self.last_cmd_time) < self.timeout)
        if fresh_cmd:
            cmd = self.last_cmd
            vx = _clamp(cmd.linear.x, self.max_h)
            vy = _clamp(cmd.linear.y, self.max_h)
            vz = _clamp(cmd.linear.z, self.max_v)
            wz = _clamp(cmd.angular.z, self.max_yaw)
            try:
                if self.body_frame:
                    # 机体系：ROS(前/左/上) -> AirSim(前/右/下) 由 SimClient 处理
                    self.sim.move_by_velocity_body(vx, vy, vz,
                                                   duration=self.cmd_duration, yaw_rate=wz)
                else:
                    self.sim.move_by_velocity_enu((vx, vy, vz),
                                                  duration=self.cmd_duration, yaw_rate=wz)
                if self.is_hovering:
                    self._publish_status("VELOCITY")
                    self.is_hovering = False
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn_throttle(2.0, "cmd_vel_sub: 下发速度指令失败: %s", exc)
            return

        # 没有新指令或已超时 -> 自动悬停
        if not self.is_hovering:
            try:
                self.sim.hover()
                self._publish_status("HOVER")
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn_throttle(2.0, "cmd_vel_sub: 悬停指令失败: %s", exc)
            self.is_hovering = True

    def spin(self):
        while not rospy.is_shutdown():
            self.step_once()
            self.rate.sleep()

    def _publish_status(self, text):
        self.pub_status.publish(String(data=text))


def main():
    rospy.init_node("uav_cmd_vel_sub", anonymous=False)
    node = CmdVelSubscriber()
    node.spin()


if __name__ == "__main__":
    main()
