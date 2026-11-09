#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""移动平台精准降落主节点：ROS 话题收发 + 状态机（算法全在 moving_landing_core）。

状态流转（与 core 里同名常量）::

    WAIT_DECK ──拿到足够的平台观测──▶ ALIGN ──连续 align_settle_s 满足对准──▶ DESCEND
    DESCEND  ──相对高度 ≤ touchdown_rel_height──▶ TOUCHDOWN ──▶ LANDED
    DESCEND  ──预计落偏超限持续 abort_hold_s──▶ ALIGN（复飞，计数 +1）
    任意状态 ──超时 / 复飞次数用完 / 穿到甲板下面 / 观测断流太久──▶ ABORT（原地悬停）

设计要点
--------
* 本节点**不直接连仿真器**：连仿真器、NED↔ENU 换算、速度限幅都由
  ``carlair_ros_bridge`` 负责，本模块只订阅平台位置、下发 ``/uav/cmd_vel``，
  因此与键盘控制、屋顶降落等模块共用同一条速度通道。
* 平台位置**只吃一路**（``/uav/platform_pose``），速度不订阅——由估计器自己算，
  这样接视觉（Apriltag/色块）还是接仿真器的另一个载具都能用。
* 高度分量「上为正还是下为正」由参数 ``frame/z_up`` 决定（桥接层实测是 ENU，
  默认 true）；启动后会打印第一帧原始高度与换算结果，便于第一时间发现约定不符。
* ``/uav/cmd_vel`` 必须以 10 Hz 持续下发，且桥接层 ``/control/cmd_duration`` 要
  调到 0.5 s（默认 0.1 s 会把还没生效的指令顶掉，飞机几乎不动），launch 里有说明。
"""
from __future__ import annotations

import json
import os
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import moving_landing_core as core                      # noqa: E402


def load_params():
    """从 ROS 参数服务器读全部参数（键名与 core.DEFAULT_PARAMS 一致）。"""
    params = {}
    for key, default in core.DEFAULT_PARAMS.items():
        params[key] = rospy.get_param(key, default)
    # 参数服务器上是字符串时统一转回数字
    for key, default in core.DEFAULT_PARAMS.items():
        if isinstance(default, (int, float)) and not isinstance(default, bool):
            try:
                params[key] = float(params[key])
            except (TypeError, ValueError):
                params[key] = default
    return params


class MovingPlatformLandingNode(object):
    def __init__(self):
        rospy.init_node("moving_platform_landing", anonymous=False)

        g = rospy.get_param
        self.t_odom = g("topic/odom", "/uav/odom")
        self.t_deck = g("topic/platform_pose", "/uav/platform_pose")
        self.t_cmd = g("topic/cmd_vel", "/uav/cmd_vel")
        self.t_state = g("topic/state", "/uav/moving_landing/state")
        self.t_lead = g("topic/lead", "/uav/moving_landing/lead_point")
        self.t_touch = g("topic/touchdown", "/uav/moving_landing/touchdown")
        self.z_up = bool(g("frame/z_up", True))
        self.state_rate_hz = float(g("mission/state_rate_hz", 2.0))
        self.publish_after_landed = float(g("mission/publish_after_landed_s", 2.0))

        self.params = load_params()
        self.ctl = core.MovingLandingController(self.params)
        self.ctl.reset()

        self.lock_mutex = threading.Lock()
        self.odom = None            # (t, pos_up, vel_up)
        self.deck = None            # core.DeckMeasurement
        self.odom_time = 0.0
        self.first_odom_logged = False
        self.landed_since = None

        self.pub_cmd = rospy.Publisher(self.t_cmd, Twist, queue_size=1)
        self.pub_state = rospy.Publisher(self.t_state, String, queue_size=2)
        self.pub_lead = rospy.Publisher(self.t_lead, PointStamped, queue_size=2)
        self.pub_touch = rospy.Publisher(self.t_touch, Point, queue_size=1)

        rospy.Subscriber(self.t_odom, Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(self.t_deck, PoseStamped, self.on_deck, queue_size=5)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("[moving_landing] 已启动：平台位置 <- %s，速度 -> %s，z_up=%s"
                      % (self.t_deck, self.t_cmd, self.z_up))
        rospy.loginfo("[moving_landing] 制导口径=%s（track 高度 %.1f m，下降 %.2f m/s，"
                      "横向上限 %.1f m/s）"
                      % (self.params["guide/lead_mode"],
                         self.params["guide/track_altitude"],
                         self.params["descend/speed"],
                         self.params["guide/max_xy_speed"]))
        rospy.loginfo("[moving_landing] 提醒：桥接层 /control/cmd_duration 建议 0.5 s，"
                      "默认 0.1 s 时 10 Hz 下发会把还没生效的速度指令顶掉。")

    # ------------------------------------------------------------ 收数据
    def on_odom(self, msg):
        t = msg.header.stamp.to_sec() if not msg.header.stamp.is_zero() \
            else rospy.Time.now().to_sec()
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        sign = 1.0 if self.z_up else -1.0
        pos = [p.x, p.y, sign * p.z]
        vel = [v.x, v.y, sign * v.z]
        with self.lock_mutex:
            self.odom = (t, pos, vel)
        if not self.first_odom_logged:
            self.first_odom_logged = True
            rospy.loginfo("[moving_landing] 首帧里程计：原始 z=%.2f -> 换算后高度 %.2f m"
                          % (p.z, pos[2]))

    def on_deck(self, msg):
        p = msg.pose.position
        sign = 1.0 if self.z_up else -1.0
        stamp = msg.header.stamp.to_sec() if not msg.header.stamp.is_zero() \
            else rospy.Time.now().to_sec()
        meas = core.DeckMeasurement(stamp, [p.x, p.y, sign * p.z])
        with self.lock_mutex:
            self.deck = meas

    # ------------------------------------------------------------ 主循环
    def spin(self):
        rate = rospy.Rate(self.tick_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            with self.lock_mutex:
                odom = self.odom
                meas = self.deck
                self.deck = None            # 每条观测只喂一次
            if odom is None:
                rate.sleep()
                continue
            out = self.ctl.step(now, odom[1], odom[2], meas)
            if out["state"] == core.WAIT_DECK:
                tr = self.ctl.tracker
                rospy.loginfo_throttle(
                    3.0,
                    "[moving_landing] 等平台观测：已收 %d 帧，最新一帧 t=%.3f 年龄 %.2f s，"
                    "now=%.3f（时间戳要填观测时刻）"
                    % (len(tr), (tr.t[-1] if tr.t else -1.0), tr.age(now), now))
            self.publish(out)
            rate.sleep()

    @property
    def tick_hz(self):
        """控制频率（Hz）。注意 rospy.Rate 要的是 Hz，别传成周期。"""
        return max(1.0, float(rospy.get_param("mission/tick_rate_hz", 10.0)))

    def publish(self, out):
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = out["cmd_vel"]
        self.pub_cmd.publish(cmd)

        # 状态只在真正变化时逐条打印，避免刷屏
        for ev in out["new_events"]:
            if ev["name"] == "STATE":
                rospy.loginfo("[moving_landing] 状态 -> %s（%s）"
                              % (ev["state"], ev.get("reason")))
            elif ev["name"] == "TOUCHDOWN":
                rospy.loginfo("[moving_landing] 接地！横向误差 %.2f m" % ev["error_xy"])
                self.pub_touch.publish(Point(0.0, 0.0, 0.0))
            else:
                rospy.logwarn("[moving_landing] %s：%s"
                              % (ev["name"], ev.get("reason", "")))

        if out["lead"] is not None:
            self.pub_lead.publish(PointStamped(
                header=self._header(), point=Point(*[float(x) for x in out["lead"]])))

        if self.state_rate_hz > 0 and \
                rospy.get_time() - getattr(self, "_last_state", 0.0) >= 1.0 / self.state_rate_hz:
            self._last_state = rospy.get_time()
            payload = {
                "state": out["state"],
                "lead_mode": out["lead_mode"],
                "t_touchdown": (None if out["time_to_touchdown"] is None
                                else round(out["time_to_touchdown"], 2)),
                "z_rel": (None if out["z_rel"] is None else round(out["z_rel"], 3)),
                "v_deck": (None if out["v_deck"] is None
                           else [round(float(x), 3) for x in out["v_deck"]]),
                "go_around": out["go_around"],
                "cmd_vel": [round(float(x), 3) for x in out["cmd_vel"]],
            }
            self.pub_state.publish(String(data=json.dumps(payload, ensure_ascii=False)))

        # 落地后持续发 0 速度一小会儿（让桥接层把飞机稳住），然后停止本节点
        if out["state"] == core.LANDED:
            if self.landed_since is None:
                self.landed_since = rospy.get_time()
            elif rospy.get_time() - self.landed_since > self.publish_after_landed:
                rospy.loginfo("[moving_landing] 任务完成：%s" % json.dumps(
                    {"landed_error_m": (None if out["landed_error"] is None
                                        else round(out["landed_error"], 3)),
                     "go_around": out["go_around"]}, ensure_ascii=False))
                rospy.signal_shutdown("landed")

    @staticmethod
    def _header():
        from std_msgs.msg import Header
        return Header(frame_id="world", stamp=rospy.Time.now())

    # ------------------------------------------------------------ 收尾
    def on_shutdown(self):
        for _ in range(3):
            self.pub_cmd.publish(Twist())      # 悬停：速度清零
            time.sleep(0.05)
        rep = self.ctl.report()
        rospy.loginfo("[moving_landing] 结束：state=%s，落点误差=%s，复飞=%d"
                      % (rep["state"],
                         ("%.2f m" % rep["landed_error"]) if rep["landed_error"] else "未落地",
                         rep["go_around"]))
        for (t, s, r) in rep["states"]:
            rospy.loginfo("[moving_landing]   状态轨迹 t=%.1f %s %s" % (t, s, r or ""))


def main():
    node = MovingPlatformLandingNode()
    try:
        node.spin()
    except rospy.ROSInterruptException:       # pragma: no cover - 交互退出
        pass


if __name__ == "__main__":
    main()
