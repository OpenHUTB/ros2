#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纵向速度控制 ROS 节点。

订阅
----
``speed/curvature``       std_msgs/Float32   前方路径曲率（1/m），负值表示右转
``speed/target``          std_msgs/Float32   目标速度（m/s），可选；缺省用参数 target_speed
``/carla/ego_vehicle/odom``  nav_msgs/Odometry  可选，用于取当前车速

发布
----
``speed/limit``           std_msgs/Float32   当前限速（m/s），含曲率约束
``speed/command``         std_msgs/Float32   加速度指令（m/s²），负值减速
``speed/control``         geometry_msgs/Twist  纵向速度指令（linear.x = 目标速度）
``speed/diagnostics``     std_msgs/Float32MultiArray
                          [当前速度, 限速, 加速度指令, 横向加速度, 是否限速生效]

与 camera_lane_keeping 的配合
-----------------------------
``carla_lane_keeping`` 的 ``lane_detector.py`` 会发布 ``lane/curvature``
（车道曲率半径的倒数）。把本节点的 ``curvature_topic`` 参数指向它即可实现
"车道线检测 → 曲率 → 自动限速"的完整链路：

.. code-block:: shell

    rosrun carla_speed_control speed_controller.py _curvature_topic:=/carla/ego_vehicle/lane/curvature

设计说明
--------
* 当前速度优先取里程计；没有里程计时退化为"用上一拍指令积分"的估计值，
  并在诊断里标注（避免下游误以为拿到了真实车速）。
* 曲率话题有超时保护：超过 ``curvature_timeout`` 秒没收到新曲率，就认为
  前方路径未知，退化为按 ``target_speed`` 定速行驶，而不是沿用旧曲率——
  沿用旧曲率会在直道上错误地保持弯道限速。
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from speed_control import (  # noqa: E402
    SpeedController,
    SpeedControllerConfig,
    VehicleLimits,
    lateral_accel_at,
    speed_limit_from_curvature,
)

# 参数默认值（与 config/speed_params.yaml 保持一致）
DEFAULTS = {
    "target_speed": 12.0,
    "kp": 1.0,
    "ki": 0.15,
    "kd": 0.35,
    "kff": 1.0,
    "integral_limit": 2.0,
    "max_accel": 3.0,
    "max_decel": 6.0,
    "max_jerk": 4.0,
    "max_lateral_accel": 3.0,
    "min_speed": 1.0,
    "lookahead_time": 1.5,
    "overspeed_tolerance": 0.2,
    "overspeed_gain": 2.0,
    "curvature_timeout": 0.5,
    "publish_rate": 20.0,
    "invert_command": False,
}


def load_config_from_params():
    """从 ROS 参数服务器读取配置。缺失的项用 DEFAULTS。"""

    def get(name):
        return rospy.get_param("~" + name, DEFAULTS[name])

    limits = VehicleLimits(
        max_accel=float(get("max_accel")),
        max_decel=float(get("max_decel")),
        max_jerk=float(get("max_jerk")),
        max_lateral_accel=float(get("max_lateral_accel")),
        min_speed=float(get("min_speed")),
    )
    cfg = SpeedControllerConfig(
        kp=float(get("kp")),
        ki=float(get("ki")),
        kd=float(get("kd")),
        kff=float(get("kff")),
        integral_limit=float(get("integral_limit")),
        limits=limits,
        lookahead_time=float(get("lookahead_time")),
        overspeed_tolerance=float(get("overspeed_tolerance")),
        overspeed_gain=float(get("overspeed_gain")),
    )
    cfg.validate()
    return cfg


class SpeedControllerNode:
    """把曲率 + 目标速度变成加速度指令的 ROS 节点。"""

    def __init__(self):
        self.config = load_config_from_params()
        self.controller = SpeedController(self.config)

        self.target_speed = float(rospy.get_param("~target_speed", DEFAULTS["target_speed"]))
        self.curvature_timeout = float(
            rospy.get_param("~curvature_timeout", DEFAULTS["curvature_timeout"])
        )
        self.publish_rate = float(rospy.get_param("~publish_rate", DEFAULTS["publish_rate"]))
        self.invert_command = bool(
            rospy.get_param("~invert_command", DEFAULTS["invert_command"])
        )

        self.curvature = 0.0
        self.curvature_stamp = None
        self.current_speed = 0.0
        self.has_odom = False

        # 话题名可通过参数覆盖，便于与其它模块对接
        curvature_topic = rospy.get_param("~curvature_topic", "speed/curvature")
        target_topic = rospy.get_param("~target_topic", "speed/target")
        odom_topic = rospy.get_param("~odom_topic", "/carla/ego_vehicle/odom")

        self.pub_limit = rospy.Publisher("speed/limit", Float32, queue_size=1)
        self.pub_cmd = rospy.Publisher("speed/command", Float32, queue_size=1)
        self.pub_twist = rospy.Publisher("speed/control", Twist, queue_size=1)
        self.pub_diag = rospy.Publisher("speed/diagnostics", Float32MultiArray, queue_size=1)

        rospy.Subscriber(curvature_topic, Float32, self.on_curvature, queue_size=1)
        rospy.Subscriber(target_topic, Float32, self.on_target, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.on_odom, queue_size=1)

        rospy.loginfo(
            "速度控制器已启动：曲率话题=%s，目标速度=%.2f m/s，"
            "横向加速度上限=%.2f m/s²",
            curvature_topic, self.target_speed, self.config.limits.max_lateral_accel,
        )

    # -- 回调 -------------------------------------------------------------

    def on_curvature(self, msg):
        """收到新曲率。曲率非有限值（NaN/inf）时按直道处理并告警。"""
        value = float(msg.data)
        if not math.isfinite(value):
            rospy.logwarn_throttle(5.0, "曲率非有限值(%.3f)，按直道处理", value)
            value = 0.0
        self.curvature = value
        self.curvature_stamp = rospy.Time.now()

    def on_target(self, msg):
        value = float(msg.data)
        if value > 0.0 and math.isfinite(value):
            self.target_speed = value

    def on_odom(self, msg):
        self.current_speed = float(msg.twist.twist.linear.x)
        self.has_odom = True

    # -- 主循环 -----------------------------------------------------------

    def curvature_valid(self):
        """曲率是否在有效期内。"""
        if self.curvature_stamp is None:
            return False
        return (rospy.Time.now() - self.curvature_stamp).to_sec() <= self.curvature_timeout

    def step(self, dt):
        """推进一个控制周期并发布结果。"""
        lim = self.config.limits

        # 曲率超时则按直道处理（不沿用旧曲率，避免直道上保持弯道限速）
        if self.curvature_valid():
            kappa = self.curvature
        else:
            kappa = 0.0

        v_curve = speed_limit_from_curvature(
            kappa, lim.max_lateral_accel, lim.min_speed
        )
        v_limit = min(self.target_speed, v_curve)

        a_cmd = self.controller.update(v_limit, self.current_speed, dt, target_accel=0.0)

        # 与仿真保持一致：超速兜底后仍需经过 jerk 限制
        from speed_control import apply_jerk_limit  # noqa: PLC0415

        overspeed = self.current_speed - (v_limit + self.config.overspeed_tolerance)
        if overspeed > 0.0:
            a_cmd = min(a_cmd, -self.config.overspeed_gain * overspeed)
        a_cmd = apply_jerk_limit(
            a_cmd, self.controller.last_accel, dt, lim.max_jerk
        )
        self.controller.sync_accel(a_cmd)

        # 没有里程计时，用指令积分粗略估计速度，并在日志里提示一次
        if not self.has_odom:
            self.current_speed = max(0.0, self.current_speed + a_cmd * dt)
            rospy.logwarn_throttle(
                10.0, "未收到里程计，当前速度由指令积分估计（不准确）"
            )

        if self.invert_command:
            a_cmd = -a_cmd

        # 发布
        self.pub_limit.publish(Float32(data=float(v_limit)))
        self.pub_cmd.publish(Float32(data=float(a_cmd)))

        twist = Twist()
        twist.linear.x = float(self.current_speed)
        self.pub_twist.publish(twist)

        diag = Float32MultiArray()
        diag.data = [
            float(self.current_speed),
            float(v_limit),
            float(a_cmd),
            float(lateral_accel_at(self.current_speed, kappa)),
            1.0 if v_limit < self.target_speed - 1e-6 else 0.0,
        ]
        self.pub_diag.publish(diag)

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        last = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last).to_sec()
            last = now
            if dt <= 0.0:
                dt = 1.0 / max(1.0, self.publish_rate)
            # 单步 dt 过大（例如仿真卡顿）时按标称周期处理，避免积分跳变
            dt = min(dt, 5.0 / max(1.0, self.publish_rate))
            self.step(dt)
            rate.sleep()


def main():
    rospy.init_node("speed_controller", anonymous=False)
    node = SpeedControllerNode()
    node.spin()


if __name__ == "__main__":
    main()
