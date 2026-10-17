#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车道保持控制器：把横向偏差与航向偏差闭环成车辆转向指令。

控制结构
--------
采用经典的**级联式横向控制**（autonomous driving 里最常见的形式）：

    横向偏差 e_y  --+
                    +--> PID --> 期望转角 delta
    航向偏差 e_psi--+

即
    delta = Kp * e_y + Ki * ∫e_y dt + Kd * de_y/dt + Kpsi * e_psi

其中 e_y 为车辆相对车道中心的横向偏差（米，正=偏右），
e_psi 为车辆航向与车道方向的夹角（弧度）。

符号约定
--------
CARLA / ROS 中：e_y > 0 表示车在车道中心右侧，应向左修正。
这里约定输出 `steering` 为正表示**向左打方向**，因此控制律里用「负的 e_y」：
    delta = -(Kp * e_y + Ki * ∫e_y + Kd * de_y/dt) - Kpsi * e_psi
若实测发现方向相反，把参数 `~invert_steering` 置 true 即可，无需改代码。

输出
----
  发布  /carla/ego_vehicle/vehicle_control_cmd  carla_msgs/CarlaEgoVehicleControl（若可用）
  发布  /carla/ego_vehicle/twist                geometry_msgs/Twist（配合 carla_twist_to_control）
  发布  lane/steering                            std_msgs/Float32（调试用，看转向量）

参数见 config/lane_params.yaml 的 control 段。
"""

import os
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from carla_msgs.msg import CarlaEgoVehicleControl  # noqa: E402

    _HAVE_CARLA_MSGS = True
except ImportError:  # pragma: no cover - 取决于环境
    _HAVE_CARLA_MSGS = False


class PidController:
    """带积分限幅与输出限幅的 PID（工程上比纯公式多一些保护）。"""

    def __init__(self, kp, ki, kd, out_limit, integral_limit):
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
        self.out_limit = float(out_limit)
        self.integral_limit = float(integral_limit)
        self._integral = 0.0
        self._prev_error = None
        self._prev_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = None
        self._prev_time = None

    def step(self, error, now=None):
        """输入误差，输出控制量。"""
        now = time.time() if now is None else now
        if self._prev_time is None:
            dt = 0.0
        else:
            dt = max(1e-3, now - self._prev_time)  # 防止 dt=0 导致微分爆炸

        # 积分项（带限幅，避免长时间偏差导致积分饱和）
        if dt > 0.0:
            self._integral += error * dt
            self._integral = float(
                np.clip(self._integral, -self.integral_limit, self.integral_limit)
            )

        # 微分项
        derivative = 0.0
        if self._prev_error is not None and dt > 0.0:
            derivative = (error - self._prev_error) / dt

        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        self._prev_error = error
        self._prev_time = now
        return float(np.clip(output, -self.out_limit, self.out_limit))


class LaneKeepingController:
    """订阅感知结果，输出车辆控制指令。"""

    def __init__(self):
        rospy.init_node("lane_keeping_controller", anonymous=False)

        # --- 控制参数 ---
        self.kp = float(rospy.get_param("~kp", 0.28))
        self.ki = float(rospy.get_param("~ki", 0.02))
        self.kd = float(rospy.get_param("~kd", 0.06))
        self.k_psi = float(rospy.get_param("~k_psi", 0.9))
        self.steering_limit = float(rospy.get_param("~steering_limit", 0.7))
        self.integral_limit = float(rospy.get_param("~integral_limit", 2.0))
        self.invert_steering = bool(rospy.get_param("~invert_steering", False))

        # --- 纵向参数 ---
        self.target_speed = float(rospy.get_param("~target_speed", 6.0))  # m/s
        self.throttle_gain = float(rospy.get_param("~throttle_gain", 0.18))
        self.speed_correction_gain = float(
            rospy.get_param("~speed_correction_gain", 0.25)
        )

        # --- 安全参数 ---
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.5))
        self.perception_timeout = float(rospy.get_param("~perception_timeout", 0.5))
        self.brake_on_departure = bool(rospy.get_param("~brake_on_departure", False))

        self.pid = PidController(
            self.kp, self.ki, self.kd, self.steering_limit, self.integral_limit
        )

        self._lateral = None
        self._heading = None
        self._confidence = 0.0
        self._last_perception_time = None
        self._departure = False

        # --- 发布者 ---
        self.pub_steering = rospy.Publisher("lane/steering", Float32, queue_size=1)
        if _HAVE_CARLA_MSGS:
            self.pub_control = rospy.Publisher(
                "/carla/ego_vehicle/vehicle_control_cmd",
                CarlaEgoVehicleControl,
                queue_size=1,
            )
        else:
            self.pub_control = None
        self.pub_twist = rospy.Publisher(
            "/carla/ego_vehicle/twist", Twist, queue_size=1
        )

        # --- 订阅者 ---
        rospy.Subscriber("lane/lateral_offset", Float32, self.on_lateral, queue_size=1)
        rospy.Subscriber("lane/heading_error", Float32, self.on_heading, queue_size=1)
        rospy.Subscriber("lane/confidence", Float32, self.on_confidence, queue_size=1)
        rospy.Subscriber("lane/departure", Bool, self.on_departure, queue_size=1)

        # 固定频率输出控制指令（比每帧回调里发更稳定）
        self.rate_hz = float(rospy.get_param("~control_rate", 20.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.on_timer)

        rospy.loginfo(
            "[lane_keeping] 已启动：Kp=%.2f Ki=%.2f Kd=%.2f Kpsi=%.2f 目标速度=%.1f m/s "
            "carla_msgs=%s",
            self.kp, self.ki, self.kd, self.k_psi, self.target_speed, _HAVE_CARLA_MSGS,
        )

    # ---------------- 订阅回调 ----------------

    def on_lateral(self, msg):
        self._lateral = float(msg.data)
        self._last_perception_time = time.time()

    def on_heading(self, msg):
        self._heading = float(msg.data)

    def on_confidence(self, msg):
        self._confidence = float(msg.data)

    def on_departure(self, msg):
        self._departure = bool(msg.data)

    # ---------------- 控制循环 ----------------

    def on_timer(self, _event):
        now = time.time()
        fresh = (
            self._last_perception_time is not None
            and (now - self._last_perception_time) <= self.perception_timeout
        )
        valid = (
            fresh
            and self._lateral is not None
            and self._heading is not None
            and self._confidence >= self.min_confidence
        )

        if not valid:
            # 感知不可用：不输出转向，让车辆滑行/停住，并把 PID 复位避免积分残留
            self.pid.reset()
            self._publish(steering=0.0, throttle=0.0, brake=0.3 if self._departure else 0.0)
            rospy.logwarn_throttle(
                2.0,
                "[lane_keeping] 感知不可用（新鲜=%s 置信度=%.2f），已停止转向输出",
                fresh,
                self._confidence,
            )
            return

        # 横向：PID 处理偏差，P 项处理航向
        pid_out = self.pid.step(self._lateral, now)
        steering = -(pid_out) - self.k_psi * self._heading
        if self.invert_steering:
            steering = -steering
        steering = float(np.clip(steering, -self.steering_limit, self.steering_limit))

        # 纵向：偏差越大越减速，保证修正时有余量
        speed_factor = float(
            np.clip(1.0 - self.speed_correction_gain * abs(self._lateral), 0.2, 1.0)
        )
        throttle = float(
            np.clip(self.throttle_gain * self.target_speed * speed_factor, 0.0, 1.0)
        )
        brake = 0.0
        if self.brake_on_departure and self._departure:
            throttle = 0.0
            brake = 0.5

        self.pub_steering.publish(Float32(data=steering))
        self._publish(steering=steering, throttle=throttle, brake=brake)
        rospy.loginfo_throttle(
            1.0,
            "[lane_keeping] e_y=%+.3f m  e_psi=%+.2f deg  steering=%+.3f  throttle=%.2f",
            self._lateral,
            np.degrees(self._heading),
            steering,
            throttle,
        )

    def _publish(self, steering, throttle, brake):
        """同时发布 CarlaEgoVehicleControl 与 Twist 两种指令，适配不同桥接方式。"""
        if self.pub_control is not None:
            cmd = CarlaEgoVehicleControl()
            cmd.steer = float(steering)
            cmd.throttle = float(throttle)
            cmd.brake = float(brake)
            cmd.hand_brake = False
            cmd.reverse = False
            cmd.gear = 1
            cmd.manual_gear_shift = False
            self.pub_control.publish(cmd)

        twist = Twist()
        # carla_twist_to_control 会把 linear.x 当油门、angular.z 当转向
        twist.linear.x = float(throttle)
        twist.angular.z = float(steering)
        self.pub_twist.publish(twist)


def main():
    try:
        LaneKeepingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
