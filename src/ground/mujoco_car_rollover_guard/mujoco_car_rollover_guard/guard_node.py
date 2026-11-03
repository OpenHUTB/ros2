#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 风险门控节点：订阅 IMU 与里程计，用训练好的网络预测 0.5s 内翻车概率，
把风险转成限速/制动后下发 /cmd_vel。

订阅：
    /imu     sensor_msgs/Imu
    /odom    nav_msgs/Odometry
发布：
    /rollover_risk  std_msgs/Float32      0~1 的翻车概率
    /cmd_vel        geometry_msgs/Twist  限速后的执行指令
    /guard_state    std_msgs/Float32MultiArray  [风险, 油门比例, 动作码 0/1/2]
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray

from mujoco_car_rollover_guard.config import RESULTS_DIR, Config
from mujoco_car_rollover_guard.features import from_imu, normalize
from mujoco_car_rollover_guard.guard import RolloverGuard
from mujoco_car_rollover_guard.model import RiskMLP


class GuardNode(Node):
    def __init__(self):
        super().__init__("mujoco_car_rollover_guard")
        self.declare_parameter("v_target", 1.1)
        self.declare_parameter("steer_ramp", 0.45)
        self.declare_parameter("run_seconds", 9.0)
        self.declare_parameter("model_path", "")

        self.cfg = Config()
        self.v_target = float(self.get_parameter("v_target").value)
        self.ramp = float(self.get_parameter("steer_ramp").value)
        self.run_seconds = float(self.get_parameter("run_seconds").value)

        model_path = self.get_parameter("model_path").value
        if not model_path:
            model_path = RESULTS_DIR / "rollover_risk.npz"
        self.net = RiskMLP.load(str(model_path))
        self.get_logger().info("已加载风险网络 {}".format(model_path))

        self.guard = RolloverGuard(self.cfg)
        self.imu = None
        self.speed = 0.0
        self.tilt = 0.0
        self.t = 0.0
        self.steer = 0.0
        self.thr_cap = 0.0
        self.last_action = "pass"
        self.brake_count = 0
        self.slow_count = 0
        self.max_risk = 0.0
        self.hold = 0                # 动作保持帧数，压住相邻帧的风险跳变
        self.hold_action = "pass"
        self.hold_thr = 0.0
        self.accel_done = False
        self.risk_hist = deque(maxlen=3)
        self.mode = "pass"
        self.calm = 0
        self.release_risk = 0.35

        self.pub_risk = self.create_publisher(Float32, "/rollover_risk", 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_state = self.create_publisher(Float32MultiArray, "/guard_state", 10)
        self.create_subscription(Imu, "/imu", self._on_imu, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

    def _on_imu(self, msg):
        self.imu = msg
        q = msg.orientation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        up_z = 1.0 - 2.0 * (qx * qx + qy * qy)
        self.tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
        # 门控由 IMU 帧驱动：一帧传感器算一次指令，和仿真步一一对应，
        # 比两个独立的 50 Hz 定时器更容易复现
        self._tick()

    def _on_odom(self, msg):
        self.speed = float(msg.twist.twist.linear.x)

    def _tick(self):
        if self.imu is None:
            return
        self.t += self.cfg.dt
        self.thr_cap = min(1.0, self.thr_cap + self.cfg.throttle_ramp * self.cfg.dt)
        thr_cmd = 0.55 + self.cfg.speed_keep_gain * (self.v_target - self.speed)
        thr_cmd = float(min(self.thr_cap, max(0.0, min(1.0, thr_cmd))))

        q = self.imu.orientation
        gyro = self.imu.angular_velocity
        acc = self.imu.linear_acceleration
        raw = from_imu((q.w, q.x, q.y, q.z),
                       (gyro.x, gyro.y, gyro.z),
                       (acc.x, acc.y, acc.z),
                       self.speed, self.steer)
        risk = float(self.net.predict_proba(normalize(raw, self.cfg)))
        self.max_risk = max(self.max_risk, risk)
        # 网络输出在临界附近会在相邻帧间跳到 0.00/1.00，单帧取值会让同一工况
        # 两次运行结果相反；门控取最近 3 帧最大值（安全侧），压低这种抖动
        self.risk_hist.append(risk)
        risk_gate = max(self.risk_hist)
        # 释放迟滞：已经限过速的车，要等风险连续 5 帧都低于 0.35 才恢复油门，
        # 否则刹车一松、车还在大转向，下一帧又翻
        if self.mode != "pass" and risk_gate < self.release_risk:
            self.calm += 1
        else:
            self.calm = 0
        if self.hold > 0:
            self.hold -= 1
            self.guard.smooth(risk_gate)
            steer_out = self.steer
            action = self.hold_action
            thr_out = self.hold_thr
        elif self.mode != "pass" and self.calm < 5:
            self.guard.smooth(risk_gate)
            steer_out = self.steer
            action = self.mode
            span = max(1e-6, 1.0 - self.cfg.risk_threshold)
            thr_out = thr_cmd * max(0.0, 1.0 - self.cfg.guard_gain *
                                    (risk_gate - self.cfg.risk_threshold) / span)
            thr_out = 0.0 if self.mode == "brake" and risk_gate >= self.cfg.guard_brake_risk else thr_out
        else:
            steer_out, thr_out, action = self.guard.gate(risk_gate, self.steer, thr_cmd)
            if action != "pass":
                self.hold = 6
                self.hold_action = action
                self.hold_thr = thr_out
        self.mode = action
        if action != self.last_action:
            self.get_logger().info("t={:.2f}s 风险={:.2f} 动作 {} -> {} 油门 {:.2f} -> {:.2f}".format(
                self.t, risk_gate, self.last_action, action, thr_cmd, thr_out))
        self.last_action = action
        if action == "brake":
            self.brake_count += 1
        elif action == "slow":
            self.slow_count += 1

        cmd = Twist()
        cmd.linear.x = thr_out * self.cfg.v_max
        cmd.angular.z = steer_out * self.cfg.omega_max
        self.pub_cmd.publish(cmd)
        self.pub_risk.publish(Float32(data=risk))
        st = Float32MultiArray()
        st.data = [risk, thr_out, float({"pass": 0, "slow": 1, "brake": 2}[action]), self.tilt]
        self.pub_state.publish(st)

        # 与 scenario.run_episode 一致：先把车直行放到 0.95 倍目标车速，再开始加转向，
        # 否则起步阶段就在加速同时压转向，会比离线评测更早侧翻
        if not self.accel_done and self.speed >= 0.95 * self.v_target:
            self.accel_done = True
        if self.accel_done:
            self.steer = min(1.0, self.steer + self.ramp * self.cfg.dt)
        if self.tilt >= self.cfg.rollover_tilt_deg:
            self.get_logger().info("侧翻发生：t={:.2f}s 倾角={:.1f}°".format(self.t, self.tilt))
            rclpy.shutdown()
        elif self.t >= self.run_seconds:
            self.get_logger().info(
                "工况结束：未翻车 时长={:.2f}s 制动帧={} 减速帧={} 最大风险={:.2f} 最终转向={:.2f}".format(
                    self.t, self.brake_count, self.slow_count, self.max_risk, self.steer))
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
