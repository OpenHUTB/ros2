#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 失效检测节点：订阅 IMU / 里程计 / 轮速，用训练好的网络预测 1.0 s 内跑偏概率，
把风险转成左右轮差速修正后下发 /cmd_vel。

节点自己产生"直行"的名义指令（不含转向任务），所以补偿量就是它对名义指令的
唯一改动，便于在 rqt / 话题里直接看出介入时机。

订阅：
    /imu          sensor_msgs/Imu
    /odom         nav_msgs/Odometry
    /wheel_speed  std_msgs/Float32MultiArray  [左轮角速度, 右轮角速度]
发布：
    /fault_risk   std_msgs/Float32           0~1 的跑偏概率
    /cmd_vel      geometry_msgs/Twist        补偿后的执行指令
    /guard_state  std_msgs/Float32MultiArray [风险, 补偿强度, 动作码 0/1/2, 横向偏差, 轮速差]
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray

from mujoco_car_drive_fault.config import RESULTS_DIR, Config
from mujoco_car_drive_fault.features import normalize
from mujoco_car_drive_fault.features import from_state
from mujoco_car_drive_fault.guard import FaultGuard, comp_steer
from mujoco_car_drive_fault.kinematics import differential_wheel_vels
from mujoco_car_drive_fault.scenario import hold_throttle
from mujoco_car_drive_fault.model import RiskMLP

ACTION_CODE = {"pass": 0, "assist": 1, "slow": 2}


class FaultGuardNode(Node):
    def __init__(self):
        super().__init__("mujoco_car_drive_fault_guard")
        self.declare_parameter("v_target", 0.9)
        self.declare_parameter("run_seconds", 6.0)
        self.declare_parameter("mode", "nn")
        self.declare_parameter("model_path", "")

        self.cfg = Config()
        self.v_target = float(self.get_parameter("v_target").value)
        self.run_seconds = float(self.get_parameter("run_seconds").value)
        self.mode = str(self.get_parameter("mode").value)

        model_path = self.get_parameter("model_path").value
        if not model_path:
            model_path = RESULTS_DIR / "drive_fault_risk.npz"
        self.net = RiskMLP.load(str(model_path))
        self.get_logger().info("已加载失效检测网络 {}".format(model_path))

        self.guard = FaultGuard(self.cfg)
        self.imu = None
        self.odom = None
        self.wl = 0.0
        self.wr = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.t = 0.0
        self.steer = 0.0
        self.thr_cap = 0.0
        self.max_risk = 0.0
        self.max_abs_y = 0.0
        self.assist_frames = 0
        self.slow_frames = 0
        self.risk_hist = deque(maxlen=3)

        self.pub_risk = self.create_publisher(Float32, "/fault_risk", 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_state = self.create_publisher(Float32MultiArray, "/guard_state", 10)
        self.create_subscription(Imu, "/imu", self._on_imu, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Float32MultiArray, "/wheel_speed", self._on_wheel, 10)

    def _on_imu(self, msg):
        self.imu = msg
        # 门控由 IMU 帧驱动：一帧传感器算一次指令，和仿真步一一对应，
        # 比两个独立的 50 Hz 定时器更容易复现
        self._tick()

    def _on_odom(self, msg):
        self.odom = msg
        self.speed = float(msg.twist.twist.linear.x)
        self.y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        self.yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        self.max_abs_y = max(self.max_abs_y, abs(self.y))

    def _on_wheel(self, msg):
        if len(msg.data) >= 2:
            self.wl, self.wr = float(msg.data[0]), float(msg.data[1])

    def _tick(self):
        if self.imu is None or self.odom is None:
            return
        self.t += self.cfg.dt
        self.thr_cap = min(1.0, self.thr_cap + self.cfg.throttle_cap_rate * self.cfg.dt)
        thr_cmd = hold_throttle(self.speed, self.v_target, self.cfg)
        thr_cmd = float(min(self.thr_cap, max(0.0, min(1.0, thr_cmd))))

        wl_cmd, wr_cmd = differential_wheel_vels(
            self.steer, thr_cmd, self.cfg.v_max, self.cfg.omega_max,
            self.cfg.wheel_r, self.cfg.track, self.cfg.max_wheel_omega)
        gyro = self.imu.angular_velocity
        acc = self.imu.linear_acceleration
        raw = from_state(self.y, self.yaw, (gyro.x, gyro.y, gyro.z),
                         (acc.x, acc.y, acc.z), self.speed,
                         self.wl, self.wr, wl_cmd, wr_cmd)
        x = normalize(raw, self.cfg)
        risk = float(self.net.predict_proba(x))
        self.max_risk = max(self.max_risk, risk)
        # 网络输出在临界附近会在相邻帧间跳到 0.00/1.00，取最近 3 帧最大值压抖动
        self.risk_hist.append(risk)
        risk_gate = max(self.risk_hist)

        gain, action = self.guard.decide(risk_gate, self.y, math.degrees(self.yaw),
                                         mode=self.mode)
        integ = self.guard.update_integ(self.y) if gain > 0 else 0.0
        self.steer = comp_steer(self.y, self.yaw, gyro.z, self.cfg, gain, integ) \
            if gain > 0 else 0.0
        if action == "assist":
            self.assist_frames += 1
        elif action == "slow":
            self.slow_frames += 1

        thr_out = thr_cmd * (0.5 if action == "slow" else 1.0)
        cmd = Twist()
        cmd.linear.x = thr_out * self.cfg.v_max
        cmd.angular.z = self.steer * self.cfg.omega_max
        self.pub_cmd.publish(cmd)
        self.pub_risk.publish(Float32(data=risk))
        st = Float32MultiArray()
        st.data = [risk, gain, float(ACTION_CODE[action]), self.y, self.wl - self.wr]
        self.pub_state.publish(st)

        if abs(self.y) >= 1.35:
            self.get_logger().info(
                "跑出车道：t={:.2f}s 横向偏差={:+.2f}m 最大风险={:.2f} 补偿帧={} 减速帧={}".format(
                    self.t, self.y, self.max_risk, self.assist_frames, self.slow_frames))
            rclpy.shutdown()
        elif self.t >= self.run_seconds:
            self.get_logger().info(
                "工况结束：时长={:.2f}s 最大偏差={:.2f}m 最大风险={:.2f} 补偿帧={} 减速帧={}".format(
                    self.t, self.max_abs_y, self.max_risk, self.assist_frames, self.slow_frames))
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FaultGuardNode()
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
