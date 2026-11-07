#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 仿真桥：把 MuJoCo 差速小车接入 ROS 2。

发布：
    /imu            sensor_msgs/Imu        机体系姿态、角速度、加速度（真实 IMU 传感器读数）
    /odom           nav_msgs/Odometry      位姿与前向速度
    /chassis_state  std_msgs/Float32MultiArray  [横滚角, 俯仰角, 总倾角, 前向速度(度/度/度/m/s)]
订阅：
    /cmd_vel        geometry_msgs/Twist    linear.x = 目标车速, angular.z = 目标偏航角速度

节点跑完后打印本工况的真实结果（是否侧翻、最大倾角、最大横滚），供 launch 汇总。
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

from mujoco_car_rollover_guard.car_env import CarEnv
from mujoco_car_rollover_guard.config import Config


class SimBridge(Node):
    def __init__(self):
        super().__init__("mujoco_car_rollover_sim_bridge")
        self.declare_parameter("scene", "car_scene_payload_mid.xml")
        self.declare_parameter("run_seconds", 9.0)
        self.declare_parameter("control_hz", 50.0)

        self.cfg = Config()
        scene = self.get_parameter("scene").value
        self.run_seconds = float(self.get_parameter("run_seconds").value)
        rate = float(self.get_parameter("control_hz").value)

        self.env = CarEnv(self.cfg, scene)
        self.env.reset()
        self.scene = scene

        self.cmd = (0.0, 0.0)          # (steer, throttle)
        self.t = 0.0
        self.max_tilt = 0.0
        self.max_roll = 0.0
        self.rollover = False
        self.rest_quat = np.array([1.0, 0.0, 0.0, 0.0])

        self.pub_imu = self.create_publisher(Imu, "/imu", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_state = self.create_publisher(Float32MultiArray, "/chassis_state", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_timer(1.0 / rate, self._tick)

    def _on_cmd(self, msg):
        steer = msg.angular.z / self.cfg.omega_max
        throttle = msg.linear.x / self.cfg.v_max
        self.cmd = (max(-1.0, min(1.0, steer)), max(0.0, min(1.0, throttle)))

    def _tick(self):
        steer, throttle = self.cmd
        self.env.set_command(steer, throttle)
        self.t += self.cfg.dt

        imu = self.env.imu()
        quat = imu["quat"]
        roll, pitch = self.env.roll_pitch(quat)
        tilt = self.env.tilt_deg()
        self.max_tilt = max(self.max_tilt, tilt)
        self.max_roll = max(self.max_roll, abs(math.degrees(roll)))
        v = self.env.forward_speed()

        stamp = self.get_clock().now().to_msg()
        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = "imu_link"
        m.orientation.w, m.orientation.x = float(quat[0]), float(quat[1])
        m.orientation.y, m.orientation.z = float(quat[2]), float(quat[3])
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (
            float(imu["gyro"][0]), float(imu["gyro"][1]), float(imu["gyro"][2]))
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = (
            float(imu["acc"][0]), float(imu["acc"][1]), float(imu["acc"][2]))
        self.pub_imu.publish(m)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        pos = self.env.pos()
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = (
            float(pos[0]), float(pos[1]), float(pos[2]))
        od.pose.pose.orientation.w = float(quat[0])
        od.pose.pose.orientation.x = float(quat[1])
        od.pose.pose.orientation.y = float(quat[2])
        od.pose.pose.orientation.z = float(quat[3])
        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = float(imu["gyro"][2])
        self.pub_odom.publish(od)

        st = Float32MultiArray()
        st.data = [float(math.degrees(roll)), float(math.degrees(pitch)), float(tilt), float(v)]
        self.pub_state.publish(st)

        if tilt >= self.cfg.rollover_tilt_deg:
            self.rollover = True
        if self.rollover or self.t >= self.run_seconds:
            self.get_logger().info(
                "工况结束：场景={} 时长={:.2f}s 翻车={} 最大横滚={:.1f}° 最大倾角={:.1f}°".format(
                    self.scene, self.t, self.rollover, self.max_roll, self.max_tilt))
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
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
