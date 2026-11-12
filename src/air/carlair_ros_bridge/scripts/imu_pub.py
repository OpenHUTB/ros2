#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IMU 发布节点：CarlaAir/AirSim -> /imu (sensor_msgs/Imu).

发布：
    /imu      sensor_msgs/Imu    姿态（ENU）、角速度/线加速度（机体系 FLU）

参数（默认值见 config/bridge.yaml）：
    sim/host, sim/airsim_port, sim/vehicle_name
    rate/imu_hz, frame/imu, topic/imu
"""
from __future__ import annotations

import os
import sys

import rospy
from sensor_msgs.msg import Imu

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_client import SimClient  # noqa: E402


class ImuPublisher(object):
    def __init__(self):
        host = rospy.get_param("sim/host", "127.0.0.1")
        port = int(rospy.get_param("sim/airsim_port", 41451))
        vehicle = rospy.get_param("sim/vehicle_name", "")
        rate_hz = float(rospy.get_param("rate/imu_hz", 50.0))
        self.frame_id = rospy.get_param("frame/imu", "base_link")
        topic = rospy.get_param("topic/imu", "/imu")

        self.sim = SimClient(host=host, port=port, vehicle_name=vehicle)
        self.sim.connect()
        rospy.loginfo("imu_pub: 已连接仿真器 %s:%d", host, port)

        self.pub = rospy.Publisher(topic, Imu, queue_size=50)
        self.rate = rospy.Rate(rate_hz)
        self.fail_count = 0

    def spin(self):
        while not rospy.is_shutdown():
            try:
                quat, ang, acc = self.sim.get_imu_enu()
                self.publish(quat, ang, acc)
                self.fail_count = 0
            except Exception as exc:  # noqa: BLE001 - 仿真断连时不能让节点崩溃
                self.fail_count += 1
                if self.fail_count in (1, 10, 100):
                    rospy.logwarn("imu_pub: 读取 IMU 失败(第 %d 次): %s",
                                  self.fail_count, exc)
            self.rate.sleep()

    def publish(self, quat, ang, acc):
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        msg.orientation.x = float(quat[0])
        msg.orientation.y = float(quat[1])
        msg.orientation.z = float(quat[2])
        msg.orientation.w = float(quat[3])

        msg.angular_velocity.x = float(ang[0])
        msg.angular_velocity.y = float(ang[1])
        msg.angular_velocity.z = float(ang[2])

        msg.linear_acceleration.x = float(acc[0])
        msg.linear_acceleration.y = float(acc[1])
        msg.linear_acceleration.z = float(acc[2])

        # 粗略方差，仅用于让 EKF/robot_localization 有非零初值；
        # 需要严格标定时按实际传感器噪声替换。
        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1

        self.pub.publish(msg)


def main():
    rospy.init_node("uav_imu_pub", anonymous=False)
    node = ImuPublisher()
    node.spin()


if __name__ == "__main__":
    main()
