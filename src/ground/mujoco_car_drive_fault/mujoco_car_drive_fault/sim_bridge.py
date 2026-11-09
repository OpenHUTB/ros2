#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 仿真桥：把一台"左/右轮被拖滞"的 MuJoCo 差速小车接入 ROS 2。

发布：
    /imu            sensor_msgs/Imu                机体系姿态、角速度、加速度
    /odom           nav_msgs/Odometry              位姿、航向角速度、前向速度
    /wheel_speed    std_msgs/Float32MultiArray     左右驱动轮实际角速度 (rad/s)
    /chassis_state  std_msgs/Float32MultiArray     [横向偏差, 航向角(度), 车速, 左轮速, 右轮速]
订阅：
    /cmd_vel        geometry_msgs/Twist            linear.x = 目标车速, angular.z = 目标偏航角速度

节点跑完后打印本工况的真实结果（最大横向偏差、末端偏差、是否跑偏），供 launch 汇总。
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

from mujoco_car_drive_fault.car_env import CarEnv
from mujoco_car_drive_fault.config import Config


class SimBridge(Node):
    def __init__(self):
        super().__init__("mujoco_car_drive_fault_sim_bridge")
        self.declare_parameter("scene", "car_scene_fault_mid.xml")
        self.declare_parameter("fault_side", "left")
        self.declare_parameter("fault_damping", 0.0)      # 0 表示取场景自带的拖滞量
        self.declare_parameter("run_seconds", 6.0)
        self.declare_parameter("control_hz", 50.0)

        self.cfg = Config()
        scene = self.get_parameter("scene").value
        side = self.get_parameter("fault_side").value
        damping = float(self.get_parameter("fault_damping").value)
        self.run_seconds = float(self.get_parameter("run_seconds").value)
        rate = float(self.get_parameter("control_hz").value)

        self.env = CarEnv(self.cfg, scene)
        if damping <= 0.0:
            damping = dict(self.cfg.scenes)[scene]
        self.env.reset(fault_side=side, fault_damping=damping)
        self.scene = scene
        self.side = side

        self.cmd = (0.0, 0.0)          # (steer, throttle)
        self.t = 0.0
        self.max_abs_y = 0.0
        self.deviate = False

        self.pub_imu = self.create_publisher(Imu, "/imu", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_wheel = self.create_publisher(Float32MultiArray, "/wheel_speed", 10)
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
        y = self.env.lateral_offset()
        yaw = self.env.yaw()
        v = self.env.forward_speed()
        wl, wr = self.env.wheel_speeds()
        self.max_abs_y = max(self.max_abs_y, abs(y))
        if abs(y) >= self.cfg.deviate_m:
            self.deviate = True

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
        pos = self.env.data.xpos[self.env.chassis_id]
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = (
            float(pos[0]), float(pos[1]), float(pos[2]))
        od.pose.pose.orientation.w = float(quat[0])
        od.pose.pose.orientation.x = float(quat[1])
        od.pose.pose.orientation.y = float(quat[2])
        od.pose.pose.orientation.z = float(quat[3])
        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = float(imu["gyro"][2])
        self.pub_odom.publish(od)

        ws = Float32MultiArray()
        ws.data = [float(wl), float(wr)]
        self.pub_wheel.publish(ws)

        st = Float32MultiArray()
        st.data = [float(y), float(math.degrees(yaw)), float(v), float(wl), float(wr)]
        self.pub_state.publish(st)

        if self.t >= self.run_seconds or abs(y) >= 1.35:
            self.get_logger().info(
                "工况结束：场景={} 失效侧={} 时长={:.2f}s 跑偏={} 最大偏差={:.2f}m 末端偏差={:+.2f}m".format(
                    self.scene, self.side, self.t, self.deviate, self.max_abs_y, y))
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
