#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AirSim 无人机桥接节点（uav_lidar_dqn_navigation）

把 AirSim MultirotorClient 封装成 ROS 话题，供键盘遥控、DQN 导航和建图节点共用：

  订阅  /drone/cmd_vel     geometry_msgs/Twist     机体系速度（m/s、rad/s）
  发布  /drone/odom        nav_msgs/Odometry       ENU 位姿与速度
  发布  /drone/state       std_msgs/Float32MultiArray
                             [x, y, z, yaw, vx, vy, vz]（NED，米/弧度）
  服务  /drone/takeoff     std_srvs/Trigger
  服务  /drone/land        std_srvs/Trigger

两个坐标系约定：
  AirSim 用 NED（北-东-地），ROS 用 ENU（东-北-天）。
  位置换算 x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned；
  偏航换算 psi_enu = pi/2 - psi_ned。
"""

import math
import os
import sys
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger, TriggerResponse

_client_dir = os.environ.get('AIRSIM_PYTHON_CLIENT')
if _client_dir and _client_dir not in sys.path:
    sys.path.insert(0, _client_dir)

import airsim  # noqa: E402


def yaw_from_quat_ned(q):
    """AirSim 四元数 -> NED 偏航角（弧度，顺时针为正）。"""
    x, y, z, w = q.x_val, q.y_val, q.z_val, q.w_val
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class DroneBridge(object):
    def __init__(self):
        self.airsim_ip = rospy.get_param('~airsim_ip', '172.25.176.1')
        self.vehicle = rospy.get_param('~vehicle_name', 'Drone_1')
        self.rate_hz = float(rospy.get_param('~control_rate', 10.0))
        self.publish_tf = bool(rospy.get_param('~publish_tf', False))
        self.auto_takeoff = bool(rospy.get_param('~auto_takeoff', False))
        self.takeoff_height = float(rospy.get_param('~takeoff_height', 4.0))
        self.altitude_hold = bool(rospy.get_param('~altitude_hold', True))

        # msgpackrpc 会话不是线程安全的：状态读取和指令下发都要串行
        self.api_lock = threading.Lock()
        self.cmd_lock = threading.Lock()
        self.cmd = Twist()
        self.has_cmd = False
        self.yaw_ned = 0.0
        self.hold_z = None

        rospy.loginfo('连接 AirSim 仿真器 %s（机体 %s）', self.airsim_ip, self.vehicle)
        self.client = airsim.MultirotorClient(ip=self.airsim_ip, timeout_value=10)
        self.client.confirmConnection()
        rospy.loginfo('AirSim 已连接')

        self.odom_pub = rospy.Publisher('/drone/odom', Odometry, queue_size=10)
        self.state_pub = rospy.Publisher('/drone/state', Float32MultiArray, queue_size=10)
        rospy.Subscriber('/drone/cmd_vel', Twist, self._on_cmd, queue_size=1)
        self.tf_br = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        rospy.Service('/drone/takeoff', Trigger, self._srv_takeoff)
        rospy.Service('/drone/land', Trigger, self._srv_land)

        if self.auto_takeoff:
            self._takeoff_to(self.takeoff_height)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)
        rospy.on_shutdown(self._on_shutdown)

    # ---------------- 起飞 / 降落 ----------------
    def _takeoff_to(self, height):
        with self.api_lock:
            self.client.enableApiControl(True, self.vehicle)
            self.client.armDisarm(True, self.vehicle)
            rospy.loginfo('起飞并爬升到 %.1f m', height)
            self.client.takeoffAsync(timeout_sec=15, vehicle_name=self.vehicle).join()
            self.client.moveToZAsync(-abs(height), 2.0, vehicle_name=self.vehicle).join()
        rospy.loginfo('已到达巡航高度')

    def _srv_takeoff(self, _req):
        try:
            self._takeoff_to(self.takeoff_height)
            return TriggerResponse(success=True, message='takeoff ok')
        except Exception as exc:  # pragma: no cover - 仿真侧异常
            return TriggerResponse(success=False, message=str(exc))

    def _srv_land(self, _req):
        try:
            with self.api_lock:
                self.client.landAsync(timeout_sec=25, vehicle_name=self.vehicle).join()
                self.client.armDisarm(False, self.vehicle)
            return TriggerResponse(success=True, message='land ok')
        except Exception as exc:  # pragma: no cover
            return TriggerResponse(success=False, message=str(exc))

    # ---------------- 指令 ----------------
    def _on_cmd(self, msg):
        with self.cmd_lock:
            self.cmd = msg
            self.has_cmd = True

    def _tick(self, _event):
        if not self.has_cmd:
            return
        with self.cmd_lock:
            cmd = self.cmd

        # 机体系速度（前/右/上）按当前偏航旋到 NED 世界系
        psi = self.yaw_ned
        vx_body, vy_body, vz_up = cmd.linear.x, cmd.linear.y, cmd.linear.z
        vx = vx_body * math.cos(psi) - vy_body * math.sin(psi)
        vy = vx_body * math.sin(psi) + vy_body * math.cos(psi)
        # ROS 偏航正方向为左（逆时针），AirSim 为顺时针，取负后再下发
        yaw_rate = -math.degrees(cmd.angular.z)

        try:
            with self.api_lock:
                if self.altitude_hold and self.hold_z is not None:
                    self.hold_z -= vz_up / self.rate_hz
                    self.client.moveByVelocityZAsync(
                        vx, vy, self.hold_z, 1.0 / self.rate_hz,
                        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                        yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate),
                        vehicle_name=self.vehicle)
                else:
                    self.client.moveByVelocityAsync(
                        vx, vy, -vz_up, 1.0 / self.rate_hz,
                        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                        yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate),
                        vehicle_name=self.vehicle)
        except Exception as exc:  # pragma: no cover
            rospy.logwarn_throttle(5.0, '速度指令下发失败: %s', exc)

    # ---------------- 状态 ----------------
    def _read_state(self):
        with self.api_lock:
            return self.client.getMultirotorState(vehicle_name=self.vehicle)

    def _publish(self):
        state = self._read_state()
        pos = state.kinematics_estimated.position
        vel = state.kinematics_estimated.linear_velocity
        ori = state.kinematics_estimated.orientation

        if self.hold_z is None or abs(pos.z_val - self.hold_z) > 3.0:
            self.hold_z = pos.z_val

        self.yaw_ned = yaw_from_quat_ned(ori)
        yaw_enu = math.pi / 2.0 - self.yaw_ned
        stamp = rospy.Time.now()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = pos.y_val
        odom.pose.pose.position.y = pos.x_val
        odom.pose.pose.position.z = -pos.z_val
        odom.pose.pose.orientation.z = math.sin(yaw_enu / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw_enu / 2.0)
        odom.twist.twist.linear.x = vel.y_val
        odom.twist.twist.linear.y = vel.x_val
        odom.twist.twist.linear.z = -vel.z_val
        self.odom_pub.publish(odom)

        raw = Float32MultiArray()
        raw.data = [pos.x_val, pos.y_val, pos.z_val, self.yaw_ned,
                    vel.x_val, vel.y_val, vel.z_val]
        self.state_pub.publish(raw)

        if self.tf_br is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = 'odom'
            tf_msg.child_frame_id = 'base_link'
            tf_msg.transform.translation.x = odom.pose.pose.position.x
            tf_msg.transform.translation.y = odom.pose.pose.position.y
            tf_msg.transform.translation.z = odom.pose.pose.position.z
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_br.sendTransform(tf_msg)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                self._publish()
            except Exception as exc:  # pragma: no cover
                rospy.logwarn_throttle(5.0, '状态读取失败: %s', exc)
            rate.sleep()

    def _on_shutdown(self):
        # 退出前必须降落再释放 API 控制，否则无人机会自由落体
        try:
            with self.cmd_lock:
                self.cmd = Twist()
            rospy.loginfo('正在降落...')
            with self.api_lock:
                self.client.landAsync(timeout_sec=25, vehicle_name=self.vehicle).join()
                self.client.armDisarm(False, self.vehicle)
                self.client.enableApiControl(False, self.vehicle)
            rospy.loginfo('已降落')
        except Exception:  # pragma: no cover
            pass


def main():
    rospy.init_node('drone_bridge')
    DroneBridge().spin()


if __name__ == '__main__':
    main()
