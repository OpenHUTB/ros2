#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AirSim 无人机桥接节点

把 AirSim 的多旋翼接口封装成 ROS 话题：
  /uav/cmd_vel   (geometry_msgs/Twist)  速度指令，线速度单位 m/s，角速度 rad/s
  /uav/odom      (nav_msgs/Odometry)    位姿与速度，坐标系已从 NED 转换到 ENU
  /uav/takeoff   (std_srvs/Trigger)     起飞
  /uav/land      (std_srvs/Trigger)     降落
"""

import math
import os
import sys
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerResponse

# 允许通过环境变量指定 AirSim PythonClient 路径，避免把第三方源码放进仓库
_client_path = os.environ.get('AIRSIM_PYTHON_CLIENT')
if _client_path and _client_path not in sys.path:
    sys.path.insert(0, _client_path)

import airsim  # noqa: E402


def quat_to_yaw_ned(q):
    """从 AirSim 的四元数（NED）解出偏航角，单位弧度，0 指向正北，顺时针为正。"""
    x, y, z, w = q.x_val, q.y_val, q.z_val, q.w_val
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class AirSimBridgeNode(object):
    def __init__(self):
        self.ip = rospy.get_param('~airsim_ip', '127.0.0.1')
        self.vehicle_name = rospy.get_param('~vehicle_name', 'Drone_1')
        self.rate_hz = float(rospy.get_param('~control_rate', 10.0))
        self.publish_tf = bool(rospy.get_param('~publish_tf', False))
        self.auto_takeoff = bool(rospy.get_param('~auto_takeoff', False))
        self.takeoff_height = float(rospy.get_param('~takeoff_height', 3.0))

        self.lock = threading.Lock()
        # msgpackrpc 的会话不是线程安全的：主线程读状态、定时器线程发指令，
        # 必须串行化，否则会报 "Existing exports of data: object cannot be re-sized"
        self.client_lock = threading.Lock()
        self.cmd = Twist()
        self.armed = False
        self.current_yaw = 0.0     # NED 偏航角（弧度），用于机体系->世界系转换
        self.target_z = None       # 高度保持的目标（NED，向下为正）
        self.altitude_hold = bool(rospy.get_param('~altitude_hold', True))
        # 收到第一条速度指令前不发任何指令，否则会和外部起飞抢控制权
        self.received_cmd = False

        rospy.loginfo('连接 AirSim: %s (vehicle=%s)', self.ip, self.vehicle_name)
        self.client = airsim.MultirotorClient(ip=self.ip, timeout_value=10)
        self.client.confirmConnection()
        rospy.loginfo('AirSim 连接成功')

        self.odom_pub = rospy.Publisher('/uav/odom', Odometry, queue_size=10)
        self.cmd_sub = rospy.Subscriber('/uav/cmd_vel', Twist, self.cmd_cb, queue_size=1)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        self.srv_takeoff = rospy.Service('/uav/takeoff', Trigger, self.takeoff_srv)
        self.srv_land = rospy.Service('/uav/land', Trigger, self.land_srv)

        self._takeoff_if_needed()
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.control_loop)

        rospy.on_shutdown(self.shutdown)

    # ---------- 起飞 / 降落 ----------
    def _takeoff_if_needed(self):
        if not self.auto_takeoff:
            return
        with self.client_lock:
            self.client.enableApiControl(True, self.vehicle_name)
            self.client.armDisarm(True, self.vehicle_name)
            self.armed = True
            rospy.loginfo('起飞到 %.1f 米', self.takeoff_height)
            self.client.takeoffAsync(timeout_sec=15,
                                     vehicle_name=self.vehicle_name).join()
            self.client.moveToZAsync(-abs(self.takeoff_height), 2.0,
                                     vehicle_name=self.vehicle_name).join()
        rospy.loginfo('已到达目标高度')

    def takeoff_srv(self, _req):
        try:
            with self.client_lock:
                self.client.enableApiControl(True, self.vehicle_name)
                self.client.armDisarm(True, self.vehicle_name)
                self.armed = True
                self.client.takeoffAsync(timeout_sec=15,
                                         vehicle_name=self.vehicle_name).join()
            return TriggerResponse(success=True, message='已起飞')
        except Exception as exc:  # pragma: no cover - 仿真侧异常
            return TriggerResponse(success=False, message=str(exc))

    def land_srv(self, _req):
        try:
            with self.client_lock:
                self.client.landAsync(timeout_sec=20,
                                      vehicle_name=self.vehicle_name).join()
                self.client.armDisarm(False, self.vehicle_name)
                self.armed = False
            return TriggerResponse(success=True, message='已降落')
        except Exception as exc:  # pragma: no cover
            return TriggerResponse(success=False, message=str(exc))

    # ---------- 控制 ----------
    def cmd_cb(self, msg):
        with self.lock:
            self.cmd = msg
            self.received_cmd = True

    def control_loop(self, _event):
        if not self.received_cmd:
            return
        with self.lock:
            cmd = self.cmd

        vx_body = cmd.linear.x          # 前（机体系）
        vy_body = cmd.linear.y          # 右（机体系）
        vz_up = cmd.linear.z            # 向上为正
        yaw_rate = math.degrees(cmd.angular.z)

        # AirSim 的 moveByVelocity 接收的是世界系速度，
        # 按当前偏航角把机体速度旋到 NED 世界系
        psi = self.current_yaw
        vx = vx_body * math.cos(psi) - vy_body * math.sin(psi)
        vy = vx_body * math.sin(psi) + vy_body * math.cos(psi)

        try:
            with self.client_lock:
                if self.altitude_hold and self.target_z is not None:
                    # 用目标高度而非垂直速度控制，避免没有升降指令时往下掉
                    self.target_z -= vz_up / self.rate_hz
                    self.client.moveByVelocityZAsync(
                        vx, vy, self.target_z, 1.0 / self.rate_hz,
                        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                        yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate),
                        vehicle_name=self.vehicle_name,
                    )
                else:
                    self.client.moveByVelocityAsync(
                        vx, vy, -vz_up, 1.0 / self.rate_hz,
                        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                        yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate),
                        vehicle_name=self.vehicle_name,
                    )
        except Exception as exc:  # pragma: no cover
            rospy.logwarn_throttle(5.0, '发送速度指令失败: %s', exc)

    # ---------- 状态 ----------
    def publish_odom(self):
        with self.client_lock:
            state = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
        pos = state.kinematics_estimated.position
        vel = state.kinematics_estimated.linear_velocity
        ori = state.kinematics_estimated.orientation

        # 高度保持的目标高度：首次读取或外部把飞机挪太远时自动重新对齐
        if self.target_z is None or abs(pos.z_val - self.target_z) > 3.0:
            self.target_z = pos.z_val

        yaw_ned = quat_to_yaw_ned(ori)
        self.current_yaw = yaw_ned
        yaw_enu = math.pi / 2.0 - yaw_ned

        stamp = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        # NED -> ENU：x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
        odom.pose.pose.position.x = pos.y_val
        odom.pose.pose.position.y = pos.x_val
        odom.pose.pose.position.z = -pos.z_val
        odom.pose.pose.orientation.z = math.sin(yaw_enu / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw_enu / 2.0)
        odom.twist.twist.linear.x = vel.y_val
        odom.twist.twist.linear.y = vel.x_val
        odom.twist.twist.linear.z = -vel.z_val
        odom.twist.twist.angular.z = 0.0
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = 'odom'
            tf_msg.child_frame_id = 'base_link'
            tf_msg.transform.translation.x = odom.pose.pose.position.x
            tf_msg.transform.translation.y = odom.pose.pose.position.y
            tf_msg.transform.translation.z = odom.pose.pose.position.z
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                self.publish_odom()
            except Exception as exc:  # pragma: no cover
                rospy.logwarn_throttle(5.0, '读取状态失败: %s', exc)
            rate.sleep()

    def shutdown(self):
        # 先降落再释放控制权：直接释放的话无人机会自由落体摔到地面
        try:
            with self.lock:
                self.cmd = Twist()
            rospy.loginfo('正在降落...')
            with self.client_lock:
                self.client.landAsync(timeout_sec=25,
                                      vehicle_name=self.vehicle_name).join()
                self.client.armDisarm(False, self.vehicle_name)
                self.client.enableApiControl(False, self.vehicle_name)
            rospy.loginfo('已降落并释放控制权')
        except Exception:  # pragma: no cover
            pass


def main():
    rospy.init_node('airsim_bridge')
    node = AirSimBridgeNode()
    node.run()


if __name__ == '__main__':
    main()
