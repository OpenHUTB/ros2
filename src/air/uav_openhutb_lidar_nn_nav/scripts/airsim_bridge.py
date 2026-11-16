#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenHUTB 桥接节点：模块内**唯一**直接调用模拟器 Python 客户端的节点。

设计要点
    · 上下游全部通过 ROS 话题/服务解耦，替换模拟器时只需改这一个文件
    · 坐标统一转成 ROS 惯例（ENU：x 前、y 左、z 上），NED 转换只在本节点内部进行
    · 未连接模拟器时节点直接报错退出，不会静默产生假数据

发布
    /uav/odom            nav_msgs/Odometry      位姿与速度（ENU）
    /uav/lidar/scan      sensor_msgs/LaserScan  前向 180° 二维扫描
    /uav/camera/image    sensor_msgs/Image      机载 RGB 图像（PNG 解码后）
订阅
    /uav/cmd_vel         geometry_msgs/Twist    速度指令（ENU 惯例，m/s、rad/s）
服务
    /uav/takeoff         std_srvs/Trigger
    /uav/land            std_srvs/Trigger

参数
    ~airsim_ip      模拟器 IP（默认取环境变量 AIRSIM_IP，否则 127.0.0.1）
    ~airsim_port    RPC 端口，默认 41451
    ~vehicle_name   载具名，默认空字符串
    ~rate           主循环频率，默认 20 Hz
    ~use_lidar      是否订阅雷达，默认 true
    ~use_camera     是否拉取相机图像，默认 true
    ~lidar_bins     二维扫描分箱数，默认 180
    ~lidar_height   只取该高度带内的点（米，相对机体系），默认 0.5
"""

import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_srvs.srv import Trigger, TriggerResponse
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class OpenHutbBridge(object):
    def __init__(self):
        self.ip = rospy.get_param("~airsim_ip", os.environ.get("AIRSIM_IP", "127.0.0.1"))
        self.port = int(rospy.get_param("~airsim_port", 41451))
        self.vehicle = rospy.get_param("~vehicle_name", "")
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.use_lidar = bool(rospy.get_param("~use_lidar", True))
        self.use_camera = bool(rospy.get_param("~use_camera", True))
        self.bins = int(rospy.get_param("~lidar_bins", 180))
        self.lidar_height = float(rospy.get_param("~lidar_height", 0.5))
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        try:
            import airsim
        except ImportError:
            rospy.logfatal(
                "未找到 airsim 客户端。OpenHUTB 自带 PythonClient，请将其路径加入 PYTHONPATH，"
                "或参考 README 的依赖安装一节。")
            sys.exit(1)

        self.airsim = airsim
        try:
            self.client = airsim.MultirotorClient(ip=self.ip, port=self.port,
                                                  vehicle_name=self.vehicle)
            self.client.confirmConnection()
        except Exception as exc:  # noqa: BLE001
            rospy.logfatal("连接 OpenHUTB 失败（%s:%d）：%s", self.ip, self.port, exc)
            rospy.logfatal("请确认模拟器已启动并进入含无人机的 AIR 场景，且 RPC 端口 41451 可达。")
            sys.exit(1)

        self.client.enableApiControl(True, vehicle_name=self.vehicle)
        self.client.armDisarm(True, vehicle_name=self.vehicle)

        auto = bool(rospy.get_param("~auto_takeoff", False))
        if auto:
            try:
                self.client.takeoffAsync(timeout_sec=20, vehicle_name=self.vehicle).join()
                self.flying = True
                rospy.loginfo("已自动起飞")
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn("自动起飞失败（可手动调 /uav/takeoff）: %s", exc)
        rospy.loginfo("已连接 OpenHUTB：%s:%d", self.ip, self.port)

        self.cmd = Twist()

        self.pub_odom = rospy.Publisher("/uav/odom", Odometry, queue_size=1)
        self.pub_scan = rospy.Publisher("/uav/lidar/scan", LaserScan, queue_size=1) \
            if self.use_lidar else None
        self.pub_img = rospy.Publisher("/uav/camera/image", Image, queue_size=1) \
            if self.use_camera else None
        self.tf_br = TransformBroadcaster()

        rospy.Subscriber("/uav/cmd_vel", Twist, self.on_cmd, queue_size=1)
        rospy.Service("/uav/takeoff", Trigger, self.on_takeoff)
        rospy.Service("/uav/land", Trigger, self.on_land)

    # ---------------- 回调 ----------------
    def on_cmd(self, msg):
        self.cmd = msg

    def on_takeoff(self, _req):
        try:
            self.client.takeoffAsync(timeout_sec=20, vehicle_name=self.vehicle).join()
            self.flying = True
            return TriggerResponse(success=True, message="已起飞")
        except Exception as exc:  # noqa: BLE001
            return TriggerResponse(success=False, message="起飞失败: %s" % exc)

    def on_land(self, _req):
        try:
            self.client.landAsync(timeout_sec=30, vehicle_name=self.vehicle).join()
            self.flying = False
            return TriggerResponse(success=True, message="已降落")
        except Exception as exc:  # noqa: BLE001
            return TriggerResponse(success=False, message="降落失败: %s" % exc)

    # ---------------- 主循环 ----------------
    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.publish_odom()
            if self.flying:
                self.send_velocity()
            if self.pub_scan is not None:
                self.publish_scan()
            if self.pub_img is not None:
                self.publish_image()
            rate.sleep()

    def send_velocity(self):
        # ROS ENU -> AirSim NED：x 前、y 左、z 上  =>  NED(vx, vy, vz) = (x, y, -z)
        try:
            self.client.moveByVelocityZAsync(
                self.cmd.linear.x, self.cmd.linear.y, -self.cmd.linear.z,
                rospy.Duration(0.2).to_sec(),
                drivetrain=self.airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=self.airsim.YawMode(is_rate=True,
                                            yaw_or_rate=math.degrees(self.cmd.angular.z)),
                vehicle_name=self.vehicle)
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(5.0, "下发速度失败: %s", exc)

    def publish_odom(self):
        try:
            st = self.client.getMultirotorState(vehicle_name=self.vehicle)
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(5.0, "读取位姿失败: %s", exc)
            return

        p = st.kinematics_estimated.position
        q = st.kinematics_estimated.orientation
        v = st.kinematics_estimated.linear_velocity

        # NED -> ENU
        x, y, z = p.y_val, p.x_val, -p.z_val
        yaw = math.atan2(2.0 * (q.w_val * q.z_val + q.x_val * q.y_val),
                         1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val))

        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        qx, qy, qz, qw = yaw_to_quat(yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v.y_val
        odom.twist.twist.linear.y = v.x_val
        odom.twist.twist.linear.z = -v.z_val
        self.pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)

    def publish_scan(self):
        try:
            data = self.client.getLidarData(lidar_name="LidarSensor1", vehicle_name=self.vehicle)
        except Exception:  # noqa: BLE001
            try:
                data = self.client.getLidarData(vehicle_name=self.vehicle)
            except Exception:  # noqa: BLE001
                return

        pts = np.asarray(data.point_cloud, dtype=np.float32)
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = self.base_frame
        scan.angle_min = -math.pi / 2.0
        scan.angle_max = math.pi / 2.0
        scan.angle_increment = (scan.angle_max - scan.angle_min) / max(self.bins - 1, 1)
        scan.range_min = 0.3
        scan.range_max = 40.0

        ranges = np.full(self.bins, scan.range_max, dtype=np.float32)
        if pts.size >= 3:
            pts = pts.reshape(-1, 3)
            # 机体系：取高度带内的点（NED 下 z 向下为正）
            band = np.abs(pts[:, 2] + self.lidar_height) < 0.6
            p = pts[band]
            if p.shape[0] > 0:
                ang = np.arctan2(p[:, 1], p[:, 0])          # NED: y 右, x 前
                rng = np.hypot(p[:, 0], p[:, 1])
                idx = ((ang - scan.angle_min) / scan.angle_increment).astype(int)
                ok = (idx >= 0) & (idx < self.bins)
                for i, r in zip(idx[ok], rng[ok]):
                    if r < ranges[i]:
                        ranges[i] = r

        scan.ranges = [float(r) for r in ranges]
        self.pub_scan.publish(scan)

    def publish_image(self):
        try:
            resp = self.client.simGetImage("0", self.airsim.ImageType.Scene,
                                           vehicle_name=self.vehicle)
        except Exception:  # noqa: BLE001
            return
        if not resp:
            return
        try:
            import cv2
            img = cv2.imdecode(np.frombuffer(resp, dtype=np.uint8), cv2.IMREAD_COLOR)
        except Exception:  # noqa: BLE001
            return
        if img is None:
            return
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.base_frame
        msg.height, msg.width = img.shape[0], img.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        self.pub_img.publish(msg)


def main():
    rospy.init_node("uav_openhutb_bridge")
    try:
        node = OpenHutbBridge()
    except rospy.ROSInitException:
        return
    node.run()


if __name__ == "__main__":
    main()
