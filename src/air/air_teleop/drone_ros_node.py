#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 桥接控制节点：drone_ros_node
# 作用：订阅 /drone/cmd_vel（geometry_msgs/Twist），把 ROS 消息翻译成 AirSim 的
#       速度指令，经 RPC（默认 41451 端口）驱动 Windows 宿主机上的多旋翼无人机，
#       实现键盘输入（drone_ros_teleop）与仿真器控制之间的消息解耦。
#
# 坐标系：AirSim 使用 NED 坐标系（x 前、y 右、z 指向地面），因此 /drone/cmd_vel
#         中的消息按 NED 机体坐标系解释：
#         linear.x/y/z -> 机体 x/y/z 轴线速度（vz < 0 上升、vz > 0 下降）
#         angular.z     -> 偏航角速度（弧度/秒，正值右偏航）
#
# 安全机制：
#   1. 看门狗：连续超过 0.5 s 未收到任何指令时自动悬停；
#   2. 收到第一条速度指令时自动解锁并起飞；
#   3. 退出（Ctrl+C / rosnode kill）时通过 rospy.on_shutdown 先悬停、再释放
#      AirSim API 控制权。

import argparse
import math
import sys
import threading
import time

import airsim
import rospy
from geometry_msgs.msg import Twist


class DroneRosBridge:
    """订阅 /drone/cmd_vel 并驱动 AirSim 无人机的桥接节点"""

    def __init__(self, host='127.0.0.1', port=41451, vehicle_name='',
                 cmd_timeout=0.5, ctrl_rate=10.0):
        self.vehicle_name = vehicle_name
        self.cmd_timeout = cmd_timeout                # 无指令超时时间（秒）
        ctrl_rate = max(0.1, ctrl_rate)               # 防止 0 频率导致除零
        self.ctrl_period = 1.0 / ctrl_rate            # 控制周期（秒）
        self.cmd_duration = self.ctrl_period * 2.0    # 指令时长略长于周期，保证指令不断档

        # 最新指令缓存（由回调写入、控制循环读取）
        self.lock = threading.Lock()
        self.last_cmd_time = None
        self.vx = self.vy = self.vz = 0.0
        self.yaw_rate = 0.0

        # 飞行状态
        self.flying = False                           # 是否已起飞
        self.hovering = False                         # 是否处于超时悬停状态
        self.shutdown_done = False                    # 安全收尾是否已完成

        # 连接 AirSim RPC 服务
        self.client = self._connect(host, port)

        # 订阅速度指令，并注册控制循环与安全退出回调
        self.cmd_sub = rospy.Subscriber('/drone/cmd_vel', Twist,
                                        self.cmd_vel_callback, queue_size=1)
        self.ctrl_timer = rospy.Timer(rospy.Duration(self.ctrl_period), self.control_loop)
        rospy.on_shutdown(self.shutdown)

    # ---------------- 连接 ----------------

    @staticmethod
    def _connect(host, port, attempts=5):
        """连接 AirSim RPC 服务，失败时重试若干次后报致命错误退出"""
        for i in range(attempts):
            try:
                client = airsim.MultirotorClient(ip=host, port=port)
                client.confirmConnection()
                rospy.loginfo('已连接 AirSim 服务（%s:%d）', host, port)
                return client
            except Exception as exc:                  # 模拟器未启动或网络不通时重试
                rospy.logwarn('连接 AirSim 失败（第 %d/%d 次）：%s', i + 1, attempts, exc)
                if i + 1 < attempts:
                    time.sleep(2.0)
        rospy.logfatal('无法连接 AirSim（%s:%d），请确认宿主机模拟器已启动', host, port)
        raise RuntimeError('AirSim connection failed')

    # ---------------- ROS 回调 ----------------

    def cmd_vel_callback(self, msg):
        """缓存最新速度指令并记录收包时刻，供控制循环按 10 Hz 下发"""
        with self.lock:
            self.vx = msg.linear.x
            self.vy = msg.linear.y
            self.vz = msg.linear.z
            self.yaw_rate = msg.angular.z
            self.last_cmd_time = time.time()

    def control_loop(self, _event):
        """控制循环：有指令则下发速度，超时则悬停"""
        if self.shutdown_done or rospy.is_shutdown():
            return
        now = time.time()
        with self.lock:
            fresh = (self.last_cmd_time is not None
                     and now - self.last_cmd_time <= self.cmd_timeout)
            if fresh:
                vx, vy, vz, yaw_rate = self.vx, self.vy, self.vz, self.yaw_rate
        if fresh:
            if not self.flying:
                self.takeoff()
            if self.hovering:
                rospy.loginfo('重新收到速度指令，恢复遥控')
                self.hovering = False
            self.send_velocity(vx, vy, vz, yaw_rate)
        elif self.flying and not self.hovering:
            self.stop_and_hover()

    # ---------------- 飞行控制 ----------------

    def takeoff(self):
        """解锁电机并爬升到安全高度，随后进入遥控状态"""
        rospy.loginfo('收到第一条速度指令，解锁并起飞')
        try:
            self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
            self.client.armDisarm(True, vehicle_name=self.vehicle_name)
            self.client.takeoffAsync(timeout_sec=10, vehicle_name=self.vehicle_name).join()
            # 起飞动作结束后先悬停一下，等姿态稳定再响应速度指令
            self.client.hoverAsync(vehicle_name=self.vehicle_name).join()
            self.flying = True
            rospy.loginfo('起飞完成，开始遥控')
        except Exception as exc:
            rospy.logerr('起飞失败：%s', exc)

    def send_velocity(self, vx, vy, vz, yaw_rate):
        """在 NED 机体坐标系下下发速度指令（异步，不阻塞控制循环）"""
        self.client.moveByVelocityBodyFrameAsync(
            vx, vy, vz, self.cmd_duration,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=math.degrees(yaw_rate)),
            vehicle_name=self.vehicle_name)

    def stop_and_hover(self):
        """看门狗触发：把三轴速度打成 0，原地保持高度"""
        rospy.loginfo('超过 %.1f s 未收到指令，自动悬停', self.cmd_timeout)
        try:
            self.client.hoverAsync(vehicle_name=self.vehicle_name).join()
            self.hovering = True
        except Exception as exc:
            rospy.logerr('悬停失败：%s', exc)

    def shutdown(self):
        """安全退出：先悬停保底，再释放 AirSim API 控制权（不切断空中动力）"""
        if self.shutdown_done:
            return
        self.shutdown_done = True
        rospy.loginfo('正在安全退出：悬停并释放控制权')
        try:
            if self.flying:
                self.client.hoverAsync(vehicle_name=self.vehicle_name).join()
        except Exception as exc:
            rospy.logwarn('退出前悬停失败：%s', exc)
        try:
            # 释放 API 控制权，把无人机交还给仿真器手动控制；不 disarm，避免空中坠落
            self.client.enableApiControl(False, vehicle_name=self.vehicle_name)
            rospy.loginfo('已释放 AirSim API 控制权')
        except Exception as exc:
            rospy.logwarn('释放控制权失败：%s', exc)


def main():
    parser = argparse.ArgumentParser(
        description='AirSim 无人机 ROS 桥接节点：/drone/cmd_vel -> AirSim RPC')
    parser.add_argument('--host', default=None, help='宿主机（AirSim）IP，默认从 ~host 私有参数读取')
    parser.add_argument('--port', type=int, default=None, help='AirSim RPC 端口，默认从 ~port 私有参数读取')
    parser.add_argument('--vehicle', default=None, help='载具名称，空串表示第一架无人机')
    args, _ = parser.parse_known_args()               # 兼容 ROS 参数重映射传入的 _xxx:=yyy

    rospy.init_node('drone_ros_node')
    # 命令行参数优先，其次读取 ROS 私有参数，最后使用默认值
    host = args.host if args.host is not None else rospy.get_param('~host', '127.0.0.1')
    port = args.port if args.port is not None else rospy.get_param('~port', 41451)
    vehicle = args.vehicle if args.vehicle is not None else rospy.get_param('~vehicle', '')
    timeout = rospy.get_param('~timeout', 0.5)        # 无指令超时自动悬停
    rate = rospy.get_param('~rate', 10.0)             # 控制频率

    try:
        bridge = DroneRosBridge(host=host, port=port, vehicle_name=vehicle,
                                cmd_timeout=timeout, ctrl_rate=rate)
    except RuntimeError:
        sys.exit(1)                                   # 连接 AirSim 失败，日志已说明原因
    rospy.loginfo('桥接节点已启动：订阅 /drone/cmd_vel，控制频率 %.1f Hz，超时 %.1f s',
                  rate, timeout)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()                             # rospy.on_shutdown 之外的兜底


if __name__ == '__main__':
    main()
