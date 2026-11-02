#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
drone_client.py - OpenHUTB 无人机桥接节点

订阅 /cmd_vel (geometry_msgs/Twist)，通过 OpenHUTB 的 airsim Python API
(airsim.MultirotorClient, RPC 端口默认 41451) 控制仿真无人机飞行，
并发布两类状态话题：

    /drone/odom      (nav_msgs/Odometry)  无人机定位（ENU 坐标, 由 NED 转换）
    /drone/collision (std_msgs/Bool)      碰撞检测标志

坐标系说明：
    OpenHUTB (AirSim) 使用 NED 坐标系: x 前, y 右, z 下, 偏航顺时针为正;
    ROS (REP-103) 使用 ENU 坐标系: x 前, y 左, z 上, 偏航逆时针为正。
    本节点在指令输入与状态输出两端均完成了 NED <-> ENU 的转换。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

import airsim

# /cmd_vel 多久没更新后自动进入悬停 (秒)
CMD_TIMEOUT = 0.5


class DroneClient(Node):

    def __init__(self):
        super().__init__('drone_client')

        # ---- 参数 ----
        self.declare_parameter('host_ip', '127.0.0.1')   # 模拟器地址
        self.declare_parameter('host_port', 41451)       # AirSim 模式 RPC 端口
        self.declare_parameter('control_rate', 10.0)     # 控制频率 (Hz)
        self.declare_parameter('state_rate', 5.0)        # 定位发布频率 (Hz)
        self.declare_parameter('collision_rate', 1.0)    # 碰撞检测频率 (Hz)
        self.declare_parameter('auto_takeoff', True)     # 启动后自动起飞

        host = self.get_parameter('host_ip').value
        port = int(self.get_parameter('host_port').value)

        # ---- 连接 OpenHUTB 模拟器 (AIR 模式) ----
        self.get_logger().info(
            f'正在连接 OpenHUTB 模拟器 {host}:{port} '
            f'(请确认模拟器已用 AIR 模式启动)...')
        self.client = airsim.MultirotorClient(ip=host, port=port)
        self.client.confirmConnection()          # 阻塞直到连接成功
        self.client.enableApiControl(True)       # 交给 API 控制
        self.client.armDisarm(True)              # 解锁电机
        self.get_logger().info('连接成功, API 控制已接管')

        if bool(self.get_parameter('auto_takeoff').value):
            self.get_logger().info('无人机自动起飞中...')
            self.client.takeoffAsync()           # 异步起飞, 不阻塞

        # ---- 话题 ----
        self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, 'drone/odom', 10)
        self.collision_pub = self.create_publisher(Bool, 'drone/collision', 10)

        # ---- 定时器 ----
        c_rate = float(self.get_parameter('control_rate').value)
        s_rate = float(self.get_parameter('state_rate').value)
        k_rate = float(self.get_parameter('collision_rate').value)
        self.create_timer(1.0 / c_rate, self._control_loop)
        self.create_timer(1.0 / s_rate, self._state_loop)
        self.create_timer(1.0 / k_rate, self._collision_loop)

        self._last_cmd_time = self.get_clock().now()
        self._last_cmd = Twist()
        self._hovering = False          # 是否已处于悬停状态
        self._was_collided = False      # 上一次碰撞标志

    # ---------------- 指令接收 ----------------

    def _cmd_cb(self, msg: Twist):
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()
        self._hovering = False

    # ---------------- 控制循环 (10 Hz) ----------------

    def _control_loop(self):
        now = self.get_clock().now()
        cmd_age = (now - self._last_cmd_time).nanoseconds * 1e-9

        # 长时间无指令 → 悬停
        if cmd_age > CMD_TIMEOUT:
            if not self._hovering:
                self.client.hoverAsync()
                self._hovering = True
            return

        # ENU(cmd_vel) -> NED(AirSim): y 取反, z 取反
        vx = self._last_cmd.linear.x
        vy = -self._last_cmd.linear.y
        vz = -self._last_cmd.linear.z
        yaw_rate = -math.radians(self._last_cmd.angular.z)  # ENU 逆时针为正

        self.client.moveByVelocityAsync(
            vx, vy, vz,
            duration=1.0 / float(self.get_parameter('control_rate').value) + 0.1,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate))

    # ---------------- 状态发布 (5 Hz) ----------------

    def _state_loop(self):
        state = self.client.getMultirotorState()
        kin = state.kinematics_estimated

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'drone'

        # NED -> ENU: x 不变, y、z 取反
        p = kin.position
        odom.pose.pose.position.x = p.x_val
        odom.pose.pose.position.y = -p.y_val
        odom.pose.pose.position.z = -p.z_val

        q = kin.orientation          # NED 机体系四元数
        odom.pose.pose.orientation.w = q.w_val
        odom.pose.pose.orientation.x = q.x_val
        odom.pose.pose.orientation.y = -q.y_val
        odom.pose.pose.orientation.z = -q.z_val

        v = kin.linear_velocity      # NED 速度 -> ENU
        odom.twist.twist.linear.x = v.x_val
        odom.twist.twist.linear.y = -v.y_val
        odom.twist.twist.linear.z = -v.z_val

        self.odom_pub.publish(odom)

    # ---------------- 碰撞检测 (1 Hz) ----------------

    def _collision_loop(self):
        info = self.client.simGetCollisionInfo()
        collided = bool(info.has_collided)
        self.collision_pub.publish(Bool(data=collided))

        if collided and not self._was_collided:
            self.get_logger().warn(
                f'检测到碰撞! 碰撞点 (NED): '
                f'x={info.position.x_val:.2f}, y={info.position.y_val:.2f}, '
                f'z={info.position.z_val:.2f}')
        elif not collided and self._was_collided:
            self.get_logger().info('无人机已脱离碰撞状态')
        self._was_collided = collided


def main(args=None):
    rclpy.init(args=args)
    node = DroneClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.client.hoverAsync()
            node.client.armDisarm(False)
            node.client.enableApiControl(False)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
