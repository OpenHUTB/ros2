#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AirSim Blocks 四旋翼无人机键盘遥控节点（ROS2 Humble）。

前提：Windows 宿主机已启动 AirSim Blocks 环境（API 端口 8080）。

用法：
    ros2 run uav_airsim_keyboard keyboard_node

按键（焦点保持在终端窗口，按住控制键即可持续动作）：
    T       起飞
    W / S   前 / 后
    A / D   左 / 右
    R / F   上升 / 下降
    Q / E   左转 / 右转
    L       降落
    Ctrl+C  退出（自动降落）

说明：使用 AirSim 的 moveByVelocityBodyFrameAsync（机体系速度控制）实现遥控，
速度指令必须 .get() 等待执行；无按键时发送零速度指令保持悬停。
"""

import sys
import time
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import airsim

AIR_SIM_IP = '10.0.2.2'
AIR_SIM_PORT = 8080
SPEED = 2.0           # 速度指令大小 m/s（机体系）
CMD_INTERVAL = 0.1    # 控制指令间隔（秒）；也是速度指令持续时间
YAW_RATE = 45.0       # 转向角速度 deg/s
CTRL_KEYS = 'wasdrfqe'  # 控制键集合
HOLD_TIMEOUT = 0.25   # 超过该时长无新输入视为松开（秒）


class NonBlockingKeyboardReader:
    """终端原始模式读取（无需回车，teleop_twist_keyboard 同款方案）。"""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)

    def enable(self):
        tty.setcbreak(self.fd)

    def disable(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read(self):
        if select.select([sys.stdin], [], [], 0.01)[0]:
            return sys.stdin.read(1).lower()
        return None


class AirSimKeyboardNode(Node):
    def __init__(self):
        super().__init__('uav_airsim_keyboard')
        self.pub_pose = self.create_publisher(PoseStamped, '/uav/pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)  # 10Hz 位姿

        self.key_press_time = {}    # 键 -> 最后收到该键的时间
        self.takeoff_pressed = False
        self.land_pressed = False
        self.landing = False
        self.armed = False
        self.last_cmd = 0.0

        self.client = airsim.MultirotorClient(ip=AIR_SIM_IP, port=AIR_SIM_PORT)
        self.client.confirmConnection()
        self.client.enableApiControl(True)   # 必须先获得 API 控制权
        self.get_logger().info(f'AirSim 连接成功: {AIR_SIM_IP}:{AIR_SIM_PORT} (Blocks)，已获取 API 控制权')

    # ---------- 按键处理 ----------
    def is_held(self, key):
        """判断按键是否仍被按住（基于最近输入时间）。"""
        return self.key_press_time.get(key, 0.0) > time.time() - HOLD_TIMEOUT

    def handle_key(self, ch):
        if ch in CTRL_KEYS:
            self.key_press_time[ch] = time.time()
        elif ch == 't':
            self.takeoff_pressed = True
        elif ch == 'l':
            self.land_pressed = True

    # ---------- 控制循环 ----------
    def control_loop(self):
        # 起飞（即时模式：arm 后立即进入控制）
        if self.takeoff_pressed and not self.armed:
            self.takeoff_pressed = False
            try:
                self.client.enableApiControl(True)
                self.client.armDisarm(True)
                self.armed = True
                self.get_logger().info('已起飞（按 R 上升 / W 前进 / A D 左右 / Q E 转向）')
            except Exception as e:
                self.get_logger().error(f'起飞指令失败: {e}')
                return
        if not self.armed:
            return
        # 降落（异步）
        if self.land_pressed:
            self.land_pressed = False
            try:
                self.client.landAsync(timeout_sec=20)
                self.landing = True
                self.get_logger().info('正在降落...')
            except Exception as e:
                self.get_logger().error(f'降落指令失败: {e}')
        # 降落中：等待落地，不发位置指令
        if self.landing:
            try:
                state = self.client.getMultirotorState()
                if state.kinematics_estimated.position.z_val > -0.5:
                    self.client.armDisarm(False)
                    self.landing = False
                    self.armed = False
                    self.get_logger().info('已降落')
            except Exception:
                pass
            return
        # 速度控制（按 CMD_INTERVAL 节流；异步指令必须 .get() 才会真正执行）
        now = time.time()
        if now - self.last_cmd < CMD_INTERVAL:
            return
        self.last_cmd = now
        try:
            fx = fy = fz = 0.0
            if self.is_held('w'):
                fx += SPEED
            if self.is_held('s'):
                fx -= SPEED
            if self.is_held('d'):
                fy += SPEED
            if self.is_held('a'):
                fy -= SPEED
            if self.is_held('r'):
                fz -= SPEED  # 机体系 NED：减 z 即上升
            if self.is_held('f'):
                fz += SPEED
            if self.is_held('q'):
                self.client.rotateByYawRateAsync(YAW_RATE, CMD_INTERVAL).get()
            elif self.is_held('e'):
                self.client.rotateByYawRateAsync(-YAW_RATE, CMD_INTERVAL).get()
            # 机体系速度指令（W=机头方向），必须 get() 等待执行
            self.client.moveByVelocityBodyFrameAsync(fx, fy, fz, CMD_INTERVAL).get()
        except Exception as e:
            self.get_logger().warn(f'控制指令发送失败: {e}')

    # ---------- 位姿发布 ----------
    def publish_pose(self):
        try:
            state = self.client.getMultirotorState()
            pos = state.kinematics_estimated.position
            ori = state.kinematics_estimated.orientation
        except Exception as e:
            self.get_logger().warn(f'读取位姿失败: {e}')
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = pos.x_val
        msg.pose.position.y = pos.y_val
        msg.pose.position.z = pos.z_val
        msg.pose.orientation.w = ori.w_val
        msg.pose.orientation.x = ori.x_val
        msg.pose.orientation.y = ori.y_val
        msg.pose.orientation.z = ori.z_val
        self.pub_pose.publish(msg)

    # ---------- 主循环 ----------
    def run(self):
        reader = NonBlockingKeyboardReader()
        reader.enable()
        self.get_logger().info(
            '就绪：按 T 起飞；W/A/S/D 前后左右、R/F 升降、Q/E 转向；L 降落；Ctrl+C 退出')
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                ch = reader.read()
                if ch is not None:
                    self.handle_key(ch)
                self.control_loop()
        except KeyboardInterrupt:
            self.get_logger().info('收到退出信号')
        finally:
            reader.disable()
            if self.armed:
                try:
                    self.client.landAsync(timeout_sec=20).join()
                    self.client.armDisarm(False)
                    self.get_logger().info('已自动降落')
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = AirSimKeyboardNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
