#!/usr/bin/env python3
"""PX4 + ROS2 + OpenCV 无人机视觉目标追踪节点（第二版）"""

import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

# ==================== 可调参数 ====================
Kp_XY = 0.12
Kp_Z = 0.08
AREA_TARGET = 15000
Kp_DIST = 0.02
TAKEOFF_HEIGHT = -2.0
VEL_LIMIT = 1.5
CONTROL_FREQ = 10.0
LOWER1 = np.array([0, 120, 70])
UPPER1 = np.array([10, 255, 255])
LOWER2 = np.array([170, 120, 70])
UPPER2 = np.array([180, 255, 255])
MIN_AREA = 300
# =================================================


class VisualTrackingNode(Node):
    def __init__(self):
        super().__init__('px4_opencv_tracking')
        self.bridge = CvBridge()
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 10)
        self.target_found = False
        self.err_x = 0.0
        self.err_y = 0.0
        self.area = 0.0
        self.armed = False
        self.t0 = time.time()
        self.create_timer(1.0 / CONTROL_FREQ, self.control_cb)
        self.get_logger().info('VisualTrackingNode 已启动')

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER1, UPPER1) | cv2.inRange(hsv, LOWER2, UPPER2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.target_found = False
            return
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < MIN_AREA:
            self.target_found = False
            return
        M = cv2.moments(c)
        if M['m00'] == 0:
            self.target_found = False
            return
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        self.err_x = cx - w / 2.0
        self.err_y = cy - h / 2.0
        self.area = float(cv2.contourArea(c))
        self.target_found = True

    def control_cb(self):
        ts = int(time.time() * 1e6)
        mode = OffboardControlMode()
        mode.timestamp = ts
        mode.velocity = True
        self.offboard_pub.publish(mode)

        vx = vy = vz = 0.0
        if self.target_found:
            vy = -Kp_XY * self.err_x
            vz = -Kp_Z * self.err_y
            vx = Kp_DIST * (AREA_TARGET - self.area)
            vx = max(-VEL_LIMIT, min(VEL_LIMIT, vx))
            vy = max(-VEL_LIMIT, min(VEL_LIMIT, vy))

        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position = [float('nan'), float('nan'), TAKEOFF_HEIGHT]
        sp.velocity = [vx, vy, vz]
        sp.yaw = 0.0
        self.traj_pub.publish(sp)

        if time.time() - self.t0 > 2.0 and not self.armed:
            self._send_cmd(176, 0.0, 6.0)
            time.sleep(0.2)
            self._send_cmd(400, 1.0, 0.0)
            self.armed = True
            self.get_logger().info('Offboard + ARM 完成')

    def _send_cmd(self, command, p1, p2):
        cmd = VehicleCommand()
        cmd.timestamp = int(time.time() * 1e6)
        cmd.command = command
        cmd.param1 = p1
        cmd.param2 = p2
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VisualTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
