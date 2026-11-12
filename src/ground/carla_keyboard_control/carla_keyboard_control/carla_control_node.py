#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 仿真节点：连接仿真器、生成自车、把控制指令施加到车辆并广播传感器数据。

ROS 2 接口（Humble）：
  订阅  /carla/ego_vehicle/vehicle_control_cmd   (std_msgs/Float32MultiArray)
          数据格式 [throttle, steer, brake, reverse]
  发布  /carla/ego_vehicle/rgb_front/image       (sensor_msgs/Image)
  发布  /carla/ego_vehicle/odometry              (nav_msgs/Odometry)
  发布  /carla/ego_vehicle/speed                 (std_msgs/Float32)

设计要点：仿真步进（固定 0.05 s）由本节点驱动，键盘节点只发布控制意图，
从而保证「感知—控制—物理」严格同步，不会因键盘读取阻塞而丢帧。
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Header
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from carla_keyboard_control import carla_common as cc


class CarlaControlNode(Node):
    """承载 CARLA 世界、自车与传感器的仿真节点。"""

    def __init__(self):
        super().__init__('carla_control_node')

        # ---------------- 参数声明 ----------------
        self.declare_parameter('host', cc.DEFAULT_HOST)
        self.declare_parameter('port', cc.DEFAULT_PORT)
        self.declare_parameter('town', cc.DEFAULT_TOWN)
        self.declare_parameter('ego_blueprint', cc.DEFAULT_EGO_BLUEPRINT)
        self.declare_parameter('fixed_delta_seconds', cc.DT)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('camera_fov', 90.0)
        self.declare_parameter('throttle_max', 0.6)
        self.declare_parameter('brake_max', 0.8)
        self.declare_parameter('steer_max', 0.6)
        self.declare_parameter('reverse_speed_threshold', 0.5)

        p = self.get_parameter
        self.dt = float(p('fixed_delta_seconds').value)
        self.width = int(p('image_width').value)
        self.height = int(p('image_height').value)

        # ---------------- 连接 CARLA ----------------
        self.get_logger().info(
            f"连接 CARLA {p('host').value}:{p('port').value}，地图 {p('town').value}")
        self.client, self.world = cc.connect(
            p('host').value, int(p('port').value), p('town').value, dt=self.dt)
        self.vehicle, tf = cc.spawn_vehicle(
            self.world, p('ego_blueprint').value)
        self.get_logger().info(f"自车已生成 @ {tf.location}")

        # ---------------- 相机（保持常驻引用，避免被回收）----------------
        self._latest_frame = None
        self._sensors = [cc.make_rgb_camera(
            self.world, self.vehicle, self._on_image,
            width=self.width, height=self.height,
            fov=float(p('camera_fov').value), tick=True)]

        # ---------------- ROS 接口 ----------------
        self.create_subscription(
            Float32MultiArray,
            '/carla/ego_vehicle/vehicle_control_cmd',
            self._on_control_cmd, 10)
        self.pub_image = self.create_publisher(
            Image, '/carla/ego_vehicle/rgb_front/image', 10)
        self.pub_odom = self.create_publisher(
            Odometry, '/carla/ego_vehicle/odometry', 10)
        self.pub_speed = self.create_publisher(
            Float32, '/carla/ego_vehicle/speed', 10)

        # 独立于 ROS 时钟的物理步进定时器（同步模式必须由本节点 tick）
        self.create_timer(self.dt, self._on_tick)
        self.get_logger().info(
            "仿真节点就绪：等待 /carla/ego_vehicle/vehicle_control_cmd 控制指令")

    # ------------------------------------------------------------------ 回调
    def _on_image(self, rgb):
        """相机回调：缓存最近一帧 RGB。"""
        self._latest_frame = rgb

    def _on_control_cmd(self, msg):
        """控制指令回调：[throttle, steer, brake, reverse] → 车辆控制。"""
        data = list(msg.data) + [0.0, 0.0, 0.0, 0.0]
        cc.apply_control(self.vehicle, throttle=data[0], steer=data[1],
                         brake=data[2], reverse=bool(data[3]))

    def _on_tick(self):
        """推进一帧物理仿真，并广播图像 / 里程计 / 速度。"""
        self.world.tick()
        stamp = self.get_clock().now().to_msg()

        if self._latest_frame is not None:
            img = Image()
            img.header = Header(stamp=stamp, frame_id='ego_vehicle')
            img.height, img.width = self._latest_frame.shape[:2]
            img.encoding = 'rgb8'
            img.is_bigendian = 0
            img.step = img.width * 3
            img.data = np.ascontiguousarray(self._latest_frame).tobytes()
            self.pub_image.publish(img)

        tf = self.vehicle.get_transform()
        speed = cc.get_speed(self.vehicle)
        vel = self.vehicle.get_velocity()

        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id='map')
        odom.child_frame_id = 'ego_vehicle'
        odom.pose.pose.position.x = tf.location.x
        odom.pose.pose.position.y = tf.location.y
        odom.pose.pose.position.z = tf.location.z
        yaw = math.radians(tf.rotation.yaw)
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = vel.x
        odom.twist.twist.linear.y = vel.y
        odom.twist.twist.linear.z = vel.z
        self.pub_odom.publish(odom)
        self.pub_speed.publish(Float32(data=float(speed)))

    # ------------------------------------------------------------------ 清理
    def destroy_node(self):
        try:
            for sensor in self._sensors:
                sensor.stop()
                sensor.destroy()
            self.vehicle.destroy()
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            self.world.apply_settings(settings)
            self.get_logger().info("已释放自车与传感器资源")
        except Exception as exc:  # noqa: BLE001 - 退出阶段尽力清理
            self.get_logger().warn(f"清理资源时出现异常：{exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarlaControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
