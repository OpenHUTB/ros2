# -*- coding: utf-8 -*-
"""sim_node.py —— 仿真节点：机载相机出图 + 四旋翼动力学 + 真值评价。

话题（默认在 /uav 下，可在 launch 里 remap）：
  发布  image      sensor_msgs/Image    机载相机 RGB 图像(160x120)
        odom       nav_msgs/Odometry     位姿/速度(用于控制器做速度记忆)
        target     geometry_msgs/PointStamped  目标真值像素(仅评价用，策略不读)
        error      geometry_msgs/PointStamped  (ex, ey, 可见标志) 真值误差
  订阅  cmd_vel    geometry_msgs/Twist   前向/右向/垂直速度 + 偏航角速度

跑满 duration 后节点把本回合指标写到 CSV（参数 csv 指定），并继续 spin
以便上层 launch 统一结束。
"""
import csv
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image

from . import params as P
from .camera import Camera
from .quadrotor import Quadrotor
from .scenes import build_scenes


class SimNode(Node):
    def __init__(self):
        super().__init__('uav_sim')
        self.declare_parameter('scene', 'moving')
        self.declare_parameter('duration', 0.0)      # 0 -> 用场景自带时长
        self.declare_parameter('publish_image', True)
        self.declare_parameter('csv', '')            # 空 -> 不写文件

        key = self.get_parameter('scene').value
        scenes = {s.key: s for s in build_scenes()}
        if key not in scenes:
            raise ValueError('未知场景 %r，可选：%s'
                             % (key, ', '.join(sorted(scenes))))
        self.scene = scenes[key]
        dur = float(self.get_parameter('duration').value)
        self.duration = dur if dur > 0.0 else self.scene.duration
        self.steps_total = max(1, int(round(self.duration / P.DT)))
        self.publish_image = bool(self.get_parameter('publish_image').value)
        self.csv_path = str(self.get_parameter('csv').value)

        self.camera = Camera()
        self.quad = Quadrotor(self.scene.uav0, self.scene.yaw0)
        self.rng = np.random.default_rng(2024)
        self.k = 0
        self.done = False
        self.cmd = np.zeros(4)          # vx, vy, vz, yaw_rate
        self.log = {'t': [], 'err': [], 'visible': [], 'cmd': []}

        self.pub_img = self.create_publisher(Image, '/uav/image', 1)
        self.pub_odom = self.create_publisher(Odometry, '/uav/odom', 10)
        self.pub_gt = self.create_publisher(PointStamped, '/uav/target', 10)
        self.pub_err = self.create_publisher(PointStamped, '/uav/error', 10)
        self.create_subscription(Twist, '/uav/cmd_vel', self._on_cmd, 10)
        self.timer = self.create_timer(P.DT, self._tick)
        self.get_logger().info(
            '场景 %s（%s），时长 %.1fs，%d 步'
            % (self.scene.key, self.scene.name, self.duration, self.steps_total))

    # ---------- 回调 ----------
    def _on_cmd(self, msg):
        """速度指令：linear.x/y 机体前向/右向，linear.z 世界系垂直，angular.z 偏航角速度。"""
        self.cmd = np.array([msg.linear.x, msg.linear.y,
                             msg.linear.z, msg.angular.z])

    # ---------- 主循环 ----------
    def _tick(self):
        if self.done:
            return
        t = self.k * P.DT
        tp = self.scene.target_fn(t)
        cam_pos = self.quad.camera_pose()
        img = self.camera.render(cam_pos, self.quad.yaw, tp,
                                 self.scene.distractors, self.scene.walls,
                                 self.rng)
        gt = self.camera.observe(cam_pos, self.quad.yaw, tp, self.scene.walls)

        self._publish(img, gt, t)

        # 指令 -> 机体速度（偏航角速度 -> 期望偏航角，模型内部自带转速限幅）
        vx, vy, vz, yaw_rate = self.cmd
        yaw_rate = float(np.clip(yaw_rate, -P.YAW_RATE_MAX, P.YAW_RATE_MAX))
        self.quad.set_cmd((vx, vy), vz,
                          self.quad.yaw + yaw_rate * P.DT)
        self.quad.step(self.scene.wind(t))

        err = float(np.hypot(gt['u'] - (P.IMG_W - 1) / 2.0,
                             gt['v'] - (P.IMG_H - 1) / 2.0)) / (P.IMG_W / 2.0)
        self.log['t'].append(t)
        self.log['err'].append(err)
        self.log['visible'].append(1.0 if gt['visible'] else 0.0)
        self.log['cmd'].append(self.cmd.copy())

        self.k += 1
        if self.k >= self.steps_total:
            self._finish()

    def _publish(self, img, gt, t):
        stamp = self.get_clock().now().to_msg()
        if self.publish_image:
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = 'uav_camera'
            msg.height, msg.width = img.shape[0], img.shape[1]
            msg.encoding = 'rgb8'
            msg.is_bigendian = 0
            msg.step = msg.width * 3
            msg.data = (np.clip(img, 0.0, 1.0) * 255.0).astype(np.uint8).tobytes()
            self.pub_img.publish(msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.quad.p[0])
        odom.pose.pose.position.y = float(self.quad.p[1])
        odom.pose.pose.position.z = float(self.quad.p[2])
        half = self.quad.yaw / 2.0
        odom.pose.pose.orientation.z = float(np.sin(half))
        odom.pose.pose.orientation.w = float(np.cos(half))
        odom.twist.twist.linear.x = float(self.quad.v[0])
        odom.twist.twist.linear.y = float(self.quad.v[1])
        odom.twist.twist.linear.z = float(self.quad.v[2])
        self.pub_odom.publish(odom)

        gt_msg = PointStamped()
        gt_msg.header.stamp = stamp
        gt_msg.header.frame_id = 'uav_camera'
        gt_msg.point.x = float(gt['u'])
        gt_msg.point.y = float(gt['v'])
        gt_msg.point.z = 1.0 if gt['visible'] else 0.0
        self.pub_gt.publish(gt_msg)

        ex = (gt['u'] - (P.IMG_W - 1) / 2.0) / (P.IMG_W / 2.0)
        ey = (gt['v'] - (P.IMG_H - 1) / 2.0) / (P.IMG_H / 2.0)
        err_msg = PointStamped()
        err_msg.header = gt_msg.header
        err_msg.point.x = float(ex)
        err_msg.point.y = float(ey)
        err_msg.point.z = 1.0 if gt['visible'] else 0.0
        self.pub_err.publish(err_msg)

    # ---------- 收尾 ----------
    def _finish(self):
        self.done = True
        self.timer.cancel()
        err = np.array(self.log['err'])
        vis = np.array(self.log['visible']) > 0.5
        err_vis = err[vis] if vis.any() else np.array([1.0])
        rms = float(np.sqrt(np.mean(err_vis ** 2)))
        vis_frac = float(vis.mean())
        speed = float(np.mean(np.linalg.norm(
            np.array(self.log['cmd'])[:, 0:2], axis=1)))
        ok = (vis_frac >= P.LOST_FRAC_OK) and (rms <= P.ERR_OK)
        self.get_logger().info(
            '回合结束：可见占比 %.3f，误差 RMS %.4f，平均速度 %.2f m/s，判定 %s'
            % (vis_frac, rms, speed, '通过' if ok else '未通过'))
        if self.csv_path:
            d = os.path.dirname(os.path.abspath(self.csv_path))
            if d and not os.path.isdir(d):
                os.makedirs(d, exist_ok=True)
            header = ['scene', 'visible_frac', 'err_rms', 'mean_speed', 'success']
            new = not os.path.exists(self.csv_path)
            with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                if new:
                    w.writerow(header)
                w.writerow([self.scene.key, '%.4f' % vis_frac, '%.4f' % rms,
                            '%.3f' % speed, int(ok)])
            self.get_logger().info('指标已写入 %s' % self.csv_path)


def main(argv=None):
    rclpy.init(args=argv)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
