# -*- coding: utf-8 -*-
"""世界节点：MuJoCo 差速小车仿真 + 圆形赛道 + 限速牌 + 车载相机。

- 物理：MuJoCo 刚体动力学；/cmd_vel (v, w) 换算为左右驱动轮目标轮速，
  由速度 actuator 跟踪，50 Hz 推进（每周期 10 个 2 ms 物理子步）；
- 里程计：由自由关节位姿差分得到速度，发布 nav_msgs/Odometry（20 Hz）；
- 相机：MuJoCo 渲染车载前视 320x240 图像（10 Hz），场景光度/噪声退化以
  图像增广叠加，rotate 退化为牌面绕法线侧倾；
- 帧输出：render_mode=file 时逐帧合成「俯视图 + HUD + 相机画中画」落盘，
  供 main.py demo 合成 GIF。
"""
import json
import math
import os
import random

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from PIL import Image as PILImage, ImageDraw

from . import scene as S
from .mujoco_scene import CX, CY, GROUND_HALF_M, MjWorld, TOP_RES
from .sign_render import _load_font, augment, draw_limit_sign


class WorldNode(Node):
    def __init__(self):
        super().__init__("world_node")
        # ---- 参数 ----
        self.declare_parameter("render_mode", "file")   # none | file
        self.declare_parameter("frame_dir", "")         # file 模式帧输出目录
        self.declare_parameter("scene", "default")
        self.declare_parameter("start_angle_deg", 180.0)
        self.render_mode = self.get_parameter("render_mode").value
        if self.render_mode == "window":                # 无头环境不再支持窗口
            self.get_logger().warn("render_mode=window 已弃用，按 file 处理")
            self.render_mode = "file"
        self.frame_dir = self.get_parameter("frame_dir").value
        self.frame_idx = 0
        self._rng = random.Random(7)
        self._last_cam = None

        self._build(self.get_parameter("scene").value,
                    float(self.get_parameter("start_angle_deg").value))

        # ---- ROS 接口 ----
        self.sub_cmd = self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.sub_cfg = self.create_subscription(String, "world/scene_config",
                                                self.on_scene_config, 10)
        self.sub_reset = self.create_subscription(Empty, "world/reset",
                                                  lambda _: self.reset(), 10)
        self.pub_odom = self.create_publisher(Odometry, "world/odom", 10)
        self.pub_img = self.create_publisher(Image, "camera/image_raw", 10)
        self.create_timer(0.02, self.step)          # 50 Hz 物理推进
        self.create_timer(0.05, self.pub_odom_now)  # 20 Hz 里程计
        self.create_timer(0.10, self.pub_camera)    # 10 Hz 相机
        self._cmd = Twist()

    # ---------------- 场景与状态管理 ----------------
    def _build(self, scene_name, start_deg):
        """按场景构建 MuJoCo 世界并复位状态。"""
        self.scene_name = scene_name
        self.signs = S.scene_from_json(S.SCENES[scene_name])
        old = getattr(self, "mj", None)
        if old is not None:
            old.close()
        self.mj = MjWorld(self.signs, start_deg)
        self.reset()
        self.get_logger().info(
            f"world(MuJoCo) 就绪: scene={scene_name}, "
            f"signs={[(s.angle_deg, s.value_kmh) for s in self.signs]}, "
            f"render_mode={self.render_mode}, frame_dir={self.frame_dir or '-'}")

    def reset(self):
        self.mj.reset()
        self.x, self.y, self.theta = self.mj.pose()
        self._prev = (self.x, self.y, self.theta)
        self.v = 0.0
        self.yaw_rate = 0.0
        self._cmd = Twist()
        self.get_logger().info("world 已复位")

    def on_scene_config(self, msg):
        """运行时注入场景（run_scenes 多场景评测用），JSON 格式同 SCENES 条目。"""
        try:
            data = json.loads(msg.data)
            signs = S.scene_from_json(data)
            self.scene_name = data.get("name", "custom")
            self.signs = signs
            old = getattr(self, "mj", None)
            if old is not None:
                old.close()
            self.mj = MjWorld(self.signs,
                              float(self.get_parameter("start_angle_deg").value))
            self.reset()
            self.get_logger().info(
                f"场景已切换: {self.scene_name}, "
                f"signs={[(s.angle_deg, s.value_kmh) for s in self.signs]}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"场景配置解析失败: {e}")

    def on_cmd(self, msg):
        self._cmd = msg

    # ---------------- 物理推进与里程计 ----------------
    def step(self):
        self.mj.set_cmd(float(self._cmd.linear.x), float(self._cmd.angular.z))
        self.mj.step()
        x, y, theta = self.mj.pose()
        px, py, pth = self._prev
        self.v = math.hypot(x - px, y - py) / self.mj.sim_dt
        dth = math.atan2(math.sin(theta - pth), math.cos(theta - pth))
        self.yaw_rate = dth / self.mj.sim_dt
        self.x, self.y, self.theta = x, y, theta
        self._prev = (x, y, theta)

    def pub_odom_now(self):
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "world"
        m.child_frame_id = "base_link"
        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        m.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        m.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        m.twist.twist.linear.x = self.v
        m.twist.twist.angular.z = self.yaw_rate
        self.pub_odom.publish(m)

    # ---------------- 车载相机 ----------------
    def pub_camera(self):
        frame = self.mj.camera_frame()
        deg = self.signs[0].degrade if self.signs else {}
        if deg:
            img = augment(PILImage.fromarray(frame), self._rng, rotate=0,
                          blur_sigma=deg.get("blur", 0),
                          brightness=deg.get("brightness", 0),
                          contrast=deg.get("contrast", 0),
                          noise_std=deg.get("noise", 0))
            frame = np.asarray(img)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical"
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.pub_img.publish(msg)
        self._last_cam = frame
        self.save_frame()

    # ---------------- 帧输出（俯视图 + HUD + 相机画中画） ----------------
    def save_frame(self):
        if self.render_mode != "file" or not self.frame_dir:
            return
        img = PILImage.fromarray(self.mj.topdown_frame())
        d = ImageDraw.Draw(img)
        # 俯视图上叠加限速牌图标（牌面竖立，俯视渲染近乎不可见）
        span = 2.0 * GROUND_HALF_M
        for s in self.signs:
            sx, sy = S.sign_xy(s.angle_deg)
            px = int((sx - (CX - GROUND_HALF_M)) / span * TOP_RES)
            py = int(TOP_RES - (sy - (CY - GROUND_HALF_M)) / span * TOP_RES)
            icon = draw_limit_sign(s.value_kmh, 36, alpha=True)
            img.paste(icon, (px - 18, py - 18), icon)
        d.rectangle([10, 10, 400, 64], fill=(255, 255, 255), outline=(60, 60, 60))
        d.text((22, 18), f"v = {self.v:.2f} m/s    scene = {self.scene_name}",
               font=_load_font(20), fill=(20, 20, 20))
        d.text((22, 44), "MuJoCo sim | CNN perception -> speed governor",
               font=_load_font(14), fill=(90, 90, 90))
        if self._last_cam is not None:
            cam = PILImage.fromarray(self._last_cam).resize(
                (240, 180), PILImage.BILINEAR)
            img.paste(cam, (TOP_RES - 250, TOP_RES - 190))
            d.rectangle([TOP_RES - 250, TOP_RES - 190,
                         TOP_RES - 10, TOP_RES - 10], outline=(60, 60, 60))
        os.makedirs(self.frame_dir, exist_ok=True)
        img.save(os.path.join(self.frame_dir, f"frame_{self.frame_idx:05d}.png"))
        self.frame_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = WorldNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
