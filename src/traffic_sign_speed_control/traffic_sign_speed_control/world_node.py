# -*- coding: utf-8 -*-
"""世界节点：差速小车运动学 + 圆形赛道 + 虚拟前视相机 + 画布渲染。

不依赖任何外部仿真器：
- 运动学：对 /cmd_vel 做一阶差速积分得到位姿（50 Hz）；
- 里程计：发布 nav_msgs/Odometry（20 Hz）；
- 虚拟相机：按针孔投影渲染 320x240 前视图像，含几何/光度退化（10 Hz）；
- 渲染：pygame 离屏画布，render_mode=file 时逐帧落盘 PNG 供合成 GIF。
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
from PIL import Image as PILImage

from . import scene as S
from .sign_render import draw_limit_sign, augment

# pygame 必须在非窗口模式下使用 dummy 驱动（在 import 后、init 前设置）
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import pygame  # noqa: E402


FONT_PATHS = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "C:/Windows/Fonts/arialbd.ttf",
]


def _font(size):
    import pygame.font
    for p in FONT_PATHS:
        if os.path.isfile(p):
            return pygame.font.Font(p, size)
    return pygame.font.Font(None, size)


class WorldNode(Node):
    def __init__(self):
        super().__init__("world_node")
        # ---- 参数 ----
        self.declare_parameter("render_mode", "file")   # none | file | window
        self.declare_parameter("frame_dir", "")         # file 模式帧输出目录
        self.declare_parameter("scene", "default")
        self.declare_parameter("start_angle_deg", 180.0)
        self.render_mode = self.get_parameter("render_mode").value
        self.frame_dir = self.get_parameter("frame_dir").value
        scene_name = self.get_parameter("scene").value
        start_deg = float(self.get_parameter("start_angle_deg").value)

        if self.render_mode == "window":
            os.environ.pop("SDL_VIDEODRIVER", None)

        pygame.init()
        self.font = _font(20)
        self.small_font = _font(14)
        self.screen = None
        if self.render_mode == "window":
            self.screen = pygame.display.set_mode((S.CANVAS, S.CANVAS))
            pygame.display.set_caption("traffic_sign_speed_control world")
        self.frame_idx = 0
        self._rng = random.Random(7)

        # ---- 场景与状态 ----
        self.signs = S.scene_from_json(S.SCENES[scene_name])
        self.scene_name = scene_name
        a0 = math.radians(start_deg)
        r0 = S.TRACK_RADIUS / S.PX_PER_M
        self.x = S.TRACK_CENTER[0] / S.PX_PER_M + r0 * math.cos(a0)
        self.y = S.TRACK_CENTER[1] / S.PX_PER_M + r0 * math.sin(a0)
        self.theta = a0 + math.pi / 2.0    # 沿行进方向（角度增大）的切向
        self.v = 0.0

        # ---- ROS 接口 ----
        self.sub_cmd = self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.sub_cfg = self.create_subscription(String, "world/scene_config",
                                                self.on_scene_config, 10)
        self.sub_reset = self.create_subscription(Empty, "world/reset",
                                                  lambda _: self.reset(), 10)
        self.pub_odom = self.create_publisher(Odometry, "world/odom", 10)
        self.pub_img = self.create_publisher(Image, "camera/image_raw", 10)
        self.create_timer(0.02, self.step)          # 50 Hz 运动学积分
        self.create_timer(0.05, self.pub_odom_now)  # 20 Hz 里程计
        self.create_timer(0.10, self.pub_camera)    # 10 Hz 相机
        self._cmd = Twist()
        self.get_logger().info(
            f"world 就绪: scene={scene_name}, signs={[(s.angle_deg, s.value_kmh) for s in self.signs]}, "
            f"render_mode={self.render_mode}, frame_dir={self.frame_dir or '-'}")

    # ---------------- 场景与状态管理 ----------------
    def reset(self):
        a0 = math.radians(float(self.get_parameter("start_angle_deg").value))
        r0 = S.TRACK_RADIUS / S.PX_PER_M
        self.x = S.TRACK_CENTER[0] / S.PX_PER_M + r0 * math.cos(a0)
        self.y = S.TRACK_CENTER[1] / S.PX_PER_M + r0 * math.sin(a0)
        self.theta = a0 + math.pi / 2.0
        self.v = 0.0
        self._cmd = Twist()
        self.get_logger().info("world 已复位")

    def on_scene_config(self, msg):
        """运行时注入场景（run_scenes 多场景评测用），JSON 格式同 SCENES 条目。"""
        try:
            data = json.loads(msg.data)
            self.signs = S.scene_from_json(data)
            self.scene_name = data.get("name", "custom")
            self.reset()
            self.get_logger().info(
                f"场景已切换: {self.scene_name}, "
                f"signs={[(s.angle_deg, s.value_kmh) for s in self.signs]}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"场景配置解析失败: {e}")

    def on_cmd(self, msg):
        self._cmd = msg

    # ---------------- 运动学 ----------------
    def step(self):
        dt = 0.02
        v = float(self._cmd.linear.x)
        w = float(self._cmd.angular.z)
        self.v = v
        self.theta += w * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

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
        m.twist.twist.angular.z = float(self._cmd.angular.z)
        self.pub_odom.publish(m)

    # ---------------- 虚拟相机 ----------------
    def visible_signs(self):
        """返回视野内标志 [(dist, alpha, sign)]，按距离由远到近。"""
        out = []
        for s in self.signs:
            sx, sy = S.sign_xy(s.angle_deg)
            dx, dy = sx - self.x, sy - self.y
            d = math.hypot(dx, dy)
            if d > S.CAM_RANGE_M or d < 1e-3:
                continue
            alpha = math.atan2(dy, dx) - self.theta
            alpha = math.atan2(math.sin(alpha), math.cos(alpha))
            if abs(math.degrees(alpha)) <= S.CAM_FOV_DEG / 2.0:
                out.append((d, alpha, s))
        return sorted(out, key=lambda t: -t[0])

    def render_camera(self):
        """渲染 320x240 前视图像（含退化），返回 uint8 HxWx3。"""
        rng = self._rng
        img = np.zeros((S.CAM_H, S.CAM_W, 3), np.uint8)
        img[:, :, 0] = 176   # 天空
        img[:, :, 1] = 196
        img[:, :, 2] = 214
        img[96:, :, :] = (72, 72, 76)         # 路面
        # 透视车道线（向消失点汇聚）+ 中线虚线
        horizon = 96
        for x_edge in (24, 296):
            for t in range(0, 100):
                y0 = horizon + int(t * 1.44)
                x0 = S.CAM_W / 2 + (x_edge - S.CAM_W / 2) * (t / 100.0)
                if y0 < S.CAM_H:
                    img[y0, int(np.clip(x0, 0, S.CAM_W - 1))] = (230,) * 3
        for t in range(0, 144, 14):
            y0 = horizon + t
            x0 = S.CAM_W / 2 - 1
            if y0 < S.CAM_H:
                img[int(y0):int(y0) + 7, int(x0) - 1:int(x0) + 2] = (235,) * 3
        deg = self.signs[0].degrade if self.signs else {}
        for d, alpha, s in self.visible_signs():
            edge = max(6.0, S.CAM_F * S.SIGN_PHYS_M / d)
            x_img = S.CAM_W / 2 + S.CAM_F * math.tan(alpha)
            y_img = horizon + S.CAM_F * (S.SIGN_CENTER_H_M - S.CAM_HEIGHT_M) / d
            x_img = float(np.clip(x_img, 0, S.CAM_W - 1))
            y_img = float(np.clip(y_img, horizon, S.CAM_H - 1))
            sprite = draw_limit_sign(s.value_kmh, 128, alpha=True)
            sprite = augment(sprite, rng, rotate=deg.get("rotate", 0),
                             blur_sigma=deg.get("blur", 0),
                             brightness=deg.get("brightness", 0),
                             contrast=deg.get("contrast", 0),
                             noise_std=deg.get("noise", 0))
            sprite = sprite.resize((int(edge), int(edge)), PILImage.BILINEAR)
            sw = sprite.width
            x0, y0 = int(x_img - sw / 2), int(y_img - sw / 2)
            sx0, sy0 = max(0, -x0), max(0, -y0)
            x0, y0 = max(0, x0), max(0, y0)
            w = min(sw - sx0, S.CAM_W - x0)
            h = min(sw - sy0, S.CAM_H - y0)
            if w > 0 and h > 0:
                spr = np.asarray(sprite)[sy0:sy0 + h, sx0:sx0 + w].astype(np.float32)
                region = img[y0:y0 + h, x0:x0 + w].astype(np.float32)
                a = spr[..., 3:4] / 255.0
                img[y0:y0 + h, x0:x0 + w] = (
                    spr[..., :3] * a + region * (1.0 - a)).astype(np.uint8)
        return img

    def pub_camera(self):
        frame = self.render_camera()
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
        self.draw_canvas()

    # ---------------- 画布渲染 ----------------
    def draw_canvas(self):
        if self.render_mode == "none":
            return
        canvas = pygame.Surface((S.CANVAS, S.CANVAS))
        canvas.fill((168, 192, 148))
        cx, cy = S.TRACK_CENTER
        r = S.TRACK_RADIUS
        pygame.draw.circle(canvas, (70, 70, 74), (cx, cy), r + 45)   # 路肩外
        pygame.draw.circle(canvas, (110, 112, 118), (cx, cy), r + 45, 2)
        pygame.draw.circle(canvas, (70, 70, 74), (cx, cy), r - 45)
        pygame.draw.circle(canvas, (110, 112, 118), (cx, cy), r - 45, 2)
        # 中央虚线
        for a in range(0, 360, 18):
            rect = pygame.Rect(0, 0, 4, 4)
            rect.center = (cx + int(r * math.cos(math.radians(a))),
                           cy + int(r * math.sin(math.radians(a))))
            pygame.draw.rect(canvas, (225, 225, 225), rect)
        # 标志牌（RGBA 圈外透明，叠加到草地底色）
        for s in self.signs:
            sx, sy = S.sign_xy(s.angle_deg)
            px, py = int(sx * S.PX_PER_M), int(sy * S.PX_PER_M)
            pygame.draw.line(canvas, (90, 90, 90), (px, py + 18), (px, py - 8), 3)
            spr_rgba = draw_limit_sign(s.value_kmh, 44, alpha=True)
            bg = PILImage.new("RGB", (44, 44), (168, 192, 148))
            bg.paste(spr_rgba, mask=spr_rgba)
            sprite = pygame.image.frombuffer(bg.tobytes(), (44, 44), "RGB")
            canvas.blit(sprite, (px - 22, py - 52))
        # 小车（红色三角形）
        px, py = self.x * S.PX_PER_M, self.y * S.PX_PER_M
        L, W = 22.0, 12.0
        pts = [(px + L * math.cos(self.theta), py + L * math.sin(self.theta)),
               (px - L / 2 * math.cos(self.theta) + W / 2 * math.sin(self.theta),
                py - L / 2 * math.sin(self.theta) - W / 2 * math.cos(self.theta)),
               (px - L / 2 * math.cos(self.theta) - W / 2 * math.sin(self.theta),
                py - L / 2 * math.sin(self.theta) + W / 2 * math.cos(self.theta))]
        pygame.draw.polygon(canvas, (210, 40, 40), [(int(a), int(b)) for a, b in pts])
        # HUD
        pygame.draw.rect(canvas, (255, 255, 255), (10, 10, 470, 64))
        pygame.draw.rect(canvas, (60, 60, 60), (10, 10, 470, 64), 1)
        canvas.blit(self.font.render(
            f"v = {self.v:.2f} m/s    scene = {self.scene_name}", True, (20, 20, 20)),
            (22, 20))
        canvas.blit(self.small_font.render(
            "CNN perception -> speed governor",
            True, (90, 90, 90)), (22, 50))
        # 相机画中画
        if getattr(self, "_last_cam", None) is not None:
            cam = self._last_cam
            surf = pygame.image.frombuffer(cam.tobytes(), (cam.shape[1], cam.shape[0]), "RGB")
            surf = pygame.transform.smoothscale(surf, (240, 180))
            canvas.blit(surf, (S.CANVAS - 250, S.CANVAS - 190))
            pygame.draw.rect(canvas, (60, 60, 60),
                             (S.CANVAS - 250, S.CANVAS - 190, 240, 180), 1)
        # 输出
        if self.screen is not None:
            self.screen.blit(canvas, (0, 0))
            pygame.display.flip()
            for ev in pygame.event.get():
                pass
        if self.render_mode == "file" and self.frame_dir:
            os.makedirs(self.frame_dir, exist_ok=True)
            small = pygame.transform.smoothscale(canvas, (400, 400))
            pygame.image.save(small, os.path.join(
                self.frame_dir, f"frame_{self.frame_idx:05d}.png"))
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
