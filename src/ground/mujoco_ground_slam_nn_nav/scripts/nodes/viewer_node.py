#!/usr/bin/python3
"""RViz-free OpenCV viewer: map, path, lidar, BEV camera and HUD.

This node uses the system Python 3.8 (rospy + OpenCV) and therefore never
imports MuJoCo.  It can also publish keyboard teleop commands.
"""

from __future__ import annotations

import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Empty, String


W, H = 1000, 600
MAP_W, MAP_H = 720, 480
MAP_OFF = (10, 70)
SCENES = ["warehouse", "obstacle_field", "crossing"]


class ViewerNode:
    def __init__(self) -> None:
        self.scene = rospy.get_param("~scene", "warehouse")
        self.teleop = bool(rospy.get_param("~teleop", True))
        self.map_data = None
        self.map_w = self.map_h = 0
        self.pose = np.array([2.0, 2.0, 0.0])
        self.truth = None
        self.path = []
        self.scan = None
        self.bev = None
        self.state = {}
        self.slam_status = {}
        self.controller = {}
        self.metrics = {}
        self.v = 0.0
        self.w = 0.0
        self.last_key = rospy.Time(0)
        self.last_cmd = rospy.Time(0)
        self.no_display = False
        self.last_frame = rospy.Time(0)

        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.pub_reset = rospy.Publisher("/gvnn/reset", Empty, queue_size=2)
        self.pub_scene = rospy.Publisher("/gvnn/scene", String, queue_size=2)
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=2)
        rospy.Subscriber("/slam_pose", PoseStamped, self.pose_cb, queue_size=5)
        rospy.Subscriber("/ground_truth/pose", PoseStamped, self.truth_cb, queue_size=5)
        rospy.Subscriber("/plan", Path, self.path_cb, queue_size=2)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=2)
        rospy.Subscriber("/camera/bev/image_raw", Image, self.bev_cb, queue_size=2)
        rospy.Subscriber("/gvnn/state", String, self.state_cb, queue_size=5)
        rospy.Subscriber("/gvnn/slam_status", String, self.slam_cb, queue_size=5)
        rospy.Subscriber("/gvnn/controller_status", String, self.ctrl_cb, queue_size=5)
        rospy.Subscriber("/gvnn/metrics", String, self.metrics_cb, queue_size=5)

    # ------------------------------------------------------------------ callbacks
    def map_cb(self, msg: OccupancyGrid) -> None:
        self.map_w, self.map_h = msg.info.width, msg.info.height
        self.map_data = np.asarray(msg.data, dtype=np.int8).reshape(self.map_h, self.map_w)

    def pose_cb(self, msg) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = np.array([msg.pose.position.x, msg.pose.position.y, yaw])

    def truth_cb(self, msg) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.truth = np.array([msg.pose.position.x, msg.pose.position.y, yaw])

    def path_cb(self, msg: Path) -> None:
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def scan_cb(self, msg: LaserScan) -> None:
        self.scan = np.asarray(msg.ranges, dtype=np.float64)

    def bev_cb(self, msg: Image) -> None:
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            img = arr.reshape(msg.height, msg.width, 3)
            if msg.encoding == "rgb8":
                img = img[:, :, ::-1]
            self.bev = img
        except Exception:
            pass

    def state_cb(self, msg: String) -> None:
        try:
            self.state = json.loads(msg.data)
        except Exception:
            pass

    def slam_cb(self, msg: String) -> None:
        try:
            self.slam_status = json.loads(msg.data)
        except Exception:
            pass

    def ctrl_cb(self, msg: String) -> None:
        try:
            self.controller = json.loads(msg.data)
        except Exception:
            pass

    def metrics_cb(self, msg: String) -> None:
        try:
            self.metrics = json.loads(msg.data)
        except Exception:
            pass

    # ------------------------------------------------------------------ drawing
    def _px(self, x: float, y: float):
        return (int(MAP_OFF[0] + x / 24.0 * MAP_W),
                int(MAP_OFF[1] + (1.0 - y / 16.0) * MAP_H))

    def _robot(self, canvas, pose, color, scale=1.0) -> None:
        x, y, yaw = pose
        cx, cy = self._px(x, y)
        length, width = 22 * scale, 14 * scale
        c, s = math.cos(yaw), math.sin(yaw)
        pts = []
        for lx, ly in ((length, width), (length, -width), (-length, -width), (-length, width)):
            pts.append((int(cx + c * lx - s * ly), int(cy - (s * lx + c * ly))))
        cv2.fillPoly(canvas, [np.array(pts, dtype=np.int32)], color)
        tip = (int(cx + c * (length + 8)), int(cy - s * (length + 8)))
        cv2.line(canvas, (cx, cy), tip, color, 2, cv2.LINE_AA)

    def render(self) -> np.ndarray:
        canvas = np.full((H, W, 3), (28, 30, 34), dtype=np.uint8)
        cv2.putText(canvas, "GVNN  Ground Vehicle Neural Navigation", (12, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(canvas, "scene: %s   teleop: %s" % (self.scene, "ON" if self.teleop else "OFF"),
                    (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 190, 220), 1, cv2.LINE_AA)

        # map panel
        if self.map_data is not None:
            img = np.full((self.map_h, self.map_w, 3), (80, 84, 88), dtype=np.uint8)
            img[self.map_data == 0] = (238, 238, 238)
            img[self.map_data == 100] = (18, 18, 20)
            img = cv2.resize(img, (MAP_W, MAP_H), interpolation=cv2.INTER_NEAREST)
            img = cv2.flip(img, 0)
            canvas[MAP_OFF[1]:MAP_OFF[1] + MAP_H, MAP_OFF[0]:MAP_OFF[0] + MAP_W] = img
        else:
            cv2.rectangle(canvas, (MAP_OFF[0], MAP_OFF[1]),
                          (MAP_OFF[0] + MAP_W, MAP_OFF[1] + MAP_H), (90, 90, 90), 1)

        # ground-truth trajectory / path
        if self.path:
            pts = np.array([self._px(*p) for p in self.path], dtype=np.int32)
            cv2.polylines(canvas, [pts], False, (0, 220, 255), 2, cv2.LINE_AA)
        # goal
        gx, gy = self._px(22.0, 14.0)
        cv2.circle(canvas, (gx, gy), 9, (60, 220, 80), 2, cv2.LINE_AA)

        # lidar points
        if self.scan is not None and self.scan.size:
            n = self.scan.size
            angles = self.pose[2] + np.arange(n) * (2.0 * math.pi / n)
            valid = np.isfinite(self.scan) & (self.scan < 8.95)
            for i in np.where(valid)[0][::3]:
                wx = self.pose[0] + self.scan[i] * math.cos(angles[i])
                wy = self.pose[1] + self.scan[i] * math.sin(angles[i])
                px, py = self._px(wx, wy)
                cv2.circle(canvas, (px, py), 1, (60, 200, 255), -1)
        if self.truth is not None:
            self._robot(canvas, self.truth, (60, 200, 60), 1.15)
        self._robot(canvas, self.pose, (230, 120, 30), 1.0)

        # BEV inset
        cv2.rectangle(canvas, (750, 70), (990, 310), (80, 80, 80), 1)
        if self.bev is not None:
            bev = cv2.resize(self.bev, (236, 236), interpolation=cv2.INTER_NEAREST)
            canvas[72:308, 752:988] = bev
        cv2.putText(canvas, "BEV camera", (752, 330), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (190, 200, 210), 1, cv2.LINE_AA)

        # telemetry
        lines = []
        if self.state:
            lines += [
                "t = %.1f s" % float(self.state.get("time", 0)),
                "v = %.2f m/s   w = %.2f rad/s" % (float(self.state.get("v", 0)),
                                                   float(self.state.get("w", 0))),
                "collisions = %s" % self.state.get("collisions", 0),
                "goal distance = %.2f m" % float(self.state.get("goal_distance", 0)),
            ]
        if self.slam_status:
            lines += [
                "map coverage = %.0f %%" % (100.0 * float(self.slam_status.get("coverage", 0))),
                "scan score = %.2f" % float(self.slam_status.get("score", 0)),
                "slam updates = %s" % self.slam_status.get("updates", 0),
            ]
        if self.controller:
            lines.append("controller = %s" % self.controller.get("mode", "?"))
        if self.metrics:
            lines.append("mission = %s" % ("SUCCESS" if self.metrics.get("success") else "running"))
        for i, text in enumerate(lines):
            cv2.putText(canvas, text, (752, 370 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX,
                        0.52, (220, 225, 230), 1, cv2.LINE_AA)
        cv2.putText(canvas, "W/A/S/D teleop   SPACE stop   R reset   1/2/3 scene   Q quit",
                    (12, 585), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 160, 170), 1, cv2.LINE_AA)
        return canvas

    # ------------------------------------------------------------------ input
    def handle_key(self, key: int) -> bool:
        if key < 0:
            return True
        ch = chr(key & 0xFF)
        if ch in ("q", "Q", "\x1b"):
            return False
        if ch in ("r", "R"):
            self.pub_reset.publish(Empty())
            self.v = self.w = 0.0
        elif ch in ("1", "2", "3"):
            self.scene = SCENES[int(ch) - 1]
            self.pub_scene.publish(String(data=self.scene))
            self.v = self.w = 0.0
        if self.teleop:
            if ch in ("w", "W"):
                self.v = min(0.9, self.v + 0.15)
            elif ch in ("s", "S"):
                self.v = max(-0.4, self.v - 0.15)
            elif ch in ("a", "A"):
                self.w = min(1.4, self.w + 0.25)
            elif ch in ("d", "D"):
                self.w = max(-1.4, self.w - 0.25)
            elif ch == " ":
                self.v = self.w = 0.0
            self.last_key = rospy.Time.now()
        return True

    def publish_cmd(self) -> None:
        if not self.teleop:
            return
        now = rospy.Time.now()
        if (now - self.last_key).to_sec() > 0.6:
            self.v *= 0.75
            self.w *= 0.75
            if abs(self.v) < 0.02:
                self.v = 0.0
            if abs(self.w) < 0.02:
                self.w = 0.0
        if (now - self.last_cmd).to_sec() >= 0.05:
            self.last_cmd = now
            msg = Twist()
            msg.linear.x = float(self.v)
            msg.angular.z = float(self.w)
            self.pub_cmd.publish(msg)

    def spin(self) -> None:
        rospy.loginfo("GVNN viewer ready (teleop=%s)", self.teleop)
        while not rospy.is_shutdown():
            self.publish_cmd()
            if not self.no_display:
                try:
                    canvas = self.render()
                    cv2.imshow("GVNN - Ground Vehicle Neural Navigation", canvas)
                    key = cv2.waitKey(30)
                    if not self.handle_key(key):
                        break
                except cv2.error as exc:
                    rospy.logwarn("OpenCV display unavailable (%s); viewer continues headless", exc)
                    self.no_display = True
            else:
                rospy.sleep(0.1)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("gvnn_viewer")
    ViewerNode().spin()
