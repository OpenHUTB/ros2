#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
face_follower.py - 基于 OpenCV Haar 人脸检测的视觉追踪节点（比例控制器）。

订阅顶置 Kinect / USB 相机的 RGB 图像，检测人脸中心，计算人脸中心相对图像中心的
横向偏差与人脸面积，通过比例（P）控制器发布 Twist 速度指令到 /cmd_vel，
实现小车自动转向面朝目标并保持距离跟随，同时弹出标注调试窗口。

话题接口：
    订阅  input_rgb_image  (sensor_msgs/Image, launch 中重映射为 /camera/rgb/image_raw)
    发布  /cmd_vel         (geometry_msgs/Twist)
    发布  ~image_out       (sensor_msgs/Image, 标注框与中心坐标后的图像)

控制律（P 控制）：
    横向偏差  err_x  = (cx - W/2) / (W/2)            # 归一化到 [-1, 1]
    角速度    w = kp_angular * err_x                 # 人脸在右 -> 右转(实测本车 w>0 即右转)
                                                     # 限幅在 [-max_angular, max_angular]

    线速度（默认 ~only_turn=True 时恒为 0，纯原地转向，彻底杜绝前冲贴脸丢失目标）：
        仅当 ~only_turn=False 时按面积偏差闭环保持距离：
            面积偏差  err_a  = (target_area - area) / target_area
            线速度    v =  kp_linear  * err_a        # 面积过小(太远) -> 前进
        防贴脸安全死区（过近时先停后撤，避免把人脸挤出视野）：
            area > ~area_stop   -> v = 0
            area > ~area_backup -> v = -~backup_linear（小幅度倒退拉开视距）
"""

from __future__ import print_function, division

import os

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class FaceFollower(object):
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        self.bridge = CvBridge()

        # ---------- 话题接口 ----------
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.image_pub = rospy.Publisher("~image_out", Image, queue_size=1)

        # 图像尺寸，首次收到图像后确定
        self.frame_width = 0
        self.frame_height = 0

        # ---------- Haar 级联分类器（多路径容错加载） ----------
        # 优先级：launch 显式指定 > 包内自带 xml > 系统常见路径 > cv2.data 备用分支
        frontal_path = self._find_cascade(
            ["haarcascade_frontalface_default.xml",
             "haarcascade_frontalface_alt.xml"],
            "~cascade_frontal")
        profile_path = self._find_cascade(
            ["haarcascade_profileface.xml"],
            "~cascade_profile")

        self.cascade_frontal = self._load_cascade(frontal_path)
        self.cascade_profile = self._load_cascade(profile_path)

        if self.cascade_frontal is None and self.cascade_profile is None:
            rospy.logerr(
                "未找到任何 Haar 级联分类器，人脸检测无法工作。"
                " 请确认 opencv 数据已安装，或通过 ~cascade_frontal / ~cascade_profile 指定 xml 路径。")
            rospy.signal_shutdown("haar cascade not found")
            return
        if self.cascade_frontal is None:
            rospy.logwarn("未找到正脸级联分类器，仅使用侧面脸分类器: %s" % profile_path)

        # ---------- 检测参数 ----------
        self.scale_factor = rospy.get_param("~scaleFactor", 1.05)
        self.min_neighbors = rospy.get_param("~minNeighbors", 3)
        self.min_size = rospy.get_param("~minSize", 40)
        self.max_size = rospy.get_param("~maxSize", 0)   # 0 表示不限制

        # ---------- 比例（P）控制参数 ----------
        self.kp_angular = rospy.get_param("~kp_angular", 1.2)    # 横向偏差 -> 角速度
        self.kp_linear = rospy.get_param("~kp_linear", 0.6)      # 面积偏差 -> 线速度
        self.max_linear = rospy.get_param("~max_linear", 0.25)   # 线速度限幅 m/s
        self.max_angular = rospy.get_param("~max_angular", 0.5)  # 角速度限幅 rad/s
        self.target_area = rospy.get_param("~target_area", 0.0)  # 目标人脸面积(像素), 0=自动
        self.dead_zone = rospy.get_param("~dead_zone", 0.05)     # 横向偏差死区(归一化)

        # 纯转向模式：视觉追踪的核心是"车头对齐目标(角速度闭环 w)"，原地旋转即可完美
        # 展现 P 控制器跟随效果，彻底杜绝向前冲撞导致目标高出视野而丢失。
        self.only_turn = rospy.get_param("~only_turn", True)

        # 防贴脸安全死区（仅在 ~only_turn=False 时生效）
        self.area_stop = rospy.get_param("~area_stop", 2500.0)    # 面积超过此值 -> v=0
        self.area_backup = rospy.get_param("~area_backup", 3500.0)  # 面积超过此值 -> 小幅度倒退
        self.backup_linear = rospy.get_param("~backup_linear", 0.15)  # 倒退线速度幅值 m/s

        self.show_image = rospy.get_param("~show_image", True)

        self.color = (50, 255, 50)

        # 跟踪状态：是否正在追踪人脸（用于目标丢失时只发一次停止指令）
        self._tracking = False

        self.image_sub = rospy.Subscriber("input_rgb_image", Image,
                                          self.image_callback, queue_size=1)
        rospy.loginfo("face_follower started, waiting for image...")

    def _package_dir(self):
        """返回 robot_vision 包路径，用于定位包内自带级联文件。"""
        try:
            import rospkg
            return rospkg.RosPack().get_path("robot_vision")
        except Exception:
            # 退回当前脚本所在包的根目录（scripts/.. 即包根目录）
            return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def _find_cascade(self, filenames, param_name):
        """按优先级查找级联分类器文件，返回第一个存在的路径，找不到返回 None。"""
        # 1) launch 显式指定（最高优先级）
        explicit = rospy.get_param(param_name, "")
        if explicit:
            if os.path.isfile(explicit):
                return explicit
            rospy.logwarn("%s 指定的文件不存在: %s，继续按默认路径查找" %
                          (param_name, explicit))

        # 2) 候选目录：包内优先，其次系统路径，最后 cv2.data 备用分支
        dirs = []
        pkg = self._package_dir()
        if pkg:
            dirs.append(os.path.join(pkg, "data", "haar_detectors"))
            dirs.append(os.path.join(pkg, "data"))
        dirs.extend([
            "/usr/share/opencv4/haarcascades",
            "/usr/share/opencv/haarcascades",
        ])
        try:
            if hasattr(cv2, "data") and hasattr(cv2.data, "haarcascades"):
                dirs.append(cv2.data.haarcascades)
        except Exception:
            pass

        for d in dirs:
            if not d:
                continue
            for name in filenames:
                path = os.path.join(d, name)
                if os.path.isfile(path):
                    return path
        return None

    @staticmethod
    def _load_cascade(path):
        if not path or not os.path.exists(path):
            return None
        return cv2.CascadeClassifier(path)

    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("cv_bridge 转换失败: %s" % e)
            return

        self.frame_height, self.frame_width = frame.shape[:2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        faces = self._detect_faces(gray)

        if len(faces) > 0:
            # 检测到人脸：立即发布比例控制指令，恢复正常跟踪
            twist = self._control(faces, frame)
            self.cmd_pub.publish(twist)
            self._tracking = True
        elif self._tracking:
            # 目标丢失瞬间：仅发布一次停止，绝不阻断后续回调；
            # 一旦立牌/人脸重新进入视野，下一帧即回到上面分支恢复跟踪。
            self.cmd_pub.publish(Twist())
            self._tracking = False
            rospy.loginfo("目标丢失，已发布一次停止指令")

        # 发布标注后的图像（供 rviz / image_view 订阅）
        if self.image_pub.get_num_connections() > 0:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr("发布图像失败: %s" % e)

        # 弹出调试窗口
        if self.show_image:
            cv2.imshow("face_follower", frame)
            cv2.waitKey(3)

    def _detect_faces(self, gray):
        min_sz = (self.min_size, self.min_size)
        max_sz = (self.max_size, self.max_size) if self.max_size > 0 else None

        faces = []
        if self.cascade_frontal is not None:
            faces = self.cascade_frontal.detectMultiScale(
                gray, self.scale_factor, self.min_neighbors,
                cv2.CASCADE_SCALE_IMAGE, min_sz, max_sz)

        # 正面检测失败，尝试侧面脸
        if len(faces) == 0 and self.cascade_profile is not None:
            faces = self.cascade_profile.detectMultiScale(
                gray, self.scale_factor, self.min_neighbors,
                cv2.CASCADE_SCALE_IMAGE, min_sz, max_sz)

        return faces

    def _control(self, faces, frame):
        twist = Twist()

        if self.frame_width <= 0 or self.frame_height <= 0:
            return twist

        if len(faces) == 0:
            rospy.logdebug("未检测到人脸，停止")
            return twist

        # 取面积最大的人脸
        x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
        cx = x + w / 2.0
        cy = y + h / 2.0
        area = float(w * h)

        # 横向偏差（归一化到 [-1, 1]，人脸在右为正）
        err_x = (cx - self.frame_width / 2.0) / (self.frame_width / 2.0)

        # 角速度：人脸偏右 -> 右转（实测本车 angular.z > 0 对应向右转，取正号消除正反馈）
        if abs(err_x) > self.dead_zone:
            angular = self.kp_angular * err_x
        else:
            angular = 0.0
        angular = max(-self.max_angular, min(self.max_angular, angular))

        # 线速度：默认纯转向(only_turn=True)时恒为 0；否则按面积偏差闭环保持距离，
        # 并叠加防贴脸安全死区（过近时先停后撤，避免把人脸挤出视野）。
        linear = 0.0
        if not self.only_turn:
            target = self.target_area
            if target <= 0:
                target = (self.frame_width * self.frame_height) / 16.0
            err_a = (target - area) / target
            linear = self.kp_linear * err_a
            linear = max(-self.max_linear, min(self.max_linear, linear))

            # 防贴脸：面积过大说明目标已贴到镜头前，必须先停后撤
            if area > self.area_backup:
                linear = -self.backup_linear
            elif area > self.area_stop:
                linear = 0.0

        twist.linear.x = linear
        twist.angular.z = angular

        # 清晰打印检测结果（2Hz 节流，避免刷屏）
        rospy.loginfo_throttle(
            0.5,
            "检测到人脸: 中心=(%d,%d) 横向偏差=%+.3f 面积=%d -> v=%.2f w=%.2f%s" %
            (int(cx), int(cy), err_x, int(area), linear, angular,
             " (纯转向)" if self.only_turn else ""))

        self._draw(frame, x, y, w, h, cx, cy, area, err_x, linear, angular)
        return twist

    def _draw(self, frame, x, y, w, h, cx, cy, area, err_x, linear, angular):
        # 人脸边界框
        cv2.rectangle(frame, (x, y), (x + w, y + h), self.color, 2)

        center_px = int(self.frame_width / 2.0)
        center_py = int(self.frame_height / 2.0)

        # 图像中心（红色十字）与人脸中心（蓝色十字）
        cv2.drawMarker(frame, (center_px, center_py), (0, 0, 255),
                       cv2.MARKER_CROSS, 20, 1)
        cv2.drawMarker(frame, (int(cx), int(cy)), (255, 0, 0),
                       cv2.MARKER_CROSS, 20, 1)
        cv2.line(frame, (center_px, center_py), (int(cx), int(cy)),
                 (255, 255, 0), 1)

        cv2.putText(frame, "face center: (%d, %d)" % (int(cx), int(cy)),
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color, 1)
        cv2.putText(frame, "deviation: %+.3f  area: %d" % (err_x, int(area)),
                    (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color, 1)
        cv2.putText(frame, "cmd: v=%.2f  w=%.2f" % (linear, angular),
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        if self.only_turn:
            mode = "TURN-ONLY"
        elif linear < 0:
            mode = "BACKUP"
        elif linear == 0:
            mode = "HOLD"
        else:
            mode = "TRACK"
        cv2.putText(frame, "mode: %s" % mode,
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    def cleanup(self):
        rospy.loginfo("face_follower 关闭，发布停止指令")
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("face_follower")
    try:
        FaceFollower()
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
