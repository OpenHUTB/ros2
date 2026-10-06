#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
spawn_face_target.py - 在 Gazebo 中生成人脸立牌并支持键盘手动控制，供 face_follower 追踪测试。

1. 使用标准 Gazebo 模型 face_board（model.config + model.sdf + OGRE 材质脚本），
   把模型目录加入 GAZEBO_MODEL_PATH，并把 model.sdf 中的 model:// 替换为 file:// 绝对路径，
   确保 Gazebo 100% 能渲染出高对比度人脸纹理。
2. 立牌默认静止在小车正前方 (x=1.3, y=0.0, z=0.4)，通过键盘手动微调位置（REP103 约定：
   +x 前方、+y 左方、+z 上方）：
       a : 向左移动 0.15 m
       d : 向右移动 0.15 m
       w : 靠近小车 0.10 m
       s : 远离小车 0.10 m
       r : 复位到初始正前方
       q : 退出控制
   每次按键后打印立牌当前相对小车的坐标，使小车产生可预期的左右转向跟随响应。

用法：
    rosrun robot_vision spawn_face_target.py
    roslaunch robot_vision spawn_face_target.launch
"""

import math
import os
import sys
import threading

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Twist

# termios/tty 仅在 Linux 可用；非 Linux 环境自动禁用键盘
try:
    import termios
    import tty
    import select
    _HAS_TERMIOS = True
except ImportError:
    _HAS_TERMIOS = False

MODEL_NAME = "face_board"


class FaceTargetSpawner(object):
    def __init__(self):
        # ---------- 初始位置（正前方）与步长 ----------
        self.x0 = rospy.get_param("~x", 1.3)
        self.y0 = rospy.get_param("~y", 0.0)
        self.z0 = rospy.get_param("~z", 0.4)
        self.yaw = rospy.get_param("~yaw", 0.0)
        self.step_xy = rospy.get_param("~step_xy", 0.15)   # 左右步长 m
        self.step_x = rospy.get_param("~step_x", 0.1)      # 前后步长 m
        self.enable_keyboard = self._as_bool(rospy.get_param("~enable_keyboard", True)) \
            and _HAS_TERMIOS

        self.x = self.x0
        self.y = self.y0
        self.z = self.z0

        # ---------- 定位模型目录并加入 GAZEBO_MODEL_PATH ----------
        self.model_dir = self._resolve_model_dir()
        models_root = os.path.dirname(self.model_dir)  # .../models
        old = os.environ.get("GAZEBO_MODEL_PATH", "")
        os.environ["GAZEBO_MODEL_PATH"] = os.pathsep.join(
            [p for p in (models_root, old) if p])
        rospy.loginfo("GAZEBO_MODEL_PATH 已加入: %s" % models_root)

        # ---------- 读取 model.sdf，把 model:// 替换为 file:// 绝对路径 ----------
        sdf_path = os.path.join(self.model_dir, "model.sdf")
        try:
            model_xml = open(sdf_path).read()
        except IOError as e:
            rospy.logerr("读取 model.sdf 失败: %s" % e)
            rospy.signal_shutdown("model.sdf not found")
            return
        model_xml = model_xml.replace(
            "model://face_board/", "file://%s/" % self.model_dir)

        # ---------- 生成模型（静止在正前方） ----------
        spawn = self._wait_service("/gazebo/spawn_sdf_model", SpawnModel, timeout=30.0)
        if spawn is None:
            rospy.signal_shutdown("gazebo service unavailable")
            return
        resp = spawn(MODEL_NAME, model_xml, "",
                     self._make_pose(self.x, self.y, self.z, self.yaw), "")
        if not resp.success:
            rospy.logerr("生成人脸立牌失败: %s" % resp.status_message)
            rospy.signal_shutdown("spawn failed")
            return
        self._log_position("已生成并静止在正前方")

        # ---------- 动态平移发布者 ----------
        self.state_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)

        # ---------- 键盘手操 ----------
        if self.enable_keyboard:
            t = threading.Thread(target=self._keyboard_loop)
            t.daemon = True
            t.start()
            rospy.loginfo("键盘手操：a 左移 / d 右移 / w 靠近 / s 远离 / r 复位 / q 退出")
        else:
            rospy.logwarn("当前环境无 TTY，键盘手操不可用，立牌保持静止")

    # ------------------------------------------------------------------ #
    def _resolve_model_dir(self):
        try:
            import rospkg
            pkg = rospkg.RosPack().get_path("robot_vision")
            return os.path.join(pkg, "models", "face_board")
        except Exception:
            return os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                "models", "face_board")

    @staticmethod
    def _wait_service(name, srv_type, timeout=10.0):
        rospy.loginfo("等待服务 %s ..." % name)
        try:
            rospy.wait_for_service(name, timeout)
        except rospy.ROSException as e:
            rospy.logerr("服务 %s 未出现（请先启动 Gazebo）: %s" % (name, e))
            return None
        return rospy.ServiceProxy(name, srv_type)

    @staticmethod
    def _as_bool(v):
        if isinstance(v, bool):
            return v
        return str(v).strip().lower() in ("true", "1", "yes", "on")

    @staticmethod
    def _make_pose(x, y, z, yaw):
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = math.sin(yaw / 2.0)
        p.orientation.w = math.cos(yaw / 2.0)
        return p

    def _publish_state(self):
        state = ModelState()
        state.model_name = MODEL_NAME
        state.reference_frame = "world"
        state.pose = self._make_pose(self.x, self.y, self.z, self.yaw)
        state.twist = Twist()
        self.state_pub.publish(state)

    def _log_position(self, action):
        rospy.loginfo("%s，立牌坐标(相对小车): x=%.2f  y=%.2f  z=%.2f" %
                      (action, self.x, self.y, self.z))

    def _move(self, dx, dy, action):
        self.x = round(self.x + dx, 3)
        self.y = round(self.y + dy, 3)
        self._publish_state()
        self._log_position(action)

    def _reset(self):
        self.x = self.x0
        self.y = self.y0
        self.z = self.z0
        self._publish_state()
        self._log_position("已复位")

    # ------------------------------------------------------------------ #
    def _keyboard_loop(self):
        while not rospy.is_shutdown():
            k = self._get_key(0.1)
            if not k:
                continue
            if k in ("a", "A"):
                self._move(0.0, self.step_xy, "向左移动 %.2f m" % self.step_xy)
            elif k in ("d", "D"):
                self._move(0.0, -self.step_xy, "向右移动 %.2f m" % self.step_xy)
            elif k in ("w", "W"):
                self._move(-self.step_x, 0.0, "靠近小车 %.2f m" % self.step_x)
            elif k in ("s", "S"):
                self._move(self.step_x, 0.0, "远离小车 %.2f m" % self.step_x)
            elif k in ("r", "R"):
                self._reset()
            elif k in ("q", "Q"):
                rospy.loginfo("退出控制")
                rospy.signal_shutdown("user quit")
                break
            elif k == "\x03":  # Ctrl-C
                break

    @staticmethod
    def _get_key(timeout=0.1):
        if not _HAS_TERMIOS:
            return ""
        try:
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            tty.setraw(fd)
            r, _, _ = select.select([sys.stdin], [], [], timeout)
            key = sys.stdin.read(1) if r else ""
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            return key
        except Exception:
            return ""


def main():
    rospy.init_node("spawn_face_target")
    FaceTargetSpawner()
    rospy.spin()


if __name__ == "__main__":
    main()
