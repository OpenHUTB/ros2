#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""屋顶降落主节点：感知状态机 + ROS 话题收发。

状态流转::

    WAIT_DATA ──收到点云与里程计──▶ DETECT
    DETECT    ──找到合格屋顶──▶ APPROACH
    APPROACH  ──/uav/status 的首词变为 GOAL_DONE──▶ DESCEND
    DESCEND   ──高度到达屋顶──▶ LANDED
    任意状态  ──超时 / 异常 / /rooftop/stop──▶ ABORT（安全悬停）

设计要点
--------
* 本节点**不直接连仿真器**：连仿真器、NED↔ENU 换算、速度限幅、指令超时保护
  全部由 ``carlair_ros_bridge`` 负责，本模块只订阅感知数据、下发目标点与速度，
  这样它与键盘控制、规划器等其它模块可以共用同一条 ``/uav/cmd_vel`` 通道。
* 感知部分（``rooftop_perception``）是不依赖 ROS 的纯函数，可直接本地回归测试。
* 高度分量是「上为正」还是「下为正」由参数 ``frame/z_up`` 决定；节点启动后会打印
  原始 odom 高度与换算结果，便于第一时间发现坐标系约定不符。
* ``/uav/status`` 只比较**第一个词**（``frames.status_token``）：桥接层到达目标点时
  发的是 ``"GOAL_DONE %.2f %.2f %.2f"``（带三个坐标），全串比较永远不成立。
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import frames                      # noqa: E402
import rooftop_perception as rp    # noqa: E402

WAIT_DATA, DETECT, APPROACH, DESCEND, LANDED, ABORT = (
    "WAIT_DATA", "DETECT", "APPROACH", "DESCEND", "LANDED", "ABORT")


class RooftopLandingNode(object):
    def __init__(self):
        rospy.init_node("rooftop_landing", anonymous=False)

        # ---------------- 参数 ----------------
        g = rospy.get_param
        self.t_lidar = g("topic/lidar", "/lidar/points")
        self.t_odom = g("topic/odom", "/uav/odom")
        self.t_status = g("topic/status", "/uav/status")
        self.t_cmd = g("topic/cmd_vel", "/uav/cmd_vel")
        self.t_goal = g("topic/goal", "/uav/goal")
        self.t_marker = g("visualization/marker_topic", "/rooftop/markers")
        self.t_landing = g("visualization/landing_topic", "/rooftop/landing_point")
        self.t_start = g("mission/start_topic", "/rooftop/start")

        self.z_up = bool(g("frame/z_up", True))
        self.autostart = bool(g("mission/autostart", True))
        self.world = g("frame/world", "world")

        self.acc_frames = int(g("perception/accumulate_frames", 3))
        self.max_tilt = float(g("perception/max_tilt_deg", 15.0))
        self.plane_tol = float(g("perception/plane_dist_thresh", 0.15))
        self.min_inliers = int(g("perception/min_plane_inliers", 120))
        self.min_area = float(g("perception/min_roof_area", 8.0))
        self.min_height = float(g("perception/min_height", 0.5))
        self.height_mode = g("perception/min_height_mode", "relative")
        self.ground_margin = float(g("perception/ground_margin", 1.0))
        self.max_planes = int(g("perception/max_planes", 5))
        self.min_radius = float(g("perception/min_landing_radius", 0.6))
        self.prefer = g("perception/prefer", "highest")
        self.cluster_cell = float(g("perception/cluster_cell", 1.0))
        self.cluster_min_points = int(g("perception/cluster_min_points", 2))

        self.safe_alt = float(g("approach/safe_altitude", 5.0))
        self.goal_timeout = float(g("approach/goal_timeout", 60.0))
        self.verify_goal = bool(g("approach/verify_goal_coords", True))
        self.goal_tol = float(g("approach/goal_match_tol", 0.5))
        self.goal_retry = float(g("approach/goal_retry_period", 5.0))
        self.desc_speed = abs(float(g("descend/speed", 0.5)))
        self.desc_tol = float(g("descend/tolerance", 0.35))
        self.desc_timeout = float(g("descend/timeout", 60.0))
        self.tick_hz = max(1.0, float(g("mission/tick_rate_hz", 10.0)))
        self.pub_markers = bool(g("visualization/publish_markers", True))

        # ---------------- 状态 ----------------
        self.lock = threading.Lock()
        self.state = WAIT_DATA
        self.points = []
        self.odom = None
        self.raw_alt = None
        self.alt = None
        self.status = ""            # 桥接层原始状态字符串（可能带坐标）
        self.status_token = ""      # 只取第一个词，专门用于状态判断
        self.status_values = ()     # 状态词后面的坐标（桥接实际完成的目标点）
        self.sent_goal = None       # 本节点下发的目标点，用来认领 GOAL_DONE
        self.goal_sent_at = 0.0
        self.target = None          # 当前选中的屋顶（重发目标点用）
        self.state_t0 = time.time()
        self.landing_alt = None
        self.detect_runs = 0
        self.raw_z_logged = 0

        # ---------------- 收发 ----------------
        self.pub_goal = rospy.Publisher(self.t_goal, Point, queue_size=1)
        self.pub_cmd = rospy.Publisher(self.t_cmd, Twist, queue_size=1)
        self.pub_land = rospy.Publisher(self.t_landing, Point, queue_size=1)
        self.pub_mark = rospy.Publisher(self.t_marker, MarkerArray, queue_size=1)

        rospy.Subscriber(self.t_lidar, PointCloud2, self.on_lidar, queue_size=1)
        rospy.Subscriber(self.t_odom, Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(self.t_status, String, self.on_status, queue_size=1)
        rospy.Subscriber(self.t_start, Bool, self.on_start, queue_size=1)

        self.rate = rospy.Rate(self.tick_hz)
        rospy.on_shutdown(self.on_shutdown)

        if self.autostart:
            self.state = DETECT
            self.state_t0 = time.time()
        rospy.loginfo("[rooftop_landing] 已启动，状态=%s，高度约定 z_up=%s"
                      % (self.state, self.z_up))
        rospy.loginfo("[rooftop_landing] 屋顶高度判定：mode=%s"
                      "（ground_margin=%.1f m，min_height=%.2f m 仅 absolute 模式生效），"
                      "主循环 %.1f Hz"
                      % (self.height_mode, self.ground_margin, self.min_height,
                         self.tick_hz))
        rospy.loginfo("[rooftop_landing] 订阅 %s / %s / %s；发布 %s / %s"
                      % (self.t_lidar, self.t_odom, self.t_status,
                         self.t_goal, self.t_cmd))

    # ------------------------------------------------------------------ #
    # 回调
    # ------------------------------------------------------------------ #
    def on_lidar(self, msg):
        pts = frames.filter_finite_xyz(
            frames.pointcloud2_to_xyz(msg.fields, msg.point_step, msg.data,
                                      msg.width, msg.height,
                                      bool(msg.is_bigendian)))
        if len(pts) == 0:
            return
        with self.lock:
            self.points.append(pts)
            if len(self.points) > self.acc_frames:
                self.points.pop(0)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        with self.lock:
            self.odom = np_position(p)
            self.raw_alt = float(p.z)
            self.alt = frames.altitude(self.raw_alt, self.z_up)

    def on_status(self, msg):
        raw = (msg.data or "").strip()
        token = frames.status_token(raw)
        values = frames.status_values(raw)
        with self.lock:
            if token != self.status_token:
                rospy.loginfo("[rooftop_landing] 桥接状态：%s" % (raw or "(空)"))
            self.status = raw
            self.status_token = token
            self.status_values = values

    def on_start(self, msg):
        if msg.data and self.state in (WAIT_DATA, LANDED, ABORT):
            self.set_state(DETECT)
            rospy.loginfo("[rooftop_landing] 收到启动指令，开始任务")

    def on_shutdown(self):
        self.stop_motion()
        rospy.loginfo("[rooftop_landing] 节点退出，已停止运动指令")

    # ------------------------------------------------------------------ #
    # 状态机
    # ------------------------------------------------------------------ #
    def set_state(self, s):
        if s != self.state:
            rospy.loginfo("[rooftop_landing] 状态 %s -> %s" % (self.state, s))
        self.state = s
        self.state_t0 = time.time()

    def stop_motion(self):
        try:
            self.pub_cmd.publish(Twist())
        except Exception:
            pass

    def tick(self):
        with self.lock:
            ready = self.odom is not None and len(self.points) >= 1
            alt = self.alt
            status_token = self.status_token
        if self.raw_alt is not None and self.raw_z_logged < 3:
            self.raw_z_logged += 1
            rospy.loginfo("[rooftop_landing] odom 原始高度 z=%.2f -> 换算高度 %.2f m"
                          "（z_up=%s）" % (self.raw_alt, self.alt, self.z_up))

        if self.state == WAIT_DATA:
            if ready:
                self.set_state(DETECT)

        elif self.state == DETECT:
            if not ready:
                return
            if self.detect_runs < 1 or (time.time() - self.state_t0) > 1.0:
                # 重置计时，保证最多 1 Hz 检测一次。
                # 不重置的话，进入 DETECT 满 1 秒后这个条件恒为真，
                # do_detect（RANSAC + polylabel）会跟着 tick 以 10 Hz 反复跑，
                # 白白吃掉 CPU 还可能拖慢回调。
                self.state_t0 = time.time()
                self.do_detect()

        elif self.state == APPROACH:
            if (time.time() - self.state_t0) > self.goal_timeout:
                rospy.logwarn("[rooftop_landing] 接近超时，进入 ABORT")
                self.set_state(ABORT)
                return
            if status_token == "GOAL_DONE" and self.owns_goal_done():
                rospy.loginfo("[rooftop_landing] 已到达降落点上方，开始下降"
                              "（/uav/status=%s）" % (self.status or "GOAL_DONE"))
                self.set_state(DESCEND)
            else:
                self.resend_goal_if_needed()

        elif self.state == DESCEND:
            if (time.time() - self.state_t0) > self.desc_timeout:
                rospy.logwarn("[rooftop_landing] 下降超时，进入 ABORT")
                self.set_state(ABORT)
                return
            self.publish_descend(alt)

        elif self.state == ABORT:
            self.stop_motion()

    # ------------------------------------------------------------------ #
    # 各状态动作
    # ------------------------------------------------------------------ #
    def do_detect(self):
        with self.lock:
            cloud = None
            if self.points:
                cloud = np.vstack(self.points)
            odom = None if self.odom is None else self.odom.copy()
        self.detect_runs += 1
        if cloud is None or len(cloud) < 50:
            rospy.logwarn_throttle(5.0, "[rooftop_landing] 点云太少，等待数据")
            return

        info = {}
        roofs = rp.detect_rooftops(
            cloud, max_tilt_deg=self.max_tilt, dist_thresh=self.plane_tol,
            min_area=self.min_area, min_inliers=self.min_inliers,
            max_planes=self.max_planes, min_height=self.min_height,
            cluster_cell=self.cluster_cell,
            cluster_min_points=self.cluster_min_points,
            min_height_mode=self.height_mode, ground_margin=self.ground_margin,
            info=info)
        rospy.loginfo("[rooftop_landing] 点云 %d 点，检测到 %d 个候选水平面"
                      % (len(cloud), len(roofs)))
        # 把「凭什么判定这是屋顶」打出来：换地图后一眼能看出阈值该往哪调
        if info.get("floor_ref") is not None:
            rospy.loginfo("[rooftop_landing] 高度基准：相对地面 %.2f m + %.1f m"
                          " → 丢弃低于 %.2f m 的平面"
                          % (info["floor_ref"], self.ground_margin,
                             info["threshold"]))
        elif self.height_mode != "relative":
            rospy.loginfo("[rooftop_landing] 高度基准：绝对阈值 %.2f m"
                          % info.get("threshold", self.min_height))
        else:
            rospy.loginfo("[rooftop_landing] 只提取到 %d 个水平面，"
                          "无法确定地面基准，本次不做高度过滤"
                          % info.get("planes", 0))
        if not roofs:
            rospy.logwarn_throttle(3.0, "[rooftop_landing] 未找到合格屋顶，继续搜索")
            return

        cur_xy = None if odom is None else (float(odom[0]), float(odom[1]))
        target = rp.select_landing_target(roofs, current_xy=cur_xy,
                                          prefer=self.prefer,
                                          min_radius=self.min_radius)
        if target is None:
            rospy.logwarn_throttle(3.0, "[rooftop_landing] 候选面都不满足安全半径"
                                        " %.2f m，继续搜索" % self.min_radius)
            return

        lx, ly = target["landing_point"]
        self.landing_alt = float(target["height"])
        self.target = target          # 目标点被桥接忽略时用它重发
        rospy.loginfo(
            "[rooftop_landing] 选中屋顶：高度 %.2f m，面积 %.1f m^2，"
            "降落点 (%.2f, %.2f)，内切圆半径 %.2f m"
            % (self.landing_alt, target["area"], lx, ly, target["radius"]))
        self.publish_landing_point(target)
        self.publish_landing_marker(target)
        self.publish_goal(target)
        self.set_state(APPROACH)

    def publish_goal(self, target):
        lx, ly = target["landing_point"]
        want_up = target["height"] + self.safe_alt
        msg = Point()
        msg.x = float(lx)
        msg.y = float(ly)
        msg.z = float(frames.to_altitude_frame(want_up, self.z_up))
        self.pub_goal.publish(msg)
        self.sent_goal = (msg.x, msg.y, msg.z)
        self.goal_sent_at = time.time()
        rospy.loginfo("[rooftop_landing] 下发目标点 (%.2f, %.2f, %.2f)，"
                      "即屋顶上方 %.1f m" % (msg.x, msg.y, msg.z, self.safe_alt))

    def owns_goal_done(self):
        """这条 ``GOAL_DONE`` 是不是**本节点下发的那个目标点**完成的。

        桥接发的是 ``"GOAL_DONE x y z"``（带的是它实际完成的目标点坐标），
        所以比对坐标就能分辨来源。状态里没有坐标时无从区分，按「是」处理，
        与只看状态词的行为保持一致。

        实测教训：先手动 ``rostopic pub /uav/goal`` 让飞机爬升，再把本节点拉起来，
        本节点会把那次爬升的 ``GOAL_DONE`` 当成自己到达，于是在**错误的位置**
        原地下降，直到 ``descend/timeout`` 超时才 ABORT。
        """
        if not self.verify_goal:
            return True
        with self.lock:
            sent, values = self.sent_goal, self.status_values
        if frames.goal_matches(values, sent, self.goal_tol):
            return True
        rospy.logwarn_throttle(
            5.0, "[rooftop_landing] 忽略别人的到达状态 GOAL_DONE %s"
                 "（本节点目标 %s）" % (values, sent))
        return False

    def resend_goal_if_needed(self):
        """目标点被桥接忽略时定期重发。

        桥接的 ``cmd_vel_sub`` 在已有目标点任务在执行时会**直接丢弃**新目标
        （只打一条 warn），而本节点只发一次，于是会一直等到 ``goal_timeout``
        才 ABORT。定期重发可以让它在桥接空闲后立刻接上。
        """
        if self.target is None or self.goal_retry <= 0:
            return
        if (time.time() - self.goal_sent_at) < self.goal_retry:
            return
        rospy.logwarn_throttle(5.0, "[rooftop_landing] 还没等到本节点目标点的到达"
                                    "状态，重发目标点")
        self.publish_goal(self.target)

    def publish_landing_point(self, target):
        """把选中的降落点报到 ``/rooftop/landing_point``。

        z 统一是**屋顶高度（上为正）**，便于外部按 ENU 直接记录。
        一次检测**只发一条**：早期版本这里发一条、``publish_landing_marker``
        里又发一条内容完全相同的消息，同一个话题上重复了两条。
        """
        self.pub_land.publish(Point(float(target["landing_point"][0]),
                                    float(target["landing_point"][1]),
                                    float(target["height"])))

    def publish_descend(self, alt):
        # 距离屋顶还有多高（上为正）
        remain = (alt - self.landing_alt) if alt is not None else None
        if remain is not None and remain <= self.desc_tol:
            self.stop_motion()
            rospy.loginfo("[rooftop_landing] 已降落：离屋顶 %.2f m <= 容差 %.2f m"
                          % (remain, self.desc_tol))
            self.set_state(LANDED)
            return
        tw = Twist()
        # cmd_vel 是**机体系**：z 向上为正（桥接层负责换算与限幅）
        tw.linear.z = -self.desc_speed
        self.pub_cmd.publish(tw)
        rospy.loginfo_throttle(1.0, "[rooftop_landing] 下降中：离屋顶 %s m"
                               % ("?" if remain is None else "%.2f" % remain))

    # ------------------------------------------------------------------ #
    # 可视化
    # ------------------------------------------------------------------ #
    def publish_landing_marker(self, target):
        if not self.pub_markers:
            return
        arr = MarkerArray()
        hull = target["hull"]
        m = Marker()
        m.header.frame_id = self.world
        m.header.stamp = rospy.Time.now()
        m.ns, m.id, m.type, m.action = "roof", 0, Marker.LINE_STRIP, Marker.ADD
        m.scale.x = 0.15
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 1.0
        m.pose.orientation.w = 1.0
        z = float(target["height"])
        for p in list(hull) + [hull[0]]:
            m.points.append(Point(float(p[0]), float(p[1]), z))
        arr.markers.append(m)

        s = Marker()
        s.header.frame_id = self.world
        s.header.stamp = rospy.Time.now()
        s.ns, s.id, s.type, s.action = "landing", 1, Marker.SPHERE, Marker.ADD
        s.pose.position.x = float(target["landing_point"][0])
        s.pose.position.y = float(target["landing_point"][1])
        s.pose.position.z = z
        s.pose.orientation.w = 1.0
        s.scale.x = s.scale.y = s.scale.z = max(0.4, 2.0 * target["radius"])
        s.color.r, s.color.g, s.color.b, s.color.a = 0.1, 1.0, 0.2, 0.6
        arr.markers.append(s)
        self.pub_mark.publish(arr)

    def spin(self):
        while not rospy.is_shutdown():
            try:
                self.tick()
            except Exception as exc:      # 任何异常都不让节点挂掉
                rospy.logerr("[rooftop_landing] tick 异常：%s" % exc)
            self.rate.sleep()


def np_position(p):
    return np.array([float(p.x), float(p.y), float(p.z)], dtype=np.float64)


def main():
    node = RooftopLandingNode()
    node.spin()


if __name__ == "__main__":
    main()
