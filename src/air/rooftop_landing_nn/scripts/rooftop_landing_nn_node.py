#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""学习型屋顶降落节点：点云 -> 高度图 -> 小 CNN -> 落点 -> 自主降落。

与 `rooftop_landing` 的关系
---------------------------
`rooftop_landing` 用「RANSAC 平面 + 凸包 + polylabel 最大内切圆」求落点，几何意义清楚、
在规则矩形屋顶上很准，但**凹形屋顶**（U 形、L 形、带天井的楼）会被凸包"填平"，
**点云稀疏**时凸包还会被拆碎；本模块用一个小 CNN 直接从高度图上回归落点，
在这两类退化下明显更稳（对比实验见 README 与 docs/air/rooftop_landing_nn.md）。

两者话题接口保持一致，可以：
* 直接跑本模块（自带感知 + 下降控制）；
* 也可以只用它发布 ``/rooftop_nn/landing_point``，让 `rooftop_landing` 或别的规划器消费。

话题
----
订阅：``/lidar/points`` ``/uav/odom`` ``/uav/status`` ``/rooftop_nn/start``
发布：``/uav/goal`` ``/uav/cmd_vel`` ``/rooftop_nn/landing_point`` ``/rooftop_nn/markers``

状态机
------
``DETECT``（累积点云 -> 推理落点） -> ``APPROACH``（飞抵屋顶上方安全高度）
-> ``DESCEND``（垂直下降） -> ``LANDED``；任何阶段超时进入 ``ABORT``（悬停待命）。
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
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import heightmap as hm_mod       # noqa: E402
import landing_cnn as lcnn       # noqa: E402

STATE_DETECT = "DETECT"
STATE_APPROACH = "APPROACH"
STATE_DESCEND = "DESCEND"
STATE_LANDED = "LANDED"
STATE_ABORT = "ABORT"


def _local_to_world(points, pose):
    """机体系局部点云 -> 世界系 ENU。

    ``pose`` 为 (x, y, z, yaw)。只绕 z 轴旋转 + 平移：LiDAR 安装在机体上、
    机体系 z 轴已按 ENU 向上对齐（桥接层做了 NED->ENU 轴变换但没加位姿时就是这种）。
    """
    x, y, z, yaw = pose
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if abs(yaw) > 1e-6:
        c, s = math.cos(yaw), math.sin(yaw)
        out = np.empty_like(pts)
        out[:, 0] = c * pts[:, 0] - s * pts[:, 1] + x
        out[:, 1] = s * pts[:, 0] + c * pts[:, 1] + y
        out[:, 2] = pts[:, 2] + z
        return out
    return pts + np.array([x, y, z], dtype=np.float32)


class RooftopLandingNNNode(object):
    def __init__(self):
        self.cells = int(rospy.get_param("~perception/cells", 64))
        self.size_m = float(rospy.get_param("~perception/size_m", 16.0))
        self.accumulate = int(rospy.get_param("~perception/accumulate_frames", 3))
        self.min_height_m = float(rospy.get_param("~perception/min_height_m", 1.0))
        rough = rospy.get_param("~perception/rough_max", 0.35)
        self.rough_max = None if rough in ("", None, -1, "-1") else float(rough)
        self.roi_center_mode = str(rospy.get_param("~perception/roi_center", "uav"))
        self.model_path = str(rospy.get_param("~model/path", ""))

        self.z_up = bool(rospy.get_param("~frame/z_up", True))
        self.frame_world = str(rospy.get_param("~frame/world", "world"))
        #: ``/lidar/points`` 给的是「机体系局部点云」（只有轴变换、没加位姿）时置 true：
        #: 节点会用 ``/uav/odom`` 的位姿把点云搬到世界系再建高度图。
        #: 有些桥接层（如 carlair_ros_bridge 的 lidar_pub）就是这样发的，默认 false
        #: 表示话题里已经是世界系点云（假桥接与单测走这条路径）。
        self.lidar_to_world = bool(rospy.get_param("~frame/lidar_to_world", False))

        self.safe_altitude = float(rospy.get_param("~approach/safe_altitude", 5.0))
        self.goal_timeout = float(rospy.get_param("~approach/goal_timeout", 60.0))
        self.goal_retry_period = float(rospy.get_param("~approach/goal_retry_period", 5.0))
        self.verify_goal_coords = bool(rospy.get_param("~approach/verify_goal_coords", True))
        self.goal_match_tol = float(rospy.get_param("~approach/goal_match_tol", 0.5))

        self.descend_speed = float(rospy.get_param("~descend/speed", 0.5))
        self.descend_tolerance = float(rospy.get_param("~descend/tolerance", 0.35))
        self.descend_timeout = float(rospy.get_param("~descend/timeout", 60.0))

        self.autostart = bool(rospy.get_param("~mission/autostart", True))
        self.tick_rate = float(rospy.get_param("~mission/tick_rate_hz", 10.0))
        self.publish_markers = bool(rospy.get_param("~visualization/publish_markers", True))

        self.topic_lidar = str(rospy.get_param("~topic/lidar", "/lidar/points"))
        self.topic_odom = str(rospy.get_param("~topic/odom", "/uav/odom"))
        self.topic_status = str(rospy.get_param("~topic/status", "/uav/status"))
        self.topic_cmd_vel = str(rospy.get_param("~topic/cmd_vel", "/uav/cmd_vel"))
        self.topic_goal = str(rospy.get_param("~topic/goal", "/uav/goal"))
        self.topic_landing = str(rospy.get_param("~topic/landing_point", "/rooftop_nn/landing_point"))
        self.topic_markers = str(rospy.get_param("~topic/markers", "/rooftop_nn/markers"))
        self.topic_start = str(rospy.get_param("~mission/start_topic", "/rooftop_nn/start"))

        if not self.model_path:
            self.model_path = os.path.join(os.path.dirname(_HERE), "models", "landing_cnn.npz")
            self.model_path = os.path.abspath(self.model_path)
        self.model = lcnn.load_model(self.model_path)
        meta = self.model.get("meta", {})
        self.cells = int(meta.get("cells", self.cells))
        self.size_m = float(meta.get("size_m", self.size_m))
        self.min_height_m = float(meta.get("min_height_m", self.min_height_m))
        if "rough_max" in meta and rough not in ("", None, -1, "-1"):
            self.rough_max = float(meta["rough_max"])

        self.lock = threading.Lock()
        self.clouds = []
        self.odom = None
        self.odom_pose = None
        self.status = ""
        self.status_xy = None
        self.state = STATE_DETECT
        self.state_since = time.time()
        self.goal = None
        self.goal_sent_at = 0.0
        self.landing = None
        self.roof_z = None
        self.abort_reason = ""

        self.pub_goal = rospy.Publisher(self.topic_goal, Point, queue_size=1)
        self.pub_cmd = rospy.Publisher(self.topic_cmd_vel, Twist, queue_size=1)
        self.pub_landing = rospy.Publisher(self.topic_landing, Point, queue_size=1)
        self.pub_markers = rospy.Publisher(self.topic_markers, MarkerArray, queue_size=1)

        rospy.Subscriber(self.topic_lidar, PointCloud2, self.on_cloud, queue_size=1)
        rospy.Subscriber(self.topic_odom, Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(self.topic_status, String, self.on_status, queue_size=10)
        rospy.Subscriber(self.topic_start, Bool, self.on_start, queue_size=1)

        self.started = self.autostart
        rospy.loginfo("[rooftop_nn] 已启动，状态 %s，模型 %s（%d 格 / %.1f m ROI）",
                      self.state, os.path.basename(self.model_path), self.cells, self.size_m)
        rospy.Timer(rospy.Duration(1.0 / max(self.tick_rate, 1e-3)), self.tick)

    # ------------------------------------------------------------------ 回调
    def on_cloud(self, msg):
        try:
            pts = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        except Exception as exc:                       # pragma: no cover
            rospy.logwarn_throttle(5.0, "[rooftop_nn] 点云解析失败: %s", exc)
            return
        if not pts:
            return
        arr = np.asarray(pts, dtype=np.float32)
        if self.lidar_to_world:
            with self.lock:
                pose = self.odom_pose
            if pose is None:
                return                     # 还没收到里程计，先不累积
            arr = _local_to_world(arr, pose)
        with self.lock:
            self.clouds.append(arr)
            if len(self.clouds) > max(self.accumulate, 1):
                self.clouds.pop(0)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.odom = (float(p.x), float(p.y), float(p.z))
            self.odom_pose = (float(p.x), float(p.y), float(p.z), float(yaw))

    def on_status(self, msg):
        raw = (msg.data or "").strip()
        word = raw.split()[0] if raw else ""
        xy = None
        parts = raw.split()
        if len(parts) >= 4:
            try:
                xy = (float(parts[1]), float(parts[2]))
            except ValueError:
                xy = None
        with self.lock:
            self.status = word
            self.status_xy = xy

    def on_start(self, msg):
        if msg.data:
            rospy.loginfo("[rooftop_nn] 收到开始指令")
            self.started = True

    # ------------------------------------------------------------------ 工具
    def altitude(self, z_raw):
        return z_raw if self.z_up else -z_raw

    def to_raw(self, alt):
        return alt if self.z_up else -alt

    def set_state(self, state, reason=""):
        if state != self.state:
            rospy.loginfo("[rooftop_nn] 状态 %s -> %s%s", self.state, state,
                          ("（%s）" % reason) if reason else "")
            self.state = state
            self.state_since = time.time()
        if reason:
            self.abort_reason = reason

    def merged_cloud(self):
        with self.lock:
            if not self.clouds:
                return None
            return np.vstack(self.clouds)

    def stop(self):
        self.pub_cmd.publish(Twist())

    # ------------------------------------------------------------------ 状态机
    def tick(self, _evt=None):
        try:
            if not self.started:
                return
            state = self.state
            if state == STATE_DETECT:
                self.step_detect()
            elif state == STATE_APPROACH:
                self.step_approach()
            elif state == STATE_DESCEND:
                self.step_descend()
            elif state == STATE_LANDED:
                self.stop()
        except Exception as exc:                       # pragma: no cover
            rospy.logerr("[rooftop_nn] tick 异常: %s", exc)
            self.stop()
            self.set_state(STATE_ABORT, "exception: %s" % exc)

    def step_detect(self):
        cloud = self.merged_cloud()
        with self.lock:
            odom = self.odom
        if cloud is None or odom is None:
            return
        if self.roi_center_mode == "uav":
            center = (odom[0], odom[1])
        else:
            center = hm_mod.roi_center_from_points(cloud, fallback_xy=(odom[0], odom[1]))

        hm = hm_mod.points_to_heightmap(cloud, center, size_m=self.size_m, cells=self.cells)
        if not hm["mask"].any():
            rospy.logwarn_throttle(5.0, "[rooftop_nn] ROI 内没有点，等待点云")
            return
        heat = lcnn.forward(self.model, hm_mod.heightmap_channels(hm))
        point = lcnn.heat_to_point(heat, hm, min_height_m=self.min_height_m,
                                   rough_max=self.rough_max)
        if not point["ok"]:
            rospy.logwarn_throttle(5.0, "[rooftop_nn] 高度图为空，无法给出落点")
            return

        roof_z = hm_mod.cell_surface_z(cloud, point["row"], point["col"], hm)
        self.landing = (point["x"], point["y"], roof_z)
        self.roof_z = roof_z
        rospy.loginfo("[rooftop_nn] 点云 %d 点，检出落点 (%.2f, %.2f)，屋面高度 %.2f m，"
                      "置信度 %.2f，净空未知（由高度图约束）",
                      cloud.shape[0], point["x"], point["y"], roof_z, point["score"])

        self.pub_landing.publish(Point(x=point["x"], y=point["y"], z=roof_z))
        if self.publish_markers:
            self.publish_markers_for(hm, point)

        goal_alt = roof_z + self.safe_altitude
        self.goal = (point["x"], point["y"], self.to_raw(goal_alt))
        self.pub_goal.publish(Point(x=self.goal[0], y=self.goal[1], z=self.goal[2]))
        self.goal_sent_at = time.time()
        rospy.loginfo("[rooftop_nn] 下发目标点 (%.2f, %.2f, %.2f)，即屋面上方 %.1f m",
                      self.goal[0], self.goal[1], self.goal[2], self.safe_altitude)
        self.set_state(STATE_APPROACH)

    def step_approach(self):
        with self.lock:
            status, sxy = self.status, self.status_xy
        now = time.time()
        if status == "GOAL_DONE":
            mine = True
            if self.verify_goal_coords and sxy is not None and self.goal is not None:
                mine = (math.hypot(sxy[0] - self.goal[0], sxy[1] - self.goal[1])
                        <= self.goal_match_tol)
            if mine:
                self.set_state(STATE_DESCEND)
                return
            rospy.loginfo_throttle(5.0, "[rooftop_nn] 忽略别人的到达状态 %s", status)
        # 目标点可能被桥接丢弃（已有任务在执行）：按周期重发
        if self.goal_retry_period > 0 and now - self.goal_sent_at > self.goal_retry_period:
            self.pub_goal.publish(Point(x=self.goal[0], y=self.goal[1], z=self.goal[2]))
            self.goal_sent_at = now
        if now - self.state_since > self.goal_timeout:
            self.stop()
            self.set_state(STATE_ABORT, "接近超时")

    def step_descend(self):
        with self.lock:
            odom = self.odom
        if odom is None or self.roof_z is None:
            return
        remain = self.altitude(odom[2]) - self.roof_z
        if remain <= self.descend_tolerance:
            self.stop()
            rospy.loginfo("[rooftop_nn] 已降落：离屋面 %.2f m <= 容差 %.2f m",
                          remain, self.descend_tolerance)
            self.set_state(STATE_LANDED)
            return
        cmd = Twist()
        cmd.linear.z = -self.descend_speed if self.z_up else self.descend_speed
        self.pub_cmd.publish(cmd)
        rospy.loginfo_throttle(2.0, "[rooftop_nn] 下降中：离屋面 %.2f m", remain)
        if time.time() - self.state_since > self.descend_timeout:
            self.stop()
            self.set_state(STATE_ABORT, "下降超时")

    # ------------------------------------------------------------------ 可视化
    def publish_markers_for(self, hm, point):
        arr = MarkerArray()
        head = Marker()
        head.header.frame_id = self.frame_world
        head.header.stamp = rospy.Time.now()
        head.ns = "rooftop_nn"
        head.id = 0
        head.type = Marker.SPHERE
        head.action = Marker.ADD
        head.pose.position.x = point["x"]
        head.pose.position.y = point["y"]
        head.pose.position.z = self.roof_z if self.roof_z is not None else 0.0
        head.scale.x = head.scale.y = head.scale.z = 0.8
        head.color.r, head.color.g, head.color.b, head.color.a = 1.0, 0.4, 0.0, 0.9
        arr.markers.append(head)

        roi = Marker()
        roi.header.frame_id = self.frame_world
        roi.header.stamp = rospy.Time.now()
        roi.ns = "rooftop_nn"
        roi.id = 1
        roi.type = Marker.LINE_STRIP
        roi.action = Marker.ADD
        roi.scale.x = 0.08
        roi.color.g, roi.color.b, roi.color.a = 0.8, 1.0, 0.8
        half = 0.5 * hm["size_m"]
        cx, cy = hm["center"]
        for (dx, dy) in ((-half, -half), (half, -half), (half, half), (-half, half), (-half, -half)):
            p = Point()
            p.x, p.y = cx + dx, cy + dy
            p.z = self.roof_z if self.roof_z is not None else 0.0
            roi.points.append(p)
        arr.markers.append(roi)
        self.pub_markers.publish(arr)


def main():
    rospy.init_node("rooftop_landing_nn", anonymous=False)
    RooftopLandingNNNode()
    rospy.spin()


if __name__ == "__main__":
    main()
