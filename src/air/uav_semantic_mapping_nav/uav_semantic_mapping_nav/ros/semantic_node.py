#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语义建图与语义路径规划节点（ROS2 Humble，Python 3.8）。

订阅 `/camera/image_raw`（RGB）、`/camera/depth`（32FC1，米）与 `/uav/odom`：
RGB 过分割网络得到类别图，与深度一起逆投影进语义栅格；收到 `/uav/goal` 时
在语义代价图上跑 A*，把路径发到 `/semantic/path`。

发布
----
* ``/semantic/map``       nav_msgs/OccupancyGrid，``data`` 为语义类别 id（-1 未知）
* ``/semantic/map_color`` sensor_msgs/Image（rgb8），同上色的可视化版本
* ``/semantic/path``      nav_msgs/Path
* ``/semantic/status``    std_msgs/String
* ``/semantic/cost``      nav_msgs/OccupancyGrid，通行代价（按 20 倍量化成 int8）

``/uav/odom`` 用的是底座发布的里程计；如果换成 VIO 后端，把 ``odom_topic``
重映射到 ``/vio/odom`` 即可复用整条链路。
"""
from __future__ import annotations

import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ..core.labels import N_CLASS, SEMANTIC_RGB
from ..core.planner import PlanConfig, astar, geometric_cost, path_length, smooth_path
from ..core.semantic_map import CameraModel, SemMapConfig, SemanticGrid
from ..core.segnet import SegNet
from ..core.training import resize_area

_SENSOR_QOS = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)


def _quat_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SemanticNode(Node):
    def __init__(self):
        super().__init__("uav_semantic_nav")
        p = self.declare_parameter
        p("map.size", 60.0)
        p("map.res", 0.20)
        p("map.stride", 3)
        p("map.use_truth_seg", False)     # true 则订阅 /camera/seg 当真值（对照用）
        p("model_dir", "")
        p("net.in_hw", [96, 96])
        p("cam.fovy_deg", 60.0)
        p("topic.rgb", "/camera/image_raw")
        p("topic.depth", "/camera/depth")
        p("topic.seg", "/camera/seg")
        p("topic.odom", "/uav/odom")
        p("topic.goal", "/uav/goal")
        p("topic.map", "/semantic/map")
        p("topic.map_color", "/semantic/map_color")
        p("topic.path", "/semantic/path")
        p("topic.status", "/semantic/status")
        p("topic.cost", "/semantic/cost")
        p("rate.report_every", 5.0)

        g = lambda k: self.get_parameter(k).value       # noqa: E731
        self.cfg = SemMapConfig(size=float(g("map.size")), res=float(g("map.res")),
                                stride=int(g("map.stride")))
        self.grid = SemanticGrid(self.cfg)
        self.in_hw = tuple(int(v) for v in g("net.in_hw"))
        self.cam = CameraModel(fovy_deg=float(g("cam.fovy_deg")))
        self.use_truth = bool(g("map.use_truth_seg"))
        self.net = self._load_net(str(g("model_dir")))

        self.rgb = None
        self.depth = None
        self.seg_truth = None
        self.pose = np.zeros(3)
        self.n_frames = 0
        self.n_plan = 0
        self.t0 = time.time()
        self.last_report = time.time()
        self.cost = None

        self.create_subscription(Image, str(g("topic.rgb")), self.on_rgb, _SENSOR_QOS)
        self.create_subscription(Image, str(g("topic.depth")), self.on_depth,
                                 _SENSOR_QOS)
        if self.use_truth:
            self.create_subscription(Image, str(g("topic.seg")), self.on_seg,
                                     _SENSOR_QOS)
        self.create_subscription(Odometry, str(g("topic.odom")), self.on_odom, 10)
        self.create_subscription(Point, str(g("topic.goal")), self.on_goal, 10)

        self.pub_map = self.create_publisher(OccupancyGrid, str(g("topic.map")), 2)
        self.pub_color = self.create_publisher(Image, str(g("topic.map_color")), 2)
        self.pub_path = self.create_publisher(Path, str(g("topic.path")), 10)
        self.pub_status = self.create_publisher(String, str(g("topic.status")), 10)
        self.pub_cost = self.create_publisher(OccupancyGrid, str(g("topic.cost")), 2)

        self.create_timer(2.0, self.publish_map)
        self.create_timer(float(g("rate.report_every")), self.report)
        self.get_logger().info(
            "语义建图节点启动：网络=%s  分割来源=%s"
            % ("已加载" if self.net else "未加载",
               "真值 /camera/seg" if self.use_truth else "网络预测"))

    # ------------------------------------------------------------------ 载入
    def _load_net(self, d):
        if self.use_truth:
            return None
        if not d:
            try:
                from ament_index_python.packages import get_package_share_directory
                d = os.path.join(
                    get_package_share_directory("uav_semantic_mapping_nav"), "data")
            except Exception:
                d = os.path.join(os.path.dirname(os.path.dirname(__file__)), "data")
        p = os.path.join(d, "segnet.npz")
        try:
            return SegNet.load(p)
        except Exception as e:                                    # noqa: BLE001
            self.get_logger().warn("分割权重未加载（%s），改用 /camera/seg" % e)
            self.use_truth = True
            return None

    # ------------------------------------------------------------------ 回调
    def on_rgb(self, m: Image) -> None:
        a = np.frombuffer(m.data, np.uint8)
        if a.size != m.height * m.width * 3:
            return
        self.rgb = a.reshape(m.height, m.width, 3)

    def on_depth(self, m: Image) -> None:
        a = np.frombuffer(m.data, np.float32)
        if a.size != m.height * m.width:
            return
        self.depth = a.reshape(m.height, m.width).astype(np.float32)
        self._integrate()

    def on_seg(self, m: Image) -> None:
        a = np.frombuffer(m.data, np.uint8)
        if a.size != m.height * m.width * 3:
            return
        rgb = a.reshape(m.height, m.width, 3).astype(np.int32)
        # 底座的分割图是按 SEMANTIC_RGB 上色的 bgr8，反查最近的类别
        pal = (SEMANTIC_RGB[:, ::-1] * 255.0)
        d = np.linalg.norm(rgb[:, :, None, :] - pal[None, None, :, :], axis=3)
        self.seg_truth = np.argmin(d, axis=2).astype(np.int32)

    def on_odom(self, m: Odometry) -> None:
        p = m.pose.pose
        self.pose = np.array([p.position.x, p.position.y, _quat_yaw(p.orientation)])

    def on_goal(self, m: Point) -> None:
        self.plan(np.array([m.x, m.y]))

    # ------------------------------------------------------------------ 处理
    def _integrate(self) -> None:
        if self.depth is None:
            return
        seg = None
        if self.use_truth:
            seg = self.seg_truth
        elif self.net is not None and self.rgb is not None:
            hw = (self.depth.shape[0], self.depth.shape[1])
            rgb = np.clip(resize_area(self.rgb, self.in_hw), 0, 255) / 255.0
            p = self.net.predict_proba(rgb[None])[0]
            lab = np.argmax(p, axis=0).astype(np.int32)
            ih = (np.arange(hw[0]) * self.in_hw[0]) // hw[0]
            iw = (np.arange(hw[1]) * self.in_hw[1]) // hw[1]
            seg = lab[ih][:, iw]
        self.grid.integrate(self.pose, self.depth, seg, self.cam)
        self.n_frames += 1

    def plan(self, goal_xy: np.ndarray) -> None:
        self.cost = self.grid.cost_map()
        s = self.grid.cell_of(np.asarray([self.pose[:2]]))[0]
        g = self.grid.cell_of(np.asarray([goal_xy]))[0]
        start = (int(s[1]), int(s[0]))
        goal = (int(g[1]), int(g[0]))
        r = astar(self.cost, start, goal, PlanConfig())
        if not r.ok:
            self.get_logger().warn("规划失败：%s" % r.reason)
            self.publish_path(np.zeros((0, 2)))
            return
        cells = smooth_path(self.cost, r.cells)
        xy = np.asarray([self.grid.centre(a, b) for a, b in cells], float)
        self.n_plan += 1
        self.publish_path(xy)
        self.get_logger().info("规划成功：%.2f m，展开 %d 节点"
                               % (path_length(self.grid, cells), r.expanded))

    # ------------------------------------------------------------------ 发布
    def _grid_msg(self, data, frame: str) -> OccupancyGrid:
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.info.resolution = float(self.cfg.res)
        m.info.width = self.grid.n
        m.info.height = self.grid.n
        m.info.origin.position.x = -self.cfg.size / 2.0
        m.info.origin.position.y = -self.cfg.size / 2.0
        m.info.origin.orientation.w = 1.0
        m.data = data.ravel().tolist()
        return m

    def publish_map(self) -> None:
        sem = self.grid.semantic()
        obs = self.grid.observed()
        out = np.where(obs, sem, -1).astype(np.int8)
        self.pub_map.publish(self._grid_msg(out, "map"))

        col = np.zeros((self.grid.n, self.grid.n, 3), np.uint8)
        col[obs] = (SEMANTIC_RGB[sem[obs]] * 255.0).astype(np.uint8)
        im = Image()
        im.header.stamp = self.get_clock().now().to_msg()
        im.header.frame_id = "map"
        im.height, im.width = self.grid.n, self.grid.n
        im.encoding = "rgb8"
        im.is_bigendian = 0
        im.step = self.grid.n * 3
        im.data = np.flipud(col).tobytes()
        self.pub_color.publish(im)

        if self.cost is not None:
            c = np.where(np.isfinite(self.cost),
                         np.clip(self.cost * 20.0, 0, 100), 100).astype(np.int8)
            c[~self.grid.observed()] = -1
            self.pub_cost.publish(self._grid_msg(c, "map"))

    def publish_path(self, xy: np.ndarray) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for x, y in xy:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def report(self) -> None:
        now = time.time()
        dt = max(now - self.last_report, 1e-6)
        self.last_report = now
        st = self.grid.stats()
        s = String()
        s.data = ("frames=%d freq=%.1fHz observed=%.0f occ=%.0f classes=%.0f "
                  "plans=%d pos=(%.1f,%.1f)"
                  % (self.n_frames, self.n_frames / max(now - self.t0, 1e-6),
                     st["observed"], st["occupied"], st["n_class_used"],
                     self.n_plan, self.pose[0], self.pose[1]))
        self.pub_status.publish(s)
        self.get_logger().info(s.data)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = SemanticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
