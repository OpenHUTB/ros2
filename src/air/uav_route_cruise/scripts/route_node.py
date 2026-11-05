#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 节点：固定路线巡航（订阅激光雷达 + 里程计，发布世界系速度指令）.

话题
----
订阅:
    /uav/odom          nav_msgs/Odometry       世界系 ENU 位姿与速度
    /lidar/points      sensor_msgs/PointCloud2  世界系点云（zh 由桥接对齐）
发布:
    /uav/cmd_vel       geometry_msgs/Twist      **世界系**速度（linear.x=东, linear.y=北, linear.z=天）

⚠️ 桥接必须用 body_frame:=false 启动，否则它会把世界系速度当机体系解释，导致乱飞。
   这与 uav_ppo_nav 的约定完全一致。

观测与动作
----------
与 offline 环境（route_env.py）**逐项对齐**，用 policy.build_observation 拼装，
保证「训练时喂什么、实机上就喂什么」。
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import rospy                                             # noqa: E402
from geometry_msgs.msg import Twist                       # noqa: E402
from nav_msgs.msg import Odometry                         # noqa: E402
from sensor_msgs.msg import PointCloud2                   # noqa: E402
from std_msgs.msg import Bool                             # noqa: E402

import policy as P                                        # noqa: E402
from route import Route, make_route                       # noqa: E402
from route_env import MAX_RANGE, N_SECTORS                 # noqa: E402


def _pointcloud2_to_xyz(msg) -> np.ndarray:
    """PointCloud2 -> (N, 3) float64（只用 x/y/z 三个字段）。"""
    try:
        import sensor_msgs.point_cloud2 as pc2
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not pts:
            return np.zeros((0, 3), dtype=np.float64)
        return np.asarray(pts, dtype=np.float64).reshape(-1, 3)
    except Exception:                                     # noqa: BLE001
        return np.zeros((0, 3), dtype=np.float64)


class RouteCruiseNode(object):
    """把训练好的 PPO 策略接到 ROS 上：观测 -> 策略 -> 速度指令。"""

    def __init__(self):
        self.weights = rospy.get_param("~weights", "models/route_ppo_s0.npz")
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.speed_scale = float(rospy.get_param("~speed_scale", 1.0))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        self.dry_run = bool(rospy.get_param("~dry_run", False))

        route_points = rospy.get_param("~route_points", None)
        if route_points:
            pts = np.asarray(route_points, dtype=np.float64)
            if pts.ndim == 1 and pts.size % 3 == 0:
                pts = pts.reshape(-1, 3)
            if pts.ndim != 2 or pts.shape[1] != 3:
                raise ValueError("~route_points 必须是 [[x,y,z], ...] 或扁平数组")
            self.route = Route(pts)
        else:
            self.route = make_route(rospy.get_param("~route", "rect"))
        self.lookahead = float(rospy.get_param("~lookahead", 4.0))

        if not os.path.isfile(self.weights):
            raise FileNotFoundError("找不到策略权重：%s" % self.weights)
        self.policy = P.MlpPolicy.from_npz(self.weights)

        self.odom = None
        self.points = np.zeros((0, 3), dtype=np.float64)
        self.last_cmd_t = None

        self.pub_cmd = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=1)
        self.pub_done = rospy.Publisher("~lap_done", Bool, queue_size=1, latch=True)
        self.sub_odom = rospy.Subscriber("/uav/odom", Odometry, self._on_odom,
                                         queue_size=1)
        self.sub_pc = rospy.Subscriber("/lidar/points", PointCloud2, self._on_points,
                                       queue_size=1)

        self.lap_done = False
        self._s_prev = None
        self._total_s = 0.0
        self._last_log = rospy.Time.now()

        rospy.loginfo("固定路线巡航节点已启动：航线 %d 个点 / %.1f m，权重 %s",
                      len(self.route.points) - 1, self.route.length, self.weights)
        rospy.loginfo("提醒：桥接需以 body_frame:=false 启动（本节点输出世界系速度）")

    # ---------------------------------------------------------- 回调
    def _on_odom(self, msg: Odometry):
        self.odom = msg

    def _on_points(self, msg: PointCloud2):
        self.points = _pointcloud2_to_xyz(msg)

    # ---------------------------------------------------------- 观测
    def _histogram(self, pos) -> np.ndarray:
        """世界系 24 扇形直方图（与 route_env.RouteEnv._histogram 同一算法）。"""
        hist = np.ones(N_SECTORS, dtype=np.float32)
        pts = self.points
        if pts.shape[0] == 0:
            return hist
        # 只统计无人机高度附近 ±1.5 m 的点，避免地面被当成障碍
        keep = np.abs(pts[:, 2] - float(pos[2])) < 1.5
        pts = pts[keep]
        if pts.shape[0] == 0:
            return hist
        # 直接在点云上求每个扇形的最近距离，而不是逐扇形做射线求交（省算力）
        dx = pts[:, 0] - float(pos[0])
        dy = pts[:, 1] - float(pos[1])
        r = np.hypot(dx, dy)
        ang = np.arctan2(dy, dx)
        sector = 2.0 * math.pi / N_SECTORS
        idx = np.floor((ang + sector / 2.0) / sector).astype(np.int64) % N_SECTORS
        for i in range(N_SECTORS):
            m = idx == i
            if m.any():
                hist[i] = float(np.clip(r[m].min() / MAX_RANGE, 0.0, 1.0))
        return hist

    def _build_obs(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        v = self.odom.twist.twist.linear
        pos = np.array([p.x, p.y, p.z], dtype=np.float64)
        vel = np.array([v.x, v.y, v.z], dtype=np.float64)

        _i, s, _proj, cross = self.route.project(pos[:2])

        # 航向偏差：速度方向与航线切向的夹角（速度为 0 时用切向代替）
        d = self.route.seg[_i, :2]
        n = np.linalg.norm(d) or 1.0
        tangent = d / n
        vn = float(np.linalg.norm(vel[:2]))
        if vn > 1e-3:
            heading = vel[:2] / vn
        else:
            heading = tangent
        dot = float(np.clip(float(heading @ tangent), -1.0, 1.0))
        crossang = float(np.arccos(dot))
        if (tangent[0] * heading[1] - tangent[1] * heading[0]) < 0:
            crossang = -crossang

        # 单个前视点（与训练环境 RouteEnv.lookahead 一致）
        tgt = self.route.waypoint_ahead(pos[:2], s, self.lookahead)
        rel = tgt[:2] - pos[:2]
        z_err = float(pos[2] - self.route.points[0, 2])
        obs = P.build_observation(self._histogram(pos), cross, crossang,
                                  rel, vn, z_err)

        # 完成一圈统计
        if self._s_prev is not None:
            ds = s - self._s_prev
            if ds < -self.route.length / 2.0:
                ds += self.route.length
            elif ds > self.route.length / 2.0:
                ds -= self.route.length
            self._total_s += ds
        self._s_prev = s
        return obs, pos, s

    # ---------------------------------------------------------- 主循环
    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            out = self._build_obs()
            if out is None:
                rate.sleep()
                continue
            obs, pos, s = out
            a = self.policy.forward(obs)
            vx, vy, vz, _yaw = P.action_to_velocity(a)
            vx *= self.speed_scale
            vy *= self.speed_scale
            vz *= self.speed_scale

            if not self.dry_run:
                cmd = Twist()
                cmd.linear.x = vx
                cmd.linear.y = vy
                cmd.linear.z = vz
                self.pub_cmd.publish(cmd)
                self.last_cmd_t = rospy.Time.now()

            if (not self.lap_done) and self._total_s >= self.route.length:
                self.lap_done = True
                self.pub_done.publish(Bool(data=True))
                rospy.loginfo("已完成一整圈航线（累计 %.1f m / %.1f m）",
                              self._total_s, self.route.length)

            now = rospy.Time.now()
            if (now - self._last_log).to_sec() > 2.0:
                self._last_log = now
                rospy.loginfo("进度 %.1f%%  横向偏差 %.2f m  速度 (%.2f, %.2f, %.2f)",
                              self.route.progress_ratio(s) * 100.0,
                              self.route.project(pos[:2])[3], vx, vy, vz)
            rate.sleep()

        # 退出时把速度清零，避免无人机保持最后一条指令继续飞
        if not self.dry_run:
            self.pub_cmd.publish(Twist())


def main(argv=None) -> int:
    rospy.init_node("uav_route_cruise", anonymous=False)
    try:
        node = RouteCruiseNode()
    except Exception as exc:                              # noqa: BLE001
        rospy.logfatal("节点初始化失败：%s", exc)
        return 1
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
