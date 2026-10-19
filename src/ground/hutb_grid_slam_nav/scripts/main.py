#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hutb_grid_slam_nav: 自研 2D 激光雷达栅格 SLAM 节点 (occupancy grid SLAM)
=============================================================
这是本模块的核心新算法: 不依赖 gmapping/cartographer 等现成包,
纯 NumPy 实现"同步定位与建图 (SLAM)":

  1. 建图 (Mapping):
     激光扫描点经当前位姿变换到全局坐标, 对栅格做贝叶斯 log-odds 更新
       l(m_i | x_{1:t}, z_{1:t}) = l(m_i | x_{1:t-1}, z_{1:t-1})
                                   + l_occ - l0      (被击中的栅格)
                                   + l_free - l0     (被穿过的栅格)
     其中 l0 = 0 是先验 log-odds, l_occ = 0.9, l_free = -0.4.
     占用概率 p = logodds2prob(l) = 1 - 1/(1+exp(l)).

  2. 定位 (scan-to-map 匹配):
     用"迭代最近点 + 栅格最近邻"在已建栅格地图上修正里程计位姿:
     对每个激光端点 p_i, 在地图中找最近被占用栅格 q_i,
     最小化 Σ||q_i - R·p_i - t||², 迭代 3 次求解位姿增量 (x, y, yaw).

  3. 数据关联:
     tf 树: map -> odom (SLAM 修正) -> base_footprint -> laser.
     本节点在 /odom 上叠加修正量发布 map->odom 变换.

输出: /map (nav_msgs/OccupancyGrid), 供 map_server 保存与 amcl/move_base 导航.

用法:
  roslaunch hutb_grid_slam_nav main.launch phase:=slam
"""
import math

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

RES = 0.05          # 栅格分辨率 m/cell
MAP_SIZE = 40.0     # 地图边长 m
L_OCC, L_FREE, L0 = 0.9, -0.4, 0.0


class GridSLAM:
    def __init__(self):
        rospy.init_node('lidar_slam', anonymous=False)
        n = int(MAP_SIZE / RES)
        self.n = n
        self.center = n // 2
        self.logodds = np.zeros((n, n), dtype=np.float32)
        # 导航阶段支持: 加载已保存地图 + 固定地图(只定位不更新) + 初始位姿
        self.fixed_map = rospy.get_param('~fixed_map', False)
        # 建图阶段匹配在空白地图上易退化, 可关闭匹配 (纯里程计积分, 仿真 odom 精度高);
        # 导航阶段地图已建立, 开启 scan-to-map 匹配定位
        self.match_enabled = rospy.get_param('~match_enabled', True)
        load_map = rospy.get_param('~load_map', '')
        if load_map:
            self.logodds = self._load_pgm(load_map)
            rospy.loginfo('加载地图 %s, 占用率 %.2f%%'
                          % (load_map, 100 * (self.logodds > 0).mean()))
        self.pose = np.array([rospy.get_param('~init_x', 0.0),
                              rospy.get_param('~init_y', 0.0),
                              rospy.get_param('~init_yaw', 0.0)])  # 地图系位姿
        self.odom_last = None                       # 上次里程计 (x, y, yaw)
        self.scan_cnt = 0
        self.pub_map = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        self.pub_tf = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        rospy.loginfo('自研栅格 SLAM 启动: 分辨率 %.2f m, 地图 %dx%d, fixed=%s'
                      % (RES, n, n, self.fixed_map))

    def _load_pgm(self, path):
        """加载 map_saver 生成的 PGM 地图 (P5/P2), 返回 log-odds 栅格 (n x n)"""
        with open(path, 'rb') as f:
            magic = f.readline().strip()
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            w, h = map(int, line.split())
            maxval = int(f.readline())
            if magic == b'P5':
                raw = np.frombuffer(f.read(w * h), dtype=np.uint8)
            else:  # P2
                raw = np.array(f.read().split(), dtype=np.uint8)
        img = raw.reshape(h, w).astype(np.float32) / float(maxval)
        # map_saver 默认 negate=0: 255=空闲, 0=占用
        p_occ = np.clip(1.0 - img, 0.02, 0.98)
        lo = np.clip(np.log(p_occ / (1.0 - p_occ)), -4.0, 8.0)
        if h != self.n or w != self.n:
            # 尺寸不一致时缩放到标准尺寸 (最近邻)
            ys = (np.linspace(0, h - 1, self.n)).astype(int)
            xs = (np.linspace(0, w - 1, self.n)).astype(int)
            lo = lo[np.ix_(ys, xs)]
        return lo.astype(np.float32)

    # ---------------- 回调 ----------------
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom_last = np.array([msg.pose.pose.position.x,
                                   msg.pose.pose.position.y, yaw])

    def scan_cb(self, msg):
        if self.odom_last is None:
            return
        # 激光端点 (机器人坐标系)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges, dtype=np.float32)
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max * 0.95)
        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])
        if len(xs) < 20:
            return

        # 1) 里程计预测位姿
        odom_pose = self.odom_last
        pred_pose = self._odom_to_map(odom_pose)

        # 2) scan-to-map 匹配修正 (迭代最近点简化版); 可关闭以纯里程计建图
        if self.match_enabled:
            self.pose = self._scan_match(xs, ys, pred_pose)
        else:
            self.pose = pred_pose

        # 3) 栅格地图更新 (贝叶斯 log-odds); fixed_map 模式下只定位不建图
        if not self.fixed_map:
            self._update_map(xs, ys)

        # 4) 发布地图与 map->odom tf
        self._publish_map()
        self._publish_tf()
        self.scan_cnt += 1
        if self.scan_cnt % 50 == 0:
            occ = (self.logodds > 0).mean()
            rospy.loginfo('scan=%d 地图占用率=%.2f%% 位姿=(%.2f, %.2f, %.1f°)'
                          % (self.scan_cnt, 100 * occ, self.pose[0], self.pose[1],
                             math.degrees(self.pose[2])))

    # ---------------- 位姿变换 ----------------
    def _odom_to_map(self, odom_pose):
        """由里程计增量积分预测地图位姿 (一阶近似)"""
        return self.pose + self._delta(odom_pose)

    def _delta(self, odom_pose):
        """相对上一时刻的里程计增量 (x, y, yaw)"""
        if not hasattr(self, '_last_odom'):
            self._last_odom = odom_pose
            return np.array([0.0, 0.0, 0.0])
        d = odom_pose - self._last_odom
        # 角度回绕
        if d[2] > math.pi: d[2] -= 2 * math.pi
        if d[2] < -math.pi: d[2] += 2 * math.pi
        self._last_odom = odom_pose
        return d

    # ---------------- scan-to-map 匹配 ----------------
    def _scan_match(self, xs, ys, init_pose, iters=4):
        """迭代最近点: 用栅格地图最近被占用点修正位姿
        目标: min Σ || q_i - R(θ)·p_i - t ||²  (q_i 为地图最近占用点)
        邻域取 ±6 栅格 (±0.3 m), 可吸收初始位姿误差; 修正量限幅防发散"""
        pose = init_pose.copy()
        for _ in range(iters):
            # 变换扫描点到地图坐标
            c, s = math.cos(pose[2]), math.sin(pose[2])
            gx = pose[0] + c * xs - s * ys
            gy = pose[1] + s * xs + c * ys
            # 转像素
            px = ((gx + MAP_SIZE / 2) / RES).astype(int)
            py = ((gy + MAP_SIZE / 2) / RES).astype(int)
            ok = (px >= 0) & (px < self.n) & (py >= 0) & (py < self.n)
            if ok.sum() < 20:
                break
            # 只对落在地图内且附近有占用栅格的点做匹配
            occ = self.logodds > 0
            # 以 13x13 邻域内最近占用点作为对应点 q
            qx = np.zeros_like(px, float); qy = np.zeros_like(py, float)
            cnt = 0
            for dx in range(-6, 7):
                for dy in range(-6, 7):
                    nx, ny = px + dx, py + dy
                    m = ok & (nx >= 0) & (nx < self.n) & (ny >= 0) & (ny < self.n) & occ[nx, ny]
                    if m.sum() == 0:
                        continue
                    qx[m] = (nx[m] - self.center) * RES
                    qy[m] = (ny[m] - self.center) * RES
                    cnt += 1
            if cnt == 0:
                break
            # 求解位姿增量 (最小二乘: 旋转+平移)
            p = np.stack([xs, ys], axis=1)                    # (N,2) 机器人系
            q = np.stack([qx, qy], axis=1)                    # (N,2) 地图系对应点
            # 先解平移 (用重心对齐), 再解旋转 (SVD)
            pc = p - p.mean(0); qc = q - q.mean(0)
            H = pc.T @ qc
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            theta = math.atan2(R[1, 0], R[0, 0])
            t = q.mean(0) - R @ p.mean(0)
            # 修正量限幅 (单步不超过 0.4 m / 25°), 防扫描退化导致发散
            scale = min(1.0, 0.4 / max(abs(t[0]), abs(t[1]), abs(theta), 1e-6))
            # 更新 (加入松弛, 防发散)
            pose[2] += 0.4 * theta * scale
            c, s = math.cos(pose[2]), math.sin(pose[2])
            pose[0] += 0.4 * t[0] * scale
            pose[1] += 0.4 * t[1] * scale
        return pose

    # ---------------- 栅格更新 ----------------
    def _update_map(self, xs, ys):
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        gx = self.pose[0] + c * xs - s * ys
        gy = self.pose[1] + s * xs + c * ys
        px = ((gx + MAP_SIZE / 2) / RES).astype(int)
        py = ((gy + MAP_SIZE / 2) / RES).astype(int)
        ok = (px >= 0) & (px < self.n) & (py >= 0) & (py < self.n)
        if ok.sum() == 0:
            return
        # 命中栅格: +l_occ
        self.logodds[py[ok], px[ok]] += L_OCC
        # 射线穿过的栅格: +l_free (Bresenham 简化: 每束取 20 个采样点)
        x0, y0 = int(self.pose[0] / RES + self.center), int(self.pose[1] / RES + self.center)
        for xi, yi in zip(px[ok][::4], py[ok][::4]):
            steps = max(abs(xi - x0), abs(yi - y0), 1)
            for t in np.linspace(0.05, 0.95, min(steps, 24)):
                cx = int(x0 + (xi - x0) * t)
                cy = int(y0 + (yi - y0) * t)
                if 0 <= cx < self.n and 0 <= cy < self.n:
                    self.logodds[cy, cx] += L_FREE
        # 截断避免饱和
        np.clip(self.logodds, -4.0, 8.0, out=self.logodds)

    # ---------------- 发布 ----------------
    def _publish_map(self):
        p = 1.0 / (1.0 + np.exp(-self.logodds))          # log-odds -> 概率 (0~1)
        occ = np.where(p > 0.65, 100, np.where(p < 0.35, 0, -1)).astype(np.int8)
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.info.resolution = RES
        msg.info.width = self.n
        msg.info.height = self.n
        msg.info.origin.position.x = -MAP_SIZE / 2
        msg.info.origin.position.y = -MAP_SIZE / 2
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0              # 单位四元数 (yaw=0)
        msg.data = occ.ravel().tolist()
        self.pub_map.publish(msg)

    def _publish_tf(self):
        """发布 map->odom 变换 (tf2_ros): SLAM 修正后的位姿"""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'      # 父坐标系
        t.child_frame_id = 'odom'      # 子坐标系
        t.transform.translation.x = self.pose[0]
        t.transform.translation.y = self.pose[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.pose[2] / 2)
        t.transform.rotation.w = math.cos(self.pose[2] / 2)
        self.pub_tf.sendTransform(t)


if __name__ == '__main__':
    GridSLAM()
    rospy.spin()
