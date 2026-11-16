#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务3：栅格建图 + A* 路径规划 + 纯追踪航点跟踪。

输入 /uav/lidar/scan、/uav/odom，输出 /uav/cmd_vel，并写出
    map.csv    占据栅格
    path.csv   A* 规划路径
    track.csv  实际飞行轨迹
"""

import csv
import heapq
import math
import os

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GridNav(object):
    def __init__(self):
        self.res = float(rospy.get_param("~resolution", 0.25))
        self.n = int(rospy.get_param("~grid_size", 80))
        self.map_seconds = float(rospy.get_param("~map_seconds", 15.0))
        self.v = float(rospy.get_param("~v", 1.2))
        self.tol = float(rospy.get_param("~goal_tolerance", 0.6))
        self.goal = (float(rospy.get_param("~goal_x", 8.0)),
                     float(rospy.get_param("~goal_y", 0.0)))
        self.out_dir = os.path.expanduser(
            rospy.get_param("~csv_dir", "~/openhutb_nav_out"))

        self.half = self.n * self.res / 2.0
        self.occ = [[0] * self.n for _ in range(self.n)]
        self.scan = None
        self.pos = None
        self.path = []
        self.track = []
        self.state = "mapping"
        self.t0 = rospy.Time.now()
        if not os.path.isdir(self.out_dir):
            os.makedirs(self.out_dir)

        # 轨迹实时落盘，进程被强杀也不丢数据
        self._track_f = open(os.path.join(self.out_dir, "track.csv"), "w", newline="")
        self._track_w = csv.writer(self._track_f)
        self._track_w.writerow(["t", "x", "y", "yaw"])
        self._track_f.flush()
        self._last_dump = rospy.Time.now()

        rospy.Subscriber("/uav/lidar/scan", LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber("/uav/odom", Odometry, self.on_odom, queue_size=1)
        self.pub = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=1)
        rospy.loginfo("栅格导航启动，目标 (%.1f, %.1f)", self.goal[0], self.goal[1])

    # ---------- 回调 ----------
    def on_scan(self, msg):
        self.scan = msg

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.pos = (p.x, p.y, yaw)
        row = [round((rospy.Time.now() - self.t0).to_sec(), 2),
               round(p.x, 3), round(p.y, 3), round(yaw, 3)]
        self.track.append(row)
        if self._track_w is not None:
            try:
                self._track_w.writerow(row)
                self._track_f.flush()
            except ValueError:
                self._track_w = None

    # ---------- 地图 ----------
    def cell(self, x, y):
        i = int((x + self.half) / self.res)
        j = int((y + self.half) / self.res)
        if 0 <= i < self.n and 0 <= j < self.n:
            return i, j
        return None

    def world(self, c):
        return -self.half + (c[0] + 0.5) * self.res, -self.half + (c[1] + 0.5) * self.res

    def map_scan(self):
        if self.scan is None or self.pos is None:
            return
        px, py, yaw = self.pos
        s = self.scan
        for i in range(0, len(s.ranges), 2):
            r = s.ranges[i]
            if not (r == r) or r <= 0.05 or r >= s.range_max * 0.98:
                continue
            a = yaw + s.angle_min + i * s.angle_increment
            ex = px + math.cos(a) * r
            ey = py + math.sin(a) * r
            c = self.cell(ex, ey)
            if c:
                self.occ[c[1]][c[0]] += 1

    # ---------- 规划 ----------
    def blocked(self, i, j):
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                ni, nj = i + di, j + dj
                if 0 <= ni < self.n and 0 <= nj < self.n and self.occ[nj][ni] > 0:
                    return True
        return False

    def astar(self):
        s = self.cell(self.pos[0], self.pos[1])
        g = self.cell(self.goal[0], self.goal[1])
        if not s or not g:
            return []
        openq = [(0.0, s)]
        came, cost = {}, {s: 0.0}
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == g:
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                return path[::-1]
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nb = (cur[0] + di, cur[1] + dj)
                if not (0 <= nb[0] < self.n and 0 <= nb[1] < self.n):
                    continue
                if self.blocked(nb[0], nb[1]):
                    continue
                nc = cost[cur] + math.hypot(di, dj)
                if nc < cost.get(nb, 1e9):
                    cost[nb] = nc
                    came[nb] = cur
                    h = math.hypot(g[0] - nb[0], g[1] - nb[1])
                    heapq.heappush(openq, (nc + h, nb))
        return []

    def plan(self):
        cells = self.astar()
        self.path = [self.world(c) for c in cells]
        rospy.loginfo("A* 规划完成：%d 个航点", len(self.path))
        with open(os.path.join(self.out_dir, "path.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["x", "y"])
            w.writerows(self.path)

    # ---------- 控制 ----------
    def follow(self, cmd):
        if not self.path:
            return False
        px, py, yaw = self.pos
        tx, ty = self.path[0]
        if math.hypot(tx - px, ty - py) < self.tol:
            self.path.pop(0)
            return bool(self.path)
        err = math.atan2(ty - py, tx - px) - yaw
        while err > math.pi:
            err -= 2 * math.pi
        while err < -math.pi:
            err += 2 * math.pi
        cmd.angular.z = max(-1.0, min(1.0, 1.6 * err))
        cmd.linear.x = self.v * max(0.0, 1.0 - abs(err) / 1.6)
        return True

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            cmd = Twist()
            if self.state == "mapping":
                self.map_scan()
                # 每 5 秒把地图落盘一次，便于中途观察
                if (rospy.Time.now() - self._last_dump).to_sec() > 5.0:
                    self._last_dump = rospy.Time.now()
                    self.save_map()
                if self.pos is not None and \
                        (rospy.Time.now() - self.t0).to_sec() > self.map_seconds:
                    self.plan()
                    self.state = "driving"
            elif self.state == "driving":
                if not self.follow(cmd):
                    self.state = "done"
                    self.save()
                    rospy.loginfo("已到达目标，数据写入 %s", self.out_dir)
            self.pub.publish(cmd)
            rate.sleep()

    def save_map(self):
        try:
            with open(os.path.join(self.out_dir, "map.csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["i", "j", "x", "y"])
                for j in range(self.n):
                    for i in range(self.n):
                        if self.occ[j][i] > 0:
                            x, y = self.world((i, j))
                            w.writerow([i, j, round(x, 3), round(y, 3)])
        except Exception:  # noqa: BLE001
            pass

    def save(self):
        """到达目标或收到停止信号时保存地图（轨迹是实时写盘的，不在这里关文件）。"""
        self.save_map()
        if self._track_f is not None:
            try:
                self._track_f.flush()
            except ValueError:
                pass

    def close(self):
        self.save()
        if self._track_f is not None:
            try:
                self._track_f.close()
            except Exception:  # noqa: BLE001
                pass
            self._track_f = None
            self._track_w = None


def main():
    rospy.init_node("uav_grid_nav")
    node = GridNav()
    rospy.on_shutdown(node.close)
    node.spin()


if __name__ == "__main__":
    main()
