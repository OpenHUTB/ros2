#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
odom_recorder.py —— 里程计轨迹记录与性能评价
==============================================

【任务书对应】
    - 「(1) 仿真环境时需给出具体的仿真步骤，包括操作步骤和截图」
    - 「推荐使用多个场景进行测试，且包含性能评价」
    - 「支持多种路径规划与控制算法的对比」

【功能说明】
    订阅 /odom 里程计话题，持续记录车辆位姿，在程序结束时输出：
      1) 轨迹总长度 (m)
      2) 累计运行时间 (s)
      3) 平均速度 (m/s)
      4) 最大速度 (m/s)
      5) 平均转向角速度 (rad/s)
      6) 位姿覆盖范围
      7) 完整轨迹点序列（CSV 格式，供绘图与对比分析）

    这些量化指标是任务书「性能评价」要求的直接体现，
    也是不同算法之间横向对比的依据。

【输出文件】
    ~/openhutb_car_results/trajectory_<时间戳>.csv
    格式：timestamp,x,y,yaw,linear_vel,angular_vel

【话题接口】
    订阅：/odom            (nav_msgs/Odometry)
    发布：/odom_recorder/markers  (visualization_msgs/Marker)，可选
          —— 用于在 RViz 中可视化历史轨迹

【作者】OpenHUTB Car Team
"""

from __future__ import print_function

import csv
import math
import os
import threading
import time

import rospy
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

try:
    from visualization_msgs.msg import Marker
    _HAS_MARKER = True
except ImportError:
    _HAS_MARKER = False


#: 结果输出目录
RESULT_DIR = os.path.expanduser("~/openhutb_car_results")

#: 轨迹抽稀距离阈值 (m)。移动超过该距离才记录一个点，避免文件过大。
MIN_SAMPLE_DISTANCE = 0.02

#: 轨迹可视化最大点数（RViz Marker 性能考虑）
MAX_MARKER_POINTS = 2000


class OdomRecorderNode(object):
    """里程计记录与性能统计节点。"""

    def __init__(self, cmd_pub=None, watchdog=None):
        """
        :param cmd_pub:  未使用，仅为与其他节点保持统一接口
        :param watchdog: 未使用，仅为与其他节点保持统一接口
        """
        self._lock = threading.Lock()

        # 位姿与速度
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._v = 0.0
        self._w = 0.0

        self._has_pose = False

        # 轨迹点 (timestamp, x, y, yaw, v, w)
        self._trail = []

        # 统计量
        self._start_time = None
        self._last_time = None
        self._distance = 0.0
        self._v_max = 0.0
        self._w_abs_sum = 0.0
        self._w_samples = 0
        self._prev_x = None
        self._prev_y = None

        # 订阅
        self._sub = rospy.Subscriber("/odom", Odometry, self._odom_callback,
                                     queue_size=10)

        # 轨迹可视化发布者
        self._marker_pub = None
        if _HAS_MARKER:
            self._marker_pub = rospy.Publisher(
                "/odom_recorder/trail", Marker, queue_size=1
            )

        rospy.loginfo("[odom_recorder] 开始记录轨迹")

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def start(self):
        self._start_time = rospy.Time.now()
        rospy.loginfo("[odom_recorder] 轨迹记录已启动")

    def stop(self):
        """结束记录，输出统计并保存 CSV。"""
        self._sub.unregister()
        self._save_csv()
        self._print_report()

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------

    def _odom_callback(self, msg):
        """处理里程计消息，累积统计量。"""
        with self._lock:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation

            self._x = pos.x
            self._y = pos.y

            # 四元数 → 偏航角
            quat = (ori.x, ori.y, ori.z, ori.w)
            euler = tft.euler_from_quaternion(quat)
            self._yaw = euler[2]

            self._v = msg.twist.twist.linear.x
            self._w = msg.twist.twist.angular.z

            now = msg.header.stamp

            if not self._has_pose:
                self._has_pose = True
                self._start_time = now
                self._prev_x = self._x
                self._prev_y = self._y
                self._trail.append((now.to_sec(), self._x, self._y,
                                    self._yaw, self._v, self._w))
                return

            self._last_time = now

            # --- 累积行驶距离 ---
            dx = self._x - self._prev_x
            dy = self._y - self._prev_y
            step = math.hypot(dx, dy)

            if step >= MIN_SAMPLE_DISTANCE:
                self._distance += step
                self._trail.append((now.to_sec(), self._x, self._y,
                                    self._yaw, self._v, self._w))
                self._prev_x = self._x
                self._prev_y = self._y

            # --- 速度统计 ---
            speed = abs(self._v)
            if speed > self._v_max:
                self._v_max = speed

            self._w_abs_sum += abs(self._w)
            self._w_samples += 1

    # ------------------------------------------------------------------
    # 输出
    # ------------------------------------------------------------------

    def _save_csv(self):
        """将轨迹保存为 CSV 文件。"""
        if not self._trail:
            rospy.logwarn("[odom_recorder] 无轨迹数据，跳过保存")
            return

        try:
            if not os.path.isdir(RESULT_DIR):
                os.makedirs(RESULT_DIR)

            stamp = time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(RESULT_DIR,
                                "trajectory_%s.csv" % stamp)

            with open(path, "w") as fp:
                writer = csv.writer(fp)
                writer.writerow(["timestamp", "x", "y", "yaw",
                                 "linear_vel", "angular_vel"])
                for row in self._trail:
                    writer.writerow(["%.4f" % v for v in row])

            rospy.loginfo("[odom_recorder] 轨迹已保存：%s (%d 点)",
                          path, len(self._trail))
        except Exception as exc:  # noqa: BLE001
            rospy.logerr("[odom_recorder] 保存失败：%s", exc)

    def _print_report(self):
        """打印性能评价报告。"""
        with self._lock:
            if not self._has_pose:
                rospy.logwarn("[odom_recorder] 未收到里程计数据")
                return

            duration = 0.0
            if self._last_time is not None and self._start_time is not None:
                duration = (self._last_time - self._start_time).to_sec()

            avg_v = self._distance / duration if duration > 1e-6 else 0.0
            avg_w = (self._w_abs_sum / self._w_samples
                     if self._w_samples > 0 else 0.0)

            xs = [p[1] for p in self._trail]
            ys = [p[2] for p in self._trail]

            report = [
                "",
                "=" * 62,
                "          无人车运行性能评价报告",
                "=" * 62,
                "  运行时长        : %8.2f s" % duration,
                "  轨迹总长度      : %8.2f m" % self._distance,
                "  平均线速度      : %8.3f m/s" % avg_v,
                "  最大线速度      : %8.3f m/s" % self._v_max,
                "  平均角速度      : %8.3f rad/s" % avg_w,
                "  轨迹采样点数    : %8d" % len(self._trail),
                "  X 范围          : %8.2f ~ %.2f m" % (min(xs), max(xs)),
                "  Y 范围          : %8.2f ~ %.2f m" % (min(ys), max(ys)),
                "=" * 62,
                "",
            ]
            for line in report:
                rospy.loginfo(line)

            # 同时打印到标准输出，便于通过日志重定向采集
            print("\n".join(report))
            sys_stdout_flush()

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------

    def update(self):
        """周期性发布轨迹可视化 Marker。"""
        if self._marker_pub is None:
            return

        with self._lock:
            if len(self._trail) < 2:
                return
            trail = self._trail[-MAX_MARKER_POINTS:]
            frame_id = "odom"

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trail"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 1.0

        for _, x, y, _, _, _ in trail:
            from geometry_msgs.msg import Point
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.02
            marker.points.append(p)

        self._marker_pub.publish(marker)


def sys_stdout_flush():
    """刷新标准输出，确保日志被及时写入文件。"""
    import sys
    try:
        sys.stdout.flush()
    except Exception:  # noqa: BLE001
        pass
