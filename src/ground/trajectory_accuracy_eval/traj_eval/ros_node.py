#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 1 (Noetic) 在线评估节点。

订阅两条轨迹，按时间戳配对后实时计算 ATE / RPE，并定期打印与发布：

订阅
----
* ``/ground_truth`` (nav_msgs/Odometry)  真值轨迹（通常来自仿真器或高精度定位）
* ``/odom``        (nav_msgs/Odometry)  待评估的里程计输出

发布
----
* ``/traj_eval/metrics`` (std_msgs/Float64MultiArray)
  ``[ATE_RMSE, ATE_MAX, RPE_delta_trans_RMSE, 配对数]``

退出（Ctrl+C）时把两条轨迹与指标写入 ``~output_dir``，便于离线复核。

说明：本节点不做仿真，只消费话题；仓库里配套的
``scripts/sim_odom_publisher.py`` 可以生成一对"真值 + 带误差里程计"用于演示。
"""

from __future__ import annotations

import bisect
import math
import os

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float64MultiArray

from traj_eval import Pose, ate, rpe, save_trajectory


def _stamp(msg) -> float:
    return msg.header.stamp.to_sec()


def _pose_of(msg) -> Pose:
    p = msg.pose.pose
    q = p.orientation
    # 只取绕 z 轴的分量（地面车辆）
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return Pose(float(p.position.x), float(p.position.y), yaw)


class TrajEvalNode:
    """配对两条轨迹并在线评估。"""

    def __init__(self) -> None:
        rospy.init_node("traj_eval_node")

        self.match_tol = float(rospy.get_param("~match_tolerance", 0.05))
        self.report_period = float(rospy.get_param("~report_period", 3.0))
        self.rpe_delta = int(rospy.get_param("~rpe_delta", 10))
        self.min_pairs = int(rospy.get_param("~min_pairs", 20))
        self.output_dir = str(rospy.get_param("~output_dir", os.path.expanduser("~/traj_eval_out")))
        self.align = bool(rospy.get_param("~align", True))

        self.gt_buf: list[tuple[float, Pose]] = []
        self.est_buf: list[tuple[float, Pose]] = []
        self.gt_stamps: list[float] = []
        self.est_stamps: list[float] = []
        self.pairs = 0
        # 用浮点秒（rospy.get_time）而不是 rospy.Time，便于离线单元测试
        self.last_report = rospy.get_time()

        self.pub = rospy.Publisher("/traj_eval/metrics", Float64MultiArray, queue_size=1)
        rospy.Subscriber("/ground_truth", Odometry, self._on_gt, queue_size=50)
        rospy.Subscriber("/odom", Odometry, self._on_est, queue_size=50)
        # 演示用的循环回放会发这个信号：清空缓冲，从新一轮开始重新评估
        rospy.Subscriber("/traj_eval/reset", Empty, self._on_reset, queue_size=1)

        rospy.loginfo("traj_eval_node 已启动: 订阅 /ground_truth 与 /odom，配对容差 %.3f s",
                      self.match_tol)
        rospy.loginfo("每 %.1f s 报告一次 ATE/RPE；Ctrl+C 退出并保存结果到 %s",
                      self.report_period, self.output_dir)
        rospy.Timer(rospy.Duration(0.5), self._on_timer)

    # ------------------------------------------------------------------ 回调
    def _on_gt(self, msg: Odometry) -> None:
        t = _stamp(msg)
        self.gt_buf.append((t, _pose_of(msg)))
        self.gt_stamps.append(t)
        self._try_pair()

    def _on_est(self, msg: Odometry) -> None:
        t = _stamp(msg)
        self.est_buf.append((t, _pose_of(msg)))
        self.est_stamps.append(t)
        self._try_pair()

    def _on_reset(self, _msg: Empty) -> None:
        """收到重置信号：清空缓冲，从新一轮轨迹重新开始评估。

        典型场景是演示数据源循环回放——若不清空，两圈轨迹会攒在同一个缓冲区里，
        循环处"瞬间跳回起点"会被当成真实运动，使 RPE 偏大数倍、ATE 也不再从 0 增长。
        """
        n = len(self.gt_buf)
        self.gt_buf.clear()
        self.est_buf.clear()
        self.gt_stamps.clear()
        self.est_stamps.clear()
        self.last_report = rospy.get_time()
        rospy.loginfo("收到重置信号，已清空 %d 组缓存，重新开始评估", n)

    def _try_pair(self) -> None:
        """把已到达的两侧消息按时间戳排序，用最近邻配对。"""
        if not self.gt_buf or not self.est_buf:
            return
        self.gt_buf.sort(key=lambda x: x[0])
        self.est_buf.sort(key=lambda x: x[0])
        self.gt_stamps = [t for t, _ in self.gt_buf]
        self.est_stamps = [t for t, _ in self.est_buf]

    def _matched_pairs(self) -> tuple[list[Pose], list[Pose]]:
        """返回按时间戳对齐后的 (gt, est) 位姿序列。"""
        if not self.gt_buf or not self.est_buf:
            return [], []
        self.gt_buf.sort(key=lambda x: x[0])
        self.est_buf.sort(key=lambda x: x[0])
        gt_ts = [t for t, _ in self.gt_buf]
        est_ts = [t for t, _ in self.est_buf]

        gt_out: list[Pose] = []
        est_out: list[Pose] = []
        for t, p in self.est_buf:
            i = bisect.bisect_left(gt_ts, t)
            best = None
            best_dt = self.match_tol
            for j in (i - 1, i):
                if 0 <= j < len(gt_ts):
                    dt = abs(gt_ts[j] - t)
                    if dt <= best_dt:
                        best_dt = dt
                        best = j
            if best is not None:
                gt_out.append(self.gt_buf[best][1])
                est_out.append(p)
        _ = est_ts
        return gt_out, est_out

    def _on_timer(self, _event) -> None:
        now = rospy.get_time()
        if now - self.last_report < self.report_period:
            return
        self.last_report = now

        gt, est = self._matched_pairs()
        if len(gt) < self.min_pairs:
            rospy.loginfo_throttle(5.0, "已配对 %d 组（至少需要 %d 组才开始评估）",
                                   len(gt), self.min_pairs)
            return

        a = ate(est, gt, align=self.align)
        r = rpe(est, gt, delta=self.rpe_delta) if len(gt) > self.rpe_delta else None
        rospy.loginfo("配对 %d 组 | ATE RMSE %.4f m (max %.4f) | RPE(%d) %.4f m",
                      len(gt), a.rmse, a.max, self.rpe_delta,
                      r.trans_rmse if r else float("nan"))

        msg = Float64MultiArray()
        msg.data = [a.rmse, a.max, (r.trans_rmse if r else float("nan")), float(len(gt))]
        self.pub.publish(msg)

    def save(self) -> None:
        gt, est = self._matched_pairs()
        if len(gt) < 2:
            rospy.logwarn("配对数不足，未保存结果")
            return
        os.makedirs(self.output_dir, exist_ok=True)
        save_trajectory(os.path.join(self.output_dir, "ground_truth.csv"), gt)
        save_trajectory(os.path.join(self.output_dir, "odometry.csv"), est)
        a = ate(est, gt, align=self.align)
        rospy.loginfo("已保存 %d 组配对到 %s | 最终 ATE RMSE = %.4f m", len(gt), self.output_dir, a.rmse)


def main() -> None:
    node = TrajEvalNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        rospy.loginfo("traj_eval_node 退出")


if __name__ == "__main__":
    main()
