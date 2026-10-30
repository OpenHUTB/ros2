#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""里程计标定主节点（ROS 1 / Noetic）。

流程：
    1) 直行实验 2 m  → 辨识路程系数 c_s 与直行漂移率 k_head
    2) 原地旋转 3 圈 → 辨识转角系数 c_theta
    3) 方形路径 4×4 m 验证 → 对比标定前后的位置/航向漂移
    4) 把结果写到 config/calib_result.yaml，并在终端打印对照表

用法：
    roslaunch diff_drive_odom_calib main.launch        # 仿真 + 标定一条龙
    rosrun  diff_drive_odom_calib main.py              # 同上（需另起 sim_robot.py）
    python3 scripts/main.py --selftest                 # 纯逻辑自检，不需要 roscore

为什么把标定做成"跑两条固定轨迹 + 读外部真值"：
    这正是 UMBmark 一类现场标定法的做法 —— 机器人自己不知道走了多远，
    必须有一个独立于里程计的基准。仿真里这个基准就是 /ground_truth。
"""

import argparse
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
try:
    from diff_drive_odom_calib.model import (DiffDrive, OdometryIntegrator,
                                             _se2_step, pose_difference, wrap_angle)
    from diff_drive_odom_calib import calibration as calib
except ImportError:                                          # pragma: no cover
    sys.path.insert(0, os.path.join(_HERE, "..", "src"))
    from diff_drive_odom_calib.model import (DiffDrive, OdometryIntegrator,
                                             _se2_step, pose_difference, wrap_angle)
    from diff_drive_odom_calib import calibration as calib


NOMINAL = dict(c_s=1.0, c_theta=1.0, k_head=0.0)


def log_to_pose(log):
    """把逐帧 (ds, dth) 真值增量积回终点位姿。"""
    pose = (0.0, 0.0, 0.0)
    for o in log:
        pose = _se2_step(pose, o["ds_true"], o["dth_true"])
    return pose


def integrate_log(log, params, r_nominal=0.05, b_nominal=0.30):
    od = OdometryIntegrator(c_s=params.get("c_s", 1.0),
                            c_theta=params.get("c_theta", 1.0),
                            k_head=params.get("k_head", 0.0),
                            r_nominal=r_nominal, b_nominal=b_nominal)
    od.reset()
    for o in log:
        od.step(o["dphi_left"], o["dphi_right"])
    return od.pose


def report_compare(log, params, r_nominal=0.05, b_nominal=0.30):
    """同一段轨迹，标定前 vs 标定后的终点误差。"""
    true_pose = log_to_pose(log)
    out = {}
    for tag, prm in (("before", NOMINAL), ("after", params)):
        est = integrate_log(log, prm, r_nominal, b_nominal)
        ex, ey, eth = pose_difference(est, true_pose)
        path = sum(o["ds_true"] for o in log)
        out[tag] = dict(err_x=ex, err_y=ey, err_theta=eth,
                        err_pos=math.hypot(ex, ey), path=path,
                        pct=100.0 * math.hypot(ex, ey) / max(path, 1e-9))
    return out


# ----------------------------------------------------------------------
# 纯逻辑自检（不需要 roscore，也不订阅任何话题）
# ----------------------------------------------------------------------
def selftest():
    dt = 0.02
    print("=" * 68)
    print("纯逻辑自检：用内置运动学模型跑完整标定流程")
    print("=" * 68)
    robot = DiffDrive(r_left=0.0505, r_right=0.0495, b_true=0.306,
                      slip_std=0.004, seed=1)
    print("真值参数：r_left=0.0505 r_right=0.0495 b_true=0.306（标称 0.05 / 0.30）")
    st = calib.straight_log(robot, dt, distance=2.0, v=0.2)
    ro = calib.rotation_log(robot, dt, turns=3.0, w=1.0)
    prm = calib.identify(st, ro)
    print("\n[辨识结果]")
    print("  c_s     = %.5f   （路程系数，真值 %.5f）" % (prm["c_s"], (0.0505 + 0.0495) / 2 / 0.05))
    print("  k_head  = %+.5f rad/m（直行漂移率，真值 %+.5f）"
          % (prm["k_head"], (0.0495 - 0.0505) / (0.05 * 0.306)))
    print("  c_theta = %.5f   （转角系数，真值 %.5f）" % (prm["c_theta"], 0.30 / 0.306))

    sq = calib.square_cmd(side=4.0, v=0.2, w=1.0, turns=4)
    b = calib.evaluate(robot, NOMINAL, dt, sq)
    a = calib.evaluate(robot, prm, dt, sq)
    print("\n[方形路径 4x4 m 验证]")
    print("  标定前：位置误差 %.3f m  航向误差 %+7.2f°  路径 %.1f m"
          % (b["err_pos"], math.degrees(b["err_theta"]), b["path_len"]))
    print("  标定后：位置误差 %.3f m  航向误差 %+7.2f°"
          % (a["err_pos"], math.degrees(a["err_theta"])))
    print("  改善：位置误差降到 %.1f%%，航向误差降到 %.1f%%"
          % (100.0 * a["err_pos"] / max(b["err_pos"], 1e-9),
             100.0 * abs(a["err_theta"]) / max(abs(b["err_theta"]), 1e-9)))
    return 0


# ----------------------------------------------------------------------
# ROS 节点
# ----------------------------------------------------------------------
class CalibNode(object):

    def __init__(self):
        import rospy
        from geometry_msgs.msg import PoseStamped, Twist
        from std_msgs.msg import Float32MultiArray
        self.rospy = rospy
        self.Twist = Twist
        self.rate_hz = 50.0
        self.r_nominal = float(rospy.get_param("~r_nominal", 0.05))
        self.b_nominal = float(rospy.get_param("~b_nominal", 0.30))
        self.straight_distance = float(rospy.get_param("~straight_distance", 2.0))
        self.straight_speed = float(rospy.get_param("~straight_speed", 0.20))
        self.rotation_turns = float(rospy.get_param("~rotation_turns", 3.0))
        self.rotation_speed = float(rospy.get_param("~rotation_speed", 1.00))
        self.square_side = float(rospy.get_param("~square_side", 4.0))
        self.result_file = rospy.get_param("~result_file", "")

        self.truth = None
        self.ticks = None
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/ground_truth", PoseStamped, self.on_truth, queue_size=50)
        rospy.Subscriber("/wheel_ticks", Float32MultiArray, self.on_ticks, queue_size=50)


    def on_truth(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)
        self.truth = (msg.pose.position.x, msg.pose.position.y, yaw)

    def on_ticks(self, msg):
        self.ticks = (float(msg.data[0]), float(msg.data[1]))

    def wait_ready(self, timeout=10.0):
        rospy = self.rospy
        t0 = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.truth is not None and self.ticks is not None:
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout:
                return False
            rate.sleep()
        return False

    def capture(self, cmd_seq):
        """执行一段指令序列并记录逐帧真值增量与编码器增量。"""
        rospy = self.rospy
        log = []
        for v, w, duration in cmd_seq:
            tw = self.Twist()
            tw.linear.x = v
            tw.angular.z = w
            self.cmd_pub.publish(tw)
            rospy.sleep(0.1)
            prev = (self.truth, self.ticks)
            t0 = rospy.Time.now()
            rate = rospy.Rate(self.rate_hz)
            while (rospy.Time.now() - t0).to_sec() < duration and not rospy.is_shutdown():
                cur = (self.truth, self.ticks)
                if prev[0] is not None and cur[0] is not None:
                    dx = cur[0][0] - prev[0][0]
                    dy = cur[0][1] - prev[0][1]
                    ds = math.hypot(dx, dy)
                    dth = wrap_angle(cur[0][2] - prev[0][2])
                    log.append(dict(ds_true=ds, dth_true=dth,
                                    dphi_left=cur[1][0] - prev[1][0],
                                    dphi_right=cur[1][1] - prev[1][1]))
                prev = cur
                rate.sleep()
        self.cmd_pub.publish(self.Twist())
        rospy.sleep(0.2)
        return log

    def run(self):
        rospy = self.rospy
        rospy.loginfo("等待仿真机器人发布 /ground_truth 与 /wheel_ticks ...")
        if not self.wait_ready():
            rospy.logerr("超时：没有收到仿真机器人的话题。先启动 sim_robot.py？")
            return 1

        rospy.loginfo("① 直行实验 %.1f m @ %.2f m/s",
                      self.straight_distance, self.straight_speed)
        st = self.capture([(self.straight_speed, 0.0,
                            self.straight_distance / self.straight_speed)])
        rospy.loginfo("② 原地旋转 %.1f 圈 @ %.2f rad/s",
                      self.rotation_turns, self.rotation_speed)
        ro = self.capture([(0.0, self.rotation_speed,
                            self.rotation_turns * 2.0 * math.pi / self.rotation_speed)])

        prm = calib.identify(st, ro, r_nominal=self.r_nominal, b_nominal=self.b_nominal)
        print("\n" + "=" * 68)
        print("辨识结果（写回里程计积分器即可使用）")
        print("=" * 68)
        print("  c_s     = %.5f   路程系数（修正平均轮径）" % prm["c_s"])
        print("  k_head  = %+.5f rad/m  直行漂移率（修正左右轮不等）" % prm["k_head"])
        print("  c_theta = %.5f   转角系数（修正轮距）" % prm["c_theta"])

        rospy.loginfo("③ 方形路径 %.1f m × 4 验证", self.square_side)
        sq = calib.square_cmd(side=self.square_side, v=self.straight_speed,
                              w=self.rotation_speed, turns=4)
        log_sq = self.capture(sq)
        cmp_ = report_compare(log_sq, prm, self.r_nominal, self.b_nominal)
        print("\n" + "=" * 68)
        print("方形路径验证（同一段轨迹，只换里程计参数）")
        print("=" * 68)
        for tag, name in (("before", "标定前"), ("after", "标定后")):
            r = cmp_[tag]
            print("  %s：位置误差 %.3f m（占路径 %.1f%%）  航向误差 %+7.2f°  "
                  "Δx=%+.3f Δy=%+.3f"
                  % (name, r["err_pos"], r["pct"], math.degrees(r["err_theta"]),
                     r["err_x"], r["err_y"]))
        print("  改善：位置 %.1f%% → %.1f%%（路径占比）"
              % (cmp_["before"]["pct"], cmp_["after"]["pct"]))

        out = self.result_file or os.path.join(
            os.path.dirname(_HERE), "config", "calib_result.yaml")
        try:
            with open(out, "w", encoding="utf-8") as f:
                f.write("# 由 diff_drive_odom_calib 自动生成\n")
                f.write("c_s: %.6f\nk_head: %.6f\nc_theta: %.6f\n"
                        % (prm["c_s"], prm["k_head"], prm["c_theta"]))
            print("\n[导出] %s" % out)
        except Exception as exc:
            print("\n[提示] 结果文件写入失败（%s），参数见上面终端输出" % exc)
        return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="差速机器人里程计标定")
    ap.add_argument("--selftest", action="store_true", help="纯逻辑自检，不需要 roscore")
    args, _unknown = ap.parse_known_args(argv)
    if args.selftest:
        return selftest()

    import rospy
    rospy.init_node("odom_calib")
    return CalibNode().run()


if __name__ == "__main__":
    sys.exit(main())
