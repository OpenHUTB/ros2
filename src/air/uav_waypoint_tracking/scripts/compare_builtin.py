#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对照组：用 AirSim 内置位置接口（moveToPositionAsync）飞同一组航点，
与 PID 的结果做对比。日志文件：builtin_<任务名>.csv / _metrics.csv"""

import csv
import math
import os
import time

import rospy
import airsim


def ned_to_enu(v):
    """AirSim NED -> ROS ENU"""
    return (v.y_val, v.x_val, -v.z_val)


def enu_to_ned(x, y, z):
    """ROS ENU -> AirSim NED"""
    return airsim.Vector3r(y, x, -z)


class BuiltinTracker(object):

    def __init__(self):
        g = rospy.get_param
        self.ip = g("/airsim/ip", "192.168.237.1")
        self.vehicle = g("/airsim/vehicle_name", "Drone1")
        self.takeoff_alt = g("/flight/takeoff_altitude", 3.0)
        self.tolerance = g("/flight/waypoint_tolerance", 0.25)
        self.rate_hz = g("/flight/control_rate", 20.0)
        self.speed = g("/flight/max_speed", 2.0)
        self.wp_timeout = g("/flight/waypoint_timeout", 20.0)
        self.settle_time = g("/flight/settle_time", 1.0)
        self.mission = g("/mission", "rectangle")
        self.waypoints = [tuple(float(v) for v in wp)
                          for wp in g("/missions/" + self.mission)]

        log_dir = g("/log_dir", os.path.join(os.path.expanduser("~"),
                                             "uav_waypoint_tracking_logs"))
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)
        tag = "builtin_%s" % self.mission
        self.csv_path = os.path.join(log_dir, tag + ".csv")
        self.metrics_path = os.path.join(log_dir, tag + "_metrics.csv")
        self.csv_file = open(self.csv_path, "w")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["t", "wp", "x_ref", "y_ref", "z_ref",
                              "x", "y", "z", "ex", "ey", "ez", "dist", "phase"])
        self.samples = []
        self.t0 = None
        self.client = None
        self.landed = False

    def connect(self):
        self.client = airsim.MultirotorClient(ip=self.ip, timeout_value=10.0)
        self.client.confirmConnection()
        self.client.enableApiControl(True, self.vehicle)
        self.client.armDisarm(True, self.vehicle)
        self.client.takeoffAsync(vehicle_name=self.vehicle).join()
        self.client.moveToZAsync(-self.takeoff_alt, 1.0,
                                 vehicle_name=self.vehicle).join()
        rospy.loginfo("已起飞到 %.1f 米（对照组：AirSim 内置接口）", self.takeoff_alt)

    def pose(self):
        st = self.client.getMultirotorState(vehicle_name=self.vehicle)
        return ned_to_enu(st.kinematics_estimated.position)

    def fly_to(self, target, index):
        start = self.pose()
        seg = [target[i] - start[i] for i in range(3)]
        seg_len = math.sqrt(sum(v * v for v in seg)) or 1e-6
        seg_dir = [v / seg_len for v in seg]

        ned = enu_to_ned(target[0], target[1], target[2])
        self.client.moveToPositionAsync(ned.x_val, ned.y_val, ned.z_val,
                                        self.speed, timeout_sec=self.wp_timeout,
                                        yaw_mode=airsim.YawMode(False, 0),
                                        vehicle_name=self.vehicle)

        t_start = time.time()
        arrival, settled, max_proj = None, [], 0.0
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            now = time.time()
            pos = self.pose()
            err = [target[i] - pos[i] for i in range(3)]
            dist = math.sqrt(sum(e * e for e in err))
            proj = sum((pos[i] - start[i]) * seg_dir[i] for i in range(3))
            max_proj = max(max_proj, proj)

            phase = "track" if arrival is None else "settle"
            self.writer.writerow([round(now - self.t0, 4), index,
                                  target[0], target[1], target[2],
                                  round(pos[0], 4), round(pos[1], 4), round(pos[2], 4),
                                  round(err[0], 4), round(err[1], 4), round(err[2], 4),
                                  round(dist, 4), phase])
            self.csv_file.flush()
            self.samples.append(pos)

            if arrival is None:
                if dist <= self.tolerance:
                    arrival = now - t_start
                elif now - t_start > self.wp_timeout:
                    rospy.logwarn("航点 %d 超时", index)
                    arrival = float("nan")
                    break
            else:
                settled.append(dist)
                if now - t_start - arrival >= self.settle_time:
                    break

            rate.sleep()

        overshoot = max(0.0, (max_proj - seg_len) / seg_len * 100.0)
        sse = sum(settled) / len(settled) if settled else float("nan")
        return arrival, sse, overshoot

    def path_rms(self):
        """实际轨迹到期望折线的 RMS 误差（总 / 水平 / 高度）"""
        pts = [(0.0, 0.0, self.takeoff_alt)] + list(self.waypoints)
        total, horiz, vert = [], [], []
        for pos in self.samples:
            best = None
            for i in range(len(pts) - 1):
                a, b = pts[i], pts[i + 1]
                ab = [b[k] - a[k] for k in range(3)]
                ab2 = sum(v * v for v in ab) or 1e-9
                ap = [pos[k] - a[k] for k in range(3)]
                t = max(0.0, min(1.0, sum(ap[k] * ab[k] for k in range(3)) / ab2))
                pr = [a[k] + t * ab[k] for k in range(3)]
                d2 = sum((pos[k] - pr[k]) ** 2 for k in range(3))
                dh = math.sqrt(sum((pos[k] - pr[k]) ** 2 for k in (0, 1)))
                if best is None or d2 < best[0]:
                    best = (d2, dh, abs(pos[2] - pr[2]))
            total.append(math.sqrt(best[0]))
            horiz.append(best[1])
            vert.append(best[2])

        def rms(vals):
            if not vals:
                return float("nan")
            return math.sqrt(sum(v * v for v in vals) / len(vals))

        return rms(total), rms(horiz), rms(vert)

    def run(self):
        self.connect()
        self.t0 = time.time()
        results = []
        for i, wp in enumerate(self.waypoints, start=1):
            rospy.loginfo("飞往航点 %d: (%.2f, %.2f, %.2f)", i, wp[0], wp[1], wp[2])
            arrival, sse, overshoot = self.fly_to(wp, i)
            results.append((i, arrival, sse, overshoot))
            rospy.loginfo("  到达 %.2f 秒，稳态误差 %.3f 米，超调 %.1f%%",
                          arrival, sse, overshoot)

        rms_all, rms_h, rms_v = self.path_rms()

        print("")
        print("=============== AirSim 内置位置接口结果 ===============")
        print("任务: %s" % self.mission)
        print("航点   到达时间(s)   稳态误差(m)   超调(%)")
        for i, arrival, sse, overshoot in results:
            print("%4d   %10.2f   %11.3f   %7.1f" % (i, arrival, sse, overshoot))
        print("轨迹 RMS 误差: 总 %.3f m | 水平 %.3f m | 高度 %.3f m" %
              (rms_all, rms_h, rms_v))
        print("采样点数: %d    数据文件: %s" % (len(self.samples), self.csv_path))
        print("======================================================")
        print("")

        with open(self.metrics_path, "w") as f:
            w = csv.writer(f)
            w.writerow(["waypoint", "arrival_time_s",
                        "steady_state_error_m", "overshoot_percent"])
            for row in results:
                w.writerow(row)
            w.writerow(["trajectory_rms_all_m", rms_all])
            w.writerow(["trajectory_rms_horizontal_m", rms_h])
            w.writerow(["trajectory_rms_vertical_m", rms_v])

    def safe_finish(self):
        try:
            if self.csv_file and not self.csv_file.closed:
                self.csv_file.close()
        except Exception:
            pass
        try:
            if self.client is not None and not self.landed:
                self.landed = True
                self.client.landAsync(vehicle_name=self.vehicle).join()
                self.client.armDisarm(False, self.vehicle)
                self.client.enableApiControl(False, self.vehicle)
                rospy.loginfo("已降落")
        except Exception as exc:
            rospy.logwarn("降落时出现问题：%s", exc)


if __name__ == "__main__":
    rospy.init_node("compare_builtin")
    node = BuiltinTracker()
    try:
        node.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("收到中断信号，实验提前结束")
    except Exception as exc:
        rospy.logerr("运行出错：%s", exc)
    finally:
        node.safe_finish()