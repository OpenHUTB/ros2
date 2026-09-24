#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四旋翼航点轨迹跟踪：PID 控制器（主节点）

流程：
  1. 连接宿主机上的 AirSim 模拟器（RPC 端口 41451）；
  2. 解锁并起飞到设定高度，稳定悬停；
  3. 依次飞往 params.yaml 中配置的航点：位置误差经 PID 换算为速度指令；
  4. 按控制频率记录期望位置、实际位置与误差到 CSV；
  5. 统计每个航点的到达时间、稳态误差、超调量，以及全程轨迹 RMS 误差；
  6. 向 RViz 发布期望轨迹、实际轨迹与当前位姿。

坐标系：
  AirSim 内部使用 NED（北-东-地），ROS 使用 ENU（东-北-天）。
  ENU = (NED.y, NED.x, -NED.z)，该变换的逆变换形式相同。
  参数文件中的航点统一使用 ENU（x 向东、y 向北、z 向上），可直接送 RViz 显示。
"""

import csv
import math
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64

import airsim


# ------------------------------------------------------------------ 坐标换算
def ned_to_enu(vec):
    """AirSim 的 NED 向量 -> ROS 的 ENU 元组"""
    return (vec.y_val, vec.x_val, -vec.z_val)


def enu_to_ned(x, y, z):
    """ROS 的 ENU 坐标（位置或速度）-> AirSim 的 NED 向量"""
    return airsim.Vector3r(y, x, -z)


# ------------------------------------------------------------------ PID 控制器
class PID(object):
    """单轴 PID：带积分限幅（抗积分饱和）与输出限幅（限制速度指令）"""

    def __init__(self, kp, ki, kd, integral_limit, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.integral = 0.0
        self.last_error = None

    def update(self, error, dt):
        if dt <= 0.0:
            dt = 1e-3
        # 积分项限幅
        self.integral += error * dt
        self.integral = max(-self.integral_limit,
                            min(self.integral_limit, self.integral))
        # 微分项：第一个周期没有历史误差，置零以避免尖峰
        if self.last_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.last_error) / dt
        self.last_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))


# ------------------------------------------------------------------ 跟踪节点
class WaypointTracker(object):

    def __init__(self):
        # ---- 参数
        self.ip = rospy.get_param("/airsim/ip", "192.168.237.1")
        self.vehicle = rospy.get_param("/airsim/vehicle_name", "Drone1")
        self.timeout_value = rospy.get_param("/airsim/timeout", 10.0)

        self.takeoff_alt = rospy.get_param("/flight/takeoff_altitude", 3.0)
        self.tolerance = rospy.get_param("/flight/waypoint_tolerance", 0.25)
        self.rate_hz = rospy.get_param("/flight/control_rate", 20.0)
        self.max_speed = rospy.get_param("/flight/max_speed", 2.0)
        self.max_vz = rospy.get_param("/flight/max_vertical_speed", 1.0)
        self.wp_timeout = rospy.get_param("/flight/waypoint_timeout", 20.0)
        self.settle_time = rospy.get_param("/flight/settle_time", 1.0)

        self.kp = rospy.get_param("/pid/kp", 1.0)
        self.ki = rospy.get_param("/pid/ki", 0.05)
        self.kd = rospy.get_param("/pid/kd", 0.2)
        self.ilim = rospy.get_param("/pid/integral_limit", 1.0)

        self.mission = rospy.get_param("/mission", "rectangle")
        self.waypoints = [tuple(float(v) for v in wp)
                          for wp in rospy.get_param("/missions/" + self.mission)]

        # ---- 日志文件（文件名带上增益，便于参数整定对比）
        log_dir = rospy.get_param(
            "/log_dir",
            os.path.join(os.path.expanduser("~"), "uav_waypoint_tracking_logs"))
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)
        tag = "pid_%s_kp%.2f_ki%.2f_kd%.2f" % (self.mission, self.kp, self.ki, self.kd)
        self.csv_path = os.path.join(log_dir, tag + ".csv")
        self.metrics_path = os.path.join(log_dir, tag + "_metrics.csv")
        self.csv_file = open(self.csv_path, "w")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["t", "wp", "x_ref", "y_ref", "z_ref",
                              "x", "y", "z", "ex", "ey", "ez", "dist", "phase"])
        self.csv_file.flush()

        # ---- RViz 话题
        self.pub_desired = rospy.Publisher("~path_desired", Path, queue_size=1, latch=True)
        self.pub_actual = rospy.Publisher("~path_actual", Path, queue_size=1)
        self.pub_pose = rospy.Publisher("~pose", PoseStamped, queue_size=1)
        self.pub_error = rospy.Publisher("~error", Float64, queue_size=10)

        self.actual_path = Path()
        self.actual_path.header.frame_id = "world"
        self.desired_path = Path()
        self.desired_path.header.frame_id = "world"

        self.samples = []          # 全部采样，用于计算轨迹 RMS 误差
        self.publish_every = 5     # 每 5 个采样发布一次实际轨迹，避免消息过大
        self.step_count = 0
        self.t0 = None
        self.landed = False
        self.client = None

    # -------------------------------------------------------------- 基础动作
    def connect(self):
        self.client = airsim.MultirotorClient(
            ip=self.ip, timeout_value=self.timeout_value)
        self.client.confirmConnection()
        rospy.loginfo("已连接 AirSim: %s (%s)", self.ip, self.vehicle)
        self.client.enableApiControl(True, self.vehicle)
        self.client.armDisarm(True, self.vehicle)
        self.client.takeoffAsync(vehicle_name=self.vehicle).join()
        self.client.moveToZAsync(-self.takeoff_alt, 1.0,
                                 vehicle_name=self.vehicle).join()
        rospy.loginfo("已起飞并稳定在 %.1f 米", self.takeoff_alt)

    def pose_enu(self):
        state = self.client.getMultirotorState(vehicle_name=self.vehicle)
        return ned_to_enu(state.kinematics_estimated.position)

    def send_velocity(self, v_enu):
        """把 ENU 速度指令发给 AirSim（内部转换为 NED 世界坐标系）"""
        v_ned = enu_to_ned(v_enu[0], v_enu[1], v_enu[2])
        self.client.moveByVelocityAsync(
            v_ned.x_val, v_ned.y_val, v_ned.z_val, 0.5,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(False, 0),
            vehicle_name=self.vehicle)

    def publish_paths(self, pos, target):
        stamp = rospy.Time.now()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "world"
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)

        self.pub_error.publish(math.sqrt(sum((a - b) ** 2 for a, b in zip(pos, target))))

        self.actual_path.poses.append(pose)
        self.step_count += 1
        if self.step_count % self.publish_every == 0:
            self.actual_path.header.stamp = stamp
            self.pub_actual.publish(self.actual_path)

    # -------------------------------------------------------------- 单个航点
    def fly_to(self, target, index):
        pids = [PID(self.kp, self.ki, self.kd, self.ilim, self.max_speed),
                PID(self.kp, self.ki, self.kd, self.ilim, self.max_speed),
                PID(self.kp, self.ki, self.kd, self.ilim, self.max_vz)]

        start = self.pose_enu()
        seg_vec = tuple(target[i] - start[i] for i in range(3))
        seg_len = math.sqrt(sum(v * v for v in seg_vec)) or 1e-6
        seg_dir = tuple(v / seg_len for v in seg_vec)

        t_start = time.time()
        last_t = t_start
        arrival_t = None
        settled = []
        max_proj = 0.0
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            now = time.time()
            dt = now - last_t
            last_t = now

            pos = self.pose_enu()
            err = tuple(target[i] - pos[i] for i in range(3))
            dist = math.sqrt(sum(e * e for e in err))

            # 沿进近方向的投影，用于计算超调量
            proj = sum((pos[i] - start[i]) * seg_dir[i] for i in range(3))
            max_proj = max(max_proj, proj)

            v = (pids[0].update(err[0], dt),
                 pids[1].update(err[1], dt),
                 pids[2].update(err[2], dt))
            self.send_velocity(v)

            phase = "track" if arrival_t is None else "settle"
            self.writer.writerow([round(now - self.t0, 4), index,
                                  target[0], target[1], target[2],
                                  round(pos[0], 4), round(pos[1], 4), round(pos[2], 4),
                                  round(err[0], 4), round(err[1], 4), round(err[2], 4),
                                  round(dist, 4), phase])
            self.csv_file.flush()

            self.samples.append(pos)
            self.publish_paths(pos, target)

            if arrival_t is None:
                if dist <= self.tolerance:
                    arrival_t = now - t_start
                elif now - t_start > self.wp_timeout:
                    rospy.logwarn("航点 %d 超时（%.1f 秒），当前距离 %.2f 米",
                                  index, self.wp_timeout, dist)
                    arrival_t = float("nan")
                    break
            else:
                settled.append(dist)
                if now - t_start - arrival_t >= self.settle_time:
                    break

            rate.sleep()

        overshoot = max(0.0, (max_proj - seg_len) / seg_len * 100.0)
        sse = sum(settled) / len(settled) if settled else float("nan")
        return arrival_t, sse, overshoot

    # -------------------------------------------------------------- 轨迹误差
    def trajectory_rms(self):
        """实际轨迹到期望折线路径的距离：总 RMS、水平分量、高度分量"""
        points = [(0.0, 0.0, self.takeoff_alt)] + list(self.waypoints)
        total, horiz, vert = [], [], []
        for pos in self.samples:
            best = None
            for i in range(len(points) - 1):
                a, b = points[i], points[i + 1]
                ab = tuple(b[k] - a[k] for k in range(3))
                ab_len2 = sum(v * v for v in ab) or 1e-9
                ap = tuple(pos[k] - a[k] for k in range(3))
                t = max(0.0, min(1.0,
                                 sum(ap[k] * ab[k] for k in range(3)) / ab_len2))
                proj = tuple(a[k] + t * ab[k] for k in range(3))
                d2 = sum((pos[k] - proj[k]) ** 2 for k in range(3))
                dh = math.sqrt(sum((pos[k] - proj[k]) ** 2 for k in (0, 1)))
                dv = abs(pos[2] - proj[2])
                if best is None or d2 < best[0]:
                    best = (d2, dh, dv)
            total.append(math.sqrt(best[0]))
            horiz.append(best[1])
            vert.append(best[2])

        def rms(values):
            if not values:
                return float("nan")
            return math.sqrt(sum(v * v for v in values) / len(values))

        return rms(total), rms(horiz), rms(vert)

    # -------------------------------------------------------------- 主流程
    def run(self):
        self.connect()

        # 期望轨迹（航点折线）先发布一次并锁存，RViz 里始终可见
        for p in [(0.0, 0.0, self.takeoff_alt)] + list(self.waypoints):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "world"
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]
            pose.pose.orientation.w = 1.0
            self.desired_path.poses.append(pose)
        self.pub_desired.publish(self.desired_path)

        self.t0 = time.time()
        results = []
        for i, wp in enumerate(self.waypoints, start=1):
            rospy.loginfo("飞往航点 %d: (%.2f, %.2f, %.2f)", i, wp[0], wp[1], wp[2])
            arrival, sse, overshoot = self.fly_to(wp, i)
            results.append((i, arrival, sse, overshoot))
            rospy.loginfo("  到达用时 %.2f 秒，稳态误差 %.3f 米，超调 %.1f%%",
                          arrival, sse, overshoot)

        rms_all, rms_h, rms_v = self.trajectory_rms()

        print("")
        print("==================== PID 航点跟踪结果 ====================")
        print("任务: %s   PID 增益: kp=%.2f ki=%.2f kd=%.2f" %
              (self.mission, self.kp, self.ki, self.kd))
        print("航点   到达时间(s)   稳态误差(m)   超调(%)")
        for i, arrival, sse, overshoot in results:
            print("%4d   %10.2f   %11.3f   %7.1f" % (i, arrival, sse, overshoot))
        print("轨迹 RMS 误差: 总 %.3f m | 水平 %.3f m | 高度 %.3f m" %
              (rms_all, rms_h, rms_v))
        print("采样点数: %d    数据文件: %s" % (len(self.samples), self.csv_path))
        print("==========================================================")
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
        """无论正常结束还是中途中断，都关闭日志并降落"""
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
    rospy.init_node("pid_tracker")
    node = WaypointTracker()
    try:
        node.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("收到中断信号，实验提前结束")
    except Exception as exc:
        rospy.logerr("运行出错：%s", exc)
    finally:
        node.safe_finish()