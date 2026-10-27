#!/usr/bin/env python3
"""Main entry point for the tracked-vehicle MLP teleoperation demo.

Modes
-----
keyboard : W/S/A/D teleoperation, 1/2/3 switch RAW / PI / MLP controller.
collect  : random command profile driven by the PI expert, records a CSV used
           by train_mlp.py (behaviour cloning).
eval     : resets the world, runs a fixed command profile for each controller,
           records per-tick CSV and writes a summary row per controller.

The node always publishes geometry_msgs/Twist on /cmd_vel, subscribes to /odom
and /imu, and uses world-frame odometry for the measured velocity.
"""
import csv
import math
import os
import select
import sys
import termios
import threading
import time
import tty

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import (  # noqa: E402
    FEATURE_NAMES,
    NeuralController,
    PIExpert,
    package_path,
)

ROW_HEADER = [
    "t", "scene", "mode", "phase",
    "v_target", "w_target", "v_cmd", "w_cmd",
    "v_meas", "w_meas", "pitch", "roll", "yaw_rate",
    "e_v", "e_w", "int_v", "int_w", "latency_ms",
]
SUMMARY_HEADER = [
    "world", "controller", "samples", "duration_s",
    "rmse_v", "rmse_w", "mae_v", "mae_w", "mean_abs_slip",
    "distance_m", "max_roll_deg", "max_pitch_deg",
    "latency_mean_ms", "latency_p95_ms", "success",
]


def _clip(value, low, high):
    return float(np.clip(value, low, high))


def _now():
    stamp = rospy.Time.now().to_sec()
    return stamp if stamp > 0.0 else time.time()


class KeyboardReader(threading.Thread):
    """Read single keys from /dev/tty without blocking the control loop."""

    def __init__(self, callback, logger):
        super().__init__(daemon=True)
        self.callback = callback
        self.logger = logger
        self.stop_event = threading.Event()
        self.fd = None

    def run(self):
        try:
            self.fd = os.open("/dev/tty", os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            self.logger.warn("无法打开 /dev/tty，键盘控制不可用: %s", exc)
            return

        old_attrs = termios.tcgetattr(self.fd)
        try:
            tty.setcbreak(self.fd)
            while not self.stop_event.is_set() and not rospy.is_shutdown():
                ready, _, _ = select.select([self.fd], [], [], 0.1)
                if not ready:
                    continue
                try:
                    key = os.read(self.fd, 1).decode("utf-8", errors="ignore")
                except OSError:
                    break
                if key:
                    self.callback(key)
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, old_attrs)
            os.close(self.fd)

    def stop(self):
        self.stop_event.set()


class TrackedVehicleNode:
    def __init__(self):
        self.mode = str(rospy.get_param("~mode", "keyboard")).lower()
        self.controller = str(rospy.get_param("~controller", "mlp")).lower()
        self.duration = float(rospy.get_param("~duration", 60.0))
        self.world = str(rospy.get_param("~world", "flat"))
        self.output_csv = str(rospy.get_param("~output_csv", ""))
        self.requested_output_csv = self.output_csv
        self.summary_csv = str(rospy.get_param("~summary_csv", ""))
        self.keyboard_enabled = bool(rospy.get_param("~keyboard", True)) and self.mode == "keyboard"
        self.record_keyboard = bool(rospy.get_param("~record_keyboard", False))

        self.max_v = float(rospy.get_param("~vehicle/max_linear_velocity", 0.90))
        self.max_w = float(rospy.get_param("~vehicle/max_angular_velocity", 1.60))
        self.control_rate = float(rospy.get_param("~vehicle/control_rate", 20.0))
        self.odom_timeout = float(rospy.get_param("~vehicle/odom_timeout", 1.0))

        self.expert = PIExpert(
            kp_v=rospy.get_param("~expert/kp_v", 1.10),
            ki_v=rospy.get_param("~expert/ki_v", 1.60),
            kp_w=rospy.get_param("~expert/kp_w", 1.80),
            ki_w=rospy.get_param("~expert/ki_w", 3.20),
            limit_v=rospy.get_param("~expert/integral_limit_v", 0.80),
            limit_w=rospy.get_param("~expert/integral_limit_w", 1.00),
            max_v=self.max_v,
            max_w=self.max_w,
        )

        self.mlp = None
        weights_path = package_path(str(rospy.get_param("~mlp/weights", "models/mlp_policy.pt")))
        norm_path = package_path(str(rospy.get_param("~mlp/norm_stats", "models/norm_stats.npz")))
        if os.path.isfile(weights_path) and os.path.isfile(norm_path):
            try:
                hidden = rospy.get_param("~mlp/hidden", [64, 64])
                self.mlp = NeuralController(
                    weights_path, norm_path, self.max_v, self.max_w, hidden=hidden
                )
                rospy.loginfo("MLP 已加载: %s", weights_path)
            except Exception as exc:  # pragma: no cover - runtime guard
                rospy.logerr("MLP 加载失败，将回退到 PI: %s", exc)
        else:
            rospy.logwarn("未找到 MLP 权重或归一化文件，MLP 模式回退到 PI。")

        if self.controller not in ("raw", "pi", "mlp", "all"):
            rospy.logwarn("未知 controller=%s，使用 mlp", self.controller)
            self.controller = "mlp"
        self.active_controller = "pi" if self.controller == "all" else self.controller
        if self.active_controller == "mlp" and self.mlp is None:
            self.active_controller = "pi"
        if self.controller == "mlp" and self.mlp is None:
            self.controller = "pi"

        self.v_meas = 0.0
        self.w_meas = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw_rate = 0.0
        self.odom_wall = 0.0
        self.imu_wall = 0.0
        self.odom_pitch = 0.0
        self.odom_roll = 0.0

        self.v_target = 0.0
        self.w_target = 0.0
        self.last_control_time = _now()

        self.csv_file = None
        self.csv_writer = None
        self.csv_rows_since_flush = 0
        self.recording = self.mode in ("collect", "eval") or self.record_keyboard
        self.rows = []
        self.eval_rows = []

        self.stop_event = threading.Event()
        self.keyboard_reader = None
        self.last_status_print = 0.0
        self.warned_stale = False

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber("/imu", Imu, self._imu_cb, queue_size=1)

        self.reset_world_srv = None
        rospy.on_shutdown(self._on_shutdown)

        if self.recording and self.mode != "eval":
            self._open_csv()

        rospy.loginfo("模式=%s 控制器=%s 场景=%s 时长=%.1fs",
                      self.mode, self.controller, self.world, self.duration)

    # ------------------------------------------------------------------ IO
    def _open_csv(self):
        path = self.requested_output_csv
        if path:
            if self.mode == "eval":
                root, ext = os.path.splitext(path)
                path = "{}_{}{}".format(root, self.active_controller, ext)
        else:
            root = os.path.expanduser("~/.ros/tracked_vehicle_mlp_teleop/data")
            os.makedirs(root, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            suffix = "_{}".format(self.active_controller) if self.mode == "eval" else ""
            path = os.path.join(root, "{}_{}_{}{}.csv".format(
                self.mode, self.world, stamp, suffix))
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.csv_file = open(path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(ROW_HEADER)
        self.output_csv = path
        rospy.loginfo("记录 CSV: %s", path)

    def _record(self, row):
        if not self.recording or self.csv_writer is None:
            return
        self.csv_writer.writerow([
            "{:.6f}".format(x) if isinstance(x, (float, np.floating)) else x
            for x in row
        ])
        self.csv_rows_since_flush += 1
        self.rows.append(row)
        if self.csv_rows_since_flush >= 20:
            self.csv_file.flush()
            self.csv_rows_since_flush = 0

    def _close_csv(self):
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            if self.output_csv:
                rospy.loginfo("CSV 已保存: %s", self.output_csv)

    # -------------------------------------------------------------- sensors
    def _odom_cb(self, msg):
        self.v_meas = float(msg.twist.twist.linear.x)
        self.w_meas = float(msg.twist.twist.angular.z)
        self.odom_wall = time.time()
        q = msg.pose.pose.orientation
        if q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z > 1e-9:
            self.odom_roll, self.odom_pitch, _ = euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )
        if self.imu_wall <= 0.0:
            self.yaw_rate = self.w_meas

    def _imu_cb(self, msg):
        q = msg.orientation
        if q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z > 1e-9:
            self.roll, self.pitch, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        else:
            self.pitch = self.odom_pitch
            self.roll = self.odom_roll
        self.yaw_rate = float(msg.angular_velocity.z)
        self.imu_wall = time.time()

    def _fresh(self):
        return (time.time() - self.odom_wall) < self.odom_timeout

    def _attitude(self):
        if time.time() - self.imu_wall < self.odom_timeout:
            return self.pitch, self.roll, self.yaw_rate
        return self.odom_pitch, self.odom_roll, self.w_meas

    # ------------------------------------------------------------- control
    def _set_target(self, v, w):
        self.v_target = _clip(v, -self.max_v, self.max_v)
        self.w_target = _clip(w, -self.max_w, self.max_w)

    def _compute_and_publish(self, phase=0):
        now = _now()
        dt = _clip(now - self.last_control_time, 0.0, 0.1)
        if dt <= 1e-4:
            dt = 1.0 / self.control_rate
        self.last_control_time = now

        pitch, roll, yaw_rate = self._attitude()
        e_v = self.v_target - self.v_meas
        e_w = self.w_target - self.w_meas

        # The PI expert is always evaluated: it supplies the labels during
        # collection, the PI baseline and the integral features for the MLP.
        pi_v, pi_w = self.expert.command(
            self.v_target, self.w_target, self.v_meas, self.w_meas, dt
        )

        latency_ms = 0.0
        if self.active_controller == "raw":
            cmd_v, cmd_w = self.v_target, self.w_target
        elif self.active_controller == "pi":
            cmd_v, cmd_w = pi_v, pi_w
        elif self.active_controller == "mlp" and self.mlp is not None:
            features = np.array([
                self.v_target, self.w_target, self.v_meas, self.w_meas,
                e_v, e_w, self.expert.int_v, self.expert.int_w,
                pitch, roll, yaw_rate,
            ], dtype=np.float32)
            start = time.perf_counter()
            cmd_v, cmd_w = self.mlp.command(features)
            latency_ms = (time.perf_counter() - start) * 1000.0
        else:
            cmd_v, cmd_w = pi_v, pi_w

        cmd_v = _clip(cmd_v, -self.max_v, self.max_v)
        cmd_w = _clip(cmd_w, -self.max_w, self.max_w)

        twist = Twist()
        if self._fresh():
            twist.linear.x = cmd_v
            twist.angular.z = cmd_w
            self.warned_stale = False
        else:
            if not self.warned_stale:
                rospy.logwarn_throttle(2.0, "等待 /odom，暂时输出零速度")
                self.warned_stale = True

        self.cmd_pub.publish(twist)

        if self.recording and self._fresh():
            self._record([
                _now(), self.world, self.active_controller, phase,
                self.v_target, self.w_target, cmd_v, cmd_w,
                self.v_meas, self.w_meas, pitch, roll, yaw_rate,
                e_v, e_w, self.expert.int_v, self.expert.int_w, latency_ms,
            ])
        return latency_ms

    def _stop_robot(self, repeats=3):
        twist = Twist()
        for _ in range(repeats):
            try:
                self.cmd_pub.publish(twist)
            except rospy.ROSException:
                break
            time.sleep(0.02)

    # ------------------------------------------------------------- keyboard
    def _on_key(self, key):
        if key in ("\x1b", "\x03"):
            rospy.loginfo("收到退出按键，停止仿真控制")
            rospy.signal_shutdown("keyboard exit")
            return
        if key in ("w", "W"):
            self._set_target(self.v_target + 0.10, self.w_target)
        elif key in ("s", "S"):
            self._set_target(self.v_target - 0.10, self.w_target)
        elif key in ("a", "A"):
            self._set_target(self.v_target, self.w_target + 0.15)
        elif key in ("d", "D"):
            self._set_target(self.v_target, self.w_target - 0.15)
        elif key in ("q", "Q", " "):
            self._set_target(0.0, 0.0)
        elif key == "1":
            self.active_controller = "raw"
            self.expert.reset()
            rospy.loginfo("切换控制器: RAW")
        elif key == "2":
            self.active_controller = "pi"
            self.expert.reset()
            rospy.loginfo("切换控制器: PI")
        elif key == "3":
            if self.mlp is None:
                rospy.logwarn("MLP 权重不可用")
            else:
                self.active_controller = "mlp"
                self.expert.reset()
                rospy.loginfo("切换控制器: MLP")
        elif key in ("n", "N"):
            order = ["raw", "pi", "mlp"] if self.mlp is not None else ["raw", "pi"]
            idx = (order.index(self.active_controller) + 1) % len(order) \
                if self.active_controller in order else 0
            self.active_controller = order[idx]
            self.expert.reset()
            rospy.loginfo("切换控制器: %s", self.active_controller.upper())
        elif key in ("r", "R"):
            self._reset_world()
        elif key in ("h", "H", "?"):
            self._print_help()
        elif key in ("p", "P"):
            rospy.loginfo("target=(%.2f, %.2f) meas=(%.2f, %.2f) controller=%s",
                          self.v_target, self.w_target, self.v_meas,
                          self.w_meas, self.active_controller)

    def _print_help(self):
        rospy.loginfo("W/S 前进/后退  A/D 左转/右转  Q/空格 停车")
        rospy.loginfo("1=RAW 2=PI 3=MLP  N=循环切换  R=复位世界  P=打印状态  H=帮助  ESC=退出")

    # --------------------------------------------------------------- modes
    def _run_keyboard(self):
        self._print_help()
        if self.keyboard_enabled:
            self.keyboard_reader = KeyboardReader(self._on_key, rospy)
            self.keyboard_reader.start()
        else:
            rospy.logwarn("键盘输入未启用；可用 rostopic pub /cmd_vel ... 测试")

        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            self._compute_and_publish(phase=0)
            now = time.time()
            if now - self.last_status_print >= 2.0:
                rospy.loginfo("target=(%.2f,%.2f) meas=(%.2f,%.2f) ctrl=%s",
                              self.v_target, self.w_target, self.v_meas,
                              self.w_meas, self.active_controller)
                self.last_status_print = now
            rate.sleep()

    def _next_collect_segment(self, rng):
        slope = self.world == "slope"
        if slope:
            v_min = float(rospy.get_param("~collect/slope_v_min", 0.10))
            v_max = float(rospy.get_param("~collect/slope_v_max", 0.65))
            w_min = float(rospy.get_param("~collect/slope_w_min", -0.35))
            w_max = float(rospy.get_param("~collect/slope_w_max", 0.35))
            v = rng.uniform(v_min, v_max)
            if rng.rand() < 0.2:
                v = -v * 0.6
        else:
            v_min = float(rospy.get_param("~collect/v_min", -0.55))
            v_max = float(rospy.get_param("~collect/v_max", 0.80))
            w_min = float(rospy.get_param("~collect/w_min", -1.10))
            w_max = float(rospy.get_param("~collect/w_max", 1.10))
            v = rng.uniform(v_min, v_max)
            w = rng.uniform(w_min, w_max)
            if abs(w) > 0.8:
                v = _clip(v, -0.25, 0.35)
            self._set_target(v, w)
            duration = rng.uniform(
                float(rospy.get_param("~collect/segment_min_seconds", 2.0)),
                float(rospy.get_param("~collect/segment_max_seconds", 4.0)),
            )
            return duration
        w = rng.uniform(w_min, w_max)
        self._set_target(v, w)
        duration = rng.uniform(
            float(rospy.get_param("~collect/segment_min_seconds", 2.0)),
            float(rospy.get_param("~collect/segment_max_seconds", 4.0)),
        )
        return duration

    def _run_collect(self):
        seed = int(rospy.get_param("~collect/random_seed", 20261027))
        rng = np.random.RandomState(seed)
        warmup = float(rospy.get_param("~collect/warmup_seconds", 2.0))
        self.active_controller = "pi"
        self.expert.reset()
        start = _now()
        next_segment = warmup
        self._set_target(0.0, 0.0)
        rate = rospy.Rate(self.control_rate)
        rospy.loginfo("开始采集：%.1fs，场景 %s", self.duration, self.world)

        while not rospy.is_shutdown():
            elapsed = _now() - start
            if elapsed >= self.duration:
                break
            if elapsed >= warmup and elapsed >= next_segment:
                seg = self._next_collect_segment(rng)
                next_segment = elapsed + seg
            if elapsed < warmup:
                self._set_target(0.0, 0.0)
            self._compute_and_publish(phase=0)
            if int(elapsed) % 10 == 0 and abs(elapsed - round(elapsed)) < 0.05:
                rospy.loginfo("采集进度 %.0f/%.0f s，样本 %d",
                              elapsed, self.duration, len(self.rows))
            rate.sleep()

        self._stop_robot()
        self._close_csv()
        rospy.loginfo("采集完成：%d 条样本", len(self.rows))
        rospy.signal_shutdown("collect done")

    def _reset_world(self):
        if self.reset_world_srv is None:
            try:
                rospy.wait_for_service("/gazebo/reset_world", timeout=2.0)
                self.reset_world_srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)
            except rospy.ROSException:
                rospy.logwarn("/gazebo/reset_world 不可用，无法复位世界")
                return False
        try:
            self.reset_world_srv()
            self.v_meas = 0.0
            self.w_meas = 0.0
            self.v_target = 0.0
            self.w_target = 0.0
            self.expert.reset()
            self.last_control_time = _now()
            rospy.loginfo("世界已复位")
            time.sleep(float(rospy.get_param("~evaluation/settle_seconds", 1.5)))
            return True
        except rospy.ServiceException as exc:
            rospy.logwarn("复位失败: %s", exc)
            return False

    def _profile_target(self, phase_start, profile):
        elapsed = _now() - phase_start
        acc = 0.0
        for v, w, duration in profile:
            if elapsed < acc + duration:
                return float(v), float(w)
            acc += duration
        # loop the last segment if the profile is shorter than the phase
        return float(profile[-1][0]), float(profile[-1][1])

    def _summary_from_rows(self, rows, controller, duration):
        if not rows:
            return None
        arr = np.asarray([[float(x) for x in row[4:]] for row in rows], dtype=np.float64)
        # columns: v_target,w_target,v_cmd,w_cmd,v_meas,w_meas,pitch,roll,yaw_rate,
        #          e_v,e_w,int_v,int_w,latency_ms
        v_t, w_t = arr[:, 0], arr[:, 1]
        v_c, w_c = arr[:, 2], arr[:, 3]
        v_m, w_m = arr[:, 4], arr[:, 5]
        pitch, roll = arr[:, 6], arr[:, 7]
        latency = arr[:, 13]
        dt = 1.0 / self.control_rate
        summary = {
            "world": self.world,
            "controller": controller,
            "samples": len(rows),
            "duration_s": duration,
            "rmse_v": math.sqrt(float(np.mean((v_m - v_t) ** 2))),
            "rmse_w": math.sqrt(float(np.mean((w_m - w_t) ** 2))),
            "mae_v": float(np.mean(np.abs(v_m - v_t))),
            "mae_w": float(np.mean(np.abs(w_m - w_t))),
            "mean_abs_slip": float(np.mean(np.abs(v_m - v_c))),
            "distance_m": float(np.sum(np.abs(v_m)) * dt),
            "max_roll_deg": float(np.max(np.abs(roll)) * 180.0 / math.pi),
            "max_pitch_deg": float(np.max(np.abs(pitch)) * 180.0 / math.pi),
            "latency_mean_ms": float(np.mean(latency)),
            "latency_p95_ms": float(np.percentile(latency, 95)),
            "success": int(np.max(np.abs(roll)) < 0.60 and np.max(np.abs(pitch)) < 0.90),
        }
        return summary

    def _write_summaries(self, summaries):
        if not summaries:
            return
        path = self.summary_csv
        if not path:
            root = os.path.expanduser("~/.ros/tracked_vehicle_mlp_teleop/data")
            os.makedirs(root, exist_ok=True)
            path = os.path.join(root, "summary_{}.csv".format(self.world))
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(SUMMARY_HEADER)
            for item in summaries:
                writer.writerow([item[key] for key in SUMMARY_HEADER])
        self.summary_csv = path
        rospy.loginfo("汇总 CSV: %s", path)

    def _run_eval(self):
        modes = ["raw", "pi", "mlp"] if self.controller == "all" else [self.controller]
        if "mlp" in modes and self.mlp is None:
            rospy.logerr("缺少 MLP 权重，评估中跳过 mlp")
            modes = [m for m in modes if m != "mlp"]
        if not modes:
            rospy.logerr("没有可评估的控制器")
            rospy.signal_shutdown("eval failed")
            return

        phase_seconds = float(rospy.get_param("~evaluation/phase_seconds", 12.0))
        profile = rospy.get_param("~evaluation/profile", [[0.5, 0.0, 3.0]])
        summaries = []

        for controller in modes:
            self._reset_world()
            self.active_controller = controller
            self.expert.reset()
            self.eval_rows = []
            self._set_target(0.0, 0.0)
            self.recording = True
            if self.csv_file is not None:
                self._close_csv()
            self._open_csv()
            start_idx = len(self.rows)
            start = _now()
            rate = rospy.Rate(self.control_rate)
            rospy.loginfo("评估 %s：%.1fs", controller, phase_seconds)
            while not rospy.is_shutdown():
                if _now() - start >= phase_seconds:
                    break
                v, w = self._profile_target(start, profile)
                self._set_target(v, w)
                self._compute_and_publish(phase=modes.index(controller))
                rate.sleep()
            self._stop_robot()
            self._close_csv()
            summary = self._summary_from_rows(self.rows[start_idx:],
                                              controller, phase_seconds)
            if summary is not None:
                summaries.append(summary)
                rospy.loginfo("评估结果 %s: rmse_v=%.3f rmse_w=%.3f slip=%.3f success=%d",
                              controller, summary["rmse_v"], summary["rmse_w"],
                              summary["mean_abs_slip"], summary["success"])

        self._write_summaries(summaries)
        rospy.loginfo("评估完成，汇总: %s", self.summary_csv)
        rospy.signal_shutdown("eval done")

    # ------------------------------------------------------------- lifecycle
    def _on_shutdown(self):
        self.stop_event.set()
        if self.keyboard_reader is not None:
            self.keyboard_reader.stop()
        self._stop_robot()
        self._close_csv()

    def run(self):
        # Wait briefly for the simulated clock / odometry.
        deadline = time.time() + 8.0
        while not rospy.is_shutdown() and time.time() < deadline:
            if self._fresh():
                break
            rospy.sleep(0.1)

        if self.mode == "collect":
            self._run_collect()
        elif self.mode == "eval":
            self._run_eval()
        else:
            self._run_keyboard()


def main():
    rospy.init_node("tracked_vehicle_mlp_teleop")
    try:
        node = TrackedVehicleNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:  # pragma: no cover - keep shutdown clean
        rospy.logerr("主节点异常: %s", exc, exc_info=True)
        raise


if __name__ == "__main__":
    main()
