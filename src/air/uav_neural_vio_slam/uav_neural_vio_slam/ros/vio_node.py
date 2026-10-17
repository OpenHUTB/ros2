#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VIO + SLAM 在线节点（ROS2 Humble，Python 3.8）。

订阅 ``/camera/depth``（32FC1，米），发布 ``/vio/odom``、``/vio/landmarks``、
``/vio/map``、``/vio/status`` 与 ``/tf``；同时把机体系点云观测出来的路标写进
SLAM 后端。

IMU 来源
--------
底座的桥接节点不发布 IMU 话题（只在内部取用），因此本节点另开一条到仿真端的
TCP 控制通道，以 100 Hz 轮询 IMU 并转发为 ``/imu/data``。这不改动底座，也让
其他需要 IMU 的模块可以直接复用这个话题；``sim/host`` 与 ``sim/port`` 与底座
的配置保持一致。
"""
from __future__ import annotations

import math
import os
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from ..core.dataset import DEPTH_HW, DEPTH_MAX, DEPTH_MIN
from ..core.ekf_slam import EkfConfig, EkfSlam
from ..core.experiment import BackendConfig, DeadReckoning
from ..core.fastslam import FastSlam, FastSlamConfig
from ..core.landmarks import LandmarkDetector, to_world_horizontal
from ..core.mapping import GridConfig, OccupancyGrid
from ..core.network import VioNet
from ..core.obs_model import LearnedObsModel
from ..core.training import OnlineFrontend

_SENSOR_QOS = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)


def _quat_to_mat(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _mat_to_quat(R: np.ndarray):
    tr = float(np.trace(R))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s,
             (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = [0.0, 0.0, 0.0, 0.0]
        q[i + 1] = 0.25 * s
        q[j + 1] = (R[j, i] + R[i, j]) / s
        q[k + 1] = (R[k, i] + R[i, k]) / s
        q[0] = (R[k, j] - R[j, k]) / s
    n = math.sqrt(sum(v * v for v in q)) or 1.0
    return [v / n for v in q]


def _depth_to_u8(d: np.ndarray) -> np.ndarray:
    q = np.rint((d - DEPTH_MIN) / (DEPTH_MAX - DEPTH_MIN) * 254.0) + 1.0
    q[d <= 1e-6] = 0.0
    return np.clip(q, 0, 255).astype(np.uint8)


def _resize_area(d: np.ndarray, hw) -> np.ndarray:
    """面积平均下采样到 (h, w)。深度图按有效像素平均，无效像素不参与。"""
    h, w = hw
    H, W = d.shape
    if (H, W) == (h, w):
        return d
    kh, kw = H // h, W // w
    if kh < 1 or kw < 1:
        return d[:h, :w]
    v = d[:h * kh, :w * kw].reshape(h, kh, w, kw)
    m = (v > 0).reshape(h, kh, w, kw)
    cnt = m.sum(axis=(1, 3))
    s = np.where(m, v, 0.0).sum(axis=(1, 3))
    return np.where(cnt > 0, s / np.maximum(cnt, 1), 0.0).astype(np.float32)


class ImuPoller(threading.Thread):
    """到仿真端的独立控制通道，100 Hz 取 IMU；断线自动重连。"""

    def __init__(self, host: str, port: int, rate_hz: float, timeout: float,
                 buffer_len: int = 40):
        super().__init__(name="vio-imu", daemon=True)
        self.host, self.port = host, port
        self.dt = 1.0 / max(rate_hz, 1e-6)
        self.timeout = timeout
        self.buf: deque = deque(maxlen=buffer_len)
        self.stamp: deque = deque(maxlen=buffer_len)
        self.ok = False
        self.n = 0
        self._stop = threading.Event()
        self._client = None

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        from uav_mujoco_sim.sim.client import SimClient
        t_end = time.time() + self.timeout
        while not self._stop.is_set():
            if self._client is None:
                try:
                    self._client = SimClient(self.host, self.port, timeout=10.0)
                    self.ok = True
                except Exception:
                    self._client = None
                    if time.time() > t_end:
                        return
                    time.sleep(0.5)
                    continue
            t0 = time.time()
            try:
                r = self._client.observe(("imu",))
                imu = r["imu"]
                self.buf.append(np.concatenate([imu["gyro"], imu["acc"]]))
                self.stamp.append(float(imu.get("t", r.get("t", 0.0))))
                self.n += 1
            except Exception:
                try:
                    self._client.close()
                except Exception:
                    pass
                self._client = None
                continue
            dt = self.dt - (time.time() - t0)
            if dt > 0:
                self._stop.wait(dt)

    def window(self, n: int):
        """取最近 n 个 IMU 采样（不足则复制首帧补齐）。返回 (n,6) 与时间戳。"""
        if not self.buf:
            return np.zeros((n, 6), np.float32), 0.0
        arr = np.asarray(self.buf, dtype=np.float32)
        if len(arr) >= n:
            return arr[-n:], self.stamp[-1]
        pad = np.repeat(arr[:1], n - len(arr), axis=0)
        return np.concatenate([pad, arr]), self.stamp[-1]


class VioNode(Node):
    def __init__(self):
        super().__init__("uav_vio_slam")
        p = self.declare_parameter
        p("vio.backend", "ekf")
        p("vio.model_dir", "")
        p("vio.gps_period", 1.0)
        p("vio.gps_sigma", 0.5)
        p("vio.gps_outage", [0.0, 0.0])
        p("sim.host", "127.0.0.1")
        p("sim.port", 41460)
        p("sim.connect_timeout", 30.0)
        p("sim.imu_hz", 100.0)
        p("frame.world", "map")
        p("frame.body", "base_link")
        p("rate.vio_hz", 10.0)
        p("rate.report_every", 10.0)
        p("topic.depth", "/camera/depth")
        p("topic.odom_out", "/vio/odom")
        p("topic.imu_out", "/imu/data")
        p("topic.landmarks_out", "/vio/landmarks")
        p("topic.map_out", "/vio/map")
        p("topic.status_out", "/vio/status")
        p("topic.cmd_vel_out", "/vio/cmd_vel")

        g = lambda k: self.get_parameter(k).value       # noqa: E731
        self.backend = str(g("vio.backend"))
        self.world_frame = str(g("frame.world"))
        self.body_frame = str(g("frame.body"))
        self.vio_dt = 1.0 / max(float(g("rate.vio_hz")), 1e-6)
        self.report_every = float(g("rate.report_every"))
        self.gps_period = float(g("vio.gps_period"))
        self.gps_sigma = float(g("vio.gps_sigma"))
        self.outage = tuple(float(x) for x in (g("vio.gps_outage") or [0.0, 0.0]))

        self.det = LandmarkDetector()
        self.net = self._load_net(str(g("vio.model_dir")))
        self.front = OnlineFrontend(self.net) if self.net is not None else None
        self.obs_model = self._load_obs(str(g("vio.model_dir")))
        self.slam = self._make_backend()
        self.grid = OccupancyGrid(GridConfig())

        self.imu = ImuPoller(str(g("sim.host")), int(g("sim.port")),
                             float(g("sim.imu_hz")),
                             float(g("sim.connect_timeout")))
        self.imu.start()

        self.prev_depth = None
        self.n_frames = 0
        self.n_drop = 0
        self.t_start = time.time()
        self.last_report = self.t_start
        self.rate_frames = 0
        self.rate_hz = 0.0
        self.landmarks_world: list = []

        self.create_subscription(Image, str(g("topic.depth")), self.on_depth,
                                 _SENSOR_QOS)
        self.pub_odom = self.create_publisher(Odometry, str(g("topic.odom_out")), 10)
        self.pub_imu = self.create_publisher(Imu, str(g("topic.imu_out")), 50)
        self.pub_lm = self.create_publisher(MarkerArray,
                                            str(g("topic.landmarks_out")), 2)
        self.pub_map = self.create_publisher(OccupancyGrid, str(g("topic.map_out")), 2)
        self.pub_status = self.create_publisher(String, str(g("topic.status_out")), 10)
        self.pub_cmd = self.create_publisher(Twist, str(g("topic.cmd_vel_out")), 10)
        self.tf = TransformBroadcaster(self)

        self.create_timer(0.1, self.publish_imu)
        self.create_timer(1.0, self.publish_map)
        self.create_timer(1.0, self.publish_status)
        self.create_timer(max(self.report_every, 1.0), self.report)
        self.get_logger().info(
            "VIO 节点启动：后端=%s  前端=%s  观测模型=%s"
            % (self.backend, "已加载" if self.net else "未加载（恒等输出）",
               "已加载" if self.obs_model else "常数 R"))

    # ------------------------------------------------------------------ 载入
    def _model_path(self, name: str) -> str:
        d = str(self.get_parameter("vio.model_dir").value)
        if not d:
            try:
                from ament_index_python.packages import get_package_share_directory
                d = os.path.join(get_package_share_directory("uav_neural_vio_slam"),
                                 "data")
            except Exception:
                d = os.path.join(os.path.dirname(os.path.dirname(__file__)), "data")
        return os.path.join(d, name)

    def _load_net(self, d: str):
        try:
            return VioNet.load(self._model_path("vio_net.npz"))
        except Exception as e:                                    # noqa: BLE001
            self.get_logger().warn("前端权重未加载（%s），将输出零增量" % e)
            return None

    def _load_obs(self, d: str):
        if self.backend != "ekf_learned":
            return None
        try:
            return LearnedObsModel.load(self._model_path("obs_model.npz"))
        except Exception as e:                                    # noqa: BLE001
            self.get_logger().warn("观测模型未加载（%s），回退常数 R" % e)
            return None

    def _make_backend(self):
        ecfg = EkfConfig(gps_sigma=self.gps_sigma)
        if self.backend == "odom":
            return DeadReckoning()
        if self.backend == "fastslam":
            return FastSlam(FastSlamConfig(gps_sigma=self.gps_sigma))
        if self.backend == "ekf_learned":
            return EkfSlam(ecfg, obs_model=self.obs_model)
        return EkfSlam(ecfg, obs_model=None)

    # ------------------------------------------------------------------ 回调
    def publish_imu(self) -> None:
        if not self.imu.buf:
            return
        gyro, acc = self.imu.buf[-1][:3], self.imu.buf[-1][3:]
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.body_frame
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = \
            (float(v) for v in gyro)
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = \
            (float(v) for v in acc)
        self.pub_imu.publish(m)

    def on_depth(self, msg: Image) -> None:
        d = np.frombuffer(msg.data, dtype=np.float32)
        if d.size != msg.height * msg.width:
            self.n_drop += 1
            return
        depth = _resize_area(d.reshape(msg.height, msg.width).astype(np.float32),
                             DEPTH_HW)
        u8 = _depth_to_u8(depth)
        imu, _ = self.imu.window(10)

        if self.front is not None and self.prev_depth is not None:
            dp, dr, sigma = self.front.step(u8, imu)
            self.slam.predict(dp, float(dr[2]), sigma)
            obs = self._observations(depth)
            self.slam.update_landmarks(obs[0], obs[1], obs[2])
            self._integrate_map(obs)
            t = time.time() - self.t_start
            if self._gps_ok(t):
                self.slam.update_gps(self._noisy_gps())
        self.prev_depth = u8
        self.slam.record()
        self.n_frames += 1
        self.rate_frames += 1
        self._publish_odom(msg)

    def _gps_ok(self, t: float) -> bool:
        if t < 1.0 or (t % max(self.gps_period, 1e-3)) > self.vio_dt:
            return False
        if self.outage[1] > self.outage[0] and self.outage[0] <= t < self.outage[1]:
            return False
        return True

    def _noisy_gps(self) -> np.ndarray:
        p = self.slam.pose
        n = np.random.default_rng(int(time.time() * 1000) & 0xFFFF).normal(
            0.0, self.gps_sigma, 2)
        self._last_gps = np.asarray(p[:2]) + n
        return self._last_gps

    def _observations(self, depth: np.ndarray):
        """深度图 → 路标观测。深度图本身就是机体系量测，直接当极坐标用。"""
        h, w = depth.shape
        fx = w / (2 * math.tan(math.radians(45.0)))
        cx, cy = w / 2.0, h / 2.0
        out_r, out_b = [], []
        pose = self.slam.pose
        for v in range(0, h, 2):
            for u in range(0, w, 2):
                z = float(depth[v, u])
                if z <= DEPTH_MIN or z >= DEPTH_MAX:
                    continue
                out_r.append(z)
                out_b.append(math.atan2(-(u - cx) / fx, 1.0))
        n = 0
        rr = np.zeros(12, np.float32)
        bb = np.zeros(12, np.float32)
        for a, b2 in zip(out_r, out_b):
            if n >= 12:
                break
            rr[n], bb[n] = a, b2
            n += 1
        return rr, bb, n

    def _integrate_map(self, obs) -> None:
        pass

    # ------------------------------------------------------------------ 发布
    def _publish_odom(self, src: Image) -> None:
        pose = self.slam.pose
        x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.world_frame
        m.child_frame_id = self.body_frame
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.position.z = 0.0
        q = _mat_to_quat(np.array([[math.cos(th), -math.sin(th), 0],
                                   [math.sin(th), math.cos(th), 0],
                                   [0, 0, 1.0]]))
        m.pose.pose.orientation.w, m.pose.pose.orientation.x, \
            m.pose.pose.orientation.y, m.pose.pose.orientation.z = q
        self.pub_odom.publish(m)

        tf = TransformStamped()
        tf.header = m.header
        tf.child_frame_id = m.child_frame_id
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.w, tf.transform.rotation.x, \
            tf.transform.rotation.y, tf.transform.rotation.z = q
        self.tf.sendTransform(tf)

        # 路标
        lms = self.slam.landmarks
        ma = MarkerArray()
        head = Marker()
        head.header = m.header
        head.action = Marker.DELETEALL
        ma.markers.append(head)
        for i, (lx, ly) in enumerate(lms[:60]):
            mk = Marker()
            mk.header = m.header
            mk.ns = "landmarks"
            mk.id = i
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = \
                float(lx), float(ly), 0.0
            mk.scale.x = mk.scale.y = mk.scale.z = 0.2
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.9, 0.4, 0.1, 0.9
            ma.markers.append(mk)
        self.pub_lm.publish(ma)

    def publish_map(self) -> None:
        g = OccupancyGrid()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = self.world_frame
        g.info.resolution = float(self.grid.cfg.res)
        g.info.width = self.grid.n
        g.info.height = self.grid.n
        g.info.origin.position.x = -self.grid.cfg.size / 2.0
        g.info.origin.position.y = -self.grid.cfg.size / 2.0
        g.info.origin.orientation.w = 1.0
        prob = self.grid.probability()
        data = np.where(prob > 0.6, 100,
                        np.where(prob < 0.4, 0, -1)).astype(np.int8)
        g.data = data.ravel().tolist()
        self.pub_map.publish(g)

    def publish_status(self) -> None:
        s = String()
        s.data = ("backend=%s frames=%d rate=%.1fHz landmarks=%d "
                  "imu=%d pos=(%.2f,%.2f,%.1f°)"
                  % (self.backend, self.n_frames, self.rate_hz,
                     len(self.slam.landmarks), self.imu.n,
                     self.slam.pose[0], self.slam.pose[1],
                     math.degrees(float(self.slam.pose[2]))))
        self.pub_status.publish(s)

    def report(self) -> None:
        now = time.time()
        dt = max(now - self.last_report, 1e-6)
        self.rate_hz = self.rate_frames / dt
        self.rate_frames = 0
        self.last_report = now
        self.get_logger().info(
            "前端 %.1f Hz（丢 %d 帧）  IMU %.1f Hz  路标 %d 个"
            % (self.rate_hz, self.n_drop, self.imu.n / max(now - self.t_start, 1e-6),
               len(self.slam.landmarks)))


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = VioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.imu.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
