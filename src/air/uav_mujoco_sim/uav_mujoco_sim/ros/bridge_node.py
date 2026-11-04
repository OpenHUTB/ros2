#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 Humble 桥接节点：把 MuJoCo 仿真服务端接成标准 ROS 话题（对外接口与课程 ``carlair_ros_bridge`` 一致）。

发布 /uav/odom(ENU)、/tf(world→base_link)、/camera/image_raw(bgr8)、/camera/depth(32FC1, m)、/camera/seg(bgr8)、/lidar/points(XYZ float32, ENU)、/uav/status(READY/HOVER/VELOCITY/GOAL/GOAL_DONE)；订阅 /uav/cmd_vel、/uav/goal（ENU）。

控制 50 Hz 与感知 10 Hz 各用独立连接和定时器（合用一路时 66 ms/帧的 RGB+深度把控制回路拖到 3.8 Hz）；``render`` 不推进物理，图像与里程计取自同一状态。
"""
from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import geometry_msgs.msg
import std_msgs.msg

# 兼容已安装包与源码目录直跑两种导入路径
try:
    from ..sim.client import SimClient, wait_for_server
    from .conversions import (to_odometry, to_tf, to_image, to_pointcloud2,
                              make_twist)
except ImportError:  # pragma: no cover - ROS 安装后包结构会变
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from uav_mujoco_sim.sim.client import SimClient, wait_for_server
    from uav_mujoco_sim.ros.conversions import (to_odometry, to_tf, to_image,
                                                to_pointcloud2, make_twist)

# 语义分割类别 id → 伪彩色（与 core/sensors.SEMANTIC_CLASSES 对应）
SEG_COLORS = np.array([
    [0, 0, 0],        # 0 背景
    [110, 110, 110],  # 1 地面
    [190, 150, 110],  # 2 建筑
    [70, 150, 70],    # 3 植被
    [200, 90, 60],    # 4 一般障碍
    [240, 200, 40],   # 5 门
    [60, 90, 200],    # 6 车辆
    [230, 120, 180],  # 7 行人
    [180, 180, 255],  # 8 自身
], dtype=np.uint8)


class UavSimBridge(Node):
    """MuJoCo 仿真 ↔ ROS2 话题 的桥接节点。"""

    def __init__(self):
        super().__init__("uav_sim_bridge")

        # ---------------------------------------------------------- 参数
        self.declare_parameter("sim/host", "127.0.0.1")
        self.declare_parameter("sim/port", 41460)
        self.declare_parameter("sim/connect_timeout", 30.0)

        self.declare_parameter("rate/control_hz", 50.0)
        self.declare_parameter("rate/image_hz", 10.0)
        self.declare_parameter("rate/lidar_hz", 10.0)
        self.declare_parameter("rate/report_every", 10.0)

        self.declare_parameter("sensor/publish_rgb", True)
        self.declare_parameter("sensor/publish_depth", True)
        self.declare_parameter("sensor/publish_seg", False)
        self.declare_parameter("sensor/publish_lidar", True)

        self.declare_parameter("control/max_horiz_speed", 5.0)
        self.declare_parameter("control/max_vert_speed", 3.0)
        self.declare_parameter("control/max_yaw_rate", 1.0)
        self.declare_parameter("control/body_frame", True)
        self.declare_parameter("control/cmd_timeout", 0.5)
        self.declare_parameter("control/auto_takeoff", True)

        self.declare_parameter("frame/world", "world")
        self.declare_parameter("frame/body", "base_link")

        self.declare_parameter("topic/odom", "/uav/odom")
        self.declare_parameter("topic/cmd_vel", "/uav/cmd_vel")
        self.declare_parameter("topic/goal", "/uav/goal")
        self.declare_parameter("topic/image", "/camera/image_raw")
        self.declare_parameter("topic/depth", "/camera/depth")
        self.declare_parameter("topic/seg", "/camera/seg")
        self.declare_parameter("topic/lidar", "/lidar/points")
        self.declare_parameter("topic/status", "/uav/status")

        self.declare_parameter("topic/qos_reliable", False)

        g = self.get_parameter
        self.host = g("sim/host").value
        self.port = int(g("sim/port").value)
        self.world_frame = g("frame/world").value
        self.body_frame_id = g("frame/body").value
        self.body_frame = bool(g("control/body_frame").value)
        self.cmd_timeout = float(g("control/cmd_timeout").value)

        self.control_hz = float(g("rate/control_hz").value)
        self.image_hz = float(g("rate/image_hz").value)
        self.lidar_hz = float(g("rate/lidar_hz").value)

        # ---------------------------------------------------------- 连接
        self.get_logger().info(
            "正在连接仿真服务端 %s:%d …" % (self.host, self.port))
        timeout = float(g("sim/connect_timeout").value)
        # 控制通道：仅位姿/IMU，报文几十字节，可跑高频
        self.sim = wait_for_server(self.host, self.port, timeout=timeout)
        info = self.sim.observe(["odom"])
        self.get_logger().info(
            "已连接。场景=%s 初始位置=%s"
            % (info.get("scene"), np.round(info["odom"]["pos"], 2).tolist()))

        # 感知通道：独立连接，承载图像/点云等大报文
        self.need_sensor = any([g("sensor/publish_rgb").value,
                                g("sensor/publish_depth").value,
                                g("sensor/publish_seg").value,
                                g("sensor/publish_lidar").value])
        self.sim_sensor = None
        if self.need_sensor:
            self.sim_sensor = wait_for_server(self.host, self.port,
                                              timeout=timeout)

        # ---------------------------------------------------------- QoS
        reliable = bool(g("topic/qos_reliable").value)
        qos_sensor = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE if reliable
            else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        qos_ctrl = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # ---------------------------------------------------------- 发布者
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image, PointCloud2
        from tf2_msgs.msg import TFMessage

        self.pub_odom = self.create_publisher(
            Odometry, g("topic/odom").value, qos_sensor)
        self.pub_tf = self.create_publisher(
            TFMessage, "/tf", qos_sensor)
        self.pub_status = self.create_publisher(
            std_msgs.msg.String, g("topic/status").value, qos_sensor)

        self.pub_img = self.create_publisher(
            Image, g("topic/image").value, qos_sensor) \
            if g("sensor/publish_rgb").value else None
        self.pub_depth = self.create_publisher(
            Image, g("topic/depth").value, qos_sensor) \
            if g("sensor/publish_depth").value else None
        self.pub_seg = self.create_publisher(
            Image, g("topic/seg").value, qos_sensor) \
            if g("sensor/publish_seg").value else None
        self.pub_lidar = self.create_publisher(
            PointCloud2, g("topic/lidar").value, qos_sensor) \
            if g("sensor/publish_lidar").value else None

        # ---------------------------------------------------------- 订阅者
        self.create_subscription(
            geometry_msgs.msg.Twist, g("topic/cmd_vel").value,
            self.on_cmd_vel, qos_ctrl)
        self.create_subscription(
            geometry_msgs.msg.Point, g("topic/goal").value,
            self.on_goal, qos_ctrl)

        # ---------------------------------------------------------- 状态
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0, 0.0, 0.0)
        self._cmd_t = -1e9
        self._goal = None
        self._pose_cache = None
        self._last_img_t = -1e9
        self._last_lidar_t = -1e9
        self._warned_world_frame = False

        # 控制与感知分属不同回调组，感知回调（几十 ms）不阻塞控制回调；需配合 main() 的 MultiThreadedExecutor
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        self.cb_ctrl = MutuallyExclusiveCallbackGroup()
        self.cb_sensor = MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(1.0 / self.control_hz, self.on_control,
                                       callback_group=self.cb_ctrl)
        self.timer_sensor = None
        if self.sim_sensor is not None:
            self.timer_sensor = self.create_timer(
                1.0 / self.image_hz, self.on_sensor,
                callback_group=self.cb_sensor)

        # 定期打印实测速率；跨虚拟机时图像回路常是瓶颈
        self._n_ctrl = 0
        self._n_sensor = 0
        self._acc_sensor_costs = []
        self._rate_t0 = time.monotonic()
        self._rate_timer = self.create_timer(
            float(g("rate/report_every").value), self.on_rate_report,
            callback_group=self.cb_ctrl)

        self.get_logger().info(
            "桥接已就绪：控制 %.0f Hz，图像 %.0f Hz，雷达 %.0f Hz，坐标=%s系"
            % (self.control_hz, self.image_hz, self.lidar_hz,
               "机体" if self.body_frame else "世界"))

    # ------------------------------------------------------------ 回调
    def on_cmd_vel(self, msg):
        with self._lock:
            self._cmd = (msg.linear.x, msg.linear.y, msg.linear.z,
                         msg.angular.z)
            self._cmd_t = time.monotonic()
            self._goal = None         # 速度指令优先，取消飞行中的目标点

    def on_goal(self, msg):
        with self._lock:
            self._goal = (msg.x, msg.y, msg.z)

    # ------------------------------------------------------------ 控制回路
    def on_control(self):
        """50 Hz：推进仿真一步并发布位姿/状态。只走小报文，不进图像。"""
        now = time.monotonic()
        with self._lock:
            cmd = self._cmd
            fresh = (now - self._cmd_t) < self.cmd_timeout
            goal = self._goal

        vx = vy = vz = wz = 0.0
        if fresh and goal is None:
            vx, vy, vz, wz = cmd
        # 指令超时保持全零 → 悬停（松键即停）

        # 世界系指令转机体系（本项目核心按机体系解释速度）
        if not self.body_frame and (vx or vy or vz):
            if not self._warned_world_frame:
                self.get_logger().warn(
                    "body_frame=false：/uav/cmd_vel 按世界系解释，"
                    "将转换为机体系后下发。多数上游模块（如 uav_ppo_nav）"
                    "输出的是世界系速度，必须这样设置。")
                self._warned_world_frame = True
            vx, vy = self._world_to_body(vx, vy)

        try:
            if goal is not None:
                frame = self.sim.frame(n=1, goal=goal, want=["odom", "imu"])
            else:
                frame = self.sim.frame(n=1, cmd_vel=[vx, vy, vz, wz],
                                       want=["odom", "imu"])
        except OSError as e:
            self.get_logger().error("控制通道通信失败：%s" % e)
            return

        if not frame.get("ok"):
            self.get_logger().warn("仿真端返回错误：%s" % frame.get("error"))
            return

        stamp = self.get_clock().now().to_msg()
        self._pose_cache = frame["odom"]
        self._n_ctrl += 1
        self.pub_odom.publish(to_odometry(frame, stamp, self.world_frame,
                                          self.body_frame_id))
        self.pub_tf.publish(to_tf(frame, stamp, self.world_frame,
                                  self.body_frame_id))
        s = std_msgs.msg.String()
        s.data = frame.get("status", "")
        self.pub_status.publish(s)

    # ------------------------------------------------------------ 感知回路
    def on_sensor(self):
        """10 Hz：渲染并发布图像/点云。不推进物理，读的是控制回路停在的状态。"""
        if self.sim_sensor is None:
            return
        want = []
        if self.pub_img is not None:
            want.append("rgb")
        if self.pub_depth is not None:
            want.append("depth")
        if self.pub_seg is not None:
            want.append("seg")
        if self.pub_lidar is not None:
            want.append("lidar")
        if not want:
            return

        t_rpc0 = time.perf_counter()
        try:
            frame = self.sim_sensor.render(want)
        except OSError as e:
            self.get_logger().error("感知通道通信失败：%s" % e)
            return
        t_rpc1 = time.perf_counter()

        stamp = self.get_clock().now().to_msg()
        if self.pub_img is not None and frame.get("rgb") is not None:
            # MuJoCo 输出 RGB，carlair_ros_bridge 发 bgr8：翻转通道使 encoding 名副其实。
            # [..., ::-1] 是视图，to_image 内 ascontiguousarray 会实化。
            self.pub_img.publish(to_image(frame["rgb"][..., ::-1], stamp,
                                          "camera_link", "bgr8"))
        if self.pub_depth is not None and frame.get("depth") is not None:
            self.pub_depth.publish(to_image(frame["depth"], stamp,
                                            "camera_link", "32FC1"))
        if self.pub_seg is not None and frame.get("seg") is not None:
            self.pub_seg.publish(to_image(self._colorize_seg(frame["seg"])[..., ::-1],
                                          stamp, "camera_link", "bgr8"))
        if self.pub_lidar is not None and frame.get("lidar") is not None:
            self.pub_lidar.publish(to_pointcloud2(frame["lidar"], stamp,
                                                  self.world_frame))
        self._n_sensor += 1
        t_pub1 = time.perf_counter()
        self._acc_sensor_costs.append(
            (t_rpc1 - t_rpc0, t_pub1 - t_rpc1))

    # ------------------------------------------------------------ 速率报告
    def on_rate_report(self):
        now = time.monotonic()
        dt = now - self._rate_t0
        if dt <= 0:
            return
        with self._lock:
            idle = (now - self._cmd_t) >= self.cmd_timeout
        detail = ""
        if self._acc_sensor_costs:
            import numpy as _np
            a = _np.asarray(self._acc_sensor_costs) * 1000.0
            detail = ("；感知分项 RPC %.0f ms + 发布 %.0f ms（n=%d）"
                      % (a[:, 0].mean(), a[:, 1].mean(), len(a)))
        self.get_logger().info(
            "实测速率：控制 %.1f Hz（目标 %.0f），感知 %.1f Hz（目标 %.0f）%s%s"
            % (self._n_ctrl / dt, self.control_hz,
               self._n_sensor / dt, self.image_hz,
               detail, "  [已超时，自动悬停中]" if idle else ""))
        self._n_ctrl = 0
        self._n_sensor = 0
        self._acc_sensor_costs = []
        self._rate_t0 = now

    # ------------------------------------------------------------ 工具
    def _world_to_body(self, vx, vy):
        """世界系水平速度 → 机体系水平速度；用缓存航向，不为高频控制回调多发一次 RPC。"""
        o = self._pose_cache
        if o is None:
            o = self.sim.observe(["odom"])["odom"]
        yaw = o["rpy"][2]
        c, s = np.cos(-yaw), np.sin(-yaw)
        return c * vx - s * vy, s * vx + c * vy

    @staticmethod
    def _colorize_seg(seg):
        """语义类别 id → 伪彩色图（RGB，发布前翻转为 bgr8）。"""
        idx = np.clip(seg.astype(np.int32), 0, len(SEG_COLORS) - 1)
        return SEG_COLORS[idx]

    def destroy_node(self):
        for c in (self.sim, self.sim_sensor):
            try:
                if c is not None:
                    c.close()
            except Exception:                     # noqa: BLE001
                pass
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    executor = None
    try:
        node = UavSimBridge()
        # 用多线程执行器：单线程下 60 ms 的图像回调会卡住 50 Hz 控制回路
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:                       # noqa: BLE001
        print("桥接节点异常退出：%s" % e)
        raise
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
