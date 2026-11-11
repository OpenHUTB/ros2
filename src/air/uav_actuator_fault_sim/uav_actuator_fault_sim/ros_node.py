# -*- coding: utf-8 -*-
"""ROS2 节点：把"带故障的飞行"作为标准话题对外暴露。

话题契约与底座 ``uav_mujoco_sim`` 保持一致，因此上层算法换个包也能用：

发布
----
``/uav/odom``      nav_msgs/Odometry     世界系 ENU 位姿与速度
``/uav/status``    std_msgs/String       飞行模式：READY / HOVER / VELOCITY / FAULT / CRASHED
``/uav/fault``     std_msgs/String       当前故障的文字描述与注入状态
``/uav/health``    std_msgs/Float32MultiArray   4 个桨的推力增益真值 g_i(t) ∈ [0,1]
``/tf``            动态变换 odom → base_link
``/camera/image_raw`` sensor_msgs/Image   仅当 ``record_frames=true``

订阅
----
``/uav/cmd_vel``   geometry_msgs/Twist    机体系速度指令（m/s, rad/s）
``/uav/goal``      geometry_msgs/Point    世界系目标点（悬停跟踪）

``cmd_vel`` 超过 0.5 s 未更新会自动回落到悬停，避免断连后飞机一直飞。
"""

from __future__ import annotations

import math

import numpy as np

from ._compat import load_base
from .fault import FaultInjector
from .runner import _make_scene
from .scenario import FAULT_SCENARIOS, make_case

#: 指令超时（秒）：超过则自动悬停
CMD_TIMEOUT = 0.5

#: 机体系速度上限（m/s）与偏航角速度上限（rad/s）
MAX_HSPEED = 1.5
MAX_VSPEED = 1.0
MAX_YAW_RATE = math.radians(90.0)


def main(argv=None) -> int:
    """``ros2 run uav_actuator_fault_sim fault_node`` 的入口。

    这里不用 ``rclpy.spin(node)``，而是自己轮询 ``spin_once``：
    节点跑满 ``seconds`` 后要把 ``finished`` 置位退出，而在定时器回调里直接调用
    ``rclpy.shutdown()`` 会和执行器互相等待（回调不返回 → 执行器不结束 → 死锁）。
    """
    import rclpy

    rclpy.init(args=argv)
    node = _instantiate()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:  # pragma: no cover - 交互式
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    return node.exit_code


def build_node_class():
    """构造节点类（延迟 import rclpy，保证无 ROS 环境也能 import 本模块）。"""
    import rclpy
    from geometry_msgs.msg import Point, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, String

    try:  # 图像发布是可选能力
        from sensor_msgs.msg import Image

        has_image = True
    except Exception:  # pragma: no cover
        Image = None
        has_image = False

    try:
        from tf2_ros import TransformBroadcaster
        from geometry_msgs.msg import TransformStamped

        has_tf = True
    except Exception:  # pragma: no cover
        TransformBroadcaster = None
        TransformStamped = None
        has_tf = False

    class _FaultFlightNode(Node):
        """本包唯一的 ROS 节点。"""

        def __init__(self) -> None:
            super().__init__("uav_actuator_fault_sim")
            self.exit_code = 0
            self.finished = False
            self._declare_parameters()
            self._build_sim()
            self._build_io()

        # ------------------------------------------------------------------ #
        def _declare_parameters(self) -> None:
            p = self.declare_parameter
            self.scene_name = p("scene", "open_ground").value
            self.seed = int(p("seed", 0).value)
            self.dt = float(p("dt", 0.01).value)
            self.substeps = int(p("substeps", 4).value)
            self.fault_name = p("fault", "single_loss").value
            self.rotor = int(p("rotor", 0).value)
            self.onset = float(p("onset", 2.0).value)
            self.severity = float(p("severity", 0.0).value)
            self.controller_name = p("controller", "geometric").value
            self.seconds = float(p("seconds", 0.0).value)
            self.publish_rate = float(p("publish_rate", 50.0).value)
            self.record_frames = bool(p("record_frames", False).value)
            self.frame_width = int(p("frame_width", 320).value)
            self.frame_height = int(p("frame_height", 240).value)

        # ------------------------------------------------------------------ #
        def _build_sim(self) -> None:
            api = load_base()
            self.api = api
            if self.fault_name not in FAULT_SCENARIOS:
                raise ValueError(
                    f"未知故障 {self.fault_name!r}，可选 {tuple(FAULT_SCENARIOS)}")

            case = make_case(self.scene_name, self.fault_name, seed=self.seed,
                             rotor=self.rotor, onset=self.onset,
                             severity=self.severity)
            self.case = case
            scene = _make_scene(api, self.scene_name, self.seed)
            self.quad = api.Quadrotor(scene=scene, dt=self.dt, substeps=self.substeps)
            self.target = np.array([0.0, 0.0, 1.5])
            self.quad.reset(pos=self.target)
            self.ctrl = api.GeometricController(self.quad.params)
            self.injector = FaultInjector(case.build_spec(),
                                          max_thrust=float(self.quad.params.max_thrust))
            self.visual = None
            if self.record_frames and has_image:
                cam = api.CameraConfig(width=self.frame_width,
                                       height=self.frame_height,
                                       quality="fast")
                self.visual = api.VisionSensors(self.quad.model, cam)

            self.t = 0.0
            self.p_d = self.target.copy()
            self.v_cmd = np.zeros(3)
            self.yaw_cmd = 0.0
            self.last_cmd_time = -1e9
            self.crashed = False
            self.mode = "READY"

        # ------------------------------------------------------------------ #
        def _build_io(self) -> None:
            self.pub_odom = self.create_publisher(Odometry, "/uav/odom", 10)
            self.pub_status = self.create_publisher(String, "/uav/status", 10)
            self.pub_fault = self.create_publisher(String, "/uav/fault", 10)
            self.pub_health = self.create_publisher(Float32MultiArray, "/uav/health", 10)
            self.pub_image = (self.create_publisher(Image, "/camera/image_raw", 5)
                              if self.visual is not None else None)
            self.tf_broadcaster = TransformBroadcaster(self) if has_tf else None

            self.create_subscription(Twist, "/uav/cmd_vel", self.on_cmd_vel, 10)
            self.create_subscription(Point, "/uav/goal", self.on_goal, 10)

            period = 1.0 / max(self.publish_rate, 1.0)
            self.create_timer(period, self.on_tick)
            self.get_logger().info(
                f"启动：场景={self.scene_name} 故障={self.fault_name}"
                f"（桨{self.rotor}，{self.onset:.1f}s 注入）dt={self.dt}"
            )

        # ------------------------------------------------------------------ #
        # 回调
        # ------------------------------------------------------------------ #
        def on_cmd_vel(self, msg: Twist) -> None:
            v = np.array([msg.linear.x, msg.linear.y, msg.linear.z], dtype=float)
            v[0] = float(np.clip(v[0], -MAX_HSPEED, MAX_HSPEED))
            v[1] = float(np.clip(v[1], -MAX_HSPEED, MAX_HSPEED))
            v[2] = float(np.clip(v[2], -MAX_VSPEED, MAX_VSPEED))
            self.v_cmd = v
            yaw_rate = float(np.clip(msg.angular.z, -MAX_YAW_RATE, MAX_YAW_RATE))
            self.yaw_cmd += yaw_rate * (1.0 / max(self.publish_rate, 1.0))
            self.last_cmd_time = self.t
            self.mode = "VELOCITY"

        def on_goal(self, msg: Point) -> None:
            self.target = np.array([msg.x, msg.y, max(0.3, msg.z)], dtype=float)
            self.p_d = self.target.copy()
            self.v_cmd = np.zeros(3)
            self.mode = "GOAL"
            self.get_logger().info(f"新目标点 {tuple(round(float(v), 2) for v in self.target)}")

        def on_tick(self) -> None:
            # 已经跑完设定时长：不再推进仿真，等 rclpy 退出。
            if self.finished:
                return

            # 指令超时 → 悬停
            if self.t - self.last_cmd_time > CMD_TIMEOUT and self.mode == "VELOCITY":
                self.v_cmd = np.zeros(3)
                self.mode = "HOVER"

            steps = max(1, int(round((1.0 / max(self.publish_rate, 1.0)) / self.dt)))
            for _ in range(steps):
                self.p_d = self.p_d + self.v_cmd * self.dt
                self.p_d[2] = max(0.3, self.p_d[2])
                u = np.asarray(self.ctrl.act(self.quad, self.p_d, self.v_cmd,
                                             np.zeros(3), self.yaw_cmd), dtype=float)
                u = self.injector.apply(u, self.t)
                self.quad.apply_rotor_thrust(u)
                self.quad.step(1)
                self.t += self.dt

                st = self.quad.state()
                if not self.crashed and st.pos[2] < 0.15:
                    self.crashed = True
                    self.mode = "CRASHED"
                    delay = self.t - self.onset
                    self.get_logger().warn(
                        f"坠机：t={self.t:.2f}s，故障后 {delay:.2f}s 触地")
                if self.seconds > 0 and self.t >= self.seconds:
                    self.mode = "DONE"
                    self.finished = True
                    self.get_logger().info(
                        f"到达设定运行时长 {self.seconds:.1f}s，发布最后一帧后退出")
                    break

            self.publish_all()

        # ------------------------------------------------------------------ #
        def publish_all(self) -> None:
            st = self.quad.state()
            stamp = self.get_clock().now().to_msg()

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = float(st.pos[0])
            odom.pose.pose.position.y = float(st.pos[1])
            odom.pose.pose.position.z = float(st.pos[2])
            qw, qx, qy, qz = (float(v) for v in st.quat)
            odom.pose.pose.orientation.w = qw
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.twist.twist.linear.x = float(st.vel[0])
            odom.twist.twist.linear.y = float(st.vel[1])
            odom.twist.twist.linear.z = float(st.vel[2])
            odom.twist.twist.angular.x = float(st.omega[0])
            odom.twist.twist.angular.y = float(st.omega[1])
            odom.twist.twist.angular.z = float(st.omega[2])
            self.pub_odom.publish(odom)

            status = String()
            status.data = "FAULT" if self.injector.active(self.t) and not self.crashed \
                else self.mode
            self.pub_status.publish(status)

            fault = String()
            state_cn = "已注入" if self.injector.active(self.t) else "未注入"
            fault.data = f"{state_cn} | {self.injector.spec.describe()}"
            self.pub_fault.publish(fault)

            health = Float32MultiArray()
            health.data = [float(g) for g in self.injector.gains(self.t)]
            self.pub_health.publish(health)

            if self.tf_broadcaster is not None:
                tf = TransformStamped()
                tf.header.stamp = stamp
                tf.header.frame_id = "odom"
                tf.child_frame_id = "base_link"
                tf.transform.translation.x = float(st.pos[0])
                tf.transform.translation.y = float(st.pos[1])
                tf.transform.translation.z = float(st.pos[2])
                tf.transform.rotation.w = qw
                tf.transform.rotation.x = qx
                tf.transform.rotation.y = qy
                tf.transform.rotation.z = qz
                self.tf_broadcaster.sendTransform(tf)

            if self.pub_image is not None and int(self.t / self.dt) % 5 == 0:
                rgb = self.visual.rgb(self.quad.data)
                img = Image()
                img.header.stamp = stamp
                img.header.frame_id = "front_cam"
                img.height = int(rgb.shape[0])
                img.width = int(rgb.shape[1])
                img.encoding = "rgb8"
                img.is_bigendian = 0
                img.step = int(rgb.shape[1] * 3)
                img.data = np.ascontiguousarray(rgb).tobytes()
                self.pub_image.publish(img)

        # ------------------------------------------------------------------ #
        def stop(self) -> None:
            if getattr(self, "visual", None) is not None:
                self.visual.close()
                self.visual = None

    return _FaultFlightNode


def _instantiate():
    """供 :func:`main` 使用：构造节点实例。"""
    cls = build_node_class()
    return cls()


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
