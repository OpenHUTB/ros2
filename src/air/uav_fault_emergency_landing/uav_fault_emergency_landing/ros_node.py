"""ROS2 节点：把"故障 → 建图 → 选点 → 迫降"整套逻辑挂到话题上。

与另外两个无人机模块一样是**单进程**：Humble 的 rclpy 就是 Python 3.10，
不需要像底座那样拆仿真/ROS 双进程。节点自己推进 MuJoCo 物理，同时把状态发出去。
"""

from __future__ import annotations

import math
import threading
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from . import _compat
from . import controller as C
from . import evaluate as E
from . import faults as F
from . import landing as L
from . import mapping as M
from . import model as Md

NODE_NAME = "uav_fault_emergency_landing"
HOVER_ALTITUDE = 4.0
DEFAULT_SCORE_CHECKPOINT = "results/score.pt"
DEFAULT_GUIDANCE_CHECKPOINT = "results/guidance.pt"
GOAL_TIMEOUT = 0.5
SCAN_PERIOD = 0.25          # 建图节流（秒，仿真时间）：一帧深度图反投影约 120 ms

# 节点参数默认值。必须与 config/landing.yaml 的键一一对应：
# `test/test_entry.py` 会读那份 yaml 逐键比对，改一处忘了另一处会直接测出来。
DEFAULT_PARAMETERS: Dict[str, Any] = {
    "scene": "urban_block",
    "seed": 0,
    "dt": 0.01,
    "substeps": 4,
    "fault": "degradation",
    "rotor": 2,
    "severity": 0.5,
    "mode": "neural",
    "duration": 40.0,
    "publish_rate": 20.0,
    "score_checkpoint": DEFAULT_SCORE_CHECKPOINT,
    "guidance_checkpoint": DEFAULT_GUIDANCE_CHECKPOINT,
    "enable_tf": False,
}


def _resolve_checkpoint(value: str) -> str:
    """按 cwd → 包目录 → install share 目录 三级查找权重文件。"""
    p = Path(value)
    if p.is_file():
        return str(p)
    pkg_root = Path(__file__).resolve().parent.parent
    candidate = pkg_root / value
    if candidate.is_file():
        return str(candidate)
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("uav_fault_emergency_landing"))
        candidate = share / value
        if candidate.is_file():
            return str(candidate)
    except Exception:  # 没有 ROS 环境时走普通路径
        pass
    tried = [str(Path(value)), str(pkg_root / value)]
    raise RuntimeError(
        "找不到权重文件：" + value + "\n"
        "已尝试：" + "、".join(tried) + "\n"
        "先训练一次：\n    python main.py --gen-data\n    python main.py --train\n"
        "或用 -p score_checkpoint:=/绝对路径 -p guidance_checkpoint:=/绝对路径 指定")


def build_node_class():
    """延迟 import rclpy，这样没有 ROS 的机器也能 import 本模块。"""
    import rclpy
    from geometry_msgs.msg import Point, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, String

    class ForcedLandingNode(Node):
        def __init__(self):
            super().__init__(NODE_NAME)
            for name, value in DEFAULT_PARAMETERS.items():
                self.declare_parameter(name, value)

            self._scan_step = 0
            self._last_scan_time = -1e9
            self._finished = False
            self._last_goal = None
            self._last_goal_time = -math.inf

            self._api = _compat.load_base()
            self._build_sim()
            self._build_io()
            self._build_nets()

            self._timer = self.create_timer(1.0 / max(float(self.publish_rate), 1.0),
                                            self.on_tick)
            self.get_logger().info(
                f"故障迫降节点启动：{self.fault.label()} | 场景 {self.scene} | "
                f"模式 {self.mode} | 时长 {self.duration:.1f} s")

        # ------------------------------------------------------------------ 构建
        def _build_sim(self) -> None:
            self.scene = str(self.get_parameter("scene").value)
            self.seed = int(self.get_parameter("seed").value)
            self.dt = float(self.get_parameter("dt").value)
            self.substeps = int(self.get_parameter("substeps").value)
            self.duration = float(self.get_parameter("duration").value)
            self.publish_rate = float(self.get_parameter("publish_rate").value)
            self.mode = str(self.get_parameter("mode").value)

            spec = dict(kind=str(self.get_parameter("fault").value),
                        rotor=int(self.get_parameter("rotor").value),
                        severity=float(self.get_parameter("severity").value))
            self.fault = F.FaultPlan(**spec)
            self.params, self.effectiveness = E.build_context(self._api)

            # 起飞点选在障碍物正上方：垂直迫降会撞上去，选点迫降应该飞到空地
            start_xy, hover_z = E.pick_obstacle_spot(self._api, self.scene, self.seed)
            self.hover_xy = start_xy
            self.hover_z = hover_z
            self.quad = self._api.Quadrotor(
                scene=self._api.make_scene(self.scene, seed=self.seed),
                dt=self.dt, substeps=self.substeps)
            self.quad.reset(pos=(float(start_xy[0]), float(start_xy[1]), float(hover_z)))
            self.mapper = M.DepthMapper(self._api).bind(self.quad)
            self.base = self._api.GeometricController(self.quad.params)
            self.time = 0.0

        def _build_nets(self) -> None:
            score_net = guidance_net = None
            try:
                score_net, _ = Md.load_checkpoint(
                    _resolve_checkpoint(str(self.get_parameter("score_checkpoint").value)))
                guidance_net, _ = Md.load_checkpoint(
                    _resolve_checkpoint(str(self.get_parameter("guidance_checkpoint").value)))
            except RuntimeError as exc:
                self.get_logger().warn(f"权重没找到，退回规则评分与专家律：{exc}")
            self.ctrl = C.ForcedLandingController(
                self.quad.params, self._api.mixer, self.base, self.effectiveness,
                mode=self.mode if guidance_net is not None else "expert",
                score_net=score_net, guidance_net=guidance_net)

        def _build_io(self) -> None:
            self.pub_odom = self.create_publisher(Odometry, "/uav/odom", 10)
            self.pub_status = self.create_publisher(String, "/uav/status", 10)
            self.pub_target = self.create_publisher(Point, "/uav/landing_target", 10)
            self.pub_health = self.create_publisher(Float32MultiArray, "/uav/health", 10)
            self.pub_score = self.create_publisher(Float32MultiArray, "/uav/landing_scores", 10)
            self.create_subscription(Twist, "/uav/cmd_vel", self.on_cmd_vel, 10)
            self.create_subscription(Point, "/uav/goal", self.on_goal, 10)
            self.enable_tf = bool(self.get_parameter("enable_tf").value)
            self.tf_broadcaster = None
            if self.enable_tf:
                from tf2_ros import TransformBroadcaster

                self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------------------------------------------------------- 回调
        def on_cmd_vel(self, msg: Twist) -> None:
            # 迫降阶段不接受外部速度指令，只用来表示"有人接管了"
            self.get_logger().debug(f"收到 /uav/cmd_vel：{msg.linear.x:.2f}")

        def on_goal(self, msg: Point) -> None:
            self._last_goal = np.array([msg.x, msg.y, msg.z], dtype=float)
            self._last_goal_time = self.time
            self.get_logger().info(f"收到人工指定落点：{self._last_goal.round(2)}")

        # ------------------------------------------------------------------ 主循环
        def on_tick(self) -> None:
            import mujoco
            from geometry_msgs.msg import Point
            from nav_msgs.msg import Odometry
            from std_msgs.msg import Float32MultiArray, String

            if self._finished:
                return
            steps = max(1, int(round((1.0 / max(self.publish_rate, 1.0)) / self.dt)))
            for _ in range(steps):
                if self._finished:
                    break
                self._step(mujoco)
            self._publish(Odometry, String, Float32MultiArray, Point)

        def _step(self, mujoco) -> None:
            t = self.time
            state = self.quad.state()
            gains = self.fault.gains(t)
            bias = self.fault.bias(t, self.params.max_thrust)
            self.ctrl.set_health(gains, bias)

            points = None
            # 深度反投影一帧约 120 ms（WSL 软件渲染），不能每 10 步扫一次，否则
            # 定时器跑不满发布频率。改成按仿真时间节流：悬停阶段每 0.25 s 一帧把地图
            # 建起来；故障后只在还没选出落点时补扫。
            need_scan = (
                (t - self._last_scan_time) >= SCAN_PERIOD
                and (t < self.fault.onset_time() or self.ctrl.plan.target_xy is None)
            )
            if need_scan:
                self._last_scan_time = t
                mujoco.mj_forward(self.quad.model, self.quad.data)
                points = self.mapper.scan(self.quad.data)
            self._scan_step += 1

            if t < self.fault.onset_time():
                thrust, tau = self.base.compute(
                    state, np.array([self.hover_xy[0], self.hover_xy[1], self.hover_z]),
                    np.zeros(3), np.zeros(3), 0.0)
                commands = self._api.mixer(thrust, tau, self.params)
                if points is not None:
                    self.ctrl.update_map(points)
            else:
                if self._last_goal is not None and (t - self._last_goal_time) < GOAL_TIMEOUT:
                    # 人工落点优先，直接把目标设过去
                    self.ctrl.plan.target_xy = self._last_goal[:2].copy()
                commands = self.ctrl.act(state, points)

            actual = np.clip(commands * gains + (0.0 if bias is None else bias),
                             0.0, self.params.max_thrust)
            self.quad.apply_rotor_thrust(actual)
            self.quad.step(1)
            self.time = t + self.dt

            st = self.quad.state()
            if st.pos[2] <= E.LAND_Z or self.time >= self.duration:
                self._finished = True
                self.get_logger().info(
                    f"迫降结束：t={self.time:.2f}s 高度={st.pos[2]:.2f}m "
                    f"结果={self.status()}")

        def status(self) -> str:
            st = self.quad.state()
            if self.time < self.fault.onset_time():
                return "HOVER"
            if abs(st.pos[2]) <= E.LAND_Z + 0.05:
                return "LANDED"
            return "DESCENDING"

        # ------------------------------------------------------------------ 发布
        def _publish(self, Odometry, String, Float32MultiArray, Point) -> None:
            st = self.quad.state()
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = float(st.pos[0])
            odom.pose.pose.position.y = float(st.pos[1])
            odom.pose.pose.position.z = float(st.pos[2])
            odom.pose.pose.orientation.w = float(st.quat[0])
            odom.pose.pose.orientation.x = float(st.quat[1])
            odom.pose.pose.orientation.y = float(st.quat[2])
            odom.pose.pose.orientation.z = float(st.quat[3])
            odom.twist.twist.linear.x = float(st.vel[0])
            odom.twist.twist.linear.y = float(st.vel[1])
            odom.twist.twist.linear.z = float(st.vel[2])
            self.pub_odom.publish(odom)

            msg = String()
            msg.data = self.status()
            self.pub_status.publish(msg)

            if self.ctrl.plan.target_xy is not None:
                target = Point()
                target.x = float(self.ctrl.plan.target_xy[0])
                target.y = float(self.ctrl.plan.target_xy[1])
                target.z = 0.0
                self.pub_target.publish(target)

            health = Float32MultiArray()
            health.data = [float(g) for g in self.fault.gains(self.time)]
            self.pub_health.publish(health)

            score = Float32MultiArray()
            try:
                grid = self.ctrl.score_map(st.pos[:2])
                score.data = [float(np.max(grid)), float(np.mean(grid))]
            except Exception:
                score.data = [0.0, 0.0]
            self.pub_score.publish(score)

            if self.tf_broadcaster is not None:
                from geometry_msgs.msg import TransformStamped

                tf = TransformStamped()
                tf.header.stamp = self.get_clock().now().to_msg()
                tf.header.frame_id = "odom"
                tf.child_frame_id = "base_link"
                tf.transform.translation.x = float(st.pos[0])
                tf.transform.translation.y = float(st.pos[1])
                tf.transform.translation.z = float(st.pos[2])
                tf.transform.rotation.w = float(st.quat[0])
                tf.transform.rotation.x = float(st.quat[1])
                tf.transform.rotation.y = float(st.quat[2])
                tf.transform.rotation.z = float(st.quat[3])
                self.tf_broadcaster.sendTransform(tf)

    return ForcedLandingNode


def main(argv=None) -> int:
    import rclpy

    rclpy.init(args=argv)
    node = build_node_class()()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
