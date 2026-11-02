"""ROS2 Humble 节点：边飞边估，把四桨健康度实时发出来。

设计取舍说明（为什么节点内部自己跑仿真）：

健康度估计的输入里有一项是"控制器下发的四个桨推力"。机载可观测的量只有 IMU、
姿态和速度，推力指令在飞控内部。真实的工程做法是把这个估计器放进飞控里；
"独立进程只读话题"这条路需要一个额外的推力指令话题，而底座目前不发布它。

所以本节点的做法与 ``uav_actuator_fault_sim`` 一致：**节点内闭环仿真 + 在线估计**，
同时把"真值"和"估计"都发出来，方便在 rviz2 / rqt 里直接对照。这样演示不需要
额外的仿真器进程，``ros2 launch`` 一条命令就能看到估计曲线跟着故障掉下去。
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from .data import FAULT_CN, PHYSICAL_SCENES, FaultPlan, make_quad
from .features import FEATURE_DIM, ROTOR_COUNT, build_step_features, specific_force
from .infer import HealthEstimator

#: 默认检查点（相对包安装目录或工作目录）
DEFAULT_CHECKPOINT = "results/detector.pt"
#: 未收到指令时的悬停目标高度
HOVER_ALTITUDE = 1.5
#: 判定"已检测到故障"的阈值
FAULT_THRESHOLD = 0.8
#: 包安装目录（share/<pkg>/results 与源码 results/ 都按这个位置去找）
_PKG_ROOT = Path(__file__).resolve().parent.parent


def _resolve_checkpoint(value: str) -> str:
    """把 ``results/detector.pt`` 这类相对路径解析成真实文件路径。

    依次尝试：当前工作目录 → 包目录（源码树）→ 安装后的 share 目录，
    这样 ``ros2 launch`` 从任意目录启动都能找到权重。
    """
    path = Path(value)
    if path.is_file():
        return str(path)

    candidates = [_PKG_ROOT / value]
    try:  # 源码树里没有 ament 索引，装好之后才有
        from ament_index_python.packages import get_package_share_directory

        candidates.append(Path(get_package_share_directory("uav_rotor_fault_detector_nn")) / value)
    except Exception:  # noqa: BLE001 - 没有 ROS 环境时就是普通 Python 场景
        pass

    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return value  # 交给 HealthEstimator 抛它自己的 FileNotFoundError


class _FlightLoop:
    """把底座的四旋翼按"一步一推进"的方式驱动起来。

    与 ``data.simulate_episode`` 用的是同一套物理与故障定义，区别只是这里
    每步返回一个特征向量交给节点发布，而不是攒成一个大数组。
    """

    def __init__(self, api, plan: FaultPlan, scene: str, seed: int, dt: float, substeps: int,
                 imu_noise: float = 0.02, hover_altitude: float = HOVER_ALTITUDE) -> None:
        self.api = api
        self.plan = plan
        self.dt = dt
        self.rng = np.random.default_rng(seed)
        self.max_thrust = 6.0

        self.quad = make_quad(api, scene, seed, dt, substeps)
        self.ctrl = api.GeometricController(self.quad.params)
        self.max_thrust = float(self.quad.params.max_thrust)
        self.quad.reset(pos=(0.0, 0.0, hover_altitude))

        self.hover_altitude = float(hover_altitude)
        self.target = np.array([0.0, 0.0, self.hover_altitude], dtype=np.float64)
        self.v_d = np.zeros(3)
        self.a_d = np.zeros(3)
        self.yaw_d = float(self.rng.uniform(-0.5, 0.5))
        self.imu_noise = float(imu_noise)
        self.prev_vel = np.zeros(3)
        self.steps = 0
        self.crashed = False
        self.crash_time = float("nan")

        self._bias, self._noise_scale = plan.sensor_factor()

    @property
    def time(self) -> float:
        return self.steps * self.dt

    def set_goal(self, x: float, y: float, z: float) -> None:
        """接 ``/uav/goal`` 设置新的悬停目标。"""
        self.target = np.array([float(x), float(y), float(np.clip(z, 0.5, 4.0))],
                               dtype=np.float64)

    def step(self) -> tuple[np.ndarray, np.ndarray, object]:
        """推进一个仿真步，返回 (特征, 真实增益, 状态)。"""
        t = self.time
        u_cmd = np.asarray(self.ctrl.act(self.quad, self.target, self.v_d, self.a_d, self.yaw_d),
                           dtype=np.float64)
        u_cmd = np.clip(u_cmd, 0.0, self.max_thrust)

        gains = self.plan.gains(t)
        self.quad.apply_rotor_thrust(np.clip(gains * u_cmd, 0.0, self.max_thrust))

        self.prev_vel = np.asarray(self.quad.state().vel, dtype=np.float64).copy()
        self.quad.step(1)
        state = self.quad.state()

        acc = specific_force(state.vel, self.prev_vel, self.dt, state.quat)
        gyro = np.asarray(state.omega, dtype=np.float64)
        if self.imu_noise > 0.0:
            acc = acc + self.rng.normal(0.0, self.imu_noise * self._noise_scale, 3)
            gyro = gyro + self.rng.normal(0.0, self.imu_noise * self._noise_scale, 3)
        if any(self._bias):
            gyro = gyro + np.asarray(self._bias, dtype=np.float64)

        step_features = build_step_features(acc, gyro, state.vel, u_cmd, self.max_thrust)

        self.steps += 1
        if float(state.pos[2]) < 0.15 and not self.crashed:
            self.crashed = True
            self.crash_time = t

        return step_features, gains, state


def build_node_class():
    """在需要时才 import rclpy，避免无 ROS 环境下一导入本模块就失败。"""
    import rclpy
    from geometry_msgs.msg import Point, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, String

    from ._compat import load_base

    class FaultDetectorNode(Node):
        """仿真 + 检测 + 发布。"""

        def __init__(self) -> None:
            super().__init__("uav_rotor_fault_detector")
            self._declare_parameters()

            self.api = load_base()
            self.plan = FaultPlan(
                kind=str(self.get_parameter("fault").value),
                rotor=int(self.get_parameter("rotor").value),
                onset=float(self.get_parameter("onset").value),
                severity=float(self.get_parameter("severity").value),
            )
            self.sim = _FlightLoop(
                self.api,
                self.plan,
                scene=str(self.get_parameter("scene").value),
                seed=int(self.get_parameter("seed").value),
                dt=float(self.get_parameter("dt").value),
                substeps=int(self.get_parameter("substeps").value),
                imu_noise=float(self.get_parameter("imu_noise").value),
            )

            checkpoint = _resolve_checkpoint(str(self.get_parameter("checkpoint").value))
            self.estimator = HealthEstimator(checkpoint,
                                             window=int(self.get_parameter("window").value))
            self.window = self.estimator.window
            self.dt = float(self.get_parameter("dt").value)
            self.substeps = int(self.get_parameter("substeps").value)
            self.duration = float(self.get_parameter("duration").value)
            self.publish_rate = float(self.get_parameter("publish_rate").value)
            self.steps_per_tick = max(1, int(round((1.0 / self.publish_rate) / self.dt)))

            self.estimated = np.ones(ROTOR_COUNT, dtype=np.float64)
            self.detected = False
            self.detect_time = float("nan")
            self.finished = False

            self._build_io()
            self.get_logger().info(
                f"健康度检测启动：{self.plan.describe()} | 场景 {self.get_parameter('scene').value} "
                f"| 窗口 {self.window} 步 | 检查点 {checkpoint}"
            )
            self.timer = self.create_timer(1.0 / self.publish_rate, self.on_tick)

        # ------------------------------------------------------------ 初始化
        def _declare_parameters(self) -> None:
            self.declare_parameter("scene", "open_ground")
            self.declare_parameter("seed", 0)
            self.declare_parameter("dt", 0.01)
            self.declare_parameter("substeps", 4)
            self.declare_parameter("fault", "loss")
            self.declare_parameter("rotor", 0)
            self.declare_parameter("onset", 2.0)
            self.declare_parameter("severity", 1.0)
            self.declare_parameter("imu_noise", 0.02)
            self.declare_parameter("checkpoint", DEFAULT_CHECKPOINT)
            self.declare_parameter("window", 0)  # 0 = 用检查点里记录的长度
            self.declare_parameter("duration", 10.0)
            self.declare_parameter("publish_rate", 50.0)

        def _build_io(self) -> None:
            self.pub_odom = self.create_publisher(Odometry, "/uav/odom", 10)
            self.pub_status = self.create_publisher(String, "/uav/status", 10)
            self.pub_true = self.create_publisher(Float32MultiArray, "/uav/health_true", 10)
            self.pub_est = self.create_publisher(Float32MultiArray, "/uav/health_est", 10)
            self.pub_fault = self.create_publisher(String, "/uav/fault", 10)

            self.create_subscription(Twist, "/uav/cmd_vel", self.on_cmd_vel, 10)
            self.create_subscription(Point, "/uav/goal", self.on_goal, 10)

        # ------------------------------------------------------------ 回调
        def on_cmd_vel(self, msg) -> None:
            """把速度指令换算成"目标点 = 当前位置 + v * 1 s"，复用位置控制器。"""
            state = self.sim.quad.state()
            grip = 1.0
            self.sim.set_goal(
                float(state.pos[0]) + msg.linear.x * grip,
                float(state.pos[1]) + msg.linear.y * grip,
                float(state.pos[2]) + msg.linear.z * grip,
            )

        def on_goal(self, msg) -> None:
            self.sim.set_goal(msg.x, msg.y, msg.z)

        def on_tick(self) -> None:
            if self.finished:
                return

            for _ in range(self.steps_per_tick):
                step_features, gains, state = self.sim.step()
                estimate = self.estimator.push(step_features)
                if estimate is not None:
                    self.estimated = estimate

                t = self.sim.time
                if np.any(gains < 0.99) and not self.detected:
                    if self.estimator.ready and float(np.min(self.estimated)) < FAULT_THRESHOLD:
                        self.detected = True
                        self.detect_time = t
                        delay = t - self.plan.onset
                        message = (f"检测到桨故障：{FAULT_CN.get(self.plan.kind, self.plan.kind)}"
                                   f"@{self.plan.onset:.2f}s，延时 {delay:.2f}s")
                        self.get_logger().warn(message)
                        self.pub_fault.publish(String(data=message))

                if t >= self.duration:
                    self.finished = True
                    break

            self._publish(state, gains)

        # ------------------------------------------------------------ 发布
        def _publish(self, state, gains) -> None:
            stamp = self.get_clock().now().to_msg()

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = float(state.pos[0])
            odom.pose.pose.position.y = float(state.pos[1])
            odom.pose.pose.position.z = float(state.pos[2])
            odom.pose.pose.orientation.w = float(state.quat[0])
            odom.pose.pose.orientation.x = float(state.quat[1])
            odom.pose.pose.orientation.y = float(state.quat[2])
            odom.pose.pose.orientation.z = float(state.quat[3])
            odom.twist.twist.linear.x = float(state.vel[0])
            odom.twist.twist.linear.y = float(state.vel[1])
            odom.twist.twist.linear.z = float(state.vel[2])
            odom.twist.twist.angular.x = float(state.omega[0])
            odom.twist.twist.angular.y = float(state.omega[1])
            odom.twist.twist.angular.z = float(state.omega[2])
            self.pub_odom.publish(odom)

            self.pub_true.publish(Float32MultiArray(data=[float(v) for v in gains]))
            self.pub_est.publish(Float32MultiArray(data=[float(v) for v in self.estimated]))

            if self.sim.crashed:
                status = "CRASHED"
            elif self.finished:
                status = "DONE"
            elif self.detected:
                status = "FAULT"
            elif self.estimator.ready:
                status = "MONITOR"
            else:
                status = "WARMUP"
            self.pub_status.publish(String(data=status))

    return FaultDetectorNode


def main(argv=None) -> int:
    """``ros2 run uav_rotor_fault_detector_nn detector_node`` 的入口。"""
    import rclpy

    rclpy.init(args=argv)
    node = None
    try:
        node = build_node_class()()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.get_logger().info("检测节点正常退出")
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
