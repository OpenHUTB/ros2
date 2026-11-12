"""ROS2 节点：把容错分配层接到话题上。

节点自己跑闭环（仿真 + 控制器都在进程内），发布飞行状态、健康度与四桨指令；
外部可以发 ``/uav/goal`` 改目标点，也可以 ``/uav/cmd_vel`` 给一个速度偏置。

与其他模块保持同样的接口约定：``/uav/odom`` 是 ``nav_msgs/Odometry``、
``/uav/status`` 是 ``std_msgs/String``、``/uav/health`` 与 ``/uav/rotor_cmd``
是 ``std_msgs/Float32MultiArray``。
"""

from __future__ import annotations

from pathlib import Path

DEFAULT_CHECKPOINT = "results/allocator.pt"
NODE_NAME = "uav_neural_fault_tolerant_control"
HOVER_ALTITUDE = 1.5
GOAL_TIMEOUT = 0.5


def _resolve_checkpoint(value: str) -> str:
    """按 cwd → 包目录 → share 目录的顺序找权重，找不到就报错并给出补救命令。"""
    candidate = Path(value).expanduser()
    if candidate.is_file():
        return str(candidate)

    pkg_root = Path(__file__).resolve().parent.parent
    for base in (pkg_root,):
        probe = base / value
        if probe.is_file():
            return str(probe)

    try:  # 安装之后权重在 share/<pkg>/results 下
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("uav_neural_fault_tolerant_control"))
        probe = share / value
        if probe.is_file():
            return str(probe)
    except Exception:  # pragma: no cover - 没有 ROS 时走普通路径
        pass

    raise RuntimeError(
        f"找不到权重文件 {value}\n"
        "先训练一个（约 30 s 数据 + 2 s 训练）：\n"
        "    python main.py --gen-data results/allocator_dataset.npz\n"
        "    python main.py --train\n"
        f"或者用绝对路径指定：-p checkpoint:=/abs/path/allocator.pt\n"
        f"已尝试：{Path.cwd() / value}、{pkg_root / value}"
    )


def build_node_class():
    """延迟构造节点类，import 本模块时不需要 rclpy。"""
    import rclpy
    from geometry_msgs.msg import Point, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, String

    from . import data as D
    from . import faults as FL
    from . import model as M
    from . import trajectory as TJ
    from ._compat import load_base
    from .controller import FaultTolerantController

    class _TolerantControlNode(Node):
        def __init__(self) -> None:
            super().__init__(NODE_NAME)
            self._declare_parameters()

            self.api = load_base()
            self.params, self.effectiveness = D.build_context(self.api)
            self.plan = self._build_plan()
            self.net = self._load_net()
            self.trajectory = self.get_parameter("trajectory").value

            self.quad = self.api.Quadrotor(
                scene=self.api.make_scene(self.get_parameter("scene").value,
                                          seed=int(self.get_parameter("seed").value)),
                dt=float(self.get_parameter("dt").value),
                substeps=int(self.get_parameter("substeps").value))
            self.quad.reset([0.0, 0.0, HOVER_ALTITUDE], 0.0)

            base = self.api.GeometricController(self.params)
            self.ctrl = FaultTolerantController(
                self.params, self.api.mixer, base, self.effectiveness,
                mode=self.get_parameter("mode").value, net=self.net)

            self._build_io()
            self.time = 0.0
            self.finished = False
            self.goal = None
            self.goal_time = -1.0
            self.offset = None
            self.cmd_time = -1.0
            self._last_status = ""

            period = 1.0 / float(self.get_parameter("publish_rate").value)
            self.create_timer(period, self.on_tick)
            self.get_logger().info(
                f"容错控制启动：模式 {self.get_parameter('mode').value} | "
                f"{self.plan.label()} | 轨迹 {self.trajectory} | "
                f"时长 {self.get_parameter('duration').value} s")

        # -- 参数 -----------------------------------------------------------
        def _declare_parameters(self) -> None:
            defaults = (
                ("scene", "open_ground"), ("seed", 0), ("dt", 0.01),
                ("substeps", 4), ("fault", "degradation"), ("rotor", 1),
                ("onset", 2.0), ("severity", 0.5), ("trajectory", "hover"),
                ("mode", "neural"), ("checkpoint", DEFAULT_CHECKPOINT),
                ("duration", 10.0), ("publish_rate", 50.0),
                ("enable_tf", True),
            )
            for name, value in defaults:
                self.declare_parameter(name, value)

        def _build_plan(self) -> "FL.FaultPlan":
            kind = self.get_parameter("fault").value
            rotor = int(self.get_parameter("rotor").value)
            onset = float(self.get_parameter("onset").value)
            severity = float(self.get_parameter("severity").value)
            kwargs = {"kind": kind, "rotor": rotor, "onset": onset,
                      "severity": severity}
            if kind == "dual_loss":
                kwargs["rotor2"] = (rotor + 2) % 4
            return FL.FaultPlan(**kwargs)

        def _load_net(self):
            mode = self.get_parameter("mode").value
            path = _resolve_checkpoint(str(self.get_parameter("checkpoint").value))
            self.get_logger().info(f"权重：{path}")
            net, meta = M.load_checkpoint(path)
            self.get_logger().info(
                f"网络参数量 {meta.get('n_parameters')}，"
                f"验证 MAE {meta.get('best_val_mae')}")
            return net if mode == "neural" else None

        # -- 话题 -----------------------------------------------------------
        def _build_io(self) -> None:
            self.pub_odom = self.create_publisher(Odometry, "/uav/odom", 10)
            self.pub_status = self.create_publisher(String, "/uav/status", 10)
            self.pub_health = self.create_publisher(Float32MultiArray, "/uav/health", 10)
            self.pub_rotor = self.create_publisher(Float32MultiArray, "/uav/rotor_cmd", 10)
            self.pub_fault = self.create_publisher(String, "/uav/fault", 10)
            self.create_subscription(Point, "/uav/goal", self.on_goal, 10)
            self.create_subscription(Twist, "/uav/cmd_vel", self.on_cmd, 10)
            self.pub_fault.publish(String(data=self.plan.label()))

        def on_goal(self, msg: Point) -> None:
            self.goal = [float(msg.x), float(msg.y), float(msg.z)]
            self.goal_time = self.time

        def on_cmd(self, msg: Twist) -> None:
            self.offset = [float(msg.linear.x), float(msg.linear.y),
                           float(msg.linear.z)]
            self.cmd_time = self.time

        # -- 主循环 ---------------------------------------------------------
        def on_tick(self) -> None:
            if self.finished:
                return
            dt = float(self.get_parameter("dt").value)
            rate = float(self.get_parameter("publish_rate").value)
            steps = max(1, int(round((1.0 / rate) / dt)))

            for _ in range(steps):
                t = self.time
                gains = self.plan.gains(t)
                self.ctrl.set_health(gains, self.plan.bias(t, self.params.hover_thrust))
                p_d, v_d, a_d, yaw_d = TJ.reference(self.trajectory, t)

                if self.goal is not None and (t - self.goal_time) < GOAL_TIMEOUT:
                    p_d = list(self.goal)
                    v_d = [0.0, 0.0, 0.0]
                    a_d = [0.0, 0.0, 0.0]
                elif self.offset is not None and (t - self.cmd_time) < GOAL_TIMEOUT:
                    p_d = [p_d[0] + self.offset[0], p_d[1] + self.offset[1],
                           p_d[2] + self.offset[2]]

                u = self.ctrl.act(self.quad, p_d, v_d, a_d, yaw_d)
                u_act = gains * u + self.plan.bias(t, self.params.hover_thrust)
                self.quad.apply_rotor_thrust(u_act)
                self.quad.step(1)
                self.time += dt

            self.publish()

            duration = float(self.get_parameter("duration").value)
            if self.time >= duration:
                self.finished = True
                self.get_logger().info(f"飞行结束：{self.time:.2f} s，正常退出")

        def publish(self) -> None:
            state = self.quad.state()
            t = self.time

            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
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
            self.pub_odom.publish(odom)

            if self.ctrl.last_u is not None:
                self.pub_rotor.publish(Float32MultiArray(
                    data=[float(v) for v in self.ctrl.last_u]))
            self.pub_health.publish(Float32MultiArray(
                data=[float(v) for v in self.plan.gains(t)]))

            status = "CRASHED" if state.pos[2] < 0.15 else (
                "FAULT" if float(self.plan.gains(t).min()) < 0.99 else "HOVER")
            if status != self._last_status:
                self.pub_status.publish(String(data=status))
                self._last_status = status

    return _TolerantControlNode


def main(argv=None) -> int:
    import rclpy

    rclpy.init(args=argv)
    node = build_node_class()()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
