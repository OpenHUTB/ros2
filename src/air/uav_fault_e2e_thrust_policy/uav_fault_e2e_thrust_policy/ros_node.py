"""ROS2 节点：策略自己飞，发布状态与指令。

节点里也是"策略输出即执行"，没有任何手工控制器兜底——否则就不叫端到端了。
故障由参数注入（``fault`` / ``rotor`` / ``onset`` / ``severity``），
健康度真值同时发布到 ``/uav/health``，方便和策略的输入对齐看。
"""

from __future__ import annotations

from typing import Any, Dict

import numpy as np

NODE_NAME = "uav_fault_e2e_thrust_policy"
DEFAULT_CHECKPOINT = "results/policy_cnn.pt"

# 参数默认值的**唯一真源**：config/policy.yaml、launch/policy.launch.py 里的默认值
# 都必须和这张表一致（test/test_entry.py 会三方比对，改一处忘另一处直接失败）。
# 之前 publish_rate 既在常量里又在参数里、fault 在三处各写一个值，就是这张表缺位的后果。
DEFAULT_PARAMETERS: Dict[str, Any] = {
    "scene": "corridor",
    "seed": 0,
    "dt": 0.01,
    "substeps": 4,
    "fault": "degradation",
    "rotor": 2,
    "onset": 2.5,
    "severity": 0.5,
    "mode": "neural",
    "checkpoint": DEFAULT_CHECKPOINT,
    "duration": 30.0,
    "publish_rate": 20.0,
}


def _resolve_checkpoint(value: str) -> str:
    """``results/xxx.pt`` 这种相对路径要能在三种环境下找到。"""
    from pathlib import Path

    p = Path(value)
    if p.is_absolute() and p.is_file():
        return str(p)
    candidates = [Path.cwd() / p, Path(__file__).resolve().parent.parent / p]
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("uav_fault_e2e_thrust_policy")) / p)
    except Exception:  # pragma: no cover - 没有 ROS 环境时走不到
        pass
    for c in candidates:
        if c.is_file():
            return str(c)
    tried = "\n  ".join(str(c) for c in candidates)
    raise RuntimeError(
        f"找不到策略权重 {value}。先跑：python main.py --gen-data && "
        f"python main.py --train，或用 -p checkpoint:=/绝对路径 指定。\n已尝试：\n  {tried}")


def build_node_class():
    import rclpy
    from geometry_msgs.msg import Point
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, String

    from . import _compat
    from . import data as D
    from . import expert as T
    from . import policy as P
    from .env import ThrustEnv

    class E2EThrustNode(Node):
        def __init__(self) -> None:
            super().__init__(NODE_NAME)
            for name, value in DEFAULT_PARAMETERS.items():
                self.declare_parameter(name, value)

            spec = {"kind": str(self.get_parameter("fault").value),
                    "rotor": int(self.get_parameter("rotor").value),
                    "onset": float(self.get_parameter("onset").value),
                    "severity": float(self.get_parameter("severity").value)}
            self.mode = str(self.get_parameter("mode").value)
            self.rate = float(self.get_parameter("publish_rate").value)
            self.duration = float(self.get_parameter("duration").value)
            self.spec = spec
            self.plan = D.make_plan(spec)

            # scene / dt / substeps 也走参数：否则 yaml 里写了这三项节点却不认，
            # 和 PR2 踩过的"yaml 静默失效"是同一类问题。
            task = T.TaskConfig(
                scene=str(self.get_parameter("scene").value),
                dt=float(self.get_parameter("dt").value),
                substeps=int(self.get_parameter("substeps").value),
                duration=self.duration,
            )
            api = _compat.load_base()
            self.env = ThrustEnv(api, task=task, plan=self.plan,
                                 seed=int(self.get_parameter("seed").value))
            self.module = None
            if self.mode == "neural":
                path = _resolve_checkpoint(str(self.get_parameter("checkpoint").value))
                self.module, self.meta = P.load_checkpoint(path)
                self.get_logger().info(
                    f"策略权重：{path}（{self.meta.get('n_parameters', '?')} 参数）")

            self.pub_odom = self.create_publisher(Odometry, "/uav/odom", 10)
            self.pub_status = self.create_publisher(String, "/uav/status", 10)
            self.pub_cmd = self.create_publisher(Float32MultiArray, "/uav/rotor_cmd", 10)
            self.pub_health = self.create_publisher(Float32MultiArray, "/uav/health", 10)
            self.pub_depth = self.create_publisher(Float32MultiArray, "/uav/depth_sample", 10)
            self.create_subscription(Point, "/uav/goal", self.on_goal, 10)

            self.obs = self.env.reset()
            self.history = [self.obs["depth"][-1].copy()] * P.DEPTH_STACK
            self.finished = False
            self._last_goal = None
            self.timer = self.create_timer(1.0 / max(self.rate, 1.0), self.on_tick)
            self.get_logger().info(
                f"端到端推力策略启动：模式 {self.mode} | {self.plan.label()} | "
                f"场景 {self.env.task.scene} | 时长 {self.duration} s")

        def on_goal(self, msg: Point) -> None:
            # 端到端策略没有"目标点"输入，这里只记录一条日志说明收到（不改变行为）
            self.get_logger().info(
                f"收到 /uav/goal（{msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}）；"
                "端到端策略不使用目标点坐标")

        def _commands(self) -> np.ndarray:
            if self.mode == "expert" or self.module is None:
                return self.env.expert_command()
            from . import policy as P2

            act = P2.predict(self.module, {
                "depth": P2.stack_depth(self.history, self.module),
                "proprio": self.obs["proprio"][None]})[0]
            return np.clip(act, 0.0, 1.0)

        def on_tick(self) -> None:
            if self.finished:
                return
            steps = max(1, int(round((1.0 / max(self.rate, 1.0)) / self.env.task.dt)))
            info = None
            for k in range(steps):
                act = self._commands()
                self.history.append(self.obs["depth"][-1].copy())
                # 只在这一拍的最后一步渲染深度图：一拍 5 个物理步，若每步都渲一次，
                # 节点只能跑到 ~3 Hz（渲染约 60 ms/帧），降到 1 次/拍后回到 20 Hz 附近。
                self.obs, info = self.env.step(act, render=(k == steps - 1))
                if info.done:
                    break
            self._publish(info)
            if info is not None and (info.done or self.env.t >= self.duration):
                self.finished = True
                status = "到达" if info.success else ("坠机" if info.crashed else "超时")
                self.get_logger().info(
                    f"飞行结束：t={info.t:.2f}s x={info.position[0]:.2f}m 结果={status}")

        def _publish(self, info) -> None:
            state = self.env.quad.state()
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

            if info is not None:
                status = "SUCCESS" if info.success else (
                    "CRASHED" if info.crashed else "RUNNING")
                self.pub_status.publish(String(data=status))
                cmd = Float32MultiArray()
                cmd.data = [float(v) for v in info.actual]
                self.pub_cmd.publish(cmd)
            health = Float32MultiArray()
            health.data = [float(v) for v in self.env.plan.gains(self.env.t)]
            self.pub_health.publish(health)
            depth = Float32MultiArray()
            depth.data = [float(v) for v in self.obs["depth"].reshape(-1)[::16]]
            self.pub_depth.publish(depth)

    return E2EThrustNode


def main(argv=None) -> int:
    import rclpy

    rclpy.init(args=argv)
    node_cls = build_node_class()
    node = node_cls()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        node.env.close()
        node.destroy_node()
        rclpy.shutdown()
    print("端到端推力策略节点正常退出")
    return 0
