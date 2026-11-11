"""ROS 2 节点：双关节肌肉骨骼手臂到达控制。

话题（与文档一致）：
    /target_position        geometry_msgs/Point           订阅，用 x、z，y 忽略
    /joint_states           sensor_msgs/JointState        肩、肘位置与速度
    /end_effector_position  geometry_msgs/Point           末端在手臂平面的 x-z
    /muscle_activations     std_msgs/Float64MultiArray    6 条肌肉激活
    /muscle_forces          std_msgs/Float64MultiArray    MuJoCo 计算的肌力 [N]

控制周期 10 ms（5 个 2 ms 物理步）。目标不可达或非有限时保留上一有效目标；
仿真异常时清零激活并停止定时器。
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .config import MUSCLES, Config, load_config
from .control import ReachController, inverse_kinematics
from .model import ArmModel


class ArmReachNode(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_musculoskeletal_arm_reach")
        self.declare_parameter("target_x", 0.38)
        self.declare_parameter("target_z", -0.17)
        self.declare_parameter("seconds", 0.0)  # >0 时跑完自动退出（便于测试）
        self.declare_parameter("initial_q1", 0.0)
        self.declare_parameter("initial_q2", 0.6)

        import dataclasses

        base = load_config()
        ctrl = dataclasses.replace(
            base.ctrl,
            target_x=float(self.get_parameter("target_x").value),
            target_z=float(self.get_parameter("target_z").value))
        self.cfg: Config = dataclasses.replace(base, ctrl=ctrl)
        self._seconds = float(self.get_parameter("seconds").value)

        self.arm = ArmModel(self.cfg)
        self.arm.reset(float(self.get_parameter("initial_q1").value),
                       float(self.get_parameter("initial_q2").value))
        self.ctl = ReachController(self.cfg)
        self.target = (self.cfg.ctrl.target_x, self.cfg.ctrl.target_z)
        if inverse_kinematics(self.target[0], self.target[1], self.cfg) is None:
            raise ValueError(f"默认目标不可达：{self.target}")

        self.pub_joint = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_tip = self.create_publisher(Point, "/end_effector_position", 10)
        self.pub_act = self.create_publisher(Float64MultiArray,
                                             "/muscle_activations", 10)
        self.pub_force = self.create_publisher(Float64MultiArray,
                                               "/muscle_forces", 10)
        self.create_subscription(Point, "/target_position",
                                 self._on_target, 10)

        self.period = self.cfg.sim.control_period
        self._elapsed = 0.0
        self._done = False
        self._failed = False
        self.get_logger().info(
            f"arm_reach 启动：目标 ({self.target[0]:.2f}, {self.target[1]:.2f})，"
            f"控制周期 {self.period * 1000:.0f} ms，6 条肌肉"
        )

    # ------------------------------------------------------------------
    def _on_target(self, msg: Point) -> None:
        x, z = float(msg.x), float(msg.z)
        if not (math.isfinite(x) and math.isfinite(z)):
            self.get_logger().warn("拒绝非有限目标，保留上一目标")
            return
        if inverse_kinematics(x, z, self.cfg) is None:
            self.get_logger().warn(
                f"拒绝不可达目标 ({x:.3f}, {z:.3f})，保留上一目标")
            return
        self.target = (x, z)
        self.get_logger().info(f"新目标 ({x:.3f}, {z:.3f})")

    @property
    def done(self) -> bool:
        return self._done or self._failed

    def step_once(self) -> None:
        try:
            self.ctl.step(self.arm, self.target)
            self.arm.step(self.cfg.sim.steps_per_control)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"仿真异常，清零激活并停止：{exc}")
            try:
                self.arm.set_activations(np.zeros(6))
            except Exception:  # noqa: BLE001
                pass
            self._failed = True
            return

        self._elapsed += self.period
        st = self.arm.state()
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["shoulder", "elbow"]
        js.position = [st.q1, st.q2]
        js.velocity = [st.dq1, st.dq2]
        self.pub_joint.publish(js)

        self.pub_tip.publish(Point(x=st.tip_x, y=0.0, z=st.tip_z))
        self.pub_act.publish(Float64MultiArray(
            data=[float(a) for a in st.activations]))
        self.pub_force.publish(Float64MultiArray(
            data=[float(f) for f in st.forces]))

        if self._seconds > 0.0 and self._elapsed >= self._seconds:
            err = float(np.linalg.norm(self.arm.tip_position
                                       - np.array(self.target)))
            self.get_logger().info(
                f"运行 {self._seconds:.1f}s 结束，末端误差 {err:.4f} m")
            self._done = True


def main(argv=None) -> int:
    import argparse
    import sys

    if argv is None:
        argv = sys.argv[1:]
    argv = list(argv)
    ros_args: list[str] = []
    if "--ros-args" in argv:
        idx = argv.index("--ros-args")
        ros_args, argv = argv[idx:], argv[:idx]

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--target-x", type=float)
    parser.add_argument("--target-z", type=float)
    parser.add_argument("--seconds", type=float)
    known, _ = parser.parse_known_args(argv)

    overrides: list[str] = []
    if known.target_x is not None:
        overrides += ["-p", f"target_x:={known.target_x}"]
    if known.target_z is not None:
        overrides += ["-p", f"target_z:={known.target_z}"]
    if known.seconds is not None:
        overrides += ["-p", f"seconds:={known.seconds}"]

    if ros_args:
        init_args = [ros_args[0], *overrides, *ros_args[1:]]
    elif overrides:
        init_args = ["--ros-args", *overrides]
    else:
        init_args = None

    rclpy.init(args=init_args)
    node = ArmReachNode()
    try:
        next_tick = time.monotonic()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.001)
            node.step_once()
            next_tick += node.period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
