"""ROS 2 节点：MuJoCo 膝关节 + 两条肌肉，PD 跟踪内置轨迹或外部目标。

话题：
    /knee/angle        std_msgs/Float64           关节角 [deg]
    /knee/activation   std_msgs/Float64MultiArray [屈肌, 伸肌] 激活 0..1
    /knee/torque       std_msgs/Float64           肌肉净力矩 [N·m]
    /knee/reference    std_msgs/Float64           外部目标角 [deg]（覆盖内置轨迹）

收到 `/knee/reference` 后进入外部跟踪模式，不再受 `duration` 限制。
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from .config import Config, load_config
from .model import KneeSim
from .simulation import sine_ref, step_ref


class KneeNode(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_musculoskeletal_knee_tracking")
        self.declare_parameter("dt", 0.005)
        self.declare_parameter("duration", 30.0)
        self.declare_parameter("trajectory_type", "sine")
        self.declare_parameter("amplitude_deg", 40.0)
        self.declare_parameter("frequency_hz", 0.4)
        self.declare_parameter("bias_deg", 30.0)
        self.declare_parameter("kp", 30.0)
        self.declare_parameter("kd", 3.0)
        self.declare_parameter("max_torque", 17.5)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("exit_after_duration", False)

        self.cfg = self._config_from_params()
        self.sim = KneeSim(self.cfg)
        self.traj = self.cfg.trajectory
        self.sim.reset(self.traj.bias_deg if self.traj.type == "sine"
                       else self.traj.step_start_deg)

        self.pub_angle = self.create_publisher(Float64, "/knee/angle", 10)
        self.pub_activation = self.create_publisher(
            Float64MultiArray, "/knee/activation", 10)
        self.pub_torque = self.create_publisher(Float64, "/knee/torque", 10)
        self.create_subscription(Float64, "/knee/reference",
                                 self._on_reference, 10)

        self._external_ref: float | None = None
        self._elapsed = 0.0
        self._done = False
        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0:
            raise ValueError("publish_rate 必须 > 0")
        self.period = 1.0 / rate
        self._steps_per_tick = max(1, int(round(self.period / self.cfg.sim.dt)))
        self._exit_after = bool(self.get_parameter("exit_after_duration").value)
        self.get_logger().info(
            f"knee_tracking 启动：dt={self.cfg.sim.dt:g}s，"
            f"trajectory={self.traj.type}，kp={self.cfg.ctrl.kp:g}，"
            f"kd={self.cfg.ctrl.kd:g}，每拍 {self._steps_per_tick} 步"
        )

    def _config_from_params(self) -> Config:
        base = load_config()
        import dataclasses

        sim = dataclasses.replace(base.sim,
                                  dt=float(self.get_parameter("dt").value),
                                  duration=float(self.get_parameter("duration").value),
                                  exit_after_duration=bool(
                                      self.get_parameter("exit_after_duration").value))
        traj = dataclasses.replace(
            base.trajectory,
            type=str(self.get_parameter("trajectory_type").value),
            amplitude_deg=float(self.get_parameter("amplitude_deg").value),
            frequency_hz=float(self.get_parameter("frequency_hz").value),
            bias_deg=float(self.get_parameter("bias_deg").value))
        ctrl = dataclasses.replace(
            base.ctrl,
            kp=float(self.get_parameter("kp").value),
            kd=float(self.get_parameter("kd").value),
            max_torque=float(self.get_parameter("max_torque").value))
        return dataclasses.replace(base, sim=sim, trajectory=traj, ctrl=ctrl)

    def _on_reference(self, msg: Float64) -> None:
        self._external_ref = float(msg.data)
        self.get_logger().info(f"收到外部目标 {self._external_ref:.2f} deg，切到外部跟踪")

    @property
    def done(self) -> bool:
        return self._done

    def reference(self, t: float) -> float:
        if self._external_ref is not None:
            return self._external_ref
        if self.traj.type == "sine":
            return sine_ref(t, self.traj.bias_deg, self.traj.amplitude_deg,
                            self.traj.frequency_hz)
        return step_ref(t, self.traj.step_start_deg, self.traj.step_target_deg,
                        self.traj.step_time)

    def step_once(self) -> None:
        import numpy as np

        for _ in range(self._steps_per_tick):
            ref = self.reference(self._elapsed)
            self.sim.step(np.radians(ref))
            self._elapsed += self.cfg.sim.dt
        st = self.sim.state()
        self.pub_angle.publish(Float64(data=float(np.degrees(st.theta_rad))))
        self.pub_activation.publish(
            Float64MultiArray(data=[float(st.flex), float(st.ext)]))
        self.pub_torque.publish(Float64(data=float(st.tau_muscle)))

        if (self._external_ref is None and self._exit_after
                and self._elapsed >= self.cfg.sim.duration):
            self._done = True
            self.get_logger().info(
                f"内置轨迹跑完 {self.cfg.sim.duration:.1f}s，安全停止"
            )


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
    parser.add_argument("--duration", type=float)
    parser.add_argument("--type", dest="trajectory_type", choices=("sine", "step"))
    parser.add_argument("--exit-after-duration", action="store_true")
    known, _ = parser.parse_known_args(argv)

    overrides: list[str] = []
    if known.duration is not None:
        overrides += ["-p", f"duration:={known.duration}"]
    if known.trajectory_type is not None:
        overrides += ["-p", f"trajectory_type:={known.trajectory_type}"]
    if known.exit_after_duration:
        overrides += ["-p", "exit_after_duration:=true"]

    if ros_args:
        init_args = [ros_args[0], *overrides, *ros_args[1:]]
    elif overrides:
        init_args = ["--ros-args", *overrides]
    else:
        init_args = None

    rclpy.init(args=init_args)
    node = KneeNode()
    try:
        next_tick = time.monotonic()
        while rclpy.ok() and not node.done:
            # timeout_sec=0.0 在 Jazzy 上会一直阻塞，这里给一个很小的非零值
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
