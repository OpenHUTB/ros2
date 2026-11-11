"""MuJoCo 仿真节点：物理步进 + LAC 策略锁步闭环。

为什么把策略放在仿真节点里，而不是像上游那样分成 4 个进程走 DDS：

  * 策略与物理必须严格锁步（每 20 ms 一帧，4 个 5 ms 子步）。分成两个进程后，
    `/joint_states` 的到达时刻与 `mj_step` 的相位会漂移，观测里出现重复帧或跳帧，
    在 50 Hz 下足以让站立变得不稳。
  * 上游依赖 CycloneDDS 域 + Xbox 手柄才能启动，评测机两样都没有。

因此本节点内做闭环，对外只暴露 ROS 接口：订阅速度/刚度/外力指令，发布本体状态。
这与上游 `inference` + `relay` + `simulate_python` 三进程方案在控制律上完全等价。
"""

from __future__ import annotations

import json
import time

import numpy as np

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

from .compliance import ComplianceRunner
from .lac.constants import JOINT_NAMES_23DOF
from .policy import (
    Command, LacPolicy, StartupFSM, STIFF_BAND, STIFF_NAMES,
)
from .sim_core import G1Sim


class SimNode(Node):
    def __init__(self):
        super().__init__('g1_compliance_sim')

        self.declare_parameter('stiffness', [250.0, 250.0, 250.0, 55.0, 55.0])
        self.declare_parameter('base_height', 0.74)
        self.declare_parameter('direct_start', True)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('publish_rate', 50.0)

        device = self.get_parameter('device').value
        self.sim = G1Sim()
        self.policy = LacPolicy(device=device)
        self.fsm = StartupFSM(direct=bool(self.get_parameter('direct_start').value))
        self.runner = ComplianceRunner(self.sim, self.policy, self.fsm)

        self.cmd = Command()
        self.cmd.stiffness = np.asarray(
            self.get_parameter('stiffness').value, dtype=np.float64)
        self.cmd.base_height = float(self.get_parameter('base_height').value)

        self._t0 = time.monotonic()
        self._perturb = None
        self._perturb_until = 0.0
        self._frames = 0
        self._infer_sum = 0.0

        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_cmd = self.create_publisher(Float64MultiArray,
                                             '/humanoid/joint_targets', 10)
        self.pub_stiff = self.create_publisher(Float32MultiArray,
                                               '/humanoid/stiffness_state', 10)
        self.pub_status = self.create_publisher(String, '/humanoid/status', 10)

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Float32MultiArray, '/humanoid/stiffness_cmd',
                                 self._on_stiffness, 10)
        self.create_subscription(Float64MultiArray, '/humanoid/external_force',
                                 self._on_force, 10)

        hz = float(self.get_parameter('publish_rate').value)
        self.dt = 1.0 / hz
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'G1 柔顺控制仿真已启动  dt={self.dt * 1000:.1f} ms  '
            f'stiffness={self.cmd.stiffness.tolist()}')

    # ---- 指令回调 ----
    def _on_cmd_vel(self, msg: Twist):
        self.cmd.lin_vel = np.array([msg.linear.x, msg.linear.y], dtype=np.float64)
        self.cmd.ang_vel_yaw = np.array([msg.angular.z], dtype=np.float64)

    def _on_stiffness(self, msg: Float32MultiArray):
        if len(msg.data) != 5:
            self.get_logger().warning(f'刚度需 5 个通道，收到 {len(msg.data)} 个')
            return
        vals = np.asarray(msg.data, dtype=np.float64)
        for i, name in enumerate(STIFF_NAMES):
            lo, hi = STIFF_BAND['K_ang' if 'ang' in name else 'K_lin']
            vals[i] = float(np.clip(vals[i], lo, hi))
        self.cmd.stiffness = vals
        self.get_logger().info(
            '[刚度] ' + '  '.join(f'{n}={v:.0f}' for n, v in zip(STIFF_NAMES, vals)))

    def _on_force(self, msg: Float64MultiArray):
        """一次性外力：[fx, fy, fz, duration_s]，作用在左腕。"""
        if len(msg.data) < 3:
            return
        now = time.monotonic() - self._t0
        dur = float(msg.data[3]) if len(msg.data) > 3 else 1.0
        self._perturb = np.asarray(msg.data[:3], dtype=np.float64)
        self._perturb_until = now + dur
        self.get_logger().info(
            f'[外力] {self._perturb.tolist()} N，持续 {dur:.2f} s')

    # ---- 主循环 ----
    def _tick(self):
        now = time.monotonic() - self._t0
        st = self.sim.read()
        t_infer = time.perf_counter()
        q_policy = self.policy.infer(st, self.cmd)
        infer_ms = (time.perf_counter() - t_infer) * 1000.0
        q_cmd, kp, kd = self.fsm.step(now, st.q, q_policy)

        self.sim.clear_forces()
        if self._perturb is not None and now < self._perturb_until:
            self.sim.apply_force(self.sim.body_id('left_wrist_roll_rubber_hand'),
                                 self._perturb)
        elif self._perturb is not None:
            self._perturb = None

        self.sim.apply_pd(q_cmd, kp, kd)
        self.sim.step(self.runner.substeps)

        st2 = self.sim.read()
        self._publish(now, st2, q_cmd)

        # 每 250 帧（5 s）报一次推理耗时，便于确认实时余量
        self._frames += 1
        self._infer_sum += infer_ms
        if self._frames % 250 == 0:
            self.get_logger().info(
                f'已推理 {self._frames} 帧，平均单帧 '
                f'{self._infer_sum / 250:.2f} ms（周期 {self.dt * 1000:.0f} ms）')
            self._infer_sum = 0.0

    def _publish(self, now: float, st, q_cmd):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINT_NAMES_23DOF)
        js.position = [float(v) for v in st.q]
        js.velocity = [float(v) for v in st.dq]
        js.effort = [float(v) for v in st.tau]
        self.pub_joint.publish(js)

        tgt = Float64MultiArray()
        tgt.data = [float(v) for v in q_cmd]
        self.pub_cmd.publish(tgt)

        sf = Float32MultiArray()
        sf.data = [float(v) for v in self.cmd.stiffness]
        self.pub_stiff.publish(sf)

        pel = self.sim.pelvis_pos()
        pg_z = float(self.sim.projected_gravity(st.quat)[2])
        status = String()
        status.data = json.dumps({
            't': round(now, 3),
            'state': self.fsm.state,
            'pelvis': [round(float(v), 4) for v in pel],
            'pg_z': round(pg_z, 4),
            'stiffness': [round(float(v), 1) for v in self.cmd.stiffness],
            'cmd_vel': [round(float(v), 3) for v in self.cmd.lin_vel],
            'fallen': bool(pg_z > -0.5),
        }, ensure_ascii=False)
        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
