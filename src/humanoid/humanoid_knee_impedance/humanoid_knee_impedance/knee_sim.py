"""膝关节阻抗阶跃响应仿真节点。

把简化人形模型固定在台架上（`useFixedBase=True`），髋与踝用位置控制
锁死，只有膝关节由阻抗律 `tau = kp*(q_des - q) - kd*qd` 驱动。
前 hold_time 秒保持初始角，随后把目标角阶跃到 step_target，
按固定仿真步长记录采样：全分辨率数据在结束时写入 CSV，同时以
knee_sample 话题发布，便于 rqt_plot 等工具实时观察。

台架化是刻意的：落地冲击的首峰由脚地接触力学主导，膝关节阻抗的贡献
被压在噪声量级；放到固定基座、只做单关节阶跃，才能干净地量出 kp/kd
对上升时间、超调、调节时间与稳态误差的影响。

注意: PyBullet 关节默认带一个目标速度为 0 的隐藏速度电机，会吃掉
外部施加的力矩；TORQUE_CONTROL 前必须先用 VELOCITY_CONTROL(force=0)
关闭默认电机，否则力矩完全无效。
"""

import csv
import os

import pybullet as p
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .controller import impedance_torque

PACKAGE = 'humanoid_knee_impedance'
KNEE_JOINTS = ('left_knee', 'right_knee')
LOCKED_JOINTS = ('left_hip', 'right_hip', 'left_ankle', 'right_ankle')
SAMPLE_KEYS = ('t', 'q', 'qd', 'tau', 'q_cmd')
BASE_HEIGHT = 0.9


class KneeSimNode(Node):
    """单关节阶跃响应仿真：固定基座 + 双膝同步阻抗控制。"""

    def __init__(self):
        super().__init__('knee_sim')
        self.declare_parameter('knee_kp', 10.0)
        self.declare_parameter('knee_kd', 0.5)
        self.declare_parameter('step_target', 0.6)
        self.declare_parameter('hold_angle', 0.0)
        self.declare_parameter('hold_time', 0.3)
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('torque_limit', 8.0)
        self.declare_parameter('use_gui', False)
        self.declare_parameter('sim_time_step', 1.0 / 1000.0)
        self.declare_parameter('timer_period', 0.005)
        self.declare_parameter('out_dir', 'results')
        # 位置控制关节的增益，显式写出便于复现（等于 PyBullet 默认值）
        self.declare_parameter('position_gain', 0.1)
        self.declare_parameter('velocity_gain', 1.0)

        self.kp = float(self.get_parameter('knee_kp').value)
        self.kd = float(self.get_parameter('knee_kd').value)
        self.step_target = float(self.get_parameter('step_target').value)
        self.hold_angle = float(self.get_parameter('hold_angle').value)
        self.hold_time = float(self.get_parameter('hold_time').value)
        self.duration = float(self.get_parameter('duration').value)
        self.torque_limit = float(self.get_parameter('torque_limit').value)
        self.sim_dt = float(self.get_parameter('sim_time_step').value)
        self.position_gain = float(self.get_parameter('position_gain').value)
        self.velocity_gain = float(self.get_parameter('velocity_gain').value)
        self.out_dir = self.get_parameter('out_dir').value
        if self.kp <= 0.0 or self.kd < 0.0:
            raise RuntimeError('knee_kp 必须为正、knee_kd 必须非负')

        self.client = self._connect()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.sim_dt)
        self.robot_id = self._load_robot()
        self.joint_indices = self._build_joint_map()
        self._reset_pose()
        self._disable_default_motors()

        # 队列深度取 1000（1 s 缓冲）：1 kHz 采样下首峰是单点极值，
        # 丢一条就会失真，不能让默认的 depth 10 成为瓶颈。
        self.publisher = self.create_publisher(
            Float64MultiArray, 'knee_sample', 1000)
        self.accumulator = 0.0
        self.last_time = self.get_clock().now()
        self.sim_time = 0.0
        self.max_substeps = 10
        self.max_lag = 0.05
        self.last_torque = 0.0
        self.samples = []
        self.running = True
        self.timer = self.create_timer(
            float(self.get_parameter('timer_period').value),
            self.simulation_step)
        self.get_logger().info(
            '膝关节阶跃仿真已启动: kp={:.2f} N·m/rad, kd={:.2f} N·m·s/rad, '
            '目标 {:.2f} rad'.format(self.kp, self.kd, self.step_target))

    def _connect(self):
        """连接 PyBullet；GUI 不可用时回退到 DIRECT。"""
        if self.get_parameter('use_gui').value:
            try:
                client = p.connect(p.GUI)
            except Exception as error:
                self.get_logger().warning(
                    'GUI 连接失败（{}），回退到 DIRECT 模式'.format(error))
                client = p.connect(p.DIRECT)
        else:
            client = p.connect(p.DIRECT)
        if client < 0:
            raise RuntimeError('无法连接 PyBullet 仿真后端')
        return client

    def _load_robot(self):
        pkg_path = get_package_share_directory(PACKAGE)
        urdf_path = os.path.join(pkg_path, 'urdf', 'simple_humanoid.urdf')
        robot_id = p.loadURDF(
            urdf_path, [0, 0, BASE_HEIGHT], useFixedBase=True)
        for link in range(-1, p.getNumJoints(robot_id)):
            p.changeDynamics(robot_id, link,
                             linearDamping=0.0, angularDamping=0.0)
        return robot_id

    def _build_joint_map(self):
        """按名字建立关节索引映射，避免依赖 URDF 中的书写顺序。"""
        mapping = {}
        for index in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, index)
            mapping[info[1].decode('utf-8')] = index
        missing = [name for name in LOCKED_JOINTS + KNEE_JOINTS
                   if name not in mapping]
        if missing:
            raise RuntimeError('模型缺少关节: {}'.format(', '.join(missing)))
        return {name: mapping[name]
                for name in LOCKED_JOINTS + KNEE_JOINTS}

    def _reset_pose(self):
        for name, index in self.joint_indices.items():
            p.resetJointState(self.robot_id, index, self.hold_angle
                              if name in KNEE_JOINTS else 0.0)

    def _disable_default_motors(self):
        """关闭膝关节默认速度电机，使 TORQUE_CONTROL 生效。"""
        for name in KNEE_JOINTS:
            p.setJointMotorControl2(
                self.robot_id, self.joint_indices[name],
                p.VELOCITY_CONTROL, force=0.0)

    def _target_now(self):
        return self.step_target if self.sim_time >= self.hold_time \
            else self.hold_angle

    def _apply_controllers(self):
        """髋/踝锁死到 0，双膝按阻抗律输出力矩。"""
        target = self._target_now()
        for name in LOCKED_JOINTS:
            p.setJointMotorControl2(
                self.robot_id, self.joint_indices[name],
                p.POSITION_CONTROL, targetPosition=0.0,
                force=self.torque_limit,
                positionGain=self.position_gain,
                velocityGain=self.velocity_gain)
        for name in KNEE_JOINTS:
            index = self.joint_indices[name]
            angle, velocity = p.getJointState(self.robot_id, index)[:2]
            torque = impedance_torque(angle, velocity, target,
                                      self.kp, self.kd, self.torque_limit)
            p.setJointMotorControl2(
                self.robot_id, index, p.TORQUE_CONTROL, force=torque)
            self.last_torque = torque

    def _record_sample(self):
        """记录一条采样并发布 [t, q, qd, tau, q_cmd]，t 以阶跃时刻为 0。"""
        if self.sim_time < self.hold_time:
            return
        index = self.joint_indices['left_knee']
        angle, velocity = p.getJointState(self.robot_id, index)[:2]
        row = {
            't': self.sim_time - self.hold_time,
            'q': angle,
            'qd': velocity,
            'tau': self.last_torque,
            'q_cmd': self._target_now(),
        }
        self.samples.append(row)
        message = Float64MultiArray()
        message.data = [row[key] for key in SAMPLE_KEYS]
        self.publisher.publish(message)

    def _write_csv(self):
        """把全分辨率采样（1 kHz）落盘。

        数据完整性不依赖话题传输：高采样率话题在 CPU 争抢时可能丢帧，
        而阶跃峰值是单点极值，丢一条就会失真，因此由仿真节点直接写盘。
        """
        os.makedirs(self.out_dir, exist_ok=True)
        path = os.path.join(
            self.out_dir,
            'samples_kp{:.2f}_kd{:.2f}.csv'.format(self.kp, self.kd))
        with open(path, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=SAMPLE_KEYS)
            writer.writeheader()
            writer.writerows(self.samples)
        expected = int(round(self.duration / self.sim_dt))
        self.get_logger().info(
            '已保存 {} 条采样到 {}（理论 {} 条）'.format(
                len(self.samples), path, expected))

    def _stop(self, message):
        """设置退出标志，由主循环的 spin_once 轮询发现后退出。

        不能在这里直接 rclpy.shutdown()：回调内关闭无法唤醒
        spin 的等待集，进程会挂死。
        """
        if not self.running:
            return
        self.running = False
        self.get_logger().info(message)

    def simulation_step(self):
        """按实际经过时间补足固定步长的仿真步进。"""
        if not self.running:
            return
        if not p.isConnected(self.client):
            self._stop('PyBullet 窗口已关闭，停止节点')
            return
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        self.accumulator += min(elapsed, self.max_lag)
        steps = 0
        try:
            while (self.accumulator >= self.sim_dt
                   and steps < self.max_substeps
                   and self.sim_time - self.hold_time < self.duration):
                self._apply_controllers()
                p.stepSimulation()
                self.sim_time += self.sim_dt
                self._record_sample()
                self.accumulator -= self.sim_dt
                steps += 1
        except Exception as error:
            self._stop('仿真步进失败，停止节点：{}'.format(error))
            return
        if self.sim_time - self.hold_time >= self.duration:
            self._write_csv()
            self._stop('阶跃仿真完成: kp={:.2f}, kd={:.2f}, 仿真 {:.2f} s'
                       .format(self.kp, self.kd,
                               self.sim_time - self.hold_time))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = KneeSimNode()
    except Exception as error:
        # 覆盖 PackageNotFoundError 等启动期异常，避免直接抛 traceback
        print('启动失败: {}: {}'.format(type(error).__name__, error))
        rclpy.shutdown()
        return
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if p.isConnected(node.client):
            p.disconnect(node.client)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
