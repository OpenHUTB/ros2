"""双足落地冲击仿真节点。

从指定高度释放蹲姿人形机器人，按控制器模式（stiff / impedance）
驱动关节，逐步发布落地过程采样到 landing_sample 话题。

两种控制器模式:
    stiff: 髋/膝/踝全部位置保持在蹲姿角（刚性保持）。
    impedance: 髋/踝位置保持，膝关节切换为虚拟弹簧-阻尼力矩控制。

注意: PyBullet 关节默认带一个目标速度为 0 的隐藏速度电机，会吃掉
外部施加的力矩；进行 TORQUE_CONTROL 前必须先用
VELOCITY_CONTROL(force=0) 关闭默认电机，否则力矩完全无效。
"""

import math
import os

import pybullet as p
import pybullet_data
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .controller import impedance_torque

GRAVITY = 9.81
ROBOT_MASS = 3.0
HOLD_JOINTS = ('left_hip', 'right_hip', 'left_ankle', 'right_ankle')
KNEE_JOINTS = ('left_knee', 'right_knee')
FOOT_LINK_JOINTS = ('left_ankle', 'right_ankle')
VALID_MODES = ('stiff', 'impedance')


class LandingSimNode(Node):
    """PyBullet 落地仿真节点：执行一次跌落实验并发布采样序列。"""

    def __init__(self):
        super().__init__('landing_sim')
        self.declare_parameter('controller_mode', 'impedance')
        self.declare_parameter('drop_height', 0.05)
        self.declare_parameter('crouch_angle', 0.3)
        self.declare_parameter('knee_kp', 12.0)
        self.declare_parameter('knee_kd', 2.0)
        self.declare_parameter('torque_limit', 8.0)
        self.declare_parameter('duration', 3.0)
        self.declare_parameter('use_gui', False)
        self.declare_parameter('sim_time_step', 1.0 / 240.0)
        self.declare_parameter('contact_stiffness', 1.0e5)
        self.declare_parameter('contact_damping', 100.0)

        self.mode = self.get_parameter('controller_mode').value
        if self.mode not in VALID_MODES:
            raise RuntimeError(
                'controller_mode 必须为 {} 之一'.format(VALID_MODES))
        self.drop_height = float(self.get_parameter('drop_height').value)
        self.crouch = float(self.get_parameter('crouch_angle').value)
        self.knee_kp = float(self.get_parameter('knee_kp').value)
        self.knee_kd = float(self.get_parameter('knee_kd').value)
        self.torque_limit = float(self.get_parameter('torque_limit').value)
        self.duration = float(self.get_parameter('duration').value)
        self.sim_dt = float(self.get_parameter('sim_time_step').value)
        self.contact_stiffness = float(
            self.get_parameter('contact_stiffness').value)
        self.contact_damping = float(
            self.get_parameter('contact_damping').value)

        # 蹲姿角: 髋/踝反向补偿膝关节，保持脚掌水平、重心居中
        self.pose = {
            'hip': -self.crouch / 2.0,
            'knee': self.crouch,
            'ankle': -self.crouch / 2.0,
        }
        self.stand_z = (0.2 + 0.3 * math.cos(self.crouch / 2.0)
                        + 0.3 * math.cos(self.crouch / 2.0) + 0.02)

        self.client = self._connect()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -GRAVITY)
        p.setTimeStep(self.sim_dt)
        self.plane_id = p.loadURDF('plane.urdf')
        p.changeDynamics(self.plane_id, -1,
                         lateralFriction=2.0,
                         contactStiffness=self.contact_stiffness,
                         contactDamping=self.contact_damping)
        self.robot_id = self._load_robot()
        self.joint_indices = self._build_joint_map()
        self._reset_pose()
        if self.mode == 'impedance':
            self._disable_default_motors()

        self.publisher = self.create_publisher(
            Float64MultiArray, 'landing_sample', 10)
        self.accumulator = 0.0
        self.last_time = self.get_clock().now()
        self.sim_time = 0.0
        self.max_substeps = 10
        self.running = True
        self.camera_ready = False
        self.timer = self.create_timer(self.sim_dt, self.simulation_step)
        self.get_logger().info(
            '落地仿真已启动: 模式 {}，跌落高度 {:.2f} m，蹲姿膝角 {:.2f} rad'
            .format(self.mode, self.drop_height, self.crouch))

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
        pkg_path = get_package_share_directory(
            'humanoid_landing_impedance')
        urdf_path = os.path.join(pkg_path, 'urdf', 'simple_humanoid.urdf')
        robot_id = p.loadURDF(
            urdf_path, [0, 0, self.stand_z + self.drop_height])
        for link in range(-1, p.getNumJoints(robot_id)):
            p.changeDynamics(robot_id, link,
                             linearDamping=0.1,
                             angularDamping=0.1,
                             lateralFriction=1.5,
                             contactStiffness=self.contact_stiffness,
                             contactDamping=self.contact_damping)
        return robot_id

    def _build_joint_map(self):
        """按名字建立关节索引映射，避免依赖 URDF 中的书写顺序。"""
        mapping = {}
        for index in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, index)
            mapping[info[1].decode('utf-8')] = index
        missing = [name for name in HOLD_JOINTS + KNEE_JOINTS
                   if name not in mapping]
        if missing:
            raise RuntimeError('模型缺少关节: {}'.format(', '.join(missing)))
        return {name: mapping[name]
                for name in HOLD_JOINTS + KNEE_JOINTS}

    def _reset_pose(self):
        """把六个关节初始化到蹲姿角。"""
        for side in ('left', 'right'):
            for joint in ('hip', 'knee', 'ankle'):
                p.resetJointState(
                    self.robot_id,
                    self.joint_indices['{}_{}'.format(side, joint)],
                    self.pose[joint])

    def _disable_default_motors(self):
        """关闭膝关节默认速度电机，使 TORQUE_CONTROL 生效。"""
        for name in KNEE_JOINTS:
            p.setJointMotorControl2(
                self.robot_id, self.joint_indices[name],
                p.VELOCITY_CONTROL, force=0.0)

    def _setup_camera(self):
        """把 GUI 相机拉近到机器人附近，便于观察与录屏。"""
        if not self.get_parameter('use_gui').value:
            return
        p.resetDebugVisualizerCamera(
            cameraDistance=1.2,
            cameraYaw=45.0,
            cameraPitch=-15.0,
            cameraTargetPosition=[0.0, 0.0, 0.55])

    def _apply_controllers(self):
        for side in ('left', 'right'):
            for joint in ('hip', 'ankle'):
                p.setJointMotorControl2(
                    self.robot_id,
                    self.joint_indices['{}_{}'.format(side, joint)],
                    p.POSITION_CONTROL,
                    targetPosition=self.pose[joint],
                    force=self.torque_limit)
            knee_index = self.joint_indices['{}_knee'.format(side)]
            if self.mode == 'stiff':
                p.setJointMotorControl2(
                    self.robot_id, knee_index, p.POSITION_CONTROL,
                    targetPosition=self.pose['knee'],
                    force=self.torque_limit)
            else:
                state = p.getJointState(self.robot_id, knee_index)
                torque = impedance_torque(
                    state[0], state[1], self.pose['knee'],
                    self.knee_kp, self.knee_kd, self.torque_limit)
                p.setJointMotorControl2(
                    self.robot_id, knee_index, p.TORQUE_CONTROL,
                    force=torque)

    def _measure_grf(self):
        """双脚与地面接触法向力之和。"""
        grf = 0.0
        for name in FOOT_LINK_JOINTS:
            for contact in p.getContactPoints(
                    self.robot_id, self.plane_id,
                    self.joint_indices[name], -1):
                grf += contact[9]
        return grf

    def _publish_sample(self):
        position, orientation = p.getBasePositionAndOrientation(self.robot_id)
        velocity, _ = p.getBaseVelocity(self.robot_id)
        euler = p.getEulerFromQuaternion(orientation)
        knee = p.getJointState(
            self.robot_id, self.joint_indices['left_knee'])[0]
        message = Float64MultiArray()
        message.data = [
            self.sim_time,
            self._measure_grf(),
            velocity[2],
            position[2],
            euler[1],
            knee,
        ]
        self.publisher.publish(message)

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
        self.accumulator += min(elapsed, 0.05)
        steps = 0
        try:
            while (self.accumulator >= self.sim_dt
                   and steps < self.max_substeps):
                self._apply_controllers()
                p.stepSimulation()
                self._publish_sample()
                self.accumulator -= self.sim_dt
                self.sim_time += self.sim_dt
                steps += 1
        except Exception as error:
            self._stop('仿真步进失败，停止节点：{}'.format(error))
            return
        if not self.camera_ready:
            self._setup_camera()
            self.camera_ready = True
        if self.sim_time >= self.duration:
            self._stop('落地仿真完成: 模式 {}，跌落 {:.2f} m，仿真 {:.2f} s'.format(
                self.mode, self.drop_height, self.sim_time))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LandingSimNode()
    except RuntimeError as error:
        print('启动失败: {}'.format(error))
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
