import os

import pybullet as p
import pybullet_data
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .lipm import GRAVITY

JOINT_NAMES = ('left_hip', 'left_knee', 'left_ankle',
               'right_hip', 'right_knee', 'right_ankle')


class PlayNode(Node):
    """PyBullet 仿真执行节点：订阅关节指令并驱动人形模型。"""

    def __init__(self):
        super().__init__('play_node')
        self.declare_parameter('use_gui', True)
        self.declare_parameter('sim_time_step', 1.0 / 240.0)
        self.declare_parameter('max_substeps', 10)
        self.sim_dt = self.get_parameter('sim_time_step').value
        self.max_substeps = self.get_parameter('max_substeps').value
        self.accumulator = 0.0
        self.last_time = self.get_clock().now()
        self.targets = [0.0] * len(JOINT_NAMES)
        self.running = True
        self.sim_time = 0.0
        self.last_report_time = 0.0
        self.last_zmp_valid = True

        self.client = self._connect()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -GRAVITY)
        p.setTimeStep(self.sim_dt)
        self.plane_id = p.loadURDF('plane.urdf')
        p.changeDynamics(self.plane_id, -1, lateralFriction=2.0)

        self.robot_id = self._load_robot()
        self.joint_indices = self._build_joint_map()
        self.camera_ready = False

        self.zmp_sub = self.create_subscription(
            Float64MultiArray, 'zmp_trajectory', self.zmp_callback, 10)
        self.joint_sub = self.create_subscription(
            Float64MultiArray, 'joint_commands', self.command_callback, 10)
        self.timer = self.create_timer(self.sim_dt, self.simulation_step)
        self.get_logger().info(
            'PyBullet 仿真节点已启动，固定步长 {:.4f} s'.format(self.sim_dt))

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
        pkg_path = get_package_share_directory('humanoid_zmp_walking')
        urdf_path = os.path.join(pkg_path, 'urdf', 'simple_humanoid.urdf')
        robot_id = p.loadURDF(urdf_path, [0, 0, 0.83])
        for link in range(-1, p.getNumJoints(robot_id)):
            p.changeDynamics(robot_id, link,
                             linearDamping=0.1,
                             angularDamping=0.1,
                             lateralFriction=1.5)
        self.get_logger().info(
            '已加载模型，共 {} 个关节'.format(p.getNumJoints(robot_id)))
        return robot_id

    def _setup_camera(self):
        """把 GUI 相机拉近到机器人附近，便于观察与录屏。"""
        if not self.get_parameter('use_gui').value:
            return
        p.resetDebugVisualizerCamera(
            cameraDistance=1.2,
            cameraYaw=45.0,
            cameraPitch=-15.0,
            cameraTargetPosition=[0.0, 0.0, 0.55])

    def _build_joint_map(self):
        """按名字建立关节索引映射，避免依赖 URDF 中的书写顺序。"""
        mapping = {}
        for index in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, index)
            mapping[info[1].decode('utf-8')] = index
        missing = [name for name in JOINT_NAMES if name not in mapping]
        if missing:
            raise RuntimeError('模型缺少关节: {}'.format(', '.join(missing)))
        return {name: mapping[name] for name in JOINT_NAMES}

    def _stop(self, message):
        """设置退出标志并关闭 rclpy，避免重复关闭。"""
        if not self.running:
            return
        self.running = False
        self.get_logger().info(message)
        if rclpy.ok():
            rclpy.shutdown()

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
                self._apply_motor_targets()
                p.stepSimulation()
                self.accumulator -= self.sim_dt
                steps += 1
        except Exception as error:
            self._stop('仿真步进失败，停止节点：{}'.format(error))
            return
        self.sim_time += steps * self.sim_dt
        if not self.camera_ready:
            self._setup_camera()
            self.camera_ready = True
        self._report()

    def _apply_motor_targets(self):
        for name, target in zip(JOINT_NAMES, self.targets):
            p.setJointMotorControl2(
                self.robot_id, self.joint_indices[name],
                p.POSITION_CONTROL, targetPosition=target, force=8.0)

    def _report(self):
        """每经过 1 s 仿真时间输出一次姿态与 ZMP 判定信息。"""
        if self.sim_time - self.last_report_time < 1.0:
            return
        self.last_report_time = self.sim_time
        position, _ = p.getBasePositionAndOrientation(self.robot_id)
        self.get_logger().info(
            '底盘高度 {:.3f} m，左髋 {:.2f} rad，右膝 {:.2f} rad，'
            'ZMP 支撑域 {}'.format(
                position[2], self.targets[0], self.targets[4],
                '有效' if self.last_zmp_valid else '越界'))

    def zmp_callback(self, message):
        if len(message.data) >= 6:
            self.last_zmp_valid = message.data[5] > 0.0

    def command_callback(self, message):
        if len(message.data) >= len(JOINT_NAMES):
            self.targets = [float(value) for value in message.data[:6]]


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PlayNode()
    except RuntimeError as error:
        print('启动失败: {}'.format(error))
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
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
