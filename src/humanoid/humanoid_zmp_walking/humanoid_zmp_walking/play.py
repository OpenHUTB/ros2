import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
import os
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

class PlayNode(Node):
    def __init__(self):
        super().__init__('play_node')
        self.get_logger().info('PyBullet simulation node started')

        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        self.plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(self.plane_id, -1, lateralFriction=2.0)

        try:
            pkg_path = get_package_share_directory('humanoid_zmp_walking')
            urdf_path = os.path.join(pkg_path, 'urdf', 'simple_humanoid.urdf')
            self.robot_id = p.loadURDF(urdf_path, [0, 0, 0.85])
            self.num_joints = p.getNumJoints(self.robot_id)
            self.get_logger().info(f'Loaded robot with {self.num_joints} joints')

            # 加阻尼和摩擦，让机器人站得更稳
            for i in range(-1, self.num_joints):
                p.changeDynamics(self.robot_id, i,
                                 linearDamping=0.1,
                                 angularDamping=0.1,
                                 lateralFriction=1.5)

            for i in range(self.num_joints):
                info = p.getJointInfo(self.robot_id, i)
                self.get_logger().info(f'Joint {i}: {info[1].decode("utf-8")}')

        except Exception as e:
            self.get_logger().error(f'Failed to load URDF: {e}')
            self.robot_id = -1

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'zmp_trajectory',
            self.listener_callback,
            10)
        self.timer = self.create_timer(1.0 / 240.0, self.simulation_step)
        self.target_angle = 0.0
        self.step_counter = 0

    def simulation_step(self):
        if self.robot_id != -1 and self.num_joints > 0:
            for i in range(self.num_joints):
                p.setJointMotorControl2(
                    self.robot_id, i,
                    p.POSITION_CONTROL,
                    targetPosition=self.target_angle,
                    force=10.0
                )
        p.stepSimulation()

        self.step_counter += 1
        if self.step_counter % 240 == 0 and self.robot_id != -1:
            pos, _ = p.getBasePositionAndOrientation(self.robot_id)
            self.get_logger().info(f'Base z={pos[2]:.3f}, target={self.target_angle:.3f}')

    def listener_callback(self, msg):
        if len(msg.data) >= 1:
            self.target_angle = msg.data[0]

def main(args=None):
    rclpy.init(args=args)
    node = PlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if p.isConnected():
                p.disconnect()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
