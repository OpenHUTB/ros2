"""端到端节点：图像 + 深度 + 目标方位通道进，速度指令出。

权重默认从 ament share 的 models/e2e.npz 读取，可用 model_path 参数覆盖。

目标方位通道（policies.goal_channel）需要"当前任务目标"——往返巡检
里去程是 B 端、返程是 A 端。节点内部持有一份场景副本，用里程计位姿
推进任务阶段（弧长判定），因此不依赖 SLAM、不依赖建图。
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image

from .model_io import default_model_dir, load_net
from .policies import E2EPolicy
from .world import make_scene


def _yaw_from_quat(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class E2ENode(Node):
    def __init__(self):
        super().__init__('rov_e2e')

        self.declare_parameter('model_path', '')
        self.declare_parameter('scene', 'straight')
        self.declare_parameter('seed', 0)
        self.declare_parameter('control_hz', 10.0)
        self.declare_parameter('max_speed', 0.45)

        mp = str(self.get_parameter('model_path').value)
        if not mp:
            import os
            mp = os.path.join(default_model_dir(), 'e2e.npz')
        self.max_v = float(self.get_parameter('max_speed').value)
        self.world = make_scene(str(self.get_parameter('scene').value),
                                int(self.get_parameter('seed').value))

        self.net = load_net(mp)
        self.policy = E2EPolicy(self.net, max_v=self.max_v)
        self._depth = None
        self._odom = None
        self.n_frames = 0

        qos = 10
        self.create_subscription(Image, '/rov/camera/image_raw', self.on_image, qos)
        self.create_subscription(Image, '/rov/camera/depth', self.on_depth, qos)
        self.create_subscription(Odometry, '/rov/odom', self.on_odom, qos)
        self.pub_cmd = self.create_publisher(Twist, '/rov/cmd_vel', qos)

        self.get_logger().info('端到端节点启动，权重 %s，场景 %s'
                               % (mp, self.world.key))

    def on_depth(self, msg):
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
        if arr.size == msg.height * msg.width:
            self._depth = arr.reshape(msg.height, msg.width)

    def on_odom(self, msg):
        p = msg.pose.pose
        self._odom = np.array([p.position.x, p.position.y,
                               _yaw_from_quat(p.orientation)], dtype=np.float64)

    def on_image(self, msg):
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if arr.size != msg.height * msg.width * 3:
            return
        rgb = arr.reshape(msg.height, msg.width, 3)
        self.n_frames += 1

        if self._odom is None:
            # 没有里程计就算不出目标方位，先保持静止，别用零向量喂网络
            if self.n_frames % 50 == 1:
                self.get_logger().warn('等待 /rov/odom ...')
            return

        # 用里程计位姿推进任务阶段，取当前端点作目标
        self.world.advance(self._odom)
        obs = {'rgb': rgb, 'depth': self._depth, 'odom': self._odom,
               'goal': self.world.goal, 'true_pose': self._odom}
        v, w = self.policy.act(obs)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)

        if self.n_frames % 100 == 0:
            d = float(np.hypot(self.world.goal[0] - self._odom[0],
                               self.world.goal[1] - self._odom[1]))
            self.get_logger().info(
                '第 %d 帧：v=%.2f m/s, w=%.2f rad/s, phase=%s, 距目标 %.2f m'
                % (self.n_frames, v, w, self.world.phase, d))


def main(argv=None):
    rclpy.init(args=argv)
    node = E2ENode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
