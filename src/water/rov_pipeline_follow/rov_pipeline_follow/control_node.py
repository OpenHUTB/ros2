"""控制节点：管道跟踪级联控制 + SLAM 位姿/建图兜底。

经典流水线的 ROS 形态，与离线 PipeSlamPolicy 同一套逻辑：
感知正常时用 (dx_err, dtheta) 级联控制跟管；感知丢失超过 LOST_LIMIT
切换到占据栅格 A* + 纯追踪。折返（往返巡检 U 形掉头）是显式机动，
回转方向按管道在鼻子哪一侧选，避免残余推力把 ROV 甩进管道。

感知可选几何检测或 CNN 回归（use_cnn=true 时加载 percep.npz），
两者输出统一为任务系 (dx_err, dtheta)。
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from sensor_msgs.msg import Image

from . import detector, params as P
from .controllers import PipeFollower, pure_pursuit, safety_from_depth
from .cnn import CNN
from .gridmap import GridMap
from .kinematics import wrap
from .model_io import default_model_dir, load_net
from .policies import PERCEP_IN_CH, _percep_input
from .planner import astar, inflate, proximity_cost
from .world import make_scene

SPIN_TIME = 1.2          # 起步原地扫描建图的时间（秒）
REPLAN_PERIOD = 2.0
LOST_LIMIT = 3.0
INFLATE_CELLS = 4


class ControlNode(Node):
    def __init__(self):
        super().__init__('rov_control')

        self.declare_parameter('scene', 'straight')
        self.declare_parameter('seed', 0)
        self.declare_parameter('control_hz', 10.0)
        self.declare_parameter('use_cnn', False)
        self.declare_parameter('percep_model_path', '')

        scene = str(self.get_parameter('scene').value)
        seed = int(self.get_parameter('seed').value)
        self.rate = float(self.get_parameter('control_hz').value)
        self.use_cnn = bool(self.get_parameter('use_cnn').value)

        # 场景副本：任务阶段（去程/返程）是任务定义的一部分，
        # 用 SLAM 位姿按弧长推进 —— 与离线仿真同一套判定
        self.world = make_scene(scene, seed)
        self.follower = PipeFollower()
        self.grid = GridMap()
        self.percep = None
        if self.use_cnn:
            mp = str(self.get_parameter('percep_model_path').value)
            if not mp:
                import os
                mp = os.path.join(default_model_dir(), 'percep.npz')
            self.percep = load_net(mp)

        self.pose = None            # SLAM 位姿
        self.rgb = None
        self.depth = None
        self.occ = None
        self.path = []
        self.last_plan = -1e9
        self.t = 0.0
        self.lost_t = 0.0
        self.n_lm_seen = 0
        self._last_phase = self.world.phase

        qos = 10
        self.create_subscription(PoseWithCovarianceStamped, '/rov/slam_pose',
                                 self.on_pose, qos)
        self.create_subscription(Image, '/rov/camera/image_raw', self.on_image, qos)
        self.create_subscription(Image, '/rov/camera/depth', self.on_depth, qos)
        self.create_subscription(OccupancyGrid, '/rov/map', self.on_map, qos)

        self.pub_cmd = self.create_publisher(Twist, '/rov/cmd_vel', qos)
        self.pub_path = self.create_publisher(Path, '/rov/path', qos)
        self.create_timer(1.0 / self.rate, self.tick)

        self.get_logger().info(
            '控制节点启动：scene=%s 感知=%s' % (scene,
                                            'CNN' if self.use_cnn else 'geometric'))

    # ------------------------------------------------------------------
    def on_pose(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = np.array([p.x, p.y,
                              2.0 * math.atan2(q.z, q.w)], dtype=np.float64)

    def on_depth(self, msg):
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
        if arr.size == msg.height * msg.width:
            self.depth = arr.reshape(msg.height, msg.width)

    def on_image(self, msg):
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if arr.size == msg.height * msg.width * 3:
            self.rgb = arr.reshape(msg.height, msg.width, 3)

    def on_map(self, msg):
        if msg.info.width != P.GW or msg.info.height != P.GH:
            return
        self.occ = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    def _publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    def _publish_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for x, y in self.path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def _perceive(self):
        """(found, dx_err, dtheta)，统一任务系（与离线 _perceive 同约定）。"""
        if self.rgb is None or self.depth is None:
            return False, 0.0, 0.0
        if self.percep is not None:
            x = _percep_input(self.rgb, self.depth)
            out = self.percep.forward(x[None, ...], train=False)[0]
            return True, float(out[0]) * 1.2, float(out[1]) * 1.5
        found, dx_raw, dtheta = detector.pipeline_observation(self.rgb, self.depth)
        if not found:
            return False, 0.0, 0.0
        mdir = self.world.mission_direction(self.pose[0], self.pose[1])
        return True, dx_raw + (mdir - 1.0) * P.FOLLOW_OFFSET, dtheta

    # ------------------------------------------------------------------
    def tick(self):
        self.t += 1.0 / self.rate
        if self.pose is None:
            self._publish_cmd(0.0, 0.0)
            return
        pose = self.pose

        # 任务阶段推进 + 折返机动触发（与 PipeSlamPolicy 一致）
        if self.world.advance(pose):
            self._publish_cmd(0.0, 0.0)
            if not getattr(self, '_done_logged', False):
                self.get_logger().info('往返巡检完成，用时 %.1fs' % self.t)
                self._done_logged = True
            return
        if self.world.phase != self._last_phase:
            self._last_phase = self.world.phase
            if self.world.phase == 'return':
                pipe = []
                if self.rgb is not None and self.depth is not None:
                    _, pipe = detector.detect(self.rgb, self.depth)
                if len(pipe):
                    rel = float(pipe[np.argmin(pipe[:, 1]), 0])
                    self.follower.begin_turnaround(1.0 if rel <= 0 else -1.0)
                else:
                    self.follower.begin_turnaround(1.0)
                self.get_logger().info('到达 B 端，折返机动开始')

        # 折返机动：v=0 原地回转，艏向误差收小后自动交还
        if self.follower.turn_dir:
            s, _lat, (tx, ty) = self.world.progress_of(pose[0], pose[1])
            mdir = self.world.mission_direction(pose[0], pose[1])
            dth = wrap(math.atan2(mdir * ty, mdir * tx) - pose[2])
            v, w = self.follower.step(0.0, dth)
            self._publish_cmd(v, w)
            return

        # 起步先原地扫一圈，把周围应答器观测攒起来
        if self.t < SPIN_TIME:
            self._publish_cmd(0.10, 0.9)
            return

        found, dx_err, dtheta = self._perceive()
        self.lost_t = 0.0 if found else self.lost_t + 1.0 / self.rate

        if found and self.lost_t <= LOST_LIMIT:
            v, w = self.follower.step(dx_err, dtheta)
            v, w = safety_from_depth(self.depth, v, w)
            self._publish_cmd(v, w)
            return

        # 感知丢失：SLAM 位姿 + 已建栅格 A* 兜底
        if self.occ is not None and (
                self.t - self.last_plan >= REPLAN_PERIOD or not self.path):
            occ = self.occ > 50
            unknown = self.occ < 0
            blocked = inflate(occ, INFLATE_CELLS)
            cost = proximity_cost(occ) + np.where(unknown, 1.2, 0.0)
            goal = self.world.goal
            self.path = astar(blocked, (pose[0], pose[1]),
                              (float(goal[0]), float(goal[1])), cost=cost)
            self.last_plan = self.t
            if self.path:
                self._publish_path()
        if self.path:
            v, w = pure_pursuit(pose, self.path)
        else:
            a = wrap(math.atan2(self.world.goal[1] - pose[1],
                                self.world.goal[0] - pose[0]) - pose[2])
            v = 0.20 * max(0.0, math.cos(a))
            w = float(np.clip(1.2 * a, -P.MAX_W, P.MAX_W))
        v, w = safety_from_depth(self.depth, v, w)
        self._publish_cmd(v, w)


def main(argv=None):
    rclpy.init(args=argv)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
