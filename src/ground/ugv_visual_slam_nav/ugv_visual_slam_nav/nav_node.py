"""导航节点：占据栅格 → A* → 纯追踪 → /cmd_vel。

只依赖 /ugv/map、/ugv/slam_pose、/ugv/depth 三个话题，
换掉 SLAM 节点也能工作（栅格从哪来它不关心）。

卡住恢复与两段式到点确认直接复用 `nav_guard.NavGuard`——和离线策略
`policies.SlamNavPolicy` 是同一份状态机，避免"文档说的"和"launch 跑的"
是两套实现。
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from sensor_msgs.msg import Image

from . import params as P
from .controllers import avoid_from_depth, pure_pursuit
from .kinematics import wrap
from .nav_guard import INFLATE_CELLS, REPLAN_PERIOD, NavGuard
from .planner import astar, inflate, proximity_cost, simplify


class NavNode(Node, NavGuard):
    def __init__(self):
        super().__init__('ugv_nav')

        self.declare_parameter('goal_x', 17.6)
        self.declare_parameter('goal_y', 11.6)
        self.declare_parameter('control_hz', 10.0)
        self.declare_parameter('spin_time', 1.6)
        self.declare_parameter('max_speed', 0.6)
        self.declare_parameter('settle_goal', True)

        self.goal = (float(self.get_parameter('goal_x').value),
                     float(self.get_parameter('goal_y').value))
        self.rate = float(self.get_parameter('control_hz').value)
        self.spin_time = float(self.get_parameter('spin_time').value)
        self.max_v = float(self.get_parameter('max_speed').value)
        self.settle_on = bool(self.get_parameter('settle_goal').value)

        self.occ = None
        self.pose = None
        self.depth = None
        self.t = 0.0
        self.done = False
        self._reset_planner()

        qos = 10
        self.create_subscription(OccupancyGrid, '/ugv/map', self.on_map, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/ugv/slam_pose', self.on_pose, qos)
        self.create_subscription(Image, '/ugv/camera/depth', self.on_depth, qos)

        self.pub_cmd = self.create_publisher(Twist, '/ugv/cmd_vel', qos)
        self.pub_path = self.create_publisher(Path, '/ugv/path', qos)
        self.create_timer(1.0 / self.rate, self.tick)

        self.get_logger().info('导航节点启动，目标 (%.2f, %.2f)' % self.goal)

    # ------------------------------------------------------------------
    def on_map(self, msg):
        if msg.info.width != P.GW or msg.info.height != P.GH:
            return
        arr = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.occ = arr

    def on_pose(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = np.array([p.x, p.y, 2.0 * math.atan2(q.z, q.w)])

    def on_depth(self, msg):
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
        if arr.size == msg.height * msg.width:
            self.depth = arr.reshape(msg.height, msg.width)

    # ------------------------------------------------------------------
    def _publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    def _publish_path(self, stamp):
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        for x, y in self.path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    # ------------------------------------------------------------------
    def _cell_blocked(self, x, y):
        """世界坐标点是否被占据：恢复动作后退前的安全检查用。"""
        if self.occ is None:
            return True
        c = int(x / P.GRID_RES)
        r = int(y / P.GRID_RES)
        if not (0 <= r < P.GH and 0 <= c < P.GW):
            return True
        sub = self.occ[max(0, r - 2):r + 3, max(0, c - 2):c + 3]
        return bool(sub.size) and bool((sub > 50).any())

    def tick(self):
        self.t += 1.0 / self.rate
        if self.occ is None or self.pose is None:
            self._publish_cmd(0.0, 0.0)
            return

        pose = self.pose

        # 到点两段式确认：先原地自转复核，再最终停车。
        # 进入判据沿用原来的 GOAL_TOL（而不是离线策略更严的 STOP_TOL），
        # 保证换实现之后"能到点就能停"。
        if self.settle_on:
            settle = self._settle_goal(pose, self.goal, entry_tol=P.GOAL_TOL)
            if settle is True:
                self._publish_cmd(0.0, 0.0)
                if not self.done:
                    self.get_logger().info('已到达目标，用时 %.1fs' % self.t)
                    self.done = True
                return
            if settle == 'spin':
                self._publish_cmd(0.0, 1.0)
                return

        # 起步先原地扫描一圈，把栅格攒起来
        if self.t < self.spin_time:
            self._publish_cmd(0.12, 1.0)
            return

        # 连续几步几乎没位移就进恢复模式（原地转向 → 查图后退）
        if self._check_stuck(pose):
            v, w = self._recovery_cmd(pose)
            self._publish_cmd(v, w)
            return

        if self.t - self.last_plan >= REPLAN_PERIOD or not self.path:
            occ = self.occ > 50
            unknown = self.occ < 0
            blocked = inflate(occ, INFLATE_CELLS)
            cost = proximity_cost(occ) + np.where(unknown, 1.8, 0.0)
            path = astar(blocked, (pose[0], pose[1]), self.goal, cost=cost)
            if path:
                self.path = simplify(inflate(blocked, 2), path)
                self.last_plan = self.t
                self._publish_path(self.get_clock().now().to_msg())

        if self.path:
            v, w = pure_pursuit(pose, self.path)
        else:
            # 地图还规划不出路：朝目标方向低速试探
            a = wrap(math.atan2(self.goal[1] - pose[1], self.goal[0] - pose[0]) - pose[2])
            v = 0.22 * max(0.0, math.cos(a))
            w = float(np.clip(1.4 * a, -P.MAX_W, P.MAX_W))

        # 记录"规划层希望走多快"：被避障砍到 0 却一直不动才是真卡住
        self._last_v = float(v)
        v, w = avoid_from_depth(self.depth, v, w)
        v = float(np.clip(v, -P.MAX_V, self.max_v))
        self._publish_cmd(v, w)


def main(argv=None):
    rclpy.init(args=argv)
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
