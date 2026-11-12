"""ROS 2（Humble）节点：把 MuJoCo 仿真里的 SLAM 与导航同步跑起来并发布话题。

发布：
  /scan            sensor_msgs/LaserScan   120 线平面雷达（frame_id=laser）
  /odom            nav_msgs/Odometry       轮式里程计（带轮滑误差，frame_id=map）
  /slam/pose       geometry_msgs/PoseStamped  EKF-SLAM 位姿
  /map             nav_msgs/OccupancyGrid  在线构建的占据栅格
  /nav/cmd         geometry_msgs/Twist     下发速度
  /nav/status      std_msgs/String         "dist=..,ate=..,mode=.."
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .env import DiffDriveEnv, LIDAR_BEAMS, LIDAR_RANGE
from .nav import NavConfig, local_policy
from .net import LocalPlannerNet
from .planner import astar
from .slam import EKFSLAM, OccupancyGrid, detect_landmarks


def _default_weights() -> str:
    """默认权重：优先包内 share/models，其次源码目录（与 arena.xml 同一套定位逻辑）。"""
    from .env import _find_asset
    return str(_find_asset("planner.npz"))


def _import_ros():
    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg, Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    return (rclpy, Node, LaserScan, Odometry, OccupancyGridMsg, PoseStamped, Twist,
            String, TransformBroadcaster, TransformStamped)


def main(argv=None) -> int:
    (rclpy, Node, LaserScan, Odometry, OccupancyGridMsg, PoseStamped, Twist,
     String, TransformBroadcaster, TransformStamped) = _import_ros()

    class SlamNavNode(Node):
        def __init__(self):
            super().__init__("mujoco_slam_nav_sync")
            self.declare_parameter("scene", "maze")
            self.declare_parameter("use_slam", True)
            self.declare_parameter("slip", 0.0)
            self.declare_parameter("weights", "")
            self.declare_parameter("rate_hz", 20.0)
            self.declare_parameter("max_steps", 1200)

            scene = self.get_parameter("scene").value
            self.use_slam = bool(self.get_parameter("use_slam").value)
            self.slip = float(self.get_parameter("slip").value)
            rate = float(self.get_parameter("rate_hz").value)
            self.max_steps = int(self.get_parameter("max_steps").value)
            weights = self.get_parameter("weights").value
            self.rate_hz = rate
            self.done = False

            self.env = DiffDriveEnv(scene)
            # 与离线 run_episode() 对齐：显式 reset 到起点，并固定随机源，
            # 否则节点的里程计噪声序列与文档里的实验不是同一条轨迹
            self.env.reset(scene=scene, seed=0)
            self.env.rng = np.random.default_rng(10007)
            self.cfg = NavConfig()
            self.net = LocalPlannerNet()
            path = weights or _default_weights()
            if path and Path(path).exists():
                self.net.load(path)
                self.get_logger().info(f"已加载权重 {path}")
            else:
                raise RuntimeError(
                    "没有找到 planner.npz：请先 python main.py --demo 训练，"
                    "或用 weights 参数指定权重文件")

            self.grid = OccupancyGrid()
            self.ekf = EKFSLAM(init_pose=(*DiffDriveEnv.START, 0.0),
                               n_landmarks=len(self.env.lm_ids))
            self.dead = np.array([*DiffDriveEnv.START, 0.0])
            self.step_i = 0
            self.path = None
            self.w_filt = 0.0
            self.stuck = 0
            self.reached = False
            self.prev_state = self.env.state()

            self.pub_scan = self.create_publisher(LaserScan, "scan", 10)
            self.pub_odom = self.create_publisher(Odometry, "odom", 10)
            self.pub_pose = self.create_publisher(PoseStamped, "slam/pose", 10)
            self.pub_map = self.create_publisher(OccupancyGridMsg, "map", 10)
            self.pub_cmd = self.create_publisher(Twist, "nav/cmd", 10)
            self.pub_status = self.create_publisher(String, "nav/status", 10)
            self.tf_broadcaster = TransformBroadcaster(self)
            self.timer = self.create_timer(1.0 / rate, self.tick)
            self.get_logger().info(
                f"场景={scene} 模式={'EKF-SLAM' if self.use_slam else 'dead-reckoning'} "
                f"轮滑={self.slip:.0%} 目标={DiffDriveEnv.GOAL}")

        # ------------------------------------------------------------ 工具
        def _stamp(self):
            return self.get_clock().now().to_msg()

        def _publish_scan(self, scan, pose):
            m = LaserScan()
            m.header.stamp = self._stamp()
            m.header.frame_id = "laser"
            m.angle_min = 0.0
            # angle_max 必须等于最后一条射线的角度，否则区间与增量不自洽
            m.angle_max = float(2.0 * np.pi - 2.0 * np.pi / LIDAR_BEAMS)
            m.angle_increment = float(2.0 * np.pi / LIDAR_BEAMS)
            m.range_min = 0.05
            m.range_max = float(LIDAR_RANGE)
            m.time_increment = 0.0                     # 120 条射线同一时刻采样
            m.scan_time = float(1.0 / max(self.rate_hz, 1e-3))
            m.ranges = [float(r) for r in scan]
            self.pub_scan.publish(m)

        def _publish_map(self):
            m = OccupancyGridMsg()
            m.header.stamp = self._stamp()
            m.header.frame_id = "map"
            m.info.resolution = float(self.grid.res)
            m.info.width = int(self.grid.w)
            m.info.height = int(self.grid.h)
            m.info.origin.position.x = float(self.grid.x0)
            m.info.origin.position.y = float(self.grid.y0)
            m.info.origin.orientation.w = 1.0
            occ = np.where(self.grid.logodds > 0.5, 100,
                           np.where(self.grid.logodds < -0.5, 0, -1)).astype(np.int8)
            m.data = occ.ravel().tolist()
            self.pub_map.publish(m)

        def _publish_odom(self, pose, v, w):
            """轮式里程计（死推算）位姿，与 SLAM 位姿是两条独立的消息。"""
            m = Odometry()
            m.header.stamp = self._stamp()
            m.header.frame_id = "map"
            m.child_frame_id = "base_link"
            m.pose.pose.position.x = float(pose[0])
            m.pose.pose.position.y = float(pose[1])
            m.pose.pose.orientation.z = float(np.sin(pose[2] / 2))
            m.pose.pose.orientation.w = float(np.cos(pose[2] / 2))
            m.twist.twist.linear.x = float(v)
            m.twist.twist.angular.z = float(w)
            self.pub_odom.publish(m)

        def _publish_pose(self, pose):
            p = PoseStamped()
            p.header.stamp = self._stamp()
            p.header.frame_id = "map"
            p.pose.position.x = float(pose[0])
            p.pose.position.y = float(pose[1])
            p.pose.orientation.z = float(np.sin(pose[2] / 2))
            p.pose.orientation.w = float(np.cos(pose[2] / 2))
            self.pub_pose.publish(p)

        def _publish_tf(self, pose):
            """map -> base_link（SLAM 位姿）+ base_link -> laser（静态外参）。

            没有 TF 时 RViz 会报 `No transform from [laser] to [map]`，
            `/scan` 与 `/map` 无法叠在一起显示。
            """
            t = TransformStamped()
            t.header.stamp = self._stamp()
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = float(pose[0])
            t.transform.translation.y = float(pose[1])
            t.transform.rotation.z = float(np.sin(pose[2] / 2))
            t.transform.rotation.w = float(np.cos(pose[2] / 2))
            self.tf_broadcaster.sendTransform(t)

            t2 = TransformStamped()
            t2.header.stamp = t.header.stamp
            t2.header.frame_id = "base_link"
            t2.child_frame_id = "laser"
            t2.transform.translation.z = 0.25          # 雷达安装高度，与 env.scan() 一致
            t2.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t2)

        # ------------------------------------------------------------ 主循环
        def tick(self):
            if self.done:
                return
            dist_goal = float(np.linalg.norm(self.env.state().pos - DiffDriveEnv.GOAL))
            if dist_goal < self.cfg.goal_tol:
                self.reached = True
            if self.env.crashed or self.reached or self.step_i >= self.max_steps:
                self.done = True
                self.timer.cancel()          # 结束后停表，避免每 50 ms 刷一条"结束"
                self.get_logger().info(
                    f"结束：reached={self.reached} crashed={self.env.crashed} "
                    f"steps={self.step_i} 终点误差={dist_goal:.2f} m")
                return

            scan = self.env.scan()

            if self.use_slam:
                self.ekf.observe(detect_landmarks(scan))
                pose = self.ekf.pose
            else:
                pose = self.dead.copy()

            self.grid.update(pose, scan)
            if self.step_i % self.cfg.replan_every == 0 or self.path is None:
                self.path = astar(self.grid.occupancy_for_planning(3), pose[:2],
                                  DiffDriveEnv.GOAL)

            # 与离线 run_episode() 共用同一套局部策略：末端停靠 + 护栏 + 卡死恢复
            ctrl = local_policy(scan, pose, self.path, self.net, self.cfg,
                                self.w_filt, self.stuck)
            v, w = ctrl["v"], ctrl["w"]
            self.w_filt, self.stuck = ctrl["w_filt"], ctrl["stuck"]

            st = self.env.step(v, w)
            # 里程计用位姿增量（编码器读数），不是下发的指令
            v_meas, w_meas = self.env.odom_from_motion(self.prev_state, st, self.cfg_dt)
            self.prev_state = st
            v_o, w_o = self.env.odom(v_meas, w_meas, self.slip, 0.003)
            dt = self.cfg_dt
            self.dead += np.array([v_o * np.cos(self.dead[2]) * dt,
                                   v_o * np.sin(self.dead[2]) * dt, w_o * dt])
            if self.use_slam:
                self.ekf.predict(v_o, w_o, dt)

            self._publish_scan(scan, pose)
            self._publish_odom(self.dead, v_o, w_o)   # /odom = 死推算位姿 + 实测轮速
            self._publish_pose(pose)                # /slam/pose = EKF-SLAM
            self._publish_tf(pose)
            if self.step_i % 5 == 0:
                self._publish_map()

            cmd = Twist()
            cmd.linear.x, cmd.angular.z = float(v), float(w)
            self.pub_cmd.publish(cmd)

            err = float(np.linalg.norm(pose[:2] - self.env.state().pos))
            st = String()
            st.data = (f"mode={'slam' if self.use_slam else 'dead'} step={self.step_i} "
                       f"dist={dist_goal:.2f} ate={err:.2f} "
                       f"reached={self.reached} cmd=({v:.2f},{w:.2f})")
            self.pub_status.publish(st)
            if self.step_i % 20 == 0:
                self.get_logger().info(st.data)
            self.step_i += 1

        @property
        def cfg_dt(self):
            from .env import DT
            return DT

    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=argv)
    node = SlamNavNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Humble 下 Ctrl+C / SIGTERM 抛的是 ExternalShutdownException，
        # 此时 context 已被 rclpy 的 signal handler 关掉，再 shutdown 会
        # `RCLError: rcl_shutdown already called`，所以要先判 ok()
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
