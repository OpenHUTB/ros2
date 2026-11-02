#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MCL 定位节点（ROS 1）。

话题
----
订阅  /scan             sensor_msgs/LaserScan        2D 激光
      /odom             nav_msgs/Odometry            轮式里程计（会漂移）
      /initialpose      geometry_msgs/PoseWithCovarianceStamped   手动重定位
      /gazebo/model_states  gazebo_msgs/ModelStates  **仅用于评估**，不参与估计
发布  /mcl_pose         geometry_msgs/PoseWithCovarianceStamped   map 系下的位姿
      /particles        geometry_msgs/PoseArray       粒子分布（rviz 可视化）
      /mcl_diagnostics  std_msgs/Float32MultiArray    [Neff, 位置散布, 航向散布, 位置误差, 航向误差]
      TF   map -> odom

设计上的两个关键点
------------------
1. **估计只用 /scan 与 /odom**。真值只用来算误差、写进诊断话题里，
   绝不参与粒子权重——否则等于作弊。
2. **map->odom 的 TF**：MCL 给出 map->base_link，里程计给出 odom->base_link，
   两者复合得到 map->odom。这样 rviz 里地图、粒子、机器人能对齐，
   而且下游导航栈可以直接接。
"""

import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

try:
    from lidar_mcl_localization.geometry import pose_error, wrap_angle
    from lidar_mcl_localization.map_model import OccupancyMap
    from lidar_mcl_localization.motion_model import MotionModel, odometry_delta
    from lidar_mcl_localization.particle_filter import ParticleFilter
    from lidar_mcl_localization.sensor_model import LikelihoodFieldModel
except ImportError:                                          # pragma: no cover
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
    from lidar_mcl_localization.geometry import pose_error, wrap_angle
    from lidar_mcl_localization.map_model import OccupancyMap
    from lidar_mcl_localization.motion_model import MotionModel, odometry_delta
    from lidar_mcl_localization.particle_filter import ParticleFilter
    from lidar_mcl_localization.sensor_model import LikelihoodFieldModel

try:
    from gazebo_msgs.msg import ModelStates
    HAVE_GAZEBO_MSGS = True
except ImportError:                                          # pragma: no cover
    ModelStates = None
    HAVE_GAZEBO_MSGS = False


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def as_floats(value, default=None):
    """把 rosparam 读到的值转成 float 列表。

    launch 里写 ``<param name="init_pose" value="[-3.5, 0, 0]"/>`` 时，
    rospy 拿到的是**字符串** "[−3.5, 0, 0]" 而不是列表（要加 ``type="yaml"`` 才是列表）。
    这两种写法都很常见，所以这里统一兼容，避免节点一启动就 ValueError。
    """
    if value is None:
        return list(default or [])
    if isinstance(value, str):
        text = value.strip().strip("[]()")
        if not text:
            return list(default or [])
        return [float(v) for v in text.replace(",", " ").split()]
    if isinstance(value, (list, tuple)):
        return [float(v) for v in value]
    return [float(value)]


class MCLNode(object):

    def __init__(self):
        # ---------------------------------------------------------- 参数
        map_file = rospy.get_param("~map_file", "")
        if not map_file:
            map_file = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "maps", "mcl_room.pgm")
        self.num_particles = int(rospy.get_param("~num_particles", 600))
        self.model_name = rospy.get_param("~ground_truth_model", "mcl_bot")
        self.global_localization = bool(rospy.get_param("~global_localization", False))
        init = as_floats(rospy.get_param("~init_pose", [-3.5, 0.0, 0.0]),
                         default=[-3.5, 0.0, 0.0])
        init_std = as_floats(rospy.get_param("~init_std", [0.30, 0.30, 0.15]),
                             default=[0.30, 0.30, 0.15])
        alpha = as_floats(rospy.get_param("~alpha", [0.12, 0.03, 0.12, 0.03]),
                          default=[0.12, 0.03, 0.12, 0.03])
        self.beam_step = int(rospy.get_param("~beam_step", 8))
        self.max_range = float(rospy.get_param("~max_range", 8.0))
        self.resample_ratio = float(rospy.get_param("~resample_ratio", 0.5))
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))

        # ---------------------------------------------------------- 模型
        self.map = OccupancyMap.from_files(map_file)
        rospy.loginfo("地图：%s", self.map)
        self.map.likelihood_field()          # 预热似然场，避免第一帧卡住

        self.rng = np.random.RandomState(int(rospy.get_param("~seed", 0)))
        self.pf = ParticleFilter(
            num_particles=self.num_particles,
            init_pose=tuple(float(v) for v in init),
            init_std=tuple(float(v) for v in init_std),
            rng=self.rng,
            global_localization=self.global_localization,
            occ_map=self.map)
        self.motion = MotionModel(alpha=alpha)
        self.sensor = LikelihoodFieldModel(sigma=float(rospy.get_param("~sigma", 0.20)),
                                           beam_step=self.beam_step)

        # ---------------------------------------------------------- 状态
        self.last_odom = None
        self._prev_odom = None
        self.last_scan = None
        self.truth = None
        self.pose = tuple(float(v) for v in init)
        self.updates = 0
        self.nis_history = []

        # ---------------------------------------------------------- 通信
        self.pub_pose = rospy.Publisher("/mcl_pose", PoseWithCovarianceStamped, queue_size=10)
        self.pub_particles = rospy.Publisher("/particles", PoseArray, queue_size=1)
        self.pub_diag = rospy.Publisher("/mcl_diagnostics", Float32MultiArray, queue_size=10)
        self.tf_br = TransformBroadcaster()

        rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.on_odom, queue_size=50)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.on_initialpose, queue_size=1)
        if HAVE_GAZEBO_MSGS:
            rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_model_states, queue_size=5)

        rospy.loginfo("MCL 启动：%d 粒子，beam_step=%d，sigma=%.2f，全局定位=%s",
                      self.num_particles, self.beam_step, self.sensor.sigma,
                      self.global_localization)
        self._publish(rospy.Time.now())

    # -------------------------------------------------------------- 回调
    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.last_odom = (p.x, p.y, quat_to_yaw(msg.pose.pose.orientation))

    def on_model_states(self, msg):
        """真值：**只用于评估**，不进粒子权重。"""
        try:
            idx = list(msg.name).index(self.model_name)
        except ValueError:
            return
        pose = msg.pose[idx]
        self.truth = (pose.position.x, pose.position.y,
                      quat_to_yaw(pose.orientation))

    def on_initialpose(self, msg):
        """手动指定初始位姿（rviz 的 2D Pose Estimate 按钮）。"""
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.pf.initialize_gaussian((p.x, p.y, yaw), (0.25, 0.25, 0.10))
        self.pose = (p.x, p.y, yaw)
        rospy.loginfo("收到初始位姿：x=%.2f y=%.2f yaw=%.2f", p.x, p.y, yaw)

    def on_scan(self, msg):
        self.last_scan = msg
        if self.last_odom is None:
            return
        self.step(msg)

    # -------------------------------------------------------------- 主循环
    def step(self, scan_msg):
        # 1) 运动模型：从上一帧里程计到这一帧的增量
        cur_odom = self.last_odom
        prev_odom = getattr(self, "_prev_odom", None)
        if prev_odom is not None:
            delta = odometry_delta(prev_odom, cur_odom)
            if delta[0] > 1e-9 or abs(delta[1]) > 1e-9:
                self.pf.predict(self.motion, delta)
        self._prev_odom = cur_odom

        # 2) 观测模型
        angles = msg_to_angles(scan_msg)
        ranges = np.asarray(scan_msg.ranges, dtype=np.float64)
        a_valid, r_valid = self.sensor.subsample(angles, ranges, self.max_range)
        if a_valid.size >= 8:
            self.pf.update(self.sensor, a_valid, r_valid, self.map, self.max_range)
            self.pf.maybe_resample(self.resample_ratio)
        self.updates += 1

        # 3) 发布
        self.pose = self.pf.estimate()
        self._publish(scan_msg.header.stamp)

    def _publish(self, stamp):
        pos_std, yaw_std = self.pf.spread()
        cov2 = self.pf.covariance()

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = self.pose[0]
        msg.pose.pose.position.y = self.pose[1]
        q = yaw_to_quat(self.pose[2])
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        cov = [0.0] * 36
        cov[0] = float(cov2[0, 0])
        cov[1] = float(cov2[0, 1])
        cov[6] = float(cov2[1, 0])
        cov[7] = float(cov2[1, 1])
        cov[35] = float(yaw_std ** 2)
        msg.pose.covariance = cov
        self.pub_pose.publish(msg)

        # 粒子云
        arr = PoseArray()
        arr.header.stamp = stamp
        arr.header.frame_id = "map"
        for p in self.pf.particles:
            ps = PoseStamped().pose
            ps.position.x = float(p[0])
            ps.position.y = float(p[1])
            qq = yaw_to_quat(float(p[2]))
            ps.orientation.z = qq[2]
            ps.orientation.w = qq[3]
            arr.poses.append(ps)
        self.pub_particles.publish(arr)

        # 诊断
        diag = Float32MultiArray()
        err_pos = err_yaw = float("nan")
        if self.truth is not None:
            err_pos, err_yaw = pose_error(self.pose, self.truth)
        diag.data = [self.pf.neff(), pos_std, yaw_std, err_pos, err_yaw,
                     float(self.pf.resample_count), float(self.updates)]
        self.pub_diag.publish(diag)

        # TF: map -> odom = (map->base) ∘ (odom->base)^-1
        if self.publish_tf and self._prev_odom is not None:
            ox, oy, oth = self._prev_odom
            mx, my, mth = self.pose
            # 复合：T_map_odom = T_map_base * T_base_odom，而 T_base_odom = (T_odom_base)^-1
            c, s = math.cos(-oth), math.sin(-oth)
            dx, dy = mx - ox, my - oy
            tx = c * dx - s * dy
            ty = s * dx + c * dy
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = "map"
            tf.child_frame_id = "odom"
            tf.transform.translation.x = tx
            tf.transform.translation.y = ty
            tq = yaw_to_quat(wrap_angle(mth - oth))
            tf.transform.rotation.z = tq[2]
            tf.transform.rotation.w = tq[3]
            self.tf_br.sendTransform(tf)


def msg_to_angles(scan_msg):
    """把 LaserScan 的角度序列算出来（等价于 msg_to_angles，便于单测替换）。"""
    n = len(scan_msg.ranges)
    return scan_msg.angle_min + scan_msg.angle_increment * np.arange(n, dtype=np.float64)


def main():
    rospy.init_node("mcl_node")
    MCLNode()
    rospy.spin()


if __name__ == "__main__":
    main()
