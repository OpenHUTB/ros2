#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
神经网络轨迹跟踪控制节点（任务 2 的控制部分）

订阅：
  /uav/lidar/features  (12 维扇区距离)
  /uav/odom            (位姿)
  /uav/goal            (目标点，ENU 坐标系)
发布：
  /uav/cmd_vel         (速度指令)

控制量由训练好的 MLP 给出（models/controller.pt）。网络之外还有一层安全兜底
（safety_shield.py）：只在动作非法、传感器过期、输出越界或近距障碍时介入，
并始终按训练量程限幅。安全层可以通过参数 ~safety_enabled:=false 关闭，
用于和"裸网络"做对比实验。
"""

import math
import os
import sys

import numpy as np
import rospy
import torch
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

# 被 import 的模块在 devel 空间里不在 sys.path 上（CMake 用的是 install(FILES)），
# 通过 rospkg 找到源码包目录补进来，rosrun 走 relay 脚本时也能正常 import
try:
    import rospkg as _rospkg
    _pkg_scripts = os.path.join(
        _rospkg.RosPack().get_path('uav_nn_perception_navigation'), 'scripts')
    if os.path.isdir(_pkg_scripts) and _pkg_scripts not in sys.path:
        sys.path.insert(0, _pkg_scripts)
except Exception:
    pass

import safety_shield  # noqa: E402
from uav_features import goal_features  # noqa: E402


class MLPPolicy(torch.nn.Module):
    """与训练脚本保持完全一致的结构。"""

    def __init__(self, in_dim=16, hidden=64):
        super(MLPPolicy, self).__init__()
        self.net = torch.nn.Sequential(
            torch.nn.Linear(in_dim, hidden),
            torch.nn.ReLU(),
            torch.nn.Linear(hidden, hidden),
            torch.nn.ReLU(),
            torch.nn.Linear(hidden, 3),
        )

    def forward(self, x):
        return self.net(x)


def resolve_model_path(model_path):
    """rosrun 执行的是 devel 空间里的副本，用 rospkg 找到源码包目录下的 models/。"""
    if os.path.isabs(model_path) and os.path.exists(model_path):
        return model_path
    try:
        import rospkg
        pkg_dir = rospkg.RosPack().get_path('uav_nn_perception_navigation')
        candidate = os.path.join(pkg_dir, 'models', os.path.basename(model_path))
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    return os.path.abspath(os.path.join(_SCRIPT_DIR, '..', 'models',
                                        os.path.basename(model_path)))


class TrajectoryControlNode(object):
    def __init__(self):
        model_path = rospy.get_param('~model_path', 'controller.pt')
        self.rate_hz = float(rospy.get_param('~rate', 10.0))
        self.altitude = float(rospy.get_param('~altitude', 5.0))
        self.kp_alt = float(rospy.get_param('~kp_altitude', 0.8))

        model_path = resolve_model_path(model_path)
        checkpoint = torch.load(model_path, map_location='cpu')
        self.model = MLPPolicy(in_dim=checkpoint.get('in_dim', 16))
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model.eval()
        self.action_scale = np.array(checkpoint.get('action_scale', [3.0, 2.0, 1.0]),
                                     dtype=np.float32)
        self.feat_mean = checkpoint.get('feat_mean')
        self.feat_std = checkpoint.get('feat_std')
        rospy.loginfo('控制器模型已加载: %s', model_path)

        self.safety_enabled = bool(rospy.get_param('~safety_enabled', True))
        self.osc = safety_shield.OscillationDetector()
        self.shield_counts = {}

        self.lidar_feats = None
        self.odom = None
        self.goal_enu = None
        self.last_lidar_t = None
        self.last_odom_t = None

        rospy.Subscriber('/uav/lidar/features', Float32MultiArray, self.lidar_cb,
                         queue_size=1)
        rospy.Subscriber('/uav/odom', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/uav/goal', PoseStamped, self.goal_cb, queue_size=1)
        self.pub = rospy.Publisher('/uav/cmd_vel', Twist, queue_size=1)
        rospy.loginfo('安全兜底: %s', '开启' if self.safety_enabled else '关闭')

    def lidar_cb(self, msg):
        self.lidar_feats = np.array(msg.data, dtype=np.float32)
        self.last_lidar_t = rospy.Time.now().to_sec()

    def odom_cb(self, msg):
        self.odom = msg
        self.last_odom_t = rospy.Time.now().to_sec()

    def goal_cb(self, msg):
        self.goal_enu = msg.pose.position
        rospy.loginfo('收到新目标点 (%.1f, %.1f, %.1f)',
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def inference(self):
        pos = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation

        # ENU -> NED
        cur_ned = (pos.y, pos.x, -pos.z)
        yaw_enu = 2.0 * math.atan2(q.z, q.w)
        yaw_ned = math.pi / 2.0 - yaw_enu

        goal_ned = (self.goal_enu.y, self.goal_enu.x, -self.goal_enu.z)

        feat = np.concatenate([
            self.lidar_feats,
            goal_features(cur_ned, goal_ned, yaw_ned),
        ]).astype(np.float32)

        if self.feat_mean is not None and self.feat_std is not None:
            feat = (feat - self.feat_mean) / self.feat_std

        with torch.no_grad():
            out = self.model(torch.from_numpy(feat).unsqueeze(0)).squeeze(0).numpy()

        vx, vy, yaw_rate = out * self.action_scale
        return float(vx), float(vy), float(yaw_rate), pos.z

    def make_cmd(self, vx, vy, yaw_rate, current_alt):
        """组装 Twist；垂直方向只做定高，同样受量程限制。"""
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        vertical = self.kp_alt * (self.altitude - current_alt)
        cmd.linear.z = max(-safety_shield.MAX_VERTICAL,
                           min(safety_shield.MAX_VERTICAL, vertical))
        cmd.angular.z = float(yaw_rate)
        return cmd

    def publish_hover(self, current_alt):
        """安全接管：水平速度与偏航归零，只保留定高。"""
        self.pub.publish(self.make_cmd(0.0, 0.0, 0.0, current_alt))

    def note_shield(self, result):
        if not result.active:
            return
        self.shield_counts[result.reason] = self.shield_counts.get(result.reason, 0) + 1
        rospy.logwarn_throttle(2.0, '安全层介入 [%s] 累计 %s',
                               result.reason, self.shield_counts[result.reason])

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            # 还没凑齐里程计/雷达/目标点时不下发指令，
            # 否则桥接节点会误以为要开始控制，干扰起飞
            if self.odom is None or self.lidar_feats is None \
                    or self.goal_enu is None:
                rate.sleep()
                continue

            current_alt = self.odom.pose.pose.position.z
            now = rospy.Time.now().to_sec()

            # 输入过期：不再信任旧特征，原地悬停等待恢复
            if safety_shield.is_stale(now, self.last_lidar_t, self.last_odom_t):
                self.publish_hover(current_alt)
                rospy.logwarn_throttle(2.0, '雷达/里程计数据过期，悬停等待')
                rate.sleep()
                continue

            try:
                vx, vy, yaw_rate, current_alt = self.inference()
            except Exception as exc:  # pragma: no cover
                self.publish_hover(current_alt)
                rospy.logwarn_throttle(5.0, '推理失败，悬停: %s', exc)
                rate.sleep()
                continue

            if self.safety_enabled:
                result = safety_shield.apply_shield(vx, vy, yaw_rate,
                                                    self.lidar_feats)
                result.yaw_rate = self.osc.update(result.yaw_rate)
                self.note_shield(result)
                vx, vy, yaw_rate = result.as_tuple()

            self.pub.publish(self.make_cmd(vx, vy, yaw_rate, current_alt))
            rate.sleep()


def main():
    rospy.init_node('trajectory_control')
    TrajectoryControlNode().run()


if __name__ == '__main__':
    main()
