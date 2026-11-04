"""
水下任务轨迹规划与生成器 (Underwater Trajectory Generator)
======================================================
生成水下机器人三维空间基准航迹，支持：
  1. 3D 空间螺旋巡检轨迹 (3D Helical Inspection Path)
  2. 割草机多航路点网格搜索轨迹 (Lawnmower Waypoint Survey)
提供实时期望位姿、线速度、期望偏航角与标准 ROS 2 nav_msgs/Path 广播接口。
"""

import math
from typing import Dict, Any
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header


class TrajectoryGenerator:
    """水下三维参考轨迹生成器"""

    def __init__(
        self,
        trajectory_type: str = "3d_helix",
        radius: float = 2.2,
        angular_speed: float = 0.18,
        base_depth: float = -1.5,
        depth_amplitude: float = 0.6,
        depth_freq: float = 0.09,
    ):
        self.traj_type = trajectory_type
        self.radius = radius
        self.omega = angular_speed
        self.base_depth = base_depth
        self.depth_amp = depth_amplitude
        self.depth_freq = depth_freq

    def get_state(self, t: float) -> Dict[str, Any]:
        """获取 t 时刻的期望空间状态 (位置, 速度, 偏航角, 偏航角速度)"""
        if self.traj_type == "3d_helix":
            # 1. 水平圆周运动
            theta = self.omega * t
            xd = self.radius * math.cos(theta)
            yd = self.radius * math.sin(theta)
            vxd = -self.radius * self.omega * math.sin(theta)
            vyd = self.radius * self.omega * math.cos(theta)

            # 2. 垂向平滑正弦升降 (在 -0.9m ~ -2.1m 间平稳起伏)
            zd = self.base_depth + self.depth_amp * math.sin(self.depth_freq * t)
            vzd = self.depth_amp * self.depth_freq * math.cos(self.depth_freq * t)

            # 3. 切线对齐期望艏向角 (ROV 机头始终对准前进速度方向)
            yaw_d = math.atan2(vyd, vxd)
            yaw_rate_d = self.omega

        elif self.traj_type == "lawnmower":
            # 割草机网格折线平移
            cycle = 40.0
            phase = (t % cycle) / cycle
            side = 3.5
            if phase < 0.25:
                # 边 1: (0, 0) -> (side, 0)
                u = phase / 0.25
                xd, yd, yaw_d = u * side - side/2, -side/2, 0.0
                vxd, vyd = side / (cycle * 0.25), 0.0
            elif phase < 0.5:
                # 边 2: 向下平移
                u = (phase - 0.25) / 0.25
                xd, yd, yaw_d = side/2, -side/2 + u * side, math.pi / 2
                vxd, vyd = 0.0, side / (cycle * 0.25)
            elif phase < 0.75:
                # 边 3: 反向向左
                u = (phase - 0.5) / 0.25
                xd, yd, yaw_d = side/2 - u * side, side/2, math.pi
                vxd, vyd = -side / (cycle * 0.25), 0.0
            else:
                # 边 4: 向上闭环
                u = (phase - 0.75) / 0.25
                xd, yd, yaw_d = -side/2, side/2 - u * side, -math.pi / 2
                vxd, vyd = 0.0, -side / (cycle * 0.25)

            zd = self.base_depth
            vzd = 0.0
            yaw_rate_d = 0.0

        else:
            xd, yd, zd = 0.0, 0.0, self.base_depth
            vxd, vyd, vzd = 0.0, 0.0, 0.0
            yaw_d, yaw_rate_d = 0.0, 0.0

        return {
            "pos": np.array([xd, yd, zd], dtype=np.float64),
            "vel": np.array([vxd, vyd, vzd], dtype=np.float64),
            "yaw": float(yaw_d),
            "yaw_rate": float(yaw_rate_d),
        }

    def generate_path_msg(self, header: Header, duration: float = 45.0, num_points: int = 120) -> Path:
        """生成供 RViz 与 3D 渲染器展示的全局参考航线 (nav_msgs/Path)"""
        path_msg = Path()
        path_msg.header = header
        path_msg.header.frame_id = "world"

        ts = np.linspace(0.0, duration, num_points)
        for t in ts:
            st = self.get_state(t)
            pose_stamped = PoseStamped()
            pose_stamped.header = header
            pose_stamped.header.frame_id = "world"
            pose_stamped.pose.position.x = float(st["pos"][0])
            pose_stamped.pose.position.y = float(st["pos"][1])
            pose_stamped.pose.position.z = float(st["pos"][2])

            yaw = st["yaw"]
            pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
            pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)

            path_msg.poses.append(pose_stamped)

        return path_msg
