"""
水下多波束成像声呐仿真模块 (Multibeam Imaging Sonar)
===================================================
基于 MuJoCo 底层 mj_ray 射线碰撞追踪 API，模拟水下机器人的前视多波束声呐探测。
功能：
  1. 在机头扇形视场角 (FOV) 内发射高密度多波束声学射线
  2. 计算与水下人工结构、障碍物、海床及浮标的碰撞距离与回波强度
  3. 输出标准 ROS 2 LaserScan 与 PointCloud2 消息
"""

import math
from typing import List, Dict, Any, Tuple
import numpy as np
import mujoco
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


class MultibeamSonar:
    """水下多波束前视声呐仿真器"""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        body_name: str = "rov",
        num_beams: int = 72,
        fov_deg: float = 120.0,
        min_range: float = 0.2,
        max_range: float = 15.0,
        sensor_offset: Tuple[float, float, float] = (0.41, 0.0, 0.0),
    ):
        self.model = model
        self.data = data
        self.body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        self.num_beams = num_beams
        self.fov_rad = math.radians(fov_deg)
        self.min_range = min_range
        self.max_range = max_range
        self.sensor_offset = np.array(sensor_offset, dtype=np.float64)

        self.angle_min = -self.fov_rad / 2.0
        self.angle_max = self.fov_rad / 2.0
        self.angle_increment = self.fov_rad / max(1, num_beams - 1)

        # 预计算各波束在机体坐标系下的单位方向向量
        self.beam_angles = np.linspace(self.angle_min, self.angle_max, num_beams)
        self.local_beam_dirs = np.zeros((num_beams, 3), dtype=np.float64)
        for i, ang in enumerate(self.beam_angles):
            self.local_beam_dirs[i] = [math.cos(ang), math.sin(ang), 0.0]

        # 缓存数据
        self.ranges = np.full(num_beams, max_range, dtype=np.float32)
        self.intensities = np.zeros(num_beams, dtype=np.float32)
        self.hit_geom_ids = np.full(num_beams, -1, dtype=np.int32)
        self.closest_obstacle_dist = max_range
        self.closest_obstacle_angle = 0.0

    @property
    def range_min(self) -> float:
        return float(self.min_range)

    @property
    def range_max(self) -> float:
        return float(self.max_range)

    @property
    def last_closest_dist(self) -> float:
        return float(self.closest_obstacle_dist)

    @property
    def last_ranges(self) -> List[float]:
        return [float(min(r, self.max_range)) if not math.isinf(r) else float(self.max_range) for r in self.ranges]

    def update(self) -> Dict[str, Any]:
        """执行射线碰撞追踪并更新声呐测距数据"""
        pos_rov = self.data.sensor("pos").data[:3].copy()
        rot_mat = self.data.xmat[self.body_id].reshape((3, 3))

        # 声呐基阵在世界坐标系下的绝对原点
        sonar_origin_world = pos_rov + rot_mat @ self.sensor_offset

        # 将机体坐标系下的各波束方向旋转到世界坐标系
        world_beam_dirs = (rot_mat @ self.local_beam_dirs.T).T

        min_dist = self.max_range
        min_angle = 0.0
        geomid_buf = np.zeros(1, dtype=np.int32)

        for i in range(self.num_beams):
            vec = world_beam_dirs[i]
            dist = mujoco.mj_ray(
                self.model,
                self.data,
                sonar_origin_world,
                vec,
                None,
                1,
                self.body_id,
                geomid_buf,
            )

            if dist >= self.min_range and dist <= self.max_range:
                self.ranges[i] = float(dist)
                self.hit_geom_ids[i] = int(geomid_buf[0])
                self.intensities[i] = float(max(0.0, 100.0 / (dist * dist + 1.0)))
                if dist < min_dist:
                    min_dist = dist
                    min_angle = float(self.beam_angles[i])
            else:
                self.ranges[i] = float('inf')
                self.intensities[i] = 0.0
                self.hit_geom_ids[i] = -1

        self.closest_obstacle_dist = float(min_dist)
        self.closest_obstacle_angle = float(min_angle)

        return {
            "ranges": self.ranges.copy(),
            "intensities": self.intensities.copy(),
            "closest_dist": self.closest_obstacle_dist,
            "closest_angle": self.closest_obstacle_angle,
        }

    def create_laserscan_msg(self, header: Header) -> LaserScan:
        """打包为 ROS 2 标准 LaserScan 消息"""
        msg = LaserScan()
        msg.header = header
        msg.header.frame_id = "sonar_link"
        msg.angle_min = float(self.angle_min)
        msg.angle_max = float(self.angle_max)
        msg.angle_increment = float(self.angle_increment)
        msg.time_increment = 0.0
        msg.scan_time = 0.05
        msg.range_min = float(self.min_range)
        msg.range_max = float(self.max_range)
        msg.ranges = [float(r) for r in self.ranges]
        msg.intensities = [float(it) for it in self.intensities]
        return msg

    def get_pointcloud_xyz(self) -> np.ndarray:
        """获取声呐探测到的局部 3D 点云坐标数组 (N, 3)"""
        valid_mask = np.isfinite(self.ranges) & (self.ranges >= self.min_range) & (self.ranges <= self.max_range)
        if not np.any(valid_mask):
            return np.empty((0, 3), dtype=np.float32)

        valid_ranges = self.ranges[valid_mask]
        valid_angles = self.beam_angles[valid_mask]

        xs = valid_ranges * np.cos(valid_angles) + self.sensor_offset[0]
        ys = valid_ranges * np.sin(valid_angles) + self.sensor_offset[1]
        zs = np.full_like(xs, self.sensor_offset[2])

        return np.column_stack([xs, ys, zs]).astype(np.float32)
