# -*- coding: utf-8 -*-
"""射线扇面测距（需要 MuJoCo）。

使用 MuJoCo 原生的 mj_ray 做射线-几何求交，等价于一个小型二维激光雷达：
    - 扇面角度、最大量程、geom group 掩码全部来自 config.py；
    - 自车几何体位于 group 3，通过 geomgroup 屏蔽，避免"打到自己的轮子"，
      同时再用 bodyexclude 兜一层；
    - 命中点的横向偏移由射线角度与距离反算，用于"本车道判定"，
      邻车道/路侧目标不参与避障决策。

注意：mj_ray 的返回值是**以传入方向向量 |vec| 为单位的距离**，未命中返回 -1。
本模块统一传入**单位向量**，因此返回值直接是米；量程限制再由 max_range 截断。
（踩过的坑：若传 `方向 × max_range`，返回值会变成 0~1 之间的参数，
表现为"静止时最近距离 0.23 m"这类量纲错误。）
"""

import math

import numpy as np

from . import config


class LidarFan:
    """二维射线扇面。"""

    def __init__(self, model, data, mujoco_module,
                 site_name=config.LIDAR_SITE_NAME,
                 angles_deg=config.LIDAR_ANGLES_DEG,
                 max_range=config.LIDAR_MAX_RANGE_M,
                 geom_groups=config.LIDAR_GEOM_GROUPS,
                 lane_half_width=config.LANE_HALF_WIDTH_M,
                 exclude_body_name="car"):
        self.mujoco = mujoco_module
        self.model = model
        self.data = data
        self.max_range = float(max_range)
        self.lane_half_width = float(lane_half_width)
        self.angles_rad = np.deg2rad(np.asarray(angles_deg, dtype=np.float64))

        self.site_id = self._name_to_id(mujoco_module.mjtObj.mjOBJ_SITE, site_name)
        self.exclude_body_id = self._name_to_id(mujoco_module.mjtObj.mjOBJ_BODY, exclude_body_name)

        self.geomgroup = np.asarray(geom_groups, dtype=np.uint8)
        self._geomid = np.zeros(1, dtype=np.int32)

    # ------------------------------------------------------------------ 工具
    def _name_to_id(self, obj_type, name):
        obj_id = self.mujoco.mj_name2id(self.model, obj_type, name)
        if obj_id < 0:
            raise ValueError("场景中找不到名为 '{}' 的对象，请检查 models/car_road_scene.xml".format(name))
        return obj_id

    @property
    def beam_count(self) -> int:
        return int(self.angles_rad.size)

    # ------------------------------------------------------------------ 扫描
    def scan(self) -> np.ndarray:
        """返回各束射线距离 (m)，长度等于束数；未命中处为 max_range。

        射线方向只在**水平面内按车身航向**解算（只取 yaw，忽略俯仰与横滚）。
        这是必要的：车身在加减速时会有几度俯仰，若直接把车体坐标系向量当作射线方向，
        实测车头上仰约 3.7° 时，7.5 m 外的障碍物（0.6 m 高）就会从射线上方"消失"
        （射线抬高 7.5*sin(3.7°) ≈ 0.48 m，正好越过障碍物顶端），
        导致测距间歇性为空、决策反复急刹。
        """
        pnt = np.array(self.data.site_xpos[self.site_id], dtype=np.float64)
        rot = np.array(self.data.site_xmat[self.site_id], dtype=np.float64).reshape(3, 3)
        yaw = math.atan2(rot[1, 0], rot[0, 0])   # 车身 x 轴在水平面的朝向

        ranges = np.empty(self.beam_count, dtype=np.float64)
        for index, angle in enumerate(self.angles_rad):
            heading = yaw + angle
            direction = np.array([math.cos(heading), math.sin(heading), 0.0],
                                 dtype=np.float64)

            self._geomid[0] = -1
            # 必须传单位向量：mj_ray 的返回值以 |vec| 为单位。
            # 实测（MuJoCo 3.15，本场景静止状态）：
            #   |vec| = 1  -> 0° 方向返回 7.4800（即 7.48 m，命中障碍物 geomid=2）
            #   |vec| = 20 -> 同一方向返回 0.3740（= 7.48 / 20）
            #   未命中返回 -1
            # 因此传单位向量时返回值就是米，量程限制由下面的 min(dist, max_range) 完成。
            dist = self.mujoco.mj_ray(
                self.model, self.data, pnt, direction,
                self.geomgroup, 1, self.exclude_body_id, self._geomid,
            )
            if self._geomid[0] < 0 or dist is None or not np.isfinite(dist) or dist < 0.0:
                ranges[index] = self.max_range
            else:
                ranges[index] = float(min(dist, self.max_range))
        return ranges

    # ------------------------------------------------------------------ 语义
    def lateral_offsets(self, ranges: np.ndarray) -> np.ndarray:
        """由距离与射线角度反算命中点横向偏移：lateral = d * sin(theta)。"""
        return ranges * np.sin(self.angles_rad)

    def in_lane_mask(self, ranges: np.ndarray) -> np.ndarray:
        """本车道掩码：在量程内且横向偏移不超过车道半宽。"""
        valid = ranges < (self.max_range - 1e-9)
        lateral = np.abs(self.lateral_offsets(ranges))
        return valid & (lateral <= self.lane_half_width)

    def min_distance(self, ranges=None):
        """本车道最近距离；无有效命中时返回 None。"""
        if ranges is None:
            ranges = self.scan()
        mask = self.in_lane_mask(ranges)
        if not np.any(mask):
            return None
        return float(np.min(ranges[mask]))
