"""纯几何计算：相机内参、世界坐标到像素的投影。

这一层不 import carla，可以在没装仿真器的机器上单独测试。
"""
import math

import numpy as np


def intrinsics(width, height, fov_deg):
    """由水平视场角算 3x3 内参矩阵。

    CARLA 的 fov 是水平视场角，所以焦距 f = (W/2) / tan(fov/2)，
    水平和垂直共用一个 f（方形像素）。
    """
    f = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return np.array([[f, 0.0, width / 2.0],
                     [0.0, f, height / 2.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def world_to_camera(points_world, camera_pose):
    """世界坐标转相机坐标。

    camera_pose 是 4x4 的相机到世界变换（CARLA 的 transform.get_matrix()），
    这里取逆。相机坐标系：x 向前、y 向右、z 向上。
    """
    points_world = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    inverse = np.linalg.inv(np.asarray(camera_pose, dtype=np.float64))
    homo = np.hstack([points_world, np.ones((points_world.shape[0], 1))])
    return (inverse @ homo.T).T[:, :3]


def project(points_world, camera_pose, k):
    """投影到像素平面，返回 (uv, depth, valid)。

    valid 同时要求点在相机前方、且落在图像范围内。
    v 方向向下，所以用 cy - f * z / x。
    """
    cam = world_to_camera(points_world, camera_pose)
    depth = cam[:, 0]
    valid = depth > 1e-3
    uv = np.full((cam.shape[0], 2), np.nan)
    uv[valid, 0] = k[0, 0] * cam[valid, 1] / depth[valid] + k[0, 2]
    uv[valid, 1] = k[1, 2] - k[1, 1] * cam[valid, 2] / depth[valid]
    inside = (valid
              & (uv[:, 0] >= 0) & (uv[:, 0] < 2 * k[0, 2])
              & (uv[:, 1] >= 0) & (uv[:, 1] < 2 * k[1, 2]))
    return uv, depth, inside


def lateral_offset(ego_xy, ego_yaw, center_xy, road_yaw):
    """带符号横向偏移：左偏为正、右偏为负（与仓库 LDW 模块同口径）。"""
    dx = ego_xy[0] - center_xy[0]
    dy = ego_xy[1] - center_xy[1]
    return -dx * math.sin(road_yaw) + dy * math.cos(road_yaw)


def wrap_angle(angle):
    """把角度折到 [-pi, pi]。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi