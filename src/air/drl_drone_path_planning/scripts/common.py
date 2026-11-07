"""Shared configuration, geometry and observation utilities.

本模块把「配置读取、坐标变换、激光扇形直方图、观测拼装、动作缩放」集中在一处，
训练环境（drone_env.py）和 ROS 部署节点（drl_nav_node.py）都从这里导入，
从代码层面保证训练/部署的接口一致，避免出现「训练时好好的，部署时乱飞」。

Developer: 汤湘粤 (学号: 2535030067)
"""

from __future__ import annotations

import math
import os
from typing import Any, Dict, Iterable, Mapping, Sequence

import numpy as np
import yaml

# ---------------------------------------------------------------------------
# 路径与配置
# ---------------------------------------------------------------------------

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_CONFIG_PATH = os.path.join(PACKAGE_DIR, "config", "drl_nav.yaml")
DEFAULT_MODEL_PATH = os.path.join(PACKAGE_DIR, "models", "policy_weights.npz")


def _deep_merge(base: Dict[str, Any], override: Mapping[str, Any]) -> Dict[str, Any]:
    """Recursively merge ``override`` into ``base`` and return ``base``.

    递归合并配置字典；override 中的同名键优先，便于 launch 参数覆盖 YAML。
    """
    for key, value in override.items():
        if isinstance(value, Mapping) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def load_config(path: str = "", overrides: Mapping[str, Any] | None = None) -> Dict[str, Any]:
    """Load ``drl_nav`` section from a YAML file.

    读取 YAML 并返回 ``drl_nav`` 子字典。若文件不存在则抛出 FileNotFoundError，
    让人一眼看到路径错误，而不是默默使用默认值。
    """
    config_path = path or DEFAULT_CONFIG_PATH
    with open(config_path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    config = dict(raw.get("drl_nav", raw))
    if overrides:
        _deep_merge(config, overrides)
    return config


def observation_layout(sectors: int = 16) -> Dict[str, slice]:
    """Return the slice of every observation block.

    返回观测各分块的切片，供节点/测试/文档共用，避免手写魔法下标。
    """
    return {
        "goal_body": slice(0, 3),
        "rpy": slice(3, 6),
        "vel_body": slice(6, 9),
        "ang_vel_body": slice(9, 12),
        "lidar": slice(12, 12 + int(sectors)),
    }


def observation_dim(cfg: Mapping[str, Any]) -> int:
    """Total observation dimension for a config."""
    return 12 + int(cfg["observation"]["sectors"])


# ---------------------------------------------------------------------------
# 四元数与坐标变换
# ---------------------------------------------------------------------------


def quat_to_rpy(quat: Sequence[float]) -> np.ndarray:
    """Convert ``(x, y, z, w)`` quaternion to roll-pitch-yaw (rad).

    采用 ROS/ENU 的 ZYX 欧拉角约定；输入四元数会先归一化。
    """
    x, y, z, w = [float(v) for v in quat]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.zeros(3, dtype=np.float64)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(float(np.clip(sinp, -1.0, 1.0)))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw], dtype=np.float64)


def rpy_to_quat(rpy: Sequence[float]) -> np.ndarray:
    """Convert roll-pitch-yaw (rad) to ``(x, y, z, w)`` quaternion."""
    roll, pitch, yaw = [float(v) for v in rpy]
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ], dtype=np.float64)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi)``."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def world_to_body(vec: Sequence[float], yaw: float) -> np.ndarray:
    """Rotate a world-frame vector into the body frame (yaw only).

    只绕 z 轴旋转，忽略 roll/pitch —— 路径规划层关心的是水平朝向。
    """
    x, y, z = [float(v) for v in vec]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * x + s * y, -s * x + c * y, z], dtype=np.float64)


def body_to_world(vec: Sequence[float], yaw: float) -> np.ndarray:
    """Rotate a body-frame vector into the world frame (yaw only)."""
    x, y, z = [float(v) for v in vec]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * x - s * y, s * x + c * y, z], dtype=np.float64)


# ---------------------------------------------------------------------------
# 激光雷达：扇形直方图与圆柱射线求交
# ---------------------------------------------------------------------------


def sector_angles(sectors: int) -> np.ndarray:
    """Sector centre angles in the body frame; sector 0 points forward (+x)."""
    return np.arange(int(sectors), dtype=np.float64) * (2.0 * math.pi / float(sectors))


def histogram_from_ranges(ranges: Iterable[float],
                          angle_min: float,
                          angle_increment: float,
                          range_min: float,
                          range_max: float,
                          sectors: int,
                          max_range: float) -> np.ndarray:
    """Convert a LaserScan range array into a ``[0, 1]`` free-space histogram.

    每个扇区取最近距离，``1`` 表示畅通、``0`` 表示贴脸障碍。角度按「0 = 正前方、
    逆时针为正」映射，与训练环境的射线方向保持一致；无效/超量程读数按畅通处理。
    """
    sectors = int(sectors)
    width = 2.0 * math.pi / sectors
    nearest = np.full(sectors, float("inf"), dtype=np.float64)
    ranges = np.asarray(list(ranges), dtype=np.float64)
    if ranges.size == 0:
        return np.ones(sectors, dtype=np.float32)

    angles = angle_min + np.arange(ranges.size, dtype=np.float64) * angle_increment
    valid = np.isfinite(ranges) & (ranges >= max(float(range_min), 1e-6))
    valid &= ranges <= max(float(range_max), float(max_range))
    if not np.any(valid):
        return np.ones(sectors, dtype=np.float32)

    angles = angles[valid]
    distances = ranges[valid]
    indices = np.floor((angles + width / 2.0) / width).astype(np.int64) % sectors
    np.minimum.at(nearest, indices, distances)
    histogram = np.ones(sectors, dtype=np.float64)
    hit = np.isfinite(nearest)
    histogram[hit] = np.clip(nearest[hit] / float(max_range), 0.0, 1.0)
    return histogram.astype(np.float32)


def ray_circle_distance(origin: Sequence[float],
                        direction: Sequence[float],
                        center: Sequence[float],
                        radius: float) -> float:
    """Distance from a 2-D ray to a circle, or ``inf`` if there is no hit.

    圆柱障碍在水平面内退化为圆，用解析求交，比逐点采样更快也更稳定。
    """
    ox, oy = float(origin[0]), float(origin[1])
    dx, dy = float(direction[0]), float(direction[1])
    cx, cy = float(center[0]), float(center[1])
    radius = float(radius)
    ocx, ocy = ox - cx, oy - cy
    b = ocx * dx + ocy * dy
    c = ocx * ocx + ocy * ocy - radius * radius
    disc = b * b - c
    if disc < 0.0:
        return float("inf")
    sqrt_disc = math.sqrt(disc)
    t_near = -b - sqrt_disc
    if t_near >= 0.0:
        return t_near
    t_far = -b + sqrt_disc
    return t_far if t_far >= 0.0 else float("inf")


def raycast_histogram(position: Sequence[float],
                      yaw: float,
                      obstacles: Iterable[Sequence[float]],
                      sectors: int,
                      max_range: float,
                      height_min: float = 0.2) -> np.ndarray:
    """Ray-cast ``sectors`` horizontal rays against cylindrical obstacles.

    障碍格式 ``[x, y, radius, height]``；无人机高于圆柱顶部时可以飞越。
    返回与 :func:`histogram_from_ranges` 同定义的畅通度直方图。
    """
    sectors = int(sectors)
    z = float(position[2])
    nearest = np.full(sectors, float("inf"), dtype=np.float64)
    world_angles = yaw + sector_angles(sectors)
    directions = np.stack([np.cos(world_angles), np.sin(world_angles)], axis=1)
    origin = np.array([float(position[0]), float(position[1])], dtype=np.float64)
    for obstacle in obstacles:
        ox, oy, radius, height = (float(v) for v in obstacle[:4])
        if z < float(height_min) or z > height:
            continue
        offset = origin - np.array([ox, oy], dtype=np.float64)
        b = directions @ offset
        c = float(offset @ offset) - radius * radius
        discriminant = b * b - c
        valid = discriminant >= 0.0
        if not np.any(valid):
            continue
        sqrt_disc = np.sqrt(discriminant[valid])
        t_near = -b[valid] - sqrt_disc
        t_far = -b[valid] + sqrt_disc
        distances = np.where(
            t_near >= 0.0, t_near, np.where(t_far >= 0.0, t_far, np.inf)
        )
        hits = np.flatnonzero(valid)
        np.minimum.at(nearest, hits, distances)
    histogram = np.ones(sectors, dtype=np.float64)
    hit = np.isfinite(nearest)
    histogram[hit] = np.clip(nearest[hit] / float(max_range), 0.0, 1.0)
    return histogram.astype(np.float32)


# ---------------------------------------------------------------------------
# 观测与动作
# ---------------------------------------------------------------------------


def build_observation(position: Sequence[float],
                      rpy: Sequence[float],
                      vel_body: Sequence[float],
                      ang_vel_body: Sequence[float],
                      goal_world: Sequence[float],
                      histogram: Sequence[float],
                      cfg: Mapping[str, Any]) -> np.ndarray:
    """Assemble the 28-D (12 + sectors) observation used by policy and env.

    布局：

    =====  ==========================================
    下标   含义
    =====  ==========================================
    0..2   目标相对位置（机体系 x 前 / y 左 / z 上，按 goal_norm 归一化）
    3..5   roll / pitch / yaw（除以 pi）
    6..8   机体线速度（按最大速度归一化）
    9..11  机体角速度（除以 pi）
    12..   激光扇形畅通度直方图，1 = 畅通
    =====  ==========================================
    """
    observation_cfg = cfg["observation"]
    limit_cfg = cfg["limits"]
    goal_norm = max(float(cfg["goal"]["goal_norm"]), 1e-6)
    max_vel_xy = max(float(limit_cfg["max_vel_xy"]), 1e-6)
    max_vel_z = max(float(limit_cfg["max_vel_z"]), 1e-6)

    position = np.asarray(position, dtype=np.float64)
    rpy = np.asarray(rpy, dtype=np.float64)
    yaw = float(rpy[2])
    goal_delta = np.asarray(goal_world, dtype=np.float64) - position
    goal_body = world_to_body(goal_delta, yaw) / goal_norm

    attitude = np.clip(rpy / math.pi, -1.0, 1.0)
    velocity = np.asarray(vel_body, dtype=np.float64).copy()
    velocity[0:2] /= max_vel_xy
    velocity[2] /= max_vel_z
    angular = np.asarray(ang_vel_body, dtype=np.float64) / math.pi
    lidar = np.asarray(histogram, dtype=np.float64)
    expected = int(observation_cfg["sectors"])
    if lidar.size != expected:
        raise ValueError(
            "lidar histogram has %d sectors, config expects %d" % (lidar.size, expected)
        )

    observation = np.concatenate([
        np.clip(goal_body, -1.0, 1.0),
        np.clip(attitude, -1.0, 1.0),
        np.clip(velocity, -1.0, 1.0),
        np.clip(angular, -1.0, 1.0),
        np.clip(lidar, 0.0, 1.0),
    ]).astype(np.float32)
    return observation


def apply_altitude_hold(action: Sequence[float],
                        position: Sequence[float],
                        goal: Sequence[float],
                        cfg: Mapping[str, Any]) -> np.ndarray:
    """Damp vertical action when the current goal is at the same altitude.

    多旋翼在巡航段通常由高度环保持定高；让策略在目标高度附近继续输出大幅
    ``vz`` 既没有收益，也容易学成「俯冲撞地」的局部最优。这里按配置把垂直
    动作衰减为 ``altitude_damping`` 倍，训练环境和 ROS 节点共用同一处理。
    """
    result = np.array(action, dtype=np.float64).reshape(-1).copy()
    if result.size < 3:
        return result
    env_cfg = cfg.get("env", {})
    band = float(env_cfg.get("altitude_hold_band", 0.0) or 0.0)
    damping = float(env_cfg.get("altitude_damping", 1.0))
    if band > 0.0 and abs(float(goal[2]) - float(position[2])) <= band:
        result[2] *= damping
    return result


def action_to_velocity(action: Sequence[float], cfg: Mapping[str, Any]) -> np.ndarray:
    """Map a normalized ``[-1, 1]`` policy action to ``(vx, vy, vz, yaw_rate)``.

    输出的线速度是**机体系**，与 ``geometry_msgs/Twist`` 的约定一致。
    """
    limits = cfg["limits"]
    clipped = np.clip(np.asarray(action, dtype=np.float64).reshape(-1), -1.0, 1.0)
    if clipped.size < 4:
        raise ValueError("action must have at least 4 elements")
    return np.array([
        clipped[0] * float(limits["max_vel_xy"]),
        clipped[1] * float(limits["max_vel_xy"]),
        clipped[2] * float(limits["max_vel_z"]),
        clipped[3] * float(limits["max_yaw_rate"]),
    ], dtype=np.float64)


def velocity_to_action(velocity: Sequence[float], cfg: Mapping[str, Any]) -> np.ndarray:
    """Inverse of :func:`action_to_velocity`, used by the geometric baseline."""
    limits = cfg["limits"]
    velocity = np.asarray(velocity, dtype=np.float64).reshape(-1)
    return np.array([
        velocity[0] / max(float(limits["max_vel_xy"]), 1e-6),
        velocity[1] / max(float(limits["max_vel_xy"]), 1e-6),
        velocity[2] / max(float(limits["max_vel_z"]), 1e-6),
        velocity[3] / max(float(limits["max_yaw_rate"]), 1e-6),
    ], dtype=np.float64)


def front_min_distance(histogram: Sequence[float],
                       cfg: Mapping[str, Any]) -> float:
    """Nearest obstacle distance inside the forward cone, in metres."""
    sectors = len(histogram)
    if sectors == 0:
        return float(cfg["observation"]["lidar_range"])
    angles = sector_angles(sectors)
    angles = np.where(angles > math.pi, angles - 2.0 * math.pi, angles)
    half_angle = float(cfg["safety"]["front_half_angle"])
    mask = np.abs(angles) <= half_angle
    if not np.any(mask):
        mask = np.zeros(sectors, dtype=bool)
        mask[0] = True
    free_space = np.asarray(histogram, dtype=np.float64)[mask]
    closest_free = float(np.min(free_space))
    return float(closest_free * float(cfg["observation"]["lidar_range"]))


def path_length(points: Iterable[Sequence[float]]) -> float:
    """Cumulative Euclidean length of a 3-D polyline."""
    points = [np.asarray(point, dtype=np.float64) for point in points]
    if len(points) < 2:
        return 0.0
    return float(sum(np.linalg.norm(points[i + 1] - points[i]) for i in range(len(points) - 1)))


def resolve_model_path(cfg: Mapping[str, Any]) -> str:
    """Resolve the configured model path, falling back to the package default.

    在 catkin 安装布局中，``common.py`` 位于 ``devel/lib/...``，而模型位于
    ``share/...``，因此默认路径还需要通过 ``rospkg`` 兜底；直接源码运行时
    ``DEFAULT_MODEL_PATH`` 已经可用。
    """
    configured = str(cfg.get("model_path", "") or "").strip()
    if configured:
        return configured
    if os.path.isfile(DEFAULT_MODEL_PATH):
        return DEFAULT_MODEL_PATH
    try:
        import rospkg  # ROS-only, imported lazily
        share_dir = rospkg.RosPack().get_path("drl_drone_path_planning")
        candidate = os.path.join(share_dir, "models", "policy_weights.npz")
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass
    return DEFAULT_MODEL_PATH
