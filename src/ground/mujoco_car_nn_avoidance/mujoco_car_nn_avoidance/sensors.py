"""前视激光射线感知：在水平面内按车头航向发出一个射线扇面。

用 MuJoCo 原生 mj_ray 做射线-几何求交，等价于一个小型二维激光雷达：
    - 射线起点取车头 site "lidar"（与车头到障碍物的真实间距同一基准）；
    - 方向只在水平面内按航向 yaw 解算（忽略俯仰/横滚），
      避免加减速时车身俯仰导致射线越过障碍物顶端而漏检；
    - 传入**单位向量**：mj_ray 的返回值以 |vec| 为单位，因此返回米；
      未命中（返回 -1）或超过量程按 lidar_range_m 截断；
    - geomgroup 掩码来自 config：group0 障碍可见，group1 地面屏蔽，
      group2 自车屏蔽（并叠加 bodyexclude 兜底），避免打到自己的轮子。

``ray_scan(..., noisy=False)`` 返回**无噪声**的真实距离。最小离障这类安全指标必须用它：
含噪射线在 σ 较大时会出现 -1 m 量级的极值采样并被 clip 到 0，看起来像「擦到障碍」，
其实只是噪声的极值统计。
"""
import math

import numpy as np
import mujoco

from .sensor_noise import apply_sensor_noise


def ray_scan(env, pos=None, yaw=None, noisy=True):
    """返回 cfg.lidar_angles_deg 对应的射线距离数组 (dtype=float32)。"""
    cfg = env.cfg
    model = env.model
    data = env.data

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "lidar")
    pnt = np.array(data.site_xpos[site_id], dtype=np.float64)
    if yaw is None:
        yaw = env.yaw()
    angles = np.deg2rad(np.asarray(cfg.lidar_angles_deg, dtype=float))
    n = len(angles)

    geomgroup = np.asarray(cfg.lidar_geom_groups, dtype=np.uint8)
    geomid = np.zeros(1, dtype=np.int32)
    ranges = np.full(n, cfg.lidar_range_m, dtype=np.float32)
    for i in range(n):
        direction = np.array([math.cos(yaw + angles[i]),
                              math.sin(yaw + angles[i]),
                              0.0], dtype=np.float64)
        geomid[0] = -1
        dist = mujoco.mj_ray(model, data, pnt, direction, geomgroup,
                             1, int(env.chassis_id), geomid)
        if geomid[0] >= 0 and dist is not None and np.isfinite(dist) and dist > 0.0:
            ranges[i] = min(float(dist), cfg.lidar_range_m)
    if not noisy:
        return ranges
    return _apply_sensor_noise(env, ranges)


def _apply_sensor_noise(env, ranges):
    """按 cfg 的 σ / 丢束率给射线加噪；两者都为 0 时原样返回。

    顺序：先判丢束、只对有效回波加噪、最后把丢束项写回量程上限，
    保证「随机无回波」的语义不被噪声污染（实现见 sensor_noise.py）。
    """
    cfg = env.cfg
    sigma = float(getattr(cfg, "lidar_noise_sigma_m", 0.0))
    dropout = float(getattr(cfg, "lidar_dropout_prob", 0.0))
    if sigma <= 0.0 and dropout <= 0.0:
        return ranges
    rng = getattr(env, "_sensor_rng", None)
    if rng is None:
        rng = np.random.default_rng(int(getattr(cfg, "sensor_seed", 0)))
        env._sensor_rng = rng
    return apply_sensor_noise(ranges, sigma, dropout, rng, cfg.lidar_range_m)


def normalize_rays(rays, range_m):
    """把距离归一化到 [0,1]（越近值越接近 0，越远越接近 1）。"""
    return np.clip(rays / range_m, 0.0, 1.0)
