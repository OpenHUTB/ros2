"""专家转向避障策略（开放度差 + 正前方危险度），用于生成训练标签的"教师"。

输入前视射线距离数组，输出 (steer, throttle)：
- 计算车体右侧（-60°~-15°，索引 0..9）与左侧（+15°~+60°，索引 15..24）
  的平均开放距离；期望航向朝更开放一侧偏转，steer 幅度正比于两侧开放度差
  并被限幅，平滑不激进。
- 正前方（±10° 内）有近障碍时，在基础上加大朝向更开放侧的转向；
  正前方空旷（>安全阈值）则保持直行（steer≈0），继续沿原航向前进通过障碍路段。
- throttle 按最近障碍距离分级减速。

只依赖射线（纯感知），不依赖任何真值；只作为数据采集时的监督信号。
"""
import numpy as np

# 前向窄带：|angle| <= 10° 视为"正前方"（对应索引 10~14，共 25 束、5° 间隔）
FWD_LO = 10
FWD_HI = 14
# 侧向带：索引 0..9 为车体右侧（-60°~-15°），索引 15..24 为车体左侧（+15°~+60°）
SIDE_LO = 10
SIDE_HI = 15


def _safe_mean(arr):
    return float(np.mean(arr)) if arr.size else 0.0


def expert_control(rays, cfg, yaw=0.0):
    """rays: (N,) 距离数组（米）；yaw: 当前航向偏差(rad)，来自 IMU/里程计。
    返回 (steer, throttle)。
    """
    r = np.asarray(rays, dtype=float)
    rng = cfg.lidar_range_m
    d_center = float(np.min(r[FWD_LO:FWD_HI + 1]))   # 正前方最近
    # 变量名沿用历史：d_left 实际取车体右侧射线（索引 0..9），
    # d_right 实际取车体左侧射线（索引 15..24），diff 顺序已按实测方向校准。
    d_left = _safe_mean(r[:SIDE_LO])                  # 车体右侧平均开放度
    d_right = _safe_mean(r[SIDE_HI:])                 # 车体左侧平均开放度

    # ---- 转向：朝更开放一侧温和偏转；前方安全则用航向反馈回正 ----
    diff = (d_right - d_left) / rng                    # 归一化开放度差，正=左侧更开放
    if d_center < 3.0 or abs(d_left - d_right) > 0.8:
        # 正前方有障碍或两侧明显不均衡 -> 温和朝更开放侧转向
        sign = 1.0 if diff >= 0.0 else -1.0
        steer = float(np.clip(sign * 0.45, -0.6, 0.6))
        if d_center < 2.0:
            steer = float(np.clip(sign * 0.55, -0.7, 0.7))
    else:
        # 前方安全且两侧均衡 -> 用航向偏差回正（yaw 来自 IMU/里程计）
        steer = float(np.clip(-yaw * 1.0, -0.3, 0.3))

    # ---- 速度：按最近障碍距离分级 ----
    d_min = float(np.min(r))
    if d_min > 3.0:
        throttle = cfg.cruise_throttle
    elif d_min > 2.0:
        throttle = 0.55
    elif d_min > 1.2:
        throttle = 0.35
    else:
        throttle = 0.20

    return steer, throttle
