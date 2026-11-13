"""回波强度辅助可通行性判断的纯算法实现。

本模块不依赖 ROS，便于离线单元测试；ROS 节点只负责话题收发。
"""

import math

DEFAULT_REF_INTENSITY = 1200.0


def confidence_from_intensity(intensity, ref=DEFAULT_REF_INTENSITY, floor=0.05):
    """把原始回波强度映射成 [floor, 1] 的置信度。

    镜面玻璃、黑色泡棉等低反射率的物体回波很弱，单纯用距离判断会
    误以为前方空旷；这里用强度给出"该回波有多可信"的权重。
    """
    if ref <= 0.0:
        raise ValueError("ref must be positive")
    if intensity <= 0.0:
        return floor
    return max(floor, min(1.0, float(intensity) / float(ref)))


def effective_range(range_m, confidence, uncertainty=0.6, min_range=0.05):
    """把低置信度回波折算成"更近"的安全距离。

    不确定度 sigma = range * uncertainty * (1 - confidence)，
    有效距离取 range - sigma，保证控制器对弱回波偏保守。
    """
    if range_m <= 0.0:
        return min_range
    sigma = range_m * uncertainty * (1.0 - confidence)
    return max(min_range, range_m - sigma)


def sector_min(beams, start_deg, end_deg):
    """返回角度区间内有效距离最小的一束（角度为度）。"""
    best = None
    for beam in beams:
        a = beam["angle_deg"]
        if start_deg <= a <= end_deg:
            if best is None or beam["effective"] < best["effective"]:
                best = beam
    return best


def build_beams(ranges, angle_min, angle_increment, intensities,
                ref=DEFAULT_REF_INTENSITY, uncertainty=0.6, range_max=12.0):
    """把 LaserScan 数组整理成带置信度的 beam 列表。"""
    beams = []
    for i, r in enumerate(ranges):
        if r is None or r != r or r <= 0.0 or r >= range_max:
            continue
        raw = intensities[i] if i < len(intensities) else 0.0
        conf = confidence_from_intensity(raw, ref)
        beams.append({
            "index": i,
            "angle_deg": math.degrees(angle_min + i * angle_increment),
            "range": float(r),
            "intensity": float(raw),
            "confidence": conf,
            "effective": effective_range(r, conf, uncertainty),
        })
    return beams


def _window_score(beams, center_deg, half_width_deg):
    seg = sector_min(beams, center_deg - half_width_deg, center_deg + half_width_deg)
    if seg is None:
        return None
    return seg


def choose_heading(beams, fov_deg=120.0, step_deg=5.0, window_deg=12.0,
                   stop_distance=0.45, slow_distance=1.2, max_speed=0.35):
    """在视场角内挑选最优通行方向。

    评分同时考虑：窗口内最小有效距离、偏离正前方带来的代价。
    返回可直接发布到 /cmd_vel 的指令与诊断信息。
    """
    half = fov_deg / 2.0
    best = None
    candidates = []
    angle = -half
    while angle <= half + 1e-9:
        seg = _window_score(beams, angle, window_deg)
        if seg is not None:
            score = seg["effective"] - 0.004 * abs(angle)
            candidates.append((score, angle, seg))
            if best is None or score > best[0]:
                best = (score, angle, seg)
        angle += step_deg

    centre = _window_score(beams, 0.0, window_deg)
    min_eff = centre["effective"] if centre else 0.0
    result = {
        "heading_deg": 0.0,
        "linear": 0.0,
        "angular": 0.0,
        "min_effective_front": min_eff,
        "action": "stop",
        "candidates": len(candidates),
    }
    if not candidates:
        return result

    score, heading, seg = best
    result["heading_deg"] = heading
    if seg["effective"] < stop_distance:
        result["action"] = "stop"
        return result

    clearance = max(seg["effective"], 1e-3)
    speed_scale = min(1.0, clearance / slow_distance)
    result["linear"] = max(0.0, max_speed * speed_scale)
    result["angular"] = max(-1.0, min(1.0, heading / 45.0))
    if abs(heading) < step_deg:
        result["action"] = "go" if speed_scale > 0.99 else "slow"
    elif heading > 0:
        result["action"] = "turn_left"
    else:
        result["action"] = "turn_right"
    if speed_scale < 0.35:
        result["action"] = "creep"
    return result
