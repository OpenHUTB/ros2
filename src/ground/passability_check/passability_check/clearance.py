"""把车辆「坐」到剖面上，逐点判断车身下缘有没有钻到路面以下（纯标准库）。

做法分三层：

1. `contact_front_x()`  给定后轴接地点，二分求前轴接地点，使两接地点弦长正好等于轴距；
2. `seat()`             由两接地点定出车身的俯仰角，得到车体位姿；
3. `scan_profile()`     让车沿剖面走一遍（后轴从剖面左端扫到右端），
                        每一档都把下缘采样点变换到世界坐标，与路面高度比较，
                        记下最大侵入量与它发生的位置（车头 / 车底中部 / 车尾）。

为什么不只用角度判据：接近角/离去角/纵向通过角对应「无限长斜面」，
真实坡道有长度、有圆角，短坡面（比如 0.7 m 宽的减速带）用角度判据会过于保守。
数值采样直接回答「这一条剖面到底蹭不蹭、蹭多深」。
"""

import math

EPS = 1e-9

SECTION_NOSE = "车头"
SECTION_MID = "车底中部"
SECTION_TAIL = "车尾"


class ClearanceError(ValueError):
    """几何求解失败（比如剖面陡到前轴无法同时接地）。"""


def _chord_error(terrain, rear_x, wheelbase):
    """后轴固定在 rear_x 时，候选前轴位置对应的 (弦长 - 轴距)。"""
    def error(front_x):
        dz = terrain.height(front_x) - terrain.height(rear_x)
        return math.hypot(front_x - rear_x, dz) - wheelbase
    return error


def contact_front_x(terrain, rear_x, wheelbase, tolerance=1e-10, max_iter=200):
    """二分求前轴接地点的 x，使两接地点弦长等于轴距。

    弦长随前轴 x 单调递增（水平距离在增加），所以二分是安全的。
    """
    if wheelbase <= 0.0:
        raise ClearanceError("轴距必须大于 0：%r" % (wheelbase,))
    error = _chord_error(terrain, rear_x, wheelbase)
    low = rear_x
    high = rear_x + wheelbase + 1.0
    if error(high) < 0.0:
        raise ClearanceError("剖面在 x=%.3f 之后无法让前轴接地" % rear_x)
    for _ in range(max_iter):
        mid = 0.5 * (low + high)
        if error(mid) < 0.0:
            low = mid
        else:
            high = mid
        if high - low < tolerance:
            break
    return 0.5 * (low + high)


def seat(terrain, vehicle, rear_x):
    """把车「坐」在剖面上：返回后轴、前轴接地点与车身俯仰角。"""
    rear_z = terrain.height(rear_x)
    front_x = contact_front_x(terrain, rear_x, vehicle.wheelbase)
    front_z = terrain.height(front_x)
    pitch = math.atan2(front_z - rear_z, front_x - rear_x)
    return {
        "rear_x": rear_x,
        "rear_z": rear_z,
        "front_x": front_x,
        "front_z": front_z,
        "pitch": pitch,
        "pitch_deg": math.degrees(pitch),
        "chord": math.hypot(front_x - rear_x, front_z - rear_z),
    }


def vehicle_to_world(pose, x_v, z_v):
    """车体坐标 → 世界坐标（原点在后轴接地点，绕后轴旋转 pitch）。"""
    cos_p = math.cos(pose["pitch"])
    sin_p = math.sin(pose["pitch"])
    return (pose["rear_x"] + x_v * cos_p - z_v * sin_p,
            pose["rear_z"] + x_v * sin_p + z_v * cos_p)


def lower_contour_world(vehicle, pose, samples=41):
    """车身下缘采样点（世界坐标），带所属区段标签。"""
    result = []
    for x_v, z_v in vehicle.lower_contour(samples):
        world_x, world_z = vehicle_to_world(pose, x_v, z_v)
        if x_v > vehicle.wheelbase + EPS:
            section = SECTION_NOSE
        elif x_v < -EPS:
            section = SECTION_TAIL
        else:
            section = SECTION_MID
        result.append({
            "x": world_x,
            "z": world_z,
            "x_v": x_v,
            "z_v": z_v,
            "section": section,
        })
    return result


def _section_of(vehicle, x_v):
    if x_v > vehicle.wheelbase + EPS:
        return SECTION_NOSE
    if x_v < -EPS:
        return SECTION_TAIL
    return SECTION_MID


def penetration_at(terrain, vehicle, pose, samples=41):
    """某一位姿下的侵入量。

    返回 `{depth, margin, x, z, x_v, section, terrain_z, points}`：
    * `depth`  最大侵入深度（米，> 0 表示托底，= 0 表示至少擦着）
    * `margin` 最小余量（米，`depth` 的相反数，> 0 表示安全）
    """
    worst = {
        "depth": -float("inf"),
        "margin": float("inf"),
        "x": 0.0,
        "z": 0.0,
        "x_v": 0.0,
        "section": SECTION_MID,
        "terrain_z": 0.0,
        "points": 0,
    }
    contour = lower_contour_world(vehicle, pose, samples)
    for point in contour:
        terrain_z = terrain.height(point["x"])
        depth = terrain_z - point["z"]
        if depth > worst["depth"]:
            worst.update({
                "depth": depth,
                "margin": -depth,
                "x": point["x"],
                "z": point["z"],
                "x_v": point["x_v"],
                "section": point["section"],
                "terrain_z": terrain_z,
            })
    worst["points"] = len(contour)
    return worst


def scan_profile(terrain, vehicle, samples=41, steps=480, margin=0.0):
    """让车沿剖面走一遍，返回最坏位姿与各区段的最小余量。

    `margin` 用来把扫描区间向两端各延一点，保证车头/车尾完全进入和离开剖面。
    """
    start = terrain.min_x - vehicle.rear_overhang - margin
    end = terrain.max_x + vehicle.front_overhang + margin
    if end <= start:
        raise ClearanceError("扫描区间为空：[%.3f, %.3f]" % (start, end))
    steps = max(2, int(steps))
    step = (end - start) / float(steps - 1)

    worst = None
    section_min = {SECTION_NOSE: float("inf"), SECTION_MID: float("inf"),
                   SECTION_TAIL: float("inf")}
    pitch_max = -float("inf")
    pitch_min = float("inf")

    for index in range(steps):
        rear_x = start + step * index
        pose = seat(terrain, vehicle, rear_x)
        record = penetration_at(terrain, vehicle, pose, samples)
        record["pitch_deg"] = pose["pitch_deg"]
        pitch_max = max(pitch_max, pose["pitch_deg"])
        pitch_min = min(pitch_min, pose["pitch_deg"])
        if worst is None or record["depth"] > worst["depth"]:
            worst = record
            worst["pose"] = pose
        section_min[record["section"]] = min(section_min[record["section"]], record["margin"])

    return {
        "worst": worst,
        "start_x": start,
        "end_x": end,
        "step": step,
        "steps": steps,
        "samples": samples,
        "max_depth": max(0.0, worst["depth"]),
        "min_margin": -worst["depth"],
        "section_min_margin": section_min,
        "pitch_min_deg": pitch_min,
        "pitch_max_deg": pitch_max,
    }
