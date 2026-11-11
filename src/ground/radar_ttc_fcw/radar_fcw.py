# -*- coding: utf-8 -*-
"""毫米波雷达目标跟踪与前向碰撞预警（TTC/FCW）核心算法。

只依赖 Python 标准库，可脱离仿真环境离线回放，便于复现与评审。

处理链路：
    原始点云 -> 速度域分离 -> 网格聚类(8 邻域合并) -> 多帧最近邻关联
    -> TTC 估计 -> 分级预警(带迟滞)

对应报告：docs/ground/radar_ttc_fcw.md

实测要点（OpenHUTB v2.10.0 / CARLA Town10HD / sensor.other.radar）：
  1. raw_data 每点 4 个 float32: (v_radial, azimuth, altitude, depth)，
     角度单位是弧度，depth 单位是 m。
  2. v_radial 是「相对径向速度」，负=接近、正=远离，单位 m/s；
     目标车以 6 m/s 接近时实测 -5.97 m/s。
  3. ego 静止时场景内静止杂波的 v_radial 恒为 0，因此速度域分离可以
     直接把静止地面杂波剔除，比按深度取最近点可靠得多。
  4. 用 enable_constant_velocity 驱动 ego 时，雷达读数会出现 -108 m/s 量级
     的异常值（实际位移仅 8 m/s），故 ego 运动场景不能直接信任瞬时多普勒，
     应以距离跟踪的斜率为准。本模块同时给出两种闭合速率估计做交叉校验。
  5. 雷达打到目标车尾部表面，而 ground truth 取的是车辆中心，存在约
     1.4 m 的系统偏差，用 DEPTH_BIAS 标定。
"""
import collections
import math

# ---------------------------------------------------------------- 参数区
DEFAULTS = {
    "v_dynamic": 1.0,      # |v_radial| >= 1.0 m/s 视为动态点，静止杂波被剔除
    "az_limit": 12.0,      # 前向锥半角（度）
    "cell": 2.0,           # 聚类网格边长（m）
    "min_pts": 3,          # 一个簇至少要有几个点
    "gate": 4.0,           # 多帧关联的距离预测门限（m）
    "confirm": 2,          # 连续命中多少帧才确认一条航迹
    "miss": 5,             # 连续丢失多少帧才删除航迹
    "max_tracks": 8,       # 同时保留的最大航迹数，抑制启动抖动产生的假航迹
    "ttc_warn": 4.0,       # WARN 的 TTC 阈值（s）
    "ttc_stop": 2.0,       # STOP 的 TTC 阈值（s，FCW 常用 1.5~2.5 s 预警线）
    "d_warn": 25.0,        # WARN 的距离阈值（m）
    "d_stop": 8.0,         # STOP 的距离阈值（m）
    "hysteresis": 3,       # 降级迟滞帧数，避免预警等级抖动
    "depth_bias": 1.4,     # 车尾表面 -> 车辆中心的标定偏差（m）
    "slope_window": 8,     # 用多少帧历史做距离斜率最小二乘估计
}


def median(values):
    """不依赖 numpy 的中位数。"""
    s = sorted(values)
    n = len(s)
    if not n:
        return 0.0
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


# ---------------------------------------------------------------- 点云解析
def to_cartesian(points):
    """把 [v_radial, azimuth_deg, altitude_deg, depth] 转成机体坐标。

    返回 (x, y, v_radial, depth, az_deg) 列表。
    数据集里角度已转成度，在线模式下由 main.py 传入的也是度。
    """
    out = []
    for vr, az_deg, alt_deg, dp in points:
        az, alt = math.radians(az_deg), math.radians(alt_deg)
        x = dp * math.cos(alt) * math.cos(az)
        y = dp * math.cos(alt) * math.sin(az)
        out.append((x, y, vr, dp, az_deg))
    return out


def extract_targets(points, cfg):
    """速度域分离 + 网格聚类 + 8 邻域合并，返回本帧的动态目标簇。

    每个簇用中位数表示：depth / v_radial / az_deg，并带上点数。
    """
    grid = collections.defaultdict(list)
    for x, y, vr, dp, az in to_cartesian(points):
        if abs(vr) < cfg["v_dynamic"] or abs(az) > cfg["az_limit"]:
            continue
        grid[(round(x / cfg["cell"]), round(y / cfg["cell"]))].append(
            (x, y, vr, dp, az))

    # 8 邻域连通域合并：近距离车体会横跨多个网格，不合并会被拆成多个假目标
    seen = set()
    clusters = []
    for key in list(grid):
        if key in seen:
            continue
        queue, group = [key], []
        seen.add(key)
        while queue:
            k = queue.pop()
            group.extend(grid[k])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nb = (k[0] + dx, k[1] + dy)
                    if nb in grid and nb not in seen:
                        seen.add(nb)
                        queue.append(nb)
        if len(group) >= cfg["min_pts"]:
            clusters.append({
                "depth": median([g[3] for g in group]),
                "v_radial": median([g[2] for g in group]),
                "az_deg": median([g[4] for g in group]),
                "n_pts": len(group),
            })
    return sorted(clusters, key=lambda c: c["depth"])


# ---------------------------------------------------------------- 多帧关联
class Track:
    """一条目标航迹：恒速预测 + 最近邻关联 + 距离历史。"""

    def __init__(self, tid, target):
        self.id = tid
        self.depth = target["depth"]
        self.v_radial = target["v_radial"]
        self.az_deg = target["az_deg"]
        self.n_pts = target["n_pts"]
        self.hits = 1
        self.misses = 0
        self.confirmed = False
        self.history = []           # (t, depth)，用于距离斜率估计
        self._stable = "SAFE"
        self._pending = "SAFE"
        self._pending_count = 0

    def predict(self, dt):
        return self.depth + self.v_radial * dt

    def update(self, target, t, confirm):
        self.depth = target["depth"]
        self.v_radial = target["v_radial"]
        self.az_deg = target["az_deg"]
        self.n_pts = target["n_pts"]
        self.hits += 1
        self.misses = 0
        self.history.append((t, self.depth))
        if self.hits >= confirm:
            self.confirmed = True


class Tracker:
    def __init__(self, cfg=None):
        self.cfg = dict(DEFAULTS)
        if cfg:
            self.cfg.update(cfg)
        self.tracks = []
        self._next_id = 1
        self.last_t = None
        self.clutter_total = 0
        self.clutter_rejected = 0

    def update(self, points, t):
        """处理一帧点云，返回确认航迹的预警记录列表。"""
        cfg = self.cfg
        dt = 0.1 if self.last_t is None else max(t - self.last_t, 1e-3)
        self.last_t = t
        self.clutter_total += len(points)
        targets = extract_targets(points, cfg)
        kept = sum(x["n_pts"] for x in targets)
        self.clutter_rejected += len(points) - kept

        for tr in self.tracks:
            tr.misses += 1

        unmatched = list(targets)
        # 按预测距离从近到远依次关联，避免远处航迹抢走本属于近处航迹的簇
        for tr in sorted(self.tracks, key=lambda x: x.predict(dt)):
            pred = tr.predict(dt)
            if not unmatched:
                break
            near = min(unmatched, key=lambda c: abs(c["depth"] - pred))
            if abs(near["depth"] - pred) <= cfg["gate"]:
                tr.update(near, t, cfg["confirm"])
                unmatched.remove(near)

        # 与已有航迹预测位置过近的簇视为同一目标的反射分裂，不另起航迹
        for c in unmatched:
            if any(abs(c["depth"] - tr.predict(dt)) <= cfg["cell"] * 1.5
                   for tr in self.tracks):
                continue
            self.tracks.append(Track(self._next_id, c))
            self._next_id += 1

        self.tracks = [tr for tr in self.tracks if tr.misses <= cfg["miss"]]
        if len(self.tracks) > cfg["max_tracks"]:
            self.tracks.sort(key=lambda x: (x.confirmed, x.hits, -x.misses),
                             reverse=True)
            self.tracks = self.tracks[:cfg["max_tracks"]]
        return [decide(tr, t, cfg) for tr in self.tracks if tr.confirmed]


# ---------------------------------------------------------------- TTC 与预警
def closing_from_slope(history, window):
    """用最近 window 帧的 (t, depth) 最小二乘斜率估计闭合速率。

    斜率 <0 表示距离在缩小；返回正值表示正在接近。
    """
    h = history[-window:]
    if len(h) < 3:
        return None
    n = len(h)
    mt = sum(p[0] for p in h) / n
    md = sum(p[1] for p in h) / n
    den = sum((p[0] - mt) ** 2 for p in h)
    if den < 1e-9:
        return None
    slope = sum((p[0] - mt) * (p[1] - md) for p in h) / den
    return -slope


def raw_level(depth, ttc, cfg):
    if depth <= cfg["d_stop"] or (ttc is not None and ttc < cfg["ttc_stop"]):
        return "STOP"
    if depth <= cfg["d_warn"] or (ttc is not None and ttc < cfg["ttc_warn"]):
        return "WARN"
    return "SAFE"


_ORDER = {"SAFE": 0, "WARN": 1, "STOP": 2}


def _apply_hysteresis(tr, level, cfg):
    """升级立即、降级延迟，避免阈值附近反复跳变。"""
    if _ORDER[level] >= _ORDER[tr._stable]:
        tr._stable = level
        tr._pending, tr._pending_count = level, 0
    else:
        if tr._pending == level:
            tr._pending_count += 1
        else:
            tr._pending, tr._pending_count = level, 1
        if tr._pending_count >= cfg["hysteresis"]:
            tr._stable = level
    return tr._stable


def decide(tr, t, cfg):
    """对一条航迹计算闭合速率、TTC 和预警等级。"""
    # 标定：雷达测到车尾表面，加上偏差还原到车辆中心
    depth = tr.depth + cfg["depth_bias"]
    v_doppler = -tr.v_radial if tr.v_radial < 0 else 0.0
    v_slope = closing_from_slope(tr.history, cfg["slope_window"])
    closing = v_doppler if v_doppler > 0.05 else (v_slope or 0.0)

    if tr.v_radial > cfg["v_dynamic"]:
        level, ttc = "RECEDING", None        # 远离目标不参与碰撞预警
    elif closing > 0.05:
        ttc = depth / closing
        level = _apply_hysteresis(tr, raw_level(depth, ttc, cfg), cfg)
    else:
        level, ttc = "SAFE", None

    return {
        "t": round(t, 2),
        "track": tr.id,
        "depth": round(depth, 2),
        "raw_depth": round(tr.depth, 2),
        "v_radial": round(tr.v_radial, 2),
        "az_deg": round(tr.az_deg, 1),
        "n_pts": tr.n_pts,
        "closing_doppler": round(v_doppler, 2),
        "closing_slope": round(v_slope, 2) if v_slope is not None else None,
        "ttc": round(ttc, 2) if ttc is not None else None,
        "level": level,
    }


def run_pipeline(frames, cfg=None):
    """离线回放整段数据，返回 (记录列表, 统计字典)。"""
    tracker = Tracker(cfg)
    records = []
    for f in frames:
        for r in tracker.update(f["pts"], f["t"]):
            r["gt_a"] = f.get("gt_a")
            r["gt_b"] = f.get("gt_b")
            records.append(r)

    # 统计：以「接近类记录中出现帧数最多」的航迹作为主目标，
    # 规避近距离车体反射分裂造成的短暂重复航迹
    approaching = [r for r in records if r["level"] in ("SAFE", "WARN", "STOP")]
    cnt = collections.Counter(r["track"] for r in approaching)
    main_track = cnt.most_common(1)[0][0] if cnt else None
    approaching = [r for r in approaching if r["track"] == main_track]
    stats = {
        "main_track": main_track,
        "frames": len(frames),
        "clutter_total": tracker.clutter_total,
        "clutter_rejected": tracker.clutter_rejected,
        "clutter_ratio": round(tracker.clutter_rejected / max(tracker.clutter_total, 1), 4),
        "tracks": tracker._next_id - 1,
        "records": len(records),
    }
    if approaching:
        stats["first_warn"] = next((r["t"] for r in approaching if r["level"] == "WARN"), None)
        stats["first_stop"] = next((r["t"] for r in approaching if r["level"] == "STOP"), None)
        stats["ttc_first"] = next((r["ttc"] for r in approaching if r["ttc"]), None)
        stats["ttc_last"] = next((r["ttc"] for r in reversed(approaching) if r["ttc"]), None)
        errs = [abs(r["depth"] - r["gt_a"]) for r in approaching
                if r["gt_a"] is not None and abs(r["az_deg"]) < 6]
        if errs:
            stats["depth_mae"] = round(sum(errs) / len(errs), 2)
            stats["depth_max_err"] = round(max(errs), 2)
    receding = [r for r in records if r["level"] == "RECEDING"]
    stats["receding_records"] = len(receding)
    stats["false_alarms"] = sum(1 for r in receding if r["level"] in ("WARN", "STOP"))
    return records, stats
