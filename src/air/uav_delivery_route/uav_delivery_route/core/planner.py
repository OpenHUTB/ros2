#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Route planner: sector clustering -> NN chain -> battery-aware split -> 2-opt.

单行英文摘要 + 中文正文。
流程分四步：1) 把站点按相对配送中心的方位角扇区预分组；2) 簇内从配送中心
出发做最近邻串接；3) 沿序列做电量检查——剩余电量低于"返航所需 x 1.25"即
封口当前架次并回中心换电，载重超限同理；4) 对簇内序列做 2-opt 改进后再
切架次。输出架次划分（Sortie 列表）与任务总时长（含 90 s 换电、20 s 悬停）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

from .cases import Case, Station
from .energy import EnergyModel, RETURN_MARGIN


@dataclass(frozen=True)
class RouteConfig:
    """All planner constants in one place."""
    cruise_speed_ms: float = 9.0
    cruise_alt_m: float = 8.0
    hover_s: float = 20.0
    swap_s: float = 90.0
    payload_cap_kg: float = 2.5
    return_margin: float = RETURN_MARGIN
    n_sectors: int = 3
    two_opt_max_iter: int = 200


@dataclass
class Sortie:
    """One battery episode: depot -> stations... -> depot."""
    index: int
    stations: List[Station]
    legs_m: List[float]                      # depot->s0, s0->s1, ..., sn->depot
    distance_m: float
    flight_s: float
    hover_s: float
    payload_max_kg: float
    energy_frac: float                       # whole sortie incl. return leg
    min_home_margin: float                   # min(remaining / need-home) over route

    def to_dict(self) -> dict:
        return dict(index=self.index,
                    stations=[s.sid for s in self.stations],
                    weights_kg=[s.weight_kg for s in self.stations],
                    legs_m=[round(v, 2) for v in self.legs_m],
                    distance_m=round(self.distance_m, 2),
                    flight_s=round(self.flight_s, 2),
                    hover_s=round(self.hover_s, 2),
                    payload_max_kg=round(self.payload_max_kg, 3),
                    energy_frac=round(self.energy_frac, 4),
                    min_home_margin=round(self.min_home_margin, 3))


@dataclass
class FleetPlan:
    """Full plan for one case: sorties + aggregate numbers."""
    case_name: str
    depot: Tuple[float, float]
    sorties: List[Sortie]
    total_duration_s: float
    total_distance_m: float
    swap_count: int
    unservable: List[str] = field(default_factory=list)

    @property
    def sortie_count(self) -> int:
        return len(self.sorties)

    def to_dict(self) -> dict:
        return dict(case=self.case_name,
                    sortie_count=len(self.sorties),
                    swap_count=self.swap_count,
                    total_duration_s=round(self.total_duration_s, 2),
                    total_distance_m=round(self.total_distance_m, 2),
                    unservable=list(self.unservable),
                    sorties=[s.to_dict() for s in self.sorties])


def dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def sector_clusters(stations: Sequence[Station], depot: Sequence[float],
                    n_sectors: int) -> List[List[Station]]:
    """Group stations into angular sectors around the depot.

    按方位角等分扇区；n_sectors=1 时全部落进同一组。角度取 [-pi, pi)，
    扇区 k 覆盖 [-pi + 2*pi*k/n, -pi + 2*pi*(k+1)/n)。
    """
    if n_sectors < 1:
        raise ValueError("n_sectors must be >= 1")
    buckets: List[List[Station]] = [[] for _ in range(n_sectors)]
    for s in stations:
        ang = math.atan2(s.y - depot[1], s.x - depot[0])
        k = int((ang + math.pi) / (2.0 * math.pi) * n_sectors)
        buckets[min(k, n_sectors - 1)].append(s)
    return [b for b in buckets if b]


def nearest_neighbor(depot: Sequence[float],
                     stations: Sequence[Station]) -> List[Station]:
    """Greedy NN chain starting from the depot."""
    left = list(stations)
    order: List[Station] = []
    cur = depot
    while left:
        nxt = min(left, key=lambda s: dist(cur, (s.x, s.y)))
        order.append(nxt)
        left.remove(nxt)
        cur = (nxt.x, nxt.y)
    return order


def tour_length(depot: Sequence[float], order: Sequence[Station]) -> float:
    """Closed tour: depot -> stations... -> depot."""
    if not order:
        return 0.0
    d = dist(depot, (order[0].x, order[0].y))
    for a, b in zip(order, order[1:]):
        d += dist((a.x, a.y), (b.x, b.y))
    return d + dist((order[-1].x, order[-1].y), depot)


def two_opt(depot: Sequence[float], order: List[Station],
            max_iter: int = 200) -> List[Station]:
    """2-opt on the closed tour, never returns a worse sequence."""
    best = list(order)
    best_len = tour_length(depot, best)
    for _ in range(max_iter):
        improved = False
        n = len(best)
        for i in range(n - 1):
            for j in range(i + 1, n):
                cand = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                d = tour_length(depot, cand)
                if d < best_len - 1e-9:
                    best, best_len, improved = cand, d, True
        if not improved:
            break
    return best


def _sortie_cost(sortie_stations: Sequence[Station], depot: Sequence[float],
                 model: EnergyModel, cfg: RouteConfig) -> Sortie:
    """Build a Sortie object with energy bookkeeping for a fixed station list."""
    pts = [depot] + [(s.x, s.y) for s in sortie_stations] + [depot]
    legs = [dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    flight_s = sum(legs) / cfg.cruise_speed_ms
    hover_total = cfg.hover_s * len(sortie_stations)
    payload_max = sum(s.weight_kg for s in sortie_stations)

    energy = 0.0
    for i in range(len(sortie_stations)):
        # 起飞时载重 = 本架次尚未投递的包裹总重，逐站投递后递减
        carried = sum(x.weight_kg for x in sortie_stations[i:])
        energy += model.leg_frac(legs[i], carried, cfg.cruise_speed_ms)
        energy += model.hover_frac(carried, cfg.hover_s)
    # 尾程空载返航
    energy += model.leg_frac(legs[-1], 0.0, cfg.cruise_speed_ms)
    return Sortie(index=0, stations=list(sortie_stations), legs_m=legs,
                  distance_m=sum(legs), flight_s=flight_s, hover_s=hover_total,
                  payload_max_kg=payload_max, energy_frac=energy,
                  min_home_margin=math.inf)


def sortie_home_margins(stations: Sequence[Station], depot: Sequence[float],
                        model: EnergyModel, cfg: RouteConfig) -> List[float]:
    """逐站返航裕量：从配送中心满载出发、按序投递的完整载重/电量轨迹。

    机上载重按"本架次全部包裹在中心一次装齐"计：飞往第 j 站时机上还有
    第 j..n 站的包裹；投递完第 j 站后机上剩第 j+1..n 站的包裹，返程按
    这个重量算。此前用"已投递包裹的重量"当剩余载重，顺序一变账目就错
    （两个 1.2 kg 远站被并进一个架次，报告裕量 1.28、真实只有 0.96）。
    """
    margins: List[float] = []
    battery = 1.0
    pos = depot
    for j, s in enumerate(stations):
        aboard = sum(x.weight_kg for x in stations[j:])       # 到 s 时机上载重
        cost = model.leg_frac(dist(pos, (s.x, s.y)), aboard,
                              cfg.cruise_speed_ms)
        cost += model.hover_frac(aboard, cfg.hover_s)
        battery -= cost
        after = sum(x.weight_kg for x in stations[j + 1:])    # 投递完 s 后剩余
        need_home = model.leg_frac(dist((s.x, s.y), depot), after,
                                   cfg.cruise_speed_ms)
        margins.append(battery / need_home if need_home > 1e-12 else math.inf)
        pos = (s.x, s.y)
    return margins


def split_sorties(depot: Sequence[float], order: List[Station],
                  model: EnergyModel, cfg: RouteConfig) -> List[Sortie]:
    """Walk the ordered stations, closing a sortie whenever battery or payload says so.

    电量判据：把下一站试排进来后，从配送中心重算整条载重/电量轨迹，
    任一站的返航裕量 < 1.25 即封口本架次（90 s 换电后重开新架次）。
    载重判据：本架次累计载重 + 下一站包裹 > 2.5 kg 时同样封口。
    特例：空架次时无条件接纳下一站，避免远超续航的站点造成死循环
    （此时 checker 会如实体检出台账违规）。
    """
    sorties: List[Sortie] = []
    cur: List[Station] = []

    def close():
        nonlocal cur
        if cur:
            s = _sortie_cost(cur, depot, model, cfg)
            s.index = len(sorties)
            s.min_home_margin = min(sortie_home_margins(cur, depot, model, cfg))
            sorties.append(s)
            cur = []

    i = 0
    while i < len(order):
        s = order[i]
        carried = sum(x.weight_kg for x in cur)
        if carried + s.weight_kg > cfg.payload_cap_kg + 1e-9:
            if cur:
                close()
                continue
            # 单件就超上限：无法装载，跳过（checker 会记录 unservable）
            i += 1
            continue
        candidate = cur + [s]
        if cur and min(sortie_home_margins(candidate, depot, model, cfg)) < cfg.return_margin:
            close()   # s 进不来本架次，重开新架次再试（循环里 continue 重试同一个 s）
            continue
        cur.append(s)
        i += 1
    close()
    return sorties


def plan_routes(case: Case, cfg: RouteConfig | None = None,
                model: EnergyModel | None = None) -> FleetPlan:
    """Full pipeline for one case; returns the batched fleet plan."""
    cfg = cfg or RouteConfig()
    model = model or EnergyModel()
    depot = case.depot

    unservable = [s.sid for s in case.stations if s.weight_kg > cfg.payload_cap_kg + 1e-9]

    clusters = sector_clusters(case.stations, depot, cfg.n_sectors)
    order_all: List[Station] = []
    for cluster in clusters:
        # 每簇内最近邻 + 2-opt 改进，再按扇区顺序串成一条总序列
        order = nearest_neighbor(depot, cluster)
        order = two_opt(depot, order, cfg.two_opt_max_iter)
        order_all.extend(order)
    sorties = split_sorties(depot, order_all, model, cfg)

    for k, s in enumerate(sorties):
        s.index = k
    total = sum(s.flight_s + s.hover_s for s in sorties)
    total += cfg.swap_s * max(0, len(sorties) - 1)
    plan = FleetPlan(case_name=case.name, depot=tuple(depot), sorties=sorties,
                     total_duration_s=total,
                     total_distance_m=sum(s.distance_m for s in sorties),
                     swap_count=max(0, len(sorties) - 1),
                     unservable=unservable)
    print("[planner] %s: %d stations -> %d sorties, %.0f m, %.0f s"
          % (case.name, len(case.stations), len(sorties),
             plan.total_distance_m, plan.total_duration_s))
    return plan


DEPOT_RADIUS_M = 2.0   # 判定"在配送中心"的水平半径，与节点到站半径一致


def decide_next(plan: FleetPlan, visited: set, pos_xy: Sequence[float],
                battery_frac: float, model: EnergyModel | None = None,
                cfg: RouteConfig | None = None,
                at_depot: bool | None = None,
                departed: bool = False) -> dict:
    """Online dispatch decision; pure function used by the ROS node.

    节点薄壳：给定已服务站集合、当前位置与电量，返回下一动作。动作取值：
    deliver（去送下一站）/ return_for_swap（架次中途电量吃紧，回中心）/
    return_to_depot（本架次送完或本架次还没装货离仓，回中心装货换电）/
    swap（在中心换电，等电量恢复）/ done（全部架次完成且已回中心）。

    departed 由节点维护：本架次货物已装并已离仓（架次切换时复位）。
    此前用"本架次是否已送过一站"推断是否已出发，去第一站途中会被
    误判成"新架次还没出发"而要求返仓，首次投递永远完不成。

    电量判据与离线规划一致：在中心看剩余电量够不够把"剩余架次账目"
    （新架次 = 全账目；中途返仓 = 剩余站账目）从中心飞完并回中心，
    不够就先换电——中途返仓到站后也能进入 swap，而不是原地反复
    return_for_swap；在途看剩余 < 1.25 x 返航所需即建议回中心。
    """
    model = model or EnergyModel()
    cfg = cfg or RouteConfig()
    depot = plan.depot
    if at_depot is None:
        at_depot = dist(pos_xy, depot) < DEPOT_RADIUS_M

    def _act(action, k, target, done=False):
        if target is not None:
            d_tgt = round(dist(pos_xy, (target["x"], target["y"])), 2)
        else:
            d_tgt = 0.0
        return dict(action=action, sortie=k, target=target,
                    distance_to_target_m=d_tgt,
                    battery_frac=round(battery_frac, 4), done=done)

    depot_t = dict(sid="depot", x=depot[0], y=depot[1])

    cur_k = None
    for k in range(len(plan.sorties)):
        if any(s.sid not in visited for s in plan.sorties[k].stations):
            cur_k = k
            break
    if cur_k is None:
        # 全部架次完成：必须先回中心才算收工（曾直接 done，无人机停在现场）
        if at_depot:
            return _act("done", -1, None, done=True)
        return _act("return_to_depot", len(plan.sorties) - 1, depot_t)

    sortie = plan.sorties[cur_k]
    remaining = [s for s in sortie.stations if s.sid not in visited]

    def _remaining_need() -> float:
        """从中心出发送完 remaining 并回中心所需电量（与离线同模型同公式）。"""
        frac = 0.0
        cur = depot
        carried = sum(s.weight_kg for s in remaining)
        for s in sortie.stations:
            if s.sid in visited:
                continue
            frac += model.leg_frac(dist(cur, (s.x, s.y)), carried,
                                   cfg.cruise_speed_ms)
            frac += model.hover_frac(carried, cfg.hover_s)
            carried -= s.weight_kg
            cur = (s.x, s.y)
        frac += model.leg_frac(dist(cur, depot), carried, cfg.cruise_speed_ms)
        return frac

    if at_depot:
        # 在中心：电量不够飞完剩余账目就先换电。中途低电返仓到站后
        # 同样落进这一分支进入 swap，不再要求"本架次还没开始"
        if battery_frac < _remaining_need():
            return _act("swap", cur_k, None)
    elif not departed:
        # 不在中心且本架次还没装货离仓：上一架次送完停在现场（或被
        # 拉走），必须先回中心装货——不能从现场直接飞去送
        return _act("return_to_depot", cur_k, depot_t)

    nxt = remaining[0]
    carried = sum(s.weight_kg for s in remaining)
    leg_in = dist(pos_xy, (nxt.x, nxt.y))
    cost = model.leg_frac(leg_in, carried, cfg.cruise_speed_ms)
    cost += model.hover_frac(carried, cfg.hover_s)
    need_home = model.leg_frac(dist((nxt.x, nxt.y), depot),
                               carried - nxt.weight_kg, cfg.cruise_speed_ms)
    if battery_frac - cost < cfg.return_margin * need_home:
        return _act("return_for_swap", cur_k, depot_t)
    return _act("deliver", cur_k,
                dict(sid=nxt.sid, x=nxt.x, y=nxt.y))
