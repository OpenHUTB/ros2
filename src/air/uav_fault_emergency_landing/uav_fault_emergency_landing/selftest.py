"""自检：不依赖 pytest，把"建图 → 评分 → 制导 → 迫降"各环节的关键性质验一遍。

缺 torch / mujoco 时对应分组打印 SKIP 而不是失败，这样在只有 numpy 的机器上也能跑。
"""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

from . import data as D
from . import landing as L
from . import mapping as M
from . import model as Md


class _Report:
    def __init__(self) -> None:
        self.items: List[Tuple[str, bool, str]] = []
        self.skipped: List[str] = []

    def check(self, name: str, ok: bool, detail: str = "") -> None:
        self.items.append((name, bool(ok), detail))

    def skip(self, name: str, why: str) -> None:
        self.skipped.append(f"{name}（{why}）")

    @property
    def failed(self) -> List[Tuple[str, bool, str]]:
        return [it for it in self.items if not it[1]]


def _close(a, b, tol=1e-6) -> bool:
    return abs(float(a) - float(b)) <= tol


# --------------------------------------------------------------------------- #
def _check_mapping(r: _Report) -> None:
    cfg = M.GridConfig(size_m=10.0, cell_m=1.0)
    r.check("网格形状", cfg.shape == (10, 10), f"shape={cfg.shape}")
    # 前两点落在同一格（验"取最高点"），第三点在另一格，第四点越界
    pts = np.array([[0.0, 0.0, 0.5], [0.1, 0.1, 1.5], [2.0, 2.0, 0.8],
                    [100.0, 0.0, 1.0]])
    ii, jj, valid = M.world_to_cells(cfg, pts)
    r.check("越界点被过滤", int(valid.sum()) == 3, f"valid={int(valid.sum())}")

    hm = M.HeightMap(cfg)
    hm.update(pts)
    r.check("取每格最高点", _close(hm.height[5, 5], 1.5, 1e-5), f"h={hm.height[5,5]:.3f}")
    r.check("观测格计数", int(hm.observed.sum()) == 2, f"observed={int(hm.observed.sum())}")

    # 障碍距离场：一格障碍，邻居距离应为 1、2、4 个 cell
    ob = np.zeros((10, 10), dtype=bool)
    ob[5, 5] = True
    dist = M._distance_to_obstacle(ob, 1.0, radius_cells=8)
    r.check("障碍距离场", _close(dist[5, 5], 0.0) and _close(dist[5, 6], 1.0)
            and _close(dist[5, 7], 2.0) and _close(dist[5, 9], 4.0),
            f"{dist[5,5]:.1f}/{dist[5,6]:.1f}/{dist[5,7]:.1f}/{dist[5,9]:.1f}")

    grid = M.height_map_input(hm)
    r.check("网络输入形状", grid.shape == (1, 10, 10), f"shape={grid.shape}")
    r.check("未观测格填 1", float(grid[0][~hm.observed].min()) >= 1.0 - 1e-6,
            f"min={float(grid[0][~hm.observed].min()):.3f}")

    score = M.rule_landing_score(hm)
    r.check("评分在 [0,1]", float(score.min()) >= 0.0 and float(score.max()) <= 1.0,
            f"[{score.min():.2f},{score.max():.2f}]")


def _check_depth(r: _Report) -> None:
    # 深度图全是 3 m：反投影后应是一块 z = cam_z - 3 的平面
    depth = np.full((32, 32), 3.0)
    cam_pos = np.array([0.0, 0.0, 5.0])
    pts = M.depth_to_points(depth, cam_pos, np.eye(3), 60.0)
    r.check("反投影点数", len(pts) == 32 * 32, f"n={len(pts)}")
    r.check("反投影高度", _close(pts[:, 2].mean(), 2.0, 1e-3), f"z={pts[:,2].mean():.3f}")
    r.check("反投影对称", _close(pts[:, 0].mean(), 0.0, 1e-6), f"x={pts[:,0].mean():.2e}")


def _check_landing(r: _Report) -> None:
    cfg = L.LandingConfig()
    gains = np.ones(4)
    v = L.expert_velocity(np.array([2.0, 0.0, -3.0]), np.zeros(3), gains, 5.0)
    r.check("专家律朝目标飞", float(v[0]) > 0.0, f"vx={v[0]:.2f}")
    r.check("专家律下降", float(v[2]) < 0.0, f"vz={v[2]:.2f}")

    v_high = L.expert_velocity(np.array([0.0, 0.0, -3.0]), np.zeros(3), gains, 4.0)
    v_low = L.expert_velocity(np.array([0.0, 0.0, -3.0]), np.zeros(3), gains, 0.2)
    r.check("触地前减速", abs(float(v_low[2])) < abs(float(v_high[2])),
            f"{v_high[2]:.2f} → {v_low[2]:.2f}")
    r.check("触地包线宽松", L.touchdown_ok(0.5, 0.4, 10.0, True), "vz=0.5 vxy=0.4")
    r.check("触地包线拒绝急降", not L.touchdown_ok(3.0, 0.4, 10.0, True), "vz=3.0")

    feats = L.guidance_features(np.array([1.0, 2.0, -3.0]), np.array([0.1, 0.0, -0.2]),
                                gains, 5.0)
    r.check("制导特征维度", feats.shape == (L.GUIDANCE_DIM,), f"shape={feats.shape}")
    r.check("制导特征有限", bool(np.all(np.isfinite(feats))), "")

    # 落点选择要避开障碍、偏好近处
    hm = M.HeightMap(M.GridConfig(size_m=10.0, cell_m=1.0))
    score = np.zeros(hm.shape)
    score[2, 2] = 0.9   # 远
    score[5, 5] = 0.9   # 近
    target = L.select_target(score, hm, position_xy=np.array([0.0, 0.0]),
                             min_score=0.1, max_distance=10.0)
    r.check("落点偏好近处", target is not None and target[0] == 5 and target[1] == 5,
            f"target={target}")


def _check_model(r: _Report) -> None:
    try:
        score_net = Md.build_score_module()
        guidance_net = Md.build_guidance_module()
    except ImportError as exc:
        r.skip("网络结构", str(exc))
        return
    r.check("评分网络参数量", Md.n_parameters(score_net) == 2625,
            f"{Md.n_parameters(score_net)}")
    r.check("制导网络参数量", Md.n_parameters(guidance_net) == 5251,
            f"{Md.n_parameters(guidance_net)}")

    out = Md.predict_score(score_net, np.zeros((48, 48), dtype=np.float32))
    r.check("评分输出范围", out.shape == (48, 48) and float(out.min()) >= 0.0
            and float(out.max()) <= 1.0, f"shape={out.shape}")
    vel = Md.predict_guidance(guidance_net, np.zeros(L.GUIDANCE_DIM, dtype=np.float32))
    r.check("制导输出维度", np.asarray(vel).reshape(-1).shape == (3,), f"{np.shape(vel)}")


def _check_data(r: _Report) -> None:
    g = D.build_guidance_dataset(64, seed=0)
    r.check("制导数据集形状", g["X"].shape == (64, L.GUIDANCE_DIM) and g["Y"].shape == (64, 3),
            f"{g['X'].shape}/{g['Y'].shape}")
    r.check("制导标签有限", bool(np.all(np.isfinite(g["Y"]))), "")
    splits = D.split_samples(64)
    r.check("数据集切分", len(splits["train"]) + len(splits["val"]) + len(splits["test"]) == 64,
            f"{len(splits['train'])}/{len(splits['val'])}/{len(splits['test'])}")


def _check_controller(r: _Report) -> None:
    try:
        from . import _compat
        from . import evaluate as E

        api = _compat.load_base()
    except Exception as exc:
        r.skip("闭环控制", f"缺少 mujoco 或底座：{exc}")
        return
    params, eff = E.build_context(api)
    from . import controller as C

    quad = api.Quadrotor(scene=api.make_scene("open_ground", seed=0), dt=0.01, substeps=4)
    base = api.GeometricController(quad.params)
    # neural 模式需要两个网络；这里用未训练的随机权重，只验形状与有界性
    try:
        score_net = Md.build_score_module()
        guidance_net = Md.build_guidance_module()
    except ImportError:
        score_net = guidance_net = None
    for mode in C.MODES:
        if mode == "neural" and score_net is None:
            r.skip("neural 模式指令形状", "缺少 torch")
            continue
        ctrl = C.ForcedLandingController(params, api.mixer, base, eff, mode=mode,
                                         score_net=score_net, guidance_net=guidance_net)
        ctrl.set_health(np.ones(4))
        commands = ctrl.act(quad.state())
        ok = commands.shape == (4,) and float(commands.min()) >= -1e-6
        r.check(f"{mode} 模式指令形状", ok, f"u={np.round(commands, 2)}")


def _check_integration(r: _Report) -> None:
    try:
        from . import _compat
        from . import evaluate as E

        api = _compat.load_base()
    except Exception as exc:
        r.skip("闭环迫降", f"缺少 mujoco 或底座：{exc}")
        return
    params, eff = E.build_context(api)
    start_xy, hover_z = E.pick_obstacle_spot(api, "urban_block", 0)
    vertical = E.run_landing(api, "vertical", dict(kind="degradation", rotor=2, severity=0.5),
                             seed=0, seconds=20.0, start_xy=start_xy, hover_z=hover_z,
                             params=params, effectiveness=eff)
    expert = E.run_landing(api, "expert", dict(kind="degradation", rotor=2, severity=0.5),
                           seed=0, seconds=30.0, start_xy=start_xy, hover_z=hover_z,
                           params=params, effectiveness=eff)
    # 判据刻意写成"相对"而不是"必须穿楼"：不同 MuJoCo 版本下射线/几何的取整略有差别，
    # pick_obstacle_spot() 可能选到同一栋楼旁边的格子，直接用 landed_on_obstacle 会随版本抖。
    # 真正要守住的性质是——垂直下降会贴近甚至穿过障碍，选点迫降不会。
    r.check("垂直迫降贴障碍或穿楼", float(vertical.min_clearance) < 1.0,
            f"clearance={vertical.min_clearance:.2f}")
    r.check("选点迫降比垂直迫降更安全",
            float(expert.min_clearance) > float(vertical.min_clearance),
            f"expert={expert.min_clearance:.2f} vs vertical={vertical.min_clearance:.2f}")
    r.check("选点迫降安全", bool(expert.landed and not expert.landed_on_obstacle),
            f"status={expert.status} t={expert.touchdown_time}")


# --------------------------------------------------------------------------- #
def run_selftest(verbose: bool = True) -> int:
    r = _Report()
    _check_mapping(r)
    _check_depth(r)
    _check_landing(r)
    _check_model(r)
    _check_data(r)
    _check_controller(r)
    _check_integration(r)

    if verbose:
        for name, ok, detail in r.items:
            print(f"[{'OK' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))
        for name in r.skipped:
            print(f"[SKIP] {name}")
    passed = len(r.items) - len(r.failed)
    print(f"\n自检通过 {passed}/{len(r.items)}" + (f"，跳过 {len(r.skipped)} 组" if r.skipped else ""))
    return 0 if not r.failed else 1


if __name__ == "__main__":
    raise SystemExit(run_selftest())
