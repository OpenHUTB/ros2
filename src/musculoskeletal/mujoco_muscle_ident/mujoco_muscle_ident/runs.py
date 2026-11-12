"""五组参数辨识实验（MuJoCo 关节模型）。"""

from __future__ import annotations

import numpy as np

from . import core_mj as C

P0 = np.array([300.0, 0.26, 0.012, 0.035, 0.08])   # 偏离真值的初值


def _err(fit):
    return {k: abs(fit[k] - C.TRUE[k]) / C.TRUE[k] * 100.0 for k in C.PARAM_NAMES}


def run_i1(noise=0.0):
    """I1 干净数据辨识：真值 vs 估计。"""
    meas, _ = C.make_measurement(C.profile_square, noise)
    fit, cov, it = C.lm_fit(P0, meas, u_fn=C.profile_square)
    e = _err(fit)
    return {
        "iterations": it,
        "mean_err_pct": float(np.mean(list(e.values()))),
        "params": [{"name": k, "true": C.TRUE[k], "est": fit[k], "err_pct": e[k]}
                   for k in C.PARAM_NAMES],
    }


def run_i2(noises=(0.0, 1.0, 2.0, 5.0), seeds=3):
    """I2 噪声敏感性（每个噪声水平跑多个 seed，取中位数）。"""
    rows = []
    for noise in noises:
        means, maxs = [], []
        for seed in range(seeds):
            meas, _ = C.make_measurement(C.profile_square, noise, seed=seed)
            fit, _, _ = C.lm_fit(P0, meas, u_fn=C.profile_square)
            e = _err(fit)
            means.append(float(np.mean(list(e.values()))))
            maxs.append(float(max(e.values())))
        rows.append({"noise_pct": noise, "seeds": seeds,
                     "mean_err_pct": float(np.median(means)),
                     "max_err_pct": float(np.median(maxs)),
                     "spread_pct": float(np.ptp(means))})
    return {"levels": rows}


def run_i3():
    """I3 参数相关性（Fisher 信息矩阵）。"""
    meas, _ = C.make_measurement(C.profile_square, 0.0)
    fit, cov, _ = C.lm_fit(P0, meas, u_fn=C.profile_square)
    F, d_opt, cond = C.fisher_info(
        np.array([fit[k] for k in C.PARAM_NAMES]), meas, u_fn=C.profile_square)
    d = np.sqrt(np.diag(cov))
    corr = cov / np.outer(d, d)
    pairs = []
    for i in range(len(C.PARAM_NAMES)):
        for j in range(i + 1, len(C.PARAM_NAMES)):
            pairs.append({"a": C.PARAM_NAMES[i], "b": C.PARAM_NAMES[j],
                          "corr": float(corr[i, j])})
    pairs.sort(key=lambda x: -abs(x["corr"]))
    return {"matrix": corr.tolist(), "names": C.PARAM_NAMES,
            "top_pairs": pairs[:3], "d_optimal": d_opt, "cond": cond}


def run_i4(ranges=((0.9, 1.1), (0.8, 1.2), (0.6, 1.5), (0.5, 2.0)),
           starts=4, seed=3):
    """I4 初值敏感性：扫描初值范围，统计收敛成功率。

    辨识是强非线性的，初值离真值太远会掉进其他极小点，
    这里给出"有效收敛域"的定量边界。
    """
    meas, _ = C.make_measurement(C.profile_square, 0.0)
    base = np.array([C.TRUE[k] for k in C.PARAM_NAMES])
    rows = []
    for lo, hi in ranges:
        rng = np.random.default_rng(seed)
        errs = []
        for _ in range(starts):
            p0 = base * rng.uniform(lo, hi, size=len(base))
            fit, _, it = C.lm_fit(p0, meas, u_fn=C.profile_square)
            e = _err(fit)
            errs.append(float(max(e.values())))
        ok = [e for e in errs if e < 5.0]
        rows.append({
            "range_lo": lo, "range_hi": hi,
            "success_rate_pct": len(ok) / len(errs) * 100.0,
            "median_err_pct": float(np.median(errs)),
            "best_err_pct": float(min(errs)),
        })
    return {"levels": rows}


def run_i5():
    """I5 激励设计：三种激励的 D-最优指标与辨识误差。"""
    rows = []
    for name, fn in C.PROFILES.items():
        meas, clean = C.make_measurement(fn, 0.0)
        fit, _, it = C.lm_fit(P0, meas, u_fn=fn)
        F, d_opt, cond = C.fisher_info(
            np.array([fit[k] for k in C.PARAM_NAMES]), meas, u_fn=fn)
        e = _err(fit)
        rows.append({
            "excitation": name,
            "angle_min": float(clean.min()), "angle_max": float(clean.max()),
            "d_optimal": float(d_opt), "cond": float(cond),
            "mean_err_pct": float(np.mean(list(e.values()))),
            "iterations": it,
        })
    rows.sort(key=lambda r: -r["d_optimal"])
    return {"levels": rows}
