"""出图：模式对比动图 + 分配精度图。

动图只标 ASCII 文本：Matplotlib 的默认字体没有中文，画中文会变成方块，
而依赖系统字体会让 CI/别人的机器上结果不一致。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np


def _matplotlib():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except Exception as exc:  # pragma: no cover - 视环境
        raise RuntimeError(f"需要 matplotlib 才能出图：{exc}") from exc


def _pil():
    from PIL import Image
    return Image


#: 动图里画的模式顺序与配色
GIF_MODES = ("geometric", "ls", "neural")
MODE_COLORS = {"geometric": "#C44E52", "naive": "#8172B3", "ls": "#4C72B0",
               "neural": "#55A868"}


def _ascii_plan(plan) -> str:
    """故障工况的英文短标题（图上只能写 ASCII）。"""
    if getattr(plan, "kind", "healthy") == "healthy":
        return "healthy"
    rotor = getattr(plan, "rotor", 0)
    severity = getattr(plan, "severity", 1.0)
    return f"{plan.kind} rotor{rotor} sev{severity:.2f}"


def make_altitude_gif(out_path, plan=None, trajectory: str = "hover",
                      seconds: float = 8.0, fps: int = 10, net=None,
                      modes=GIF_MODES, api=None) -> str:
    """把三种模式的跟踪曲线逐帧画出来，存成 gif。

    左图是高度曲线（虚线是参考），右图是当前故障桨的推力权限；
    时间轴一路推进，动图最后定格在故障后几秒，能直观看到
    ``ls``/``neural`` 还维持得住，``geometric`` 已经掉下去。
    """
    from . import data as D
    from . import evaluate as E
    from . import faults as FL

    plt = _matplotlib()
    if plan is None:
        plan = FL.FaultPlan(kind="degradation", rotor=1, onset=2.0, severity=0.5)
    if api is None:
        from ._compat import load_base
        api = load_base()

    params, effectiveness = D.build_context(api)
    traces = {}
    for mode in modes:
        traces[mode] = E.trace_mode(api, mode, plan, trajectory, seconds,
                                    net=net, params=params,
                                    effectiveness=effectiveness)

    n_frames = int(round(seconds * fps))
    step = max(1, int(round(1.0 / (fps * 0.01))))
    frames = []
    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.4), dpi=80)

    for frame in range(n_frames):
        k = min(frame * step, max(len(t) for t in
                                  (tr["t"] for tr in traces.values())) - 1)
        for ax in axes:
            ax.clear()
        axes[0].set_xlim(0.0, seconds)
        axes[0].set_ylim(0.0, 2.0)
        axes[0].set_xlabel("time (s)", fontsize=8)
        axes[0].set_ylabel("altitude (m)", fontsize=8)
        axes[0].set_title("altitude tracking", fontsize=9)
        axes[0].axhline(0.15, color="0.6", lw=0.8, ls=":")
        axes[0].tick_params(labelsize=7)

        ref_t = None
        for mode in modes:
            tr = traces[mode]
            m = min(k, len(tr["t"]) - 1)
            axes[0].plot(tr["t"][:m + 1], tr["alt"][:m + 1],
                         color=MODE_COLORS.get(mode), lw=1.4, label=mode)
            if ref_t is None:
                ref_t = tr
        if ref_t is not None:
            m = min(k, len(ref_t["t"]) - 1)
            axes[0].plot(ref_t["t"][:m + 1], ref_t["alt_d"][:m + 1],
                         color="0.4", ls="--", lw=1.0, label="reference")
        axes[0].legend(fontsize=6, loc="lower left")

        tr0 = traces[modes[0]]
        m = min(k, len(tr0["t"]) - 1)
        gains = tr0["gains"][m]
        axes[1].bar(range(1, 5), gains, color=["#4C72B0" if g > 0.99 else "#C44E52"
                                               for g in gains])
        axes[1].set_ylim(0.0, 1.05)
        axes[1].set_xticks(range(1, 5))
        axes[1].set_xlabel("rotor", fontsize=8)
        axes[1].set_ylabel("thrust authority g", fontsize=8)
        axes[1].set_title(_ascii_plan(plan), fontsize=9)
        axes[1].tick_params(labelsize=7)

        t_now = float(tr0["t"][m])
        fig.suptitle(f"t = {t_now:4.2f} s", fontsize=9)
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        frames.append(_pil().fromarray(img.copy()))
    plt.close(fig)

    path = Path(out_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(path, save_all=True, append_images=frames[1:],
                   duration=int(round(1000 / fps)), loop=0, optimize=True)
    return str(path)


def make_summary_png(rows, out_path) -> str:
    """两种模式的生存率与位置误差条形图。"""
    plt = _matplotlib()
    modes = list(dict.fromkeys(r.mode for r in rows))
    survival, rmse = [], []
    for mode in modes:
        items = [r for r in rows if r.mode == mode]
        survival.append(sum(0 if it.crashed else 1 for it in items) / max(len(items), 1))
        rmse.append(float(np.mean([it.pos_rmse for it in items])))

    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.2), dpi=90)
    n_flights = len(rows) // max(len(modes), 1)
    axes[0].bar(modes, survival, color=[MODE_COLORS.get(m, "#4C72B0") for m in modes])
    axes[0].set_ylim(0.0, 1.05)
    axes[0].set_ylabel("survival rate")
    axes[0].set_title(f"survival rate ({n_flights} flights)")
    for i, value in enumerate(survival):
        axes[0].text(i, value + 0.03, f"{value:.2f}", ha="center", fontsize=7)

    axes[1].bar(modes, rmse, color=[MODE_COLORS.get(m, "#4C72B0") for m in modes])
    axes[1].set_ylabel("position RMSE (m)")
    axes[1].set_title("tracking error")
    for i, value in enumerate(rmse):
        axes[1].text(i, value * 1.02 + 0.005, f"{value:.3f}", ha="center", fontsize=7)
    axes[1].set_ylim(0.0, max(rmse + [0.05]) * 1.3)
    for ax in axes:
        ax.tick_params(labelsize=7)
    fig.tight_layout()

    path = Path(out_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return str(path)


def make_allocator_png(dataset, net, out_path, n_samples: int = 800,
                       seed: int = 0) -> str:
    """神经网络残差 vs 近似解：指令误差直方图。"""
    from . import model as M

    plt = _matplotlib()
    x = dataset["X"]
    y = dataset["Y"]
    rng = np.random.default_rng(int(seed))
    n = int(min(n_samples, x.shape[0]))
    idx = rng.choice(x.shape[0], size=n, replace=False)
    pred = M.predict_numpy(net, x[idx]) if net is not None else np.zeros_like(y[idx])
    scale = float(dataset.get("max_thrust", 6.0))
    err_neural = np.mean(np.abs(pred - y[idx]), axis=1) * scale
    err_ls = np.mean(np.abs(y[idx]), axis=1) * scale

    fig, ax = plt.subplots(figsize=(6.4, 3.4), dpi=90)
    bins = np.linspace(0.0, float(max(err_ls.max(), err_neural.max())) * 1.05, 40)
    ax.hist(err_ls, bins=bins, alpha=0.6, label="ls (approx)", color="#4C72B0")
    ax.hist(err_neural, bins=bins, alpha=0.7, label="neural residual", color="#55A868")
    ax.set_xlabel("mean thrust error vs exact QP (N)")
    ax.set_ylabel("samples")
    ax.set_title(f"allocation error, n={n}")
    ax.legend(fontsize=7)
    ax.tick_params(labelsize=7)
    fig.tight_layout()

    path = Path(out_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return str(path)
