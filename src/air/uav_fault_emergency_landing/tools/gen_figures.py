#!/usr/bin/env python3
"""生成文档用图：评分/高度图、模式对比、迫降过程动图。

用法（在包目录下）：

    python tools/gen_figures.py            # 全量
    python tools/gen_figures.py --skip-gif # 只出静态图（动图较慢）
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_PKG_DIR = _HERE.parent
_REPO_ROOT = _HERE.parents[3]
sys.path.insert(0, str(_PKG_DIR))

IMG_DIR = _REPO_ROOT / "docs" / "img" / "air" / "uav_fault_emergency_landing"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="生成 PR4 文档配图")
    ap.add_argument("--skip-gif", action="store_true", help="跳过动图（较慢）")
    args = ap.parse_args(argv)

    try:
        import numpy as np
    except ImportError:
        print("缺少 numpy，先 pip install numpy")
        return 1

    from uav_fault_emergency_landing import _compat, evaluate as E, mapping as M, model as Md

    api = _compat.load_base()
    params, eff = E.build_context(api)
    IMG_DIR.mkdir(parents=True, exist_ok=True)

    score_net = guidance_net = None
    score_path = _PKG_DIR / "results" / "score.pt"
    guidance_path = _PKG_DIR / "results" / "guidance.pt"
    if score_path.is_file():
        score_net, _ = Md.load_checkpoint(score_path)
    if guidance_path.is_file():
        guidance_net, _ = Md.load_checkpoint(guidance_path)

    # 1) 建图与评分图：用 3×3 个高位观测点扫一遍场景（模拟"被迫降前先看清楚地面"）
    import mujoco

    quad = api.Quadrotor(scene=api.make_scene("urban_block", seed=0), dt=0.01, substeps=4)
    mapper = M.DepthMapper(api).bind(quad)
    height_map = M.HeightMap()
    for gx in (-8.0, -2.0, 4.0):
        for gy in (-8.0, -2.0, 4.0):
            quad.data.qpos[:3] = [gx, gy, 12.0]
            quad.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
            mujoco.mj_forward(quad.model, quad.data)
            height_map.update(mapper.scan(quad.data))
    start_xy, hover_z = E.pick_obstacle_spot(api, "urban_block", 0)
    score = M.rule_landing_score(height_map, position_xy=np.asarray(start_xy, float))
    if score_net is not None:
        from uav_fault_emergency_landing import controller as C
        from uav_fault_emergency_landing import evaluate as _E

        params2, eff2 = _E.build_context(api)
        ctrl = C.ForcedLandingController(params2, api.mixer, api.GeometricController(params2),
                                         eff2, mode="neural", score_net=score_net)
        ctrl.height_map = height_map
        score = ctrl.score_map(np.asarray(start_xy, float))
    out = IMG_DIR / "landing_map.png"
    from uav_fault_emergency_landing import render as R

    R.make_map_png(height_map, score, out)
    print(f"已生成 {out}")

    # 2) 三模式对比图
    results = E.flight_table(api, scene="urban_block", seeds=(0,), seconds=40.0,
                             score_net=score_net, guidance_net=guidance_net,
                             params=params, effectiveness=eff)
    out = IMG_DIR / "mode_summary.png"
    R.make_summary_png(results, out)
    print(f"已生成 {out}")

    # 3) 迫降过程动图
    if not args.skip_gif:
        out = IMG_DIR / "landing_process.gif"
        R.make_landing_gif(out, scene="urban_block", seed=0, modes=("vertical", "expert"),
                           score_net=score_net, guidance_net=guidance_net, api=api)
        print(f"已生成 {out}（{out.stat().st_size / 1024:.0f} KB）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
