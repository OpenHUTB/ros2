"""命令行入口：一条命令跑完"生成数据 → 训练 → 评测 → 出图"。

各子命令内部延迟 import（仿真侧要 py>=3.10、rclpy 只在 ROS 环境里有），
所以 `python main.py --selftest` 在没有 ROS 的机器上也能跑。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

DEFAULT_SCORE_DATA = "results/score_dataset.npz"
DEFAULT_GUIDANCE_DATA = "results/guidance_dataset.npz"
DEFAULT_SCORE_CKPT = "results/score.pt"
DEFAULT_GUIDANCE_CKPT = "results/guidance.pt"
DEFAULT_SUMMARY_JSON = "results/landing_comparison.json"


def _hint(kind: str, path: str) -> str:
    if kind == "score":
        cmd = f"python main.py --gen-data {path} --maps 120"
    elif kind == "guidance":
        cmd = f"python main.py --gen-data {path} --guidance-samples 4000"
    else:
        cmd = "python main.py --train"
    return (f"找不到{kind}数据集/权重：{path}\n"
            f"先跑一遍：\n    {cmd}\n"
            f"（两套数据合起来约 1-2 分钟，两个网络训练约 1 分钟）")


def _require(path: str, kind: str) -> Path:
    p = Path(path)
    if not p.is_file():
        print(_hint(kind, path))
        raise SystemExit(2)
    return p


# --------------------------------------------------------------------------- #
def cmd_selftest(args) -> int:
    from .selftest import run_selftest

    return run_selftest(verbose=True)


def cmd_gen_data(args) -> int:
    from . import _compat, data as D

    api = _compat.load_base()
    out_score = Path(args.score_data)
    out_guidance = Path(args.guidance_data)
    out_score.parent.mkdir(parents=True, exist_ok=True)
    out_guidance.parent.mkdir(parents=True, exist_ok=True)

    print(f"生成评分数据集（{args.maps} 张地图，扫描建图）…")
    ds = D.build_score_dataset(api, n_maps=args.maps, seed=args.seed)
    D.save_dataset(out_score, ds)
    print(f"  → {out_score}  X={ds['X'].shape}")

    print(f"生成制导数据集（{args.guidance_samples} 条，解析式打标签）…")
    g = D.build_guidance_dataset(args.guidance_samples, seed=args.seed)
    D.save_dataset(out_guidance, g)
    print(f"  → {out_guidance}  X={g['X'].shape}")
    return 0


def cmd_train(args) -> int:
    from . import train as T

    cfg = T.TrainConfig(epochs=args.epochs, batch_size=args.batch_size, lr=args.lr,
                        patience=args.patience, seed=args.seed, threads=args.threads)
    score_ds = _require(args.score_data, "score")
    guidance_ds = _require(args.guidance_data, "guidance")
    Path(DEFAULT_SCORE_CKPT).parent.mkdir(parents=True, exist_ok=True)

    print("训练落点评分网络（蒸馏规则评分）…")
    meta = T.train_score(str(score_ds), str(Path(args.score_checkpoint)), cfg, verbose=True)
    print(f"  → {args.score_checkpoint}  test MAE={meta.get('test_mae'):.4f}")

    print("训练迫降制导网络（回归专家律）…")
    meta2 = T.train_guidance(str(guidance_ds), str(Path(args.guidance_checkpoint)), cfg,
                             verbose=True)
    print(f"  → {args.guidance_checkpoint}  test MAE={meta2.get('test_mae'):.4f}")
    return 0


def cmd_eval(args) -> int:
    from . import _compat, evaluate as E, model as Md

    api = _compat.load_base()
    params, effectiveness = E.build_context(api)
    score_net = guidance_net = None
    if Path(args.score_checkpoint).is_file():
        score_net, _ = Md.load_checkpoint(args.score_checkpoint)
    else:
        print(_hint("score", args.score_checkpoint))
    if Path(args.guidance_checkpoint).is_file():
        guidance_net, _ = Md.load_checkpoint(args.guidance_checkpoint)
    else:
        print(_hint("guidance", args.guidance_checkpoint))

    rows = E.flight_table(api, modes=args.modes, scene=args.scene, seeds=tuple(args.seeds),
                          seconds=args.seconds, score_net=score_net,
                          guidance_net=guidance_net, params=params,
                          effectiveness=effectiveness)
    print()
    print(E.format_dicts([r.summary() for r in rows]))
    E.save_json(args.json, {"results": [r.summary() for r in rows]})
    print(f"\n结果已写入 {args.json}")
    return 0


def cmd_demo(args) -> int:
    from . import _compat, evaluate as E, mapping as M, model as Md, render as R

    api = _compat.load_base()
    score_net = guidance_net = None
    if Path(args.score_checkpoint).is_file():
        score_net, _ = Md.load_checkpoint(args.score_checkpoint)
    if Path(args.guidance_checkpoint).is_file():
        guidance_net, _ = Md.load_checkpoint(args.guidance_checkpoint)

    modes = ["vertical", "expert"] + (["neural"] if guidance_net is not None else [])
    out = Path(args.gif)
    out.parent.mkdir(parents=True, exist_ok=True)
    R.make_landing_gif(str(out), plan_spec=dict(kind=args.fault, rotor=args.rotor,
                                               severity=args.severity),
                       scene=args.scene, seed=args.seed, modes=tuple(modes),
                       fps=args.fps, seconds=args.seconds, score_net=score_net,
                       guidance_net=guidance_net, api=api)
    print(f"已生成 {out}")
    return 0


def cmd_node(args) -> int:
    from .ros_node import main as node_main

    return node_main(sys.argv[1:])


# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="uav_fault_emergency_landing",
        description="无人机桨故障下的神经迫降：深度图建图 → CNN 落点评分 → 制导迫降",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    sub = ap.add_mutually_exclusive_group(required=True)
    sub.add_argument("--selftest", action="store_true", help="跑自检")
    sub.add_argument("--gen-data", action="store_true", help="生成两套数据集")
    sub.add_argument("--train", action="store_true", help="训练两个网络")
    sub.add_argument("--eval", action="store_true", help="三种模式 × 故障工况对比")
    sub.add_argument("--demo", action="store_true", help="生成迫降对比动图")
    sub.add_argument("--node", action="store_true", help="启动 ROS2 节点")

    ap.add_argument("--scene", default="urban_block")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0])
    ap.add_argument("--fault", default="degradation")
    ap.add_argument("--rotor", type=int, default=2)
    ap.add_argument("--severity", type=float, default=0.5)
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--maps", type=int, default=120)
    ap.add_argument("--guidance-samples", type=int, default=4000)
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch-size", type=int, default=32)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--patience", type=int, default=8)
    ap.add_argument("--threads", type=int, default=None)
    ap.add_argument("--modes", nargs="+", default=["vertical", "expert", "neural"])
    ap.add_argument("--score-data", default=DEFAULT_SCORE_DATA)
    ap.add_argument("--guidance-data", default=DEFAULT_GUIDANCE_DATA)
    ap.add_argument("--score-checkpoint", default=DEFAULT_SCORE_CKPT)
    ap.add_argument("--guidance-checkpoint", default=DEFAULT_GUIDANCE_CKPT)
    ap.add_argument("--json", default=DEFAULT_SUMMARY_JSON)
    ap.add_argument("--gif", default="results/landing_demo.gif")
    ap.add_argument("--fps", type=int, default=10)
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.selftest:
        return cmd_selftest(args)
    if args.gen_data:
        return cmd_gen_data(args)
    if args.train:
        return cmd_train(args)
    if args.eval:
        return cmd_eval(args)
    if args.demo:
        return cmd_demo(args)
    if args.node:
        return cmd_node(args)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
