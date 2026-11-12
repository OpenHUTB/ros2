"""命令行入口：自检 / 采数据 / 训练 / 评估 / 出图 / ROS 节点。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

DEFAULT_DATASET = "results/dagger_dataset.npz"
DEFAULT_COMPARISON = "results/policy_comparison.json"


def _pkg_dir() -> Path:
    return Path(__file__).resolve().parent


def _require(path: str, what: str) -> Path:
    p = Path(path)
    if not p.is_absolute():
        p = _pkg_dir().parent / p
    if not p.is_file():
        print(f"[缺文件] {what}：{p}", file=sys.stderr)
        if "数据" in what:
            print("  先跑：python main.py --gen-data", file=sys.stderr)
        else:
            print("  先跑：python main.py --gen-data && python main.py --train",
                  file=sys.stderr)
        raise SystemExit(2)
    return p


def _base():
    from . import _compat

    try:
        return _compat.load_base()
    except ImportError as exc:
        print(f"[缺依赖] {exc}", file=sys.stderr)
        raise SystemExit(2)


def cmd_selftest(a) -> int:
    from .selftest import run_selftest

    return run_selftest(verbose=not a.quiet)


def cmd_gen_data(a) -> int:
    from . import data as D

    api = _base()
    out = Path(a.out or DEFAULT_DATASET)
    if not out.is_absolute():
        out = _pkg_dir().parent / out
    print(f"DAgger 采数据：{a.rounds} 轮 × {len(D.TRAIN_CASES)} 工况 × "
          f"{a.episodes} 次飞行")
    policy_module = None
    dataset = None
    stats = []
    for r in range(a.rounds):
        # beta = 听专家的概率：第一轮全听专家（纯 BC），之后按轮次线性降到 0.5。
        # 不降到更低：单步随机接管 + 极低 beta 会让数据几乎全是策略失控后的状态，
        # 专家在这些状态下的标签没有意义，反而把网络训坏（实测 MAE 从 0.026 涨到 0.083）。
        beta = 1.0 if a.rounds <= 1 else 0.9 + (0.5 - 0.9) * (r / max(a.rounds - 1, 1))
        dataset, new_stats = D.collect(
            api, policy_module=policy_module, rounds=1, beta=beta,
            episodes_per_case=a.episodes, seed=a.seed, verbose=True,
        )
        stats.extend(new_stats)
        D.save_dataset(out, dataset, stats)
        if r < a.rounds - 1:
            from . import train as T

            meta = T.train_policy(out, out.parent / "_warmup.pt", arch="cnn",
                                  cfg=T.TrainConfig(epochs=a.epochs, threads=a.threads),
                                  verbose=False)
            from . import policy as P

            policy_module, _ = P.load_checkpoint(out.parent / "_warmup.pt")
            print(f"  轮 {r} 预热训练完成：验证 MAE {meta['best_val_mae']:.4f}")
    print(f"数据写进 {out}，共 {stats and dataset['depth'].shape[0]} 个样本")
    return 0


def cmd_train(a) -> int:
    from . import policy as P
    from . import train as T

    data_path = _require(a.dataset, "DAgger 数据集")
    arches = P.ARCHS if a.arch == "all" else (a.arch,)
    out_dir = Path(a.out or "results")
    if not out_dir.is_absolute():
        out_dir = _pkg_dir().parent / out_dir
    cfg = T.TrainConfig(epochs=a.epochs, batch_size=a.batch_size, lr=a.lr,
                        patience=a.patience, seed=a.seed, threads=a.threads)
    for arch in arches:
        print(f"训练结构 {arch} …")
        T.train_policy(data_path, out_dir / f"policy_{arch}.pt", arch=arch, cfg=cfg)
    return 0


def cmd_eval(a) -> int:
    from . import evaluate as E
    from . import policy as P

    policies = {}
    for arch in P.ARCHS:
        ckpt = _pkg_dir().parent / "results" / f"policy_{arch}.pt"
        if ckpt.is_file():
            module, _ = P.load_checkpoint(ckpt)
            policies[arch] = module
        else:
            print(f"[提示] 没有 {ckpt.name}，跳过该结构")
    if not policies:
        print("[缺文件] 没有可用的策略权重，先跑 --train", file=sys.stderr)
        return 2
    api = _base()
    rows = E.flight_table(api, policies=policies, seeds=tuple(a.seeds))
    print()
    print(E.format_table(rows))
    print()
    summary = E.aggregate(rows)
    print(E.format_summary(summary))
    out = Path(a.json or DEFAULT_COMPARISON)
    if not out.is_absolute():
        out = _pkg_dir().parent / out
    E.save_json(out, {"summary": summary, "rows": [r.as_dict() for r in rows]})
    print(f"\n结果写进 {out}")
    return 0


def cmd_demo(a) -> int:
    from . import data as D
    from . import policy as P
    from .render import make_policy_gif, make_summary_png

    out = Path(a.out or "results/policy_demo.gif")
    if not out.is_absolute():
        out = _pkg_dir().parent / out
    ckpt = _pkg_dir().parent / "results" / f"policy_{a.arch}.pt"
    if not ckpt.is_file():
        print(f"[缺文件] 权重 {ckpt}，先跑 --train", file=sys.stderr)
        return 2
    module, _ = P.load_checkpoint(ckpt)
    spec = dict(D.EVAL_CASES)[a.case] if a.case in dict(D.EVAL_CASES) else {"kind": "healthy"}
    make_policy_gif(out, api=_base(), policy_module=module, spec=spec, seed=a.seed,
                    fps=a.fps, arch=a.arch)
    print(f"已生成 {out}（{out.stat().st_size / 1024:.0f} KB）")
    cmp_path = _pkg_dir().parent / DEFAULT_COMPARISON
    if cmp_path.is_file():
        payload = json.loads(cmp_path.read_text(encoding="utf-8"))
        png = out.with_name("policy_summary.png")
        make_summary_png(payload["summary"], png)
        print(f"已生成 {png}")
    return 0


def cmd_node(a) -> int:
    from .ros_node import main as node_main

    argv = sys.argv[1:]
    if "--node" in argv:
        argv.remove("--node")
    return node_main(argv) or 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="无人机桨故障下的端到端深度图推力策略（DAgger + CNN）")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--selftest", action="store_true", help="无权重自检")
    g.add_argument("--gen-data", action="store_true", help="采 DAgger 数据")
    g.add_argument("--train", action="store_true", help="监督训练")
    g.add_argument("--eval", action="store_true", help="闭环对比评估")
    g.add_argument("--demo", action="store_true", help="录演示 gif")
    g.add_argument("--node", action="store_true", help="启动 ROS2 节点")

    ap.add_argument("--out", default=None, help="输出路径（数据/图/目录）")
    ap.add_argument("--dataset", default=DEFAULT_DATASET)
    ap.add_argument("--json", default=DEFAULT_COMPARISON)
    ap.add_argument("--arch", default="cnn", choices=("mlp", "cnn", "cnn_stack", "all"))
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--batch-size", type=int, default=128)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--patience", type=int, default=10)
    ap.add_argument("--threads", type=int, default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0])
    ap.add_argument("--case", default="healthy",
                    help="demo 用哪个工况（见 data.EVAL_CASES）")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--quiet", action="store_true")
    return ap


def main(argv=None) -> int:
    ap = build_parser()
    a, _unknown = ap.parse_known_args(argv)
    if a.selftest:
        return cmd_selftest(a)
    if a.gen_data:
        return cmd_gen_data(a)
    if a.train:
        return cmd_train(a)
    if a.eval:
        return cmd_eval(a)
    if a.demo:
        return cmd_demo(a)
    return cmd_node(a)
