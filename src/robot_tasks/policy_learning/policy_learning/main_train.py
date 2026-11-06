"""离线入口：采集示范 → 训练 → DAgger 纠偏 → 闭环评测。

用法：
    python -m policy_learning.main_train --all
    python -m policy_learning.main_train --datagen --episodes 150
    python -m policy_learning.main_train --train --epochs 120
    python -m policy_learning.main_train --evaluate --episodes 50
"""
from __future__ import annotations

import argparse
import json
import os

import numpy as np

from .dataset import Dataset, collect, dagger, merge, split
from .mlp import PolicyMLP
from .runner import evaluate, latency
from .train import TrainConfig, train

DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "data")


def _p(name: str) -> str:
    return os.path.join(DATA_DIR, name)


def _save_ds(ds: Dataset) -> None:
    os.makedirs(DATA_DIR, exist_ok=True)
    np.savez_compressed(_p("demonstrations.npz"), x=ds.x, y=ds.y,
                        episode=ds.episode,
                        meta_json=np.asarray(json.dumps(ds.stats,
                                                        ensure_ascii=False)))


def _load_ds() -> Dataset:
    d = np.load(_p("demonstrations.npz"), allow_pickle=False)
    meta = json.loads(str(d["meta_json"])) if "meta_json" in d.files else {}
    return Dataset(d["x"], d["y"], d["episode"], meta)


def cmd_datagen(a) -> int:
    ds = collect(episodes=a.episodes, seed=a.seed, verbose=True)
    _save_ds(ds)
    print("样本 %d，episode %d → %s"
          % (len(ds), len(np.unique(ds.episode)), _p("demonstrations.npz")))
    return 0


def _fit(ds: Dataset, a, tag: str):
    tr, va = split(ds, 0.2, seed=0)
    print("=== %s：训练集 %d / 验证集 %d" % (tag, len(tr), len(va)))
    net, hist = train(tr, va, TrainConfig(epochs=a.epochs, seed=a.seed),
                      verbose=True)
    return net, hist, tr, va


def cmd_train(a) -> int:
    ds = _load_ds()
    net, hist, tr, va = _fit(ds, a, "训练")
    net.save(_p("policy.npz"),
             {"train_samples": len(tr), "val_samples": len(va),
              "best_epoch": int(hist["best_epoch"])})
    with open(_p("train_hist.json"), "w", encoding="utf-8") as f:
        json.dump(hist, f, ensure_ascii=False, indent=2)
    print("模型 %s（%d 参数）" % (_p("policy.npz"), net.n_params()))
    return 0


def cmd_evaluate(a) -> int:
    ds = _load_ds()
    tr, va = split(ds, 0.2, seed=0)
    net = PolicyMLP.load(_p("policy.npz"))
    pred = net.predict(va.x)
    res = {
        "regression": {
            "rmse_fwd": float(np.sqrt(np.mean((pred[:, 0] - va.y[:, 0]) ** 2))),
            "rmse_lat": float(np.sqrt(np.mean((pred[:, 1] - va.y[:, 1]) ** 2))),
        },
        "closed_loop": evaluate(net, n_episodes=a.episodes, seed=5000),
        "latency": latency(net, va.x[0]),
        "model_params": net.n_params(),
        "val_samples": len(va),
    }
    with open(_p("results.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    print(json.dumps(res, ensure_ascii=False, indent=2))
    return 0


def cmd_all(a) -> int:
    if cmd_datagen(a):
        return 1
    ds = _load_ds()

    for r in range(int(a.dagger)):
        net, _, _, _ = _fit(ds, a, "第 %d 轮" % (r + 1))
        print("  按当前策略采集纠正样本 ...")
        extra = dagger(net, episodes=a.dagger_episodes, seed=100 + r)
        print("  新增 %d 条" % len(extra))
        if len(extra) == 0:
            break
        ds = merge(ds, extra, offset=(r + 1) * 100000)
        _save_ds(ds)

    net, hist, tr, va = _fit(ds, a, "最终")
    net.save(_p("policy.npz"),
             {"train_samples": len(tr), "val_samples": len(va),
              "best_epoch": int(hist["best_epoch"]),
              "samples": len(ds)})
    with open(_p("train_hist.json"), "w", encoding="utf-8") as f:
        json.dump(hist, f, ensure_ascii=False, indent=2)
    print("模型 %s（%d 参数）" % (_p("policy.npz"), net.n_params()))
    return cmd_evaluate(a)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="policy_learning")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--datagen", action="store_true", help="采集专家示范")
    g.add_argument("--train", action="store_true", help="训练策略网络")
    g.add_argument("--evaluate", action="store_true", help="闭环评测")
    g.add_argument("--all", action="store_true", help="采集+训练+DAgger+评测")
    p.add_argument("--episodes", type=int, default=150)
    p.add_argument("--epochs", type=int, default=120)
    p.add_argument("--dagger", type=int, default=2, help="DAgger 轮数")
    p.add_argument("--dagger-episodes", type=int, default=25,
                   dest="dagger_episodes")
    p.add_argument("--seed", type=int, default=0)
    a = p.parse_args(argv)

    if a.datagen:
        return cmd_datagen(a)
    if a.train:
        return cmd_train(a)
    if a.evaluate:
        return cmd_evaluate(a)
    if a.all:
        return cmd_all(a)
    p.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
