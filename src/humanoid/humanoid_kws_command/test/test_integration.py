#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集成测试：用随包特征训练一个小模型，验证抗噪排序与拒识。

直接运行：python3 src/humanoid/humanoid_kws_command/test/test_integration.py
"""

import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

import numpy as np                                              # noqa: E402

import humanoid_kws_command.config as config                    # noqa: E402
import humanoid_kws_command.corpus as corpus                    # noqa: E402
import humanoid_kws_command.eval as kws_eval                    # noqa: E402
from humanoid_kws_command.kws import MLP                        # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    ok = bool(cond)
    if not ok:
        FAILS.append(name)
    print(f"[{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


def main():
    feat = PKG / "data" / config.FEATURE_FILE
    d = np.load(feat, allow_pickle=True)
    X, y, cond, snr = d["X"], d["y"], d["cond"], d["snr"]

    check("特征文件可加载", X.shape[1] == 2 * config.N_MELS, f"{X.shape}")

    # 只在训练态 SNR 上训练（干净 + 20 + 10）
    train = np.isin(snr, [corpus.snr_tag(s) for s in config.TRAIN_SNRS])
    m = MLP(X.shape[1])
    m.fit(X[train], y[train], epochs=60)

    curve = dict((s, a) for s, a, _ in kws_eval.snr_sweep_from_features(m, X, y, snr))
    clean = curve.get(None, 0.0)
    check("干净档准确率 > 95%", clean > 0.95, f"{clean*100:.1f}%")
    check("20 dB 准确率 > 90%", curve.get(20, 0) > 0.90, f"{curve.get(20, 0)*100:.1f}%")
    # 抗噪必须单调下降：噪声越大不会更准
    ordered = [curve[s] for s in (None, 20, 10, 5, 0, -5) if s in curve]
    check("准确率随噪声单调不增", all(a >= b - 1e-9 for a, b in zip(ordered, ordered[1:])),
          " > ".join(f"{v*100:.0f}%" for v in ordered))

    clean_mask = snr == "clean"
    far, frr = kws_eval.rejection_metrics(m, X[clean_mask], y[clean_mask])
    check("未知词误接受率 FAR < 20%", far < 0.2, f"{far*100:.1f}%")
    check("已知词拒绝率 FRR < 20%", frr < 0.2, f"{frr*100:.1f}%")

    cm = kws_eval.confusion(m, X[clean_mask], y[clean_mask])
    check("混淆矩阵对角占优", np.trace(cm) / max(cm.sum(), 1) > 0.9,
          f"{np.trace(cm)}/{cm.sum()}")

    print()
    print(f"{len(FAILS)} 项失败" if FAILS else "全部通过")
    return 1 if FAILS else 0


if __name__ == "__main__":
    raise SystemExit(main())