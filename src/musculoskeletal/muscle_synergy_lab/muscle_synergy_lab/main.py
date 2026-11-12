# -*- coding: utf-8 -*-
"""命令行入口：不需要 ROS，直接用合成数据验证协同分解。

用法::

    python3 main.py                       # 默认 8 块肌肉、真值 3 个协同
    python3 main.py --noise 0.10          # 加大噪声
    python3 main.py --muscles 12 --synergies 4
    python3 main.py --json out.json
"""

import argparse
import json
import sys

from . import cli
from .core import k_for_vaf, nmf, scan_k, synth_data

MUSCLE_NAMES = [
    "tib_ant", "gas_med", "gas_lat", "sol",
    "vas_lat", "vas_med", "rec_fem", "bi_fem",
    "glut_max", "glut_med", "ham", "iliopsoas",
]


def _say(text=""):
    """GBK 控制台下安全打印。"""
    try:
        print(text)
    except UnicodeEncodeError:
        enc = getattr(sys.stdout, "encoding", None) or "ascii"
        print(text.encode(enc, "replace").decode(enc))


def main(argv=None):
    ap = argparse.ArgumentParser(description="肌肉协同降维与恢复质量评估")
    ap.add_argument("--muscles", type=int, default=8, help="肌肉数")
    ap.add_argument("--synergies", type=int, default=3, help="真值协同数（合成数据用）")
    ap.add_argument("--samples", type=int, default=200, help="时间采样点数")
    ap.add_argument("--noise", type=float, default=0.03, help="噪声相对强度")
    ap.add_argument("--seed", type=int, default=0, help="随机种子")
    ap.add_argument("--json", default="", help="写入 JSON")
    args = ap.parse_args(argv)

    k_true = min(args.synergies, args.muscles)
    d = synth_data(n_muscles=args.muscles, n_synergies=k_true,
                   n_samples=args.samples, noise=args.noise, seed=args.seed)
    names = (MUSCLE_NAMES * 3)[:args.muscles]

    _say("## 数据：%d 块肌肉 × %d 个采样点，真值 %d 个协同，噪声 %.0f%%"
         % (d.E.shape[0], d.E.shape[1], k_true, 100 * args.noise))
    rows = scan_k(d.E, k_max=args.muscles, seed=args.seed)
    _say(cli.table_scan(rows))
    _say()
    _say("达到 VAF ≥ 0.95 所需协同数：**%s**（真值 %d）"
         % (k_for_vaf(rows, 0.95), k_true))
    _say()

    _say("## 恢复质量：k 取太小、刚好、太大时的对比")
    ks = sorted({max(1, k_true - 1), k_true, min(args.muscles, k_true + 1)})
    _say(cli.table_recovery(d.E, d.W_true, ks))
    _say()

    W, _H = nmf(d.E, k_true, seed=args.seed)
    _say("## k = %d 时估计出的协同权重" % k_true)
    _say(cli.table_synergy_vectors(W.tolist(), names))

    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(cli.json_summary(rows, d.E, d.W_true, k_true), f,
                      ensure_ascii=False, indent=2)
        _say("已写入 %s" % args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
