# -*- coding: utf-8 -*-
"""命令行入口：不需要 ROS，直接算出三种机型的失效包线。

用法::

    python3 main.py            # 对比四/六/八旋翼 + 四旋翼明细
    python3 main.py --detail hexa
    python3 main.py --json out.json
"""

import argparse
import json
import sys

from . import cli
from .core import hexa, max_hover, octo, quad_x


def _say(text=""):
    """GBK 控制台下安全打印。"""
    try:
        print(text)
    except UnicodeEncodeError:
        enc = getattr(sys.stdout, "encoding", None) or "ascii"
        print(text.encode(enc, "replace").decode(enc))


def _pick(name):
    table = {"quad": quad_x, "quad-x": quad_x, "hexa": hexa, "octo": octo}
    if name not in table:
        raise SystemExit("未知机型 %r，可选：quad / hexa / octo" % name)
    return table[name]()


def main(argv=None):
    ap = argparse.ArgumentParser(description="多旋翼单桨失效的悬停可行域")
    ap.add_argument("--detail", default="quad", help="打印哪种机型的逐桨明细：quad/hexa/octo")
    ap.add_argument("--payload", type=float, default=0.0, help="额外的任务载荷 kg（仅打印判断）")
    ap.add_argument("--json", default="", help="写入 JSON")
    args = ap.parse_args(argv)

    configs = [quad_x(), hexa(), octo()]

    _say("## 三种机型的失效包线（单桨失效）")
    _say(cli.table_configs(configs))
    _say()

    cfg = _pick(args.detail)
    _say("## %s 逐桨明细" % cfg.name)
    _say(cli.table_detail(cfg))
    _say()

    _say("## %s 失效 #0 的最优推力分配" % cfg.name)
    _say(cli.table_thrusts(cfg, failed=(0,)))
    _say()

    if args.payload > 0:
        for c in configs:
            r = max_hover(c, {0})
            ok = "能" if r["mass_limit"] >= c.mass + args.payload else "不能"
            _say("%-7s 自旋悬停：载重上限 %.2f kg，机重 %.2f + 载荷 %.2f → %s吊起"
                 % (c.name, r["mass_limit"], c.mass, args.payload, ok))

    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(cli.json_summary(configs), f, ensure_ascii=False, indent=2)
        _say("已写入 %s" % args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
