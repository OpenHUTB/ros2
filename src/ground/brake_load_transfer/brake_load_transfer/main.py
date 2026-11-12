# -*- coding: utf-8 -*-
"""命令行入口：无需 ROS，直接用 numpy 算出结论表。

用法::

    python3 main.py                  # 空载基准
    python3 main.py --case payload   # 载重 300 kg 居中
    python3 main.py --sweep          # 载重前后位置扫描
    python3 main.py --bias           # 制动力分配扫描
    python3 main.py --lateral        # 左右偏置 → 跑偏
    python3 main.py --all --json out.json
"""

import argparse
import json
import sys

from . import cli
from .core import (VehicleSpec, compare_with_ideal, ideal_stopping_distance,
                   simulate_braking, simulate_lateral)


def _say(text=""):
    """GBK 控制台下安全打印（与仓库其它模块保持一致的写法）。"""
    try:
        print(text)
    except UnicodeEncodeError:
        enc = getattr(sys.stdout, "encoding", None) or "ascii"
        print(text.encode(enc, "replace").decode(enc))


def build_spec(args) -> VehicleSpec:
    return VehicleSpec(payload=args.payload, payload_x=args.payload_x,
                       payload_y=args.payload_y, brake_bias=args.bias,
                       mu=args.mu, v0=args.v0 / 3.6)


def main(argv=None):
    ap = argparse.ArgumentParser(description="载重偏置对制动距离与跑偏的影响")
    ap.add_argument("--payload", type=float, default=0.0, help="载重 kg")
    ap.add_argument("--payload-x", type=float, default=1.35, help="载重质心距前轴 m")
    ap.add_argument("--payload-y", type=float, default=0.0, help="载重横向偏移 m")
    ap.add_argument("--bias", type=float, default=0.62, help="前轴制动力占比")
    ap.add_argument("--mu", type=float, default=0.90, help="路面附着系数")
    ap.add_argument("--v0", type=float, default=100.0, help="制动初速 km/h")
    ap.add_argument("--case", action="store_true", help="打印当前工况的单次结果")
    ap.add_argument("--sweep", action="store_true", help="载重前后位置扫描")
    ap.add_argument("--bias-sweep", action="store_true", help="制动力分配扫描")
    ap.add_argument("--lateral", action="store_true", help="左右偏置 → 跑偏")
    ap.add_argument("--all", action="store_true", help="以上全部")
    ap.add_argument("--json", default="", help="把关键数字写入 JSON")
    args = ap.parse_args(argv)

    spec = build_spec(args)
    do_all = args.all or not (args.case or args.sweep or args.bias_sweep or args.lateral)

    if do_all or args.case:
        _say("## 当前工况")
        _say(cli.table_case(spec))
        _say()

    if do_all or args.sweep:
        _say("## 载重前后位置扫描（载重 %.0f kg，β=%.2f）" % (args.payload or 300.0, spec.brake_bias))
        _say(cli.table_payload_sweep(spec, payload=args.payload or 300.0))
        _say()

    if do_all or args.bias_sweep:
        _say("## 制动力分配扫描（载重后移到 2.4 m）")
        _say(cli.table_bias_sweep(spec))
        _say()

    if do_all or args.lateral:
        _say("## 左右偏置导致的跑偏（载重 300 kg）")
        lat_spec = VehicleSpec(**{**spec.__dict__, "payload": args.payload or 300.0})
        _say(cli.table_lateral_sweep(lat_spec))
        _say()

    if args.json:
        out = {
            "spec": {k: v for k, v in spec.__dict__.items()},
            "case": compare_with_ideal(spec),
            "ideal_m": ideal_stopping_distance(spec),
            "braking": simulate_braking(spec),
            "lateral": simulate_lateral(spec),
        }
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2)
        _say("已写入 %s" % args.json)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
