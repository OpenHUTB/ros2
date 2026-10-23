#!/usr/bin/env python3
"""main.py — 统一入口。"""
import argparse, os, sys
_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)
from mjc_rov_rl_compare import rl_compare

def main():
    p = argparse.ArgumentParser(prog="mjc_rov_rl_compare")
    sub = p.add_subparsers(dest="cmd")
    sub.add_parser("compare")
    args = p.parse_args()
    if not args.cmd or args.cmd == "compare":
        rl_compare.run_compare()

if __name__ == "__main__":
    main()
