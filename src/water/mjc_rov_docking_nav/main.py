#!/usr/bin/env python3
"""main.py — 统一入口。"""
import argparse, os, sys
_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)
from mjc_rov_docking_nav import dock_cnn

def main():
    p = argparse.ArgumentParser(prog="mjc_rov_docking_nav")
    sub = p.add_subparsers(dest="cmd")
    sub.add_parser("train")
    sub.add_parser("test")
    args = p.parse_args()
    if not args.cmd or args.cmd == "all":
        print("== [1/2] 训练 CNN ==")
        dock_cnn.train_dock_cnn(epochs=20)
        print("== [2/2] 测试 ==")
        dock_cnn.test_dock_cnn()
    elif args.cmd == "train":
        dock_cnn.train_dock_cnn(epochs=20)
    elif args.cmd == "test":
        dock_cnn.test_dock_cnn()

if __name__ == "__main__":
    main()
