#!/usr/bin/env python3
"""
main.py — 统一入口。
子命令: train / test / openset / all（默认不带参数=全链路）
"""
import argparse
import os
import sys

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from mjc_rov_sonar_cnn import cnn_model


def main():
    parser = argparse.ArgumentParser(prog="mjc_rov_sonar_cnn")
    sub = parser.add_subparsers(dest="cmd")
    sub.add_parser("train", help="训练三分类 CNN")
    t = sub.add_parser("test", help="测试三分类准确率")
    t.add_argument("--seed", type=int, default=42,
                   help="测试集随机种子；与训练集相同(42)时结果因数据泄漏而虚高")
    o = sub.add_parser("openset", help="背景类 + 置信度拒识（独立种子评估）")
    o.add_argument("--epochs", type=int, default=30)
    o.add_argument("--clutter", type=int, default=2,
                   help="每张图叠加的随机假目标数量")
    args = parser.parse_args()

    if not args.cmd or args.cmd == "all":
        print("== [1/2] 训练 CNN ==")
        cnn_model.train_cnn(epochs=30)
        print("== [2/2] 测试准确率 ==")
        cnn_model.test_cnn()
    elif args.cmd == "train":
        cnn_model.train_cnn(epochs=30)
    elif args.cmd == "test":
        cnn_model.test_cnn(seed=args.seed)
    elif args.cmd == "openset":
        from mjc_rov_sonar_cnn import open_set
        open_set.train_and_evaluate(epochs=args.epochs, clutter=args.clutter)


if __name__ == "__main__":
    main()
