#!/usr/bin/env python3
"""
main.py — 统一入口。
子命令: collect / train / test / all（默认不带参数=全链路）
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
    sub.add_parser("train", help="训练 CNN")
    sub.add_parser("test", help="测试准确率")
    args = parser.parse_args()

    if not args.cmd or args.cmd == "all":
        print("== [1/2] 训练 CNN ==")
        cnn_model.train_cnn(epochs=30)
        print("== [2/2] 测试准确率 ==")
        cnn_model.test_cnn()
    elif args.cmd == "train":
        cnn_model.train_cnn(epochs=30)
    elif args.cmd == "test":
        cnn_model.test_cnn()


if __name__ == "__main__":
    main()
