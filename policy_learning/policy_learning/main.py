"""包内入口：一条命令跑通 采集 → 训练 → DAgger → 闭环评测。"""
from __future__ import annotations

import sys

from .main_train import main as train_main


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        argv = ["--all", "--episodes", "120", "--epochs", "150",
                "--dagger", "2", "--dagger-episodes", "20"]
    return train_main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
