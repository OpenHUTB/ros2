#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线单元测试：前端、模型存取、特征数据形状。不需要 ROS / 语料。

直接运行：python3 test/test_standalone.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np                                          # noqa: E402

import humanoid_kws_command.config as config                # noqa: E402
import humanoid_kws_command.corpus as corpus                # noqa: E402
from humanoid_kws_command.kws import MLP, params_count      # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    ok = bool(cond)
    if not ok:
        FAILS.append(name)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


def test_frontend():
    x = np.sin(2 * np.pi * 300 * np.arange(16000) / 16000).astype(np.float32)
    mel = corpus.log_mel(x)
    check("log-mel 列数 = N_MELS", mel.shape[1] == config.N_MELS, f"{mel.shape}")
    check("log-mel 有限", bool(np.isfinite(mel).all()))
    check("池化维度 = 2*N_MELS", corpus.pool(mel).shape == (2 * config.N_MELS,))
    check("确定性前端", np.allclose(corpus.pool(mel), corpus.pool(corpus.log_mel(x))))
    check("帧移对应 10 ms", config.HOP / config.SAMPLE_RATE == 0.01)


def test_model():
    m = MLP(2 * config.N_MELS)
    check("参数量 < 10k", params_count(m) < 10000, f"{params_count(m)}")
    check("标签数一致", m.W2.shape[1] == len(corpus.LABELS), f"{len(corpus.LABELS)}")
    check("unknown 在标签表里", config.UNKNOWN_LABEL in corpus.LABELS)


def test_shipped_data():
    p = Path(__file__).resolve().parent.parent / "data" / config.FEATURE_FILE
    check("随包特征文件存在", p.is_file(), p.name)
    if p.is_file():
        d = np.load(p, allow_pickle=True)
        check("特征形状为 (n, 80)", d["X"].shape[1] == 2 * config.N_MELS, f"{d['X'].shape}")
        check("含 6 个 SNR 档", len(set(d["snr"])) == 6, f"{sorted(set(d['snr']))}")
        check("标签覆盖 11 类", len(set(d["y"])) == len(corpus.LABELS), f"{len(set(d['y']))}")


def main():
    for fn in (test_frontend, test_model, test_shipped_data):
        print(f"\n--- {fn.__name__} ---")
        fn()
    print()
    print(f"{len(FAILS)} 项失败" if FAILS else "全部通过")
    return 1 if FAILS else 0


if __name__ == "__main__":
    raise SystemExit(main())