#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""人形机器人离线语音指令控制（log-mel + 小型 MLP 的 KWS）。

    python3 main.py --version                     版本
    python3 main.py --build-features <wav_dir>    从 WAV 语料提取特征（需要语料）
    python3 main.py --train                       训练 + 留一条件交叉验证
    python3 main.py --eval                        抗噪曲线 / 拒识 / 混淆矩阵（出图）
    python3 main.py --test                        离线自检
"""

import argparse
import glob
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np                                              # noqa: E402

import humanoid_kws_command.config as config                    # noqa: E402
import humanoid_kws_command.corpus as corpus                    # noqa: E402


def _real(p):
    q = Path(p)
    return q if q.is_absolute() else (HERE / q)


def cmd_version():
    print(f"{config.PACKAGE_NAME} {config.VERSION}")
    print(f"  frontend: {config.N_MELS} mel, {config.WIN/16:.0f} ms win / {config.HOP/16:.0f} ms hop")
    print(f"  classifier: MLP {2*config.N_MELS}-{config.HIDDEN}-{len(corpus.LABELS)}")
    return 0


def cmd_build_features(wav_dir, out):
    X, y, cond, snr_col = corpus.build_dataset(wav_dir)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, X=X, y=y, cond=cond.astype(str), snr=snr_col.astype(str),
                        labels=np.asarray(corpus.LABELS, dtype=str))
    print(f"特征已写入 {out}  样本 {X.shape}  条件 {sorted(set(cond))}")
    print(f"SNR 档位 {sorted(set(snr_col))}  文件 {out.stat().st_size//1024} KB")
    return 0


def cmd_train(feat_path, model_path, epochs):
    from humanoid_kws_command import eval as kws_eval
    from humanoid_kws_command.kws import MLP, params_count

    d = np.load(feat_path, allow_pickle=True)
    X, y, cond, snr_col = d["X"], d["y"], d["cond"], d["snr"]
    print(f"样本 {X.shape}  条件 {len(set(cond))}  类别 {len(corpus.LABELS)}  "
          f"SNR 档位 {sorted(set(snr_col))}")

    # 留一条件交叉验证只在干净子集上做（衡量对变调/变速的泛化）
    clean = snr_col == "clean"
    accs = kws_eval.leave_one_condition_out(X[clean], y[clean], cond[clean], epochs=epochs)
    print(f"留一条件交叉验证（干净）：{len(accs)} 折  平均 {np.mean(accs)*100:.1f}%  "
          f"最低 {np.min(accs)*100:.1f}%  最高 {np.max(accs)*100:.1f}%")

    # 训练用「干净 + 20 dB + 10 dB」：5/0/-5 dB 留作未见过档
    train_mask = np.isin(snr_col, [corpus.snr_tag(s) for s in config.TRAIN_SNRS])
    print(f"训练样本 {int(train_mask.sum())} 条（含噪声增强）")
    model = MLP(X.shape[1])
    hist = model.fit(X[train_mask], y[train_mask], epochs=epochs)
    print(f"全量训练完成，损失 {hist[0]:.3f} -> {hist[-1]:.4f}，参数量 {params_count(model)}")
    model_path.parent.mkdir(parents=True, exist_ok=True)
    model.save(model_path)
    print(f"权重已写入 {model_path}（{model_path.stat().st_size//1024} KB）")
    return 0


def cmd_eval(feat_path, model_path, wav_dir, outdir):
    from humanoid_kws_command import eval as kws_eval
    from humanoid_kws_command import figures
    from humanoid_kws_command.kws import MLP, params_count

    model = MLP.load(model_path)
    d = np.load(feat_path, allow_pickle=True)
    X, y = d["X"], d["y"]
    print(f"模型参数量 {params_count(model)}，权重 {model_path.stat().st_size//1024} KB")

    far, frr = kws_eval.rejection_metrics(model, X[d["snr"] == "clean"], y[d["snr"] == "clean"])
    print(f"干净语料：FAR（未知词误接受）={far*100:.1f}%   FRR（已知词拒绝）={frr*100:.1f}%")

    snr_col = d["snr"]
    curve = kws_eval.snr_sweep_from_features(model, X, y, snr_col)
    print(f"\n{'SNR':>10} {'样本':>5} {'准确率':>8}")
    for snr, acc, n in curve:
        tag = "clean" if snr is None else f"{snr} dB"
        mark = "" if snr in config.TRAIN_SNRS else "  (训练未见过)"
        print(f"{tag:>10} {n:5d} {acc*100:7.1f}%{mark}")
    figures.plot_snr([(s, a) for s, a, _ in curve], outdir / "snr_curve.png", config.TRAIN_SNRS)

    # 混淆矩阵与拒识只看干净子集
    clean = snr_col == "clean"
    cm = kws_eval.confusion(model, X[clean], y[clean])
    figures.plot_confusion(cm, corpus.LABELS, outdir / "confusion.png")
    print(f"图已写入 {outdir}")
    return 0


def cmd_test():
    from humanoid_kws_command.kws import MLP, params_count

    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(f"[{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    x = np.sin(2 * np.pi * 220 * np.arange(16000) / 16000).astype(np.float32)
    mel = corpus.log_mel(x)
    check("log-mel 形状正确", mel.shape[1] == config.N_MELS and mel.shape[0] > 50,
          f"{mel.shape}")
    check("log-mel 有限值", np.isfinite(mel).all())
    check("池化维度 = 2*N_MELS", corpus.pool(mel).shape == (2 * config.N_MELS,),
          f"{corpus.pool(mel).shape}")
    check("同一输入特征可复现",
          np.allclose(corpus.pool(corpus.log_mel(x)), corpus.pool(corpus.log_mel(x))))
    noisy = corpus.add_noise(x, 10, np.random.default_rng(0))
    check("加噪后能量更高", float(np.mean(noisy ** 2)) > float(np.mean(x ** 2)))

    Xd = np.random.default_rng(1).normal(size=(40, 2 * config.N_MELS)).astype(np.float32)
    yd = np.random.default_rng(2).integers(0, len(corpus.LABELS), 40)
    m = MLP(Xd.shape[1])
    check("参数量在 10k 以内", params_count(m) < 10000, f"{params_count(m)}")
    m.fit(Xd, yd, epochs=5)
    check("训练后能预测", m.predict(Xd).shape == (40,))
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "m.npz"
        m.save(p)
        m2 = MLP.load(p)
        check("权重存读一致", np.allclose(m.predict(Xd), m2.predict(Xd)))

    print()
    print(f"{len(fails)} 项失败" if fails else "全部通过")
    return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="离线语音指令控制（KWS）")
    ap.add_argument("--version", action="store_true")
    ap.add_argument("--build-features", type=str, default=None, metavar="WAV_DIR")
    ap.add_argument("--train", action="store_true")
    ap.add_argument("--eval", action="store_true")
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--epochs", type=int, default=config.EPOCHS)
    ap.add_argument("--features", type=str, default=str(HERE / "data" / config.FEATURE_FILE))
    ap.add_argument("--model", type=str, default=str(HERE / "weights" / config.MODEL_NAME))
    ap.add_argument("--wav-dir", type=str, default=str(HERE / "corpus_wav"))
    ap.add_argument("--outdir", type=str, default="results")
    a = ap.parse_args(argv)

    if a.version:
        return cmd_version()
    if a.test:
        return cmd_test()
    if a.build_features:
        return cmd_build_features(Path(a.build_features), _real(a.features))
    if a.train:
        return cmd_train(_real(a.features), _real(a.model), a.epochs)
    if a.eval:
        return cmd_eval(_real(a.features), _real(a.model), _real(a.wav_dir), _real(a.outdir))
    ap.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())