# -*- coding: utf-8 -*-
"""语料与前端：WAV 读取、log-mel 特征、特征池化。纯 numpy + 标准库。

不依赖 librosa / torch：WAV 用标准库 ``wave`` 解，频谱用 ``numpy.fft`` 算。
"""

import wave
from pathlib import Path

import numpy as np

from humanoid_kws_command import config

# 前端参数（与训练、在线推理必须完全一致）


def load_wav(path):
    """读 16-bit PCM WAV，返回 float32 单声道波形（归一化到 [-1, 1]）。"""
    with wave.open(str(path), "rb") as fh:
        n_ch = fh.getnchannels()
        sw = fh.getsampwidth()
        sr = fh.getframerate()
        raw = fh.readframes(fh.getnframes())
    if sw != 2:
        raise ValueError(f"{path}: 只支持 16-bit PCM，实际 {sw * 8} bit")
    x = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    if n_ch > 1:
        x = x.reshape(-1, n_ch).mean(axis=1)
    if sr != config.SAMPLE_RATE:
        # 线性插值重采样（语料本来就是 16 kHz，这里只是兜底）
        n = int(round(len(x) * config.SAMPLE_RATE / sr))
        x = np.interp(np.linspace(0, len(x) - 1, n), np.arange(len(x)), x).astype(np.float32)
    return x


def _mel_filterbank():
    """三角 mel 滤波器组，形状 (config.N_MELS, config.N_FFT//2+1)。"""
    def hz2mel(f):
        return 2595.0 * np.log10(1.0 + f / 700.0)

    def mel2hz(m):
        return 700.0 * (10.0 ** (m / 2595.0) - 1.0)

    edges = np.linspace(hz2mel(config.F_MIN), hz2mel(config.F_MAX), config.N_MELS + 2)
    hz = mel2hz(edges)
    bins = np.floor((config.N_FFT + 1) * hz / config.SAMPLE_RATE).astype(int)
    fb = np.zeros((config.N_MELS, config.N_FFT // 2 + 1), dtype=np.float32)
    for i in range(config.N_MELS):
        l, c, r = bins[i], bins[i + 1], bins[i + 2]
        if c == l:
            c = l + 1
        if r == c:
            r = c + 1
        for k in range(l, min(c, fb.shape[1])):
            fb[i, k] = (k - l) / max(c - l, 1)
        for k in range(c, min(r, fb.shape[1])):
            fb[i, k] = (r - k) / max(r - c, 1)
    return fb


_FB = _mel_filterbank()


def log_mel(x):
    """log-mel 谱，返回 (frames, config.N_MELS)。"""
    if len(x) < config.WIN:
        x = np.pad(x, (0, config.WIN - len(x)))
    x = np.append(x[0], x[1:] - config.PREEMPH * x[:-1]).astype(np.float32)
    n_frames = 1 + (len(x) - config.WIN) // config.HOP
    idx = np.arange(config.WIN)[None, :] + config.HOP * np.arange(n_frames)[:, None]
    frames = x[idx] * np.hamming(config.WIN).astype(np.float32)[None, :]
    spec = np.abs(np.fft.rfft(frames, n=config.N_FFT)) ** 2
    mel = spec @ _FB.T
    return np.log(mel + 1e-8).astype(np.float32)


def pool(mel):
    """把帧级 log-mel 压成定长向量：每个 mel 带的均值与标准差（2*config.N_MELS 维）。"""
    return np.concatenate([mel.mean(axis=0), mel.std(axis=0)]).astype(np.float32)


def features_from_wav(path):
    """WAV -> 定长特征向量。"""
    return pool(log_mel(load_wav(path)))


def add_noise(x, snr_db, rng):
    """按指定 SNR（dB）叠加高斯白噪声。"""
    p_sig = float(np.mean(x ** 2)) + 1e-12
    p_noise = p_sig / (10.0 ** (snr_db / 10.0))
    return (x + rng.normal(0.0, np.sqrt(p_noise), size=x.shape)).astype(np.float32)


CMD_WORDS = ["forward", "backward", "turn_left", "turn_right", "stop",
             "faster", "slower", "wave", "crouch", "stand_up"]
UNK_WORDS = ["hello", "weather", "music", "light_on", "thanks", "open_door"]
LABELS = CMD_WORDS + ["unknown"]
CN = {"forward": "前进", "backward": "后退", "turn_left": "左转", "turn_right": "右转",
      "stop": "停止", "faster": "加速", "slower": "减速", "wave": "挥手",
      "crouch": "蹲下", "stand_up": "起立", "unknown": "其他/未知"}



def snr_tag(snr):
    """SNR -> 字符串标签，"" 表示干净。"""
    return "clean" if snr is None else str(int(snr))


def build_dataset(wav_dir, snrs=(None, 20, 10, 5, 0, -5), seed=7):
    """对所有 SNR 一次性提特征，返回 X, y, cond, snr（字符串数组）。

    把各 SNR 的**特征**固化进 data/features.npz 之后，训练与评测都不再需要 WAV，
    因此没有 TTS 的 Linux / CI 也能完整复现抗噪曲线。
    """
    import glob
    paths = sorted(glob.glob(str(Path(wav_dir) / "*.wav")))
    if not paths:
        raise FileNotFoundError(f"{wav_dir} 下没有 wav")
    X, y, cond, snr_col = [], [], [], []
    for snr in snrs:
        rng = np.random.default_rng(seed)
        for p_ in paths:
            parts = Path(p_).stem.split("__")
            if len(parts) != 3:
                continue
            word = parts[1]
            label = word if word in CMD_WORDS else "unknown"
            x = load_wav(p_)
            if snr is not None:
                x = add_noise(x, snr, rng)
            X.append(pool(log_mel(x)))
            y.append(LABELS.index(label))
            cond.append(parts[2])
            snr_col.append(snr_tag(snr))
    return (np.asarray(X, dtype=np.float32), np.asarray(y, dtype=np.int64),
            np.asarray(cond), np.asarray(snr_col))