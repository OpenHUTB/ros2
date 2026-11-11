"""把 TTS 生成的音频规整成 16 kHz / 16 bit / 单声道 wav 并放进 audio/。

口令音频用系统 TTS 生成（Windows 可用 SAPI，Linux 可用 espeak-ng 或 edge-tts），
生成后用本脚本统一采样率，减小仓库体积。用法::

    python scripts/prepare_audio.py <源目录> <目标目录>
"""

import os
import sys

import numpy as np
import soundfile as sf

TARGET_SR = 16000


def convert(src: str, dst: str):
    data, sr = sf.read(src, dtype='float32', always_2d=False)
    if data.ndim > 1:
        data = data.mean(axis=1)
    if sr != TARGET_SR:
        n = int(round(len(data) * TARGET_SR / sr))
        idx = np.linspace(0.0, len(data) - 1.0, n)
        data = np.interp(idx, np.arange(len(data)), data).astype(np.float32)
    peak = float(np.max(np.abs(data))) if data.size else 0.0
    if peak > 1e-6:
        data = data / peak * 0.95
    sf.write(dst, data, TARGET_SR, subtype='PCM_16')
    return len(data) / TARGET_SR, os.path.getsize(dst)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    src_dir, dst_dir = sys.argv[1], sys.argv[2]
    os.makedirs(dst_dir, exist_ok=True)
    for name in sorted(os.listdir(src_dir)):
        if not name.lower().endswith('.wav'):
            continue
        dur, size = convert(os.path.join(src_dir, name), os.path.join(dst_dir, name))
        print('%-18s %5.2f s  %6d bytes' % (name, dur, size))
    return 0


if __name__ == '__main__':
    sys.exit(main())
