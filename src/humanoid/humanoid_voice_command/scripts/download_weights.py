"""预取 Whisper 权重到本地缓存（仓库不携带 72 MB 的 tiny.pt）。

用法::

    python scripts/download_weights.py             # 下载 tiny 并校验 sha256
    python scripts/download_weights.py --check     # 只检查本地是否已有
    python scripts/download_weights.py --model base
    python scripts/download_weights.py --copy-to <包内 weights/whisper 目录>

评测机能联网时先跑一次，之后 `--wav / --demo / ros2 launch` 全程离线。
完全断网的机器可以：在任意联网机器上跑 `--copy-to` 把文件拷进包内
``weights/whisper/``，``asr.py`` 会自动优先使用包内权重。
"""

import argparse
import hashlib
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..')))

from humanoid_voice_command.asr import default_cache_dir, find_local_model  # noqa: E402

# Whisper 官方发布的权重指纹（与 openai/whisper 仓库 README 一致）
KNOWN_SHA256 = {
    'tiny': '65147644a518d12f04e32d6f3b26facc3f8dd46e5390956a9424a650c0ce22b9',
}


def sha256_of(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser(description='预取 Whisper 权重并校验')
    ap.add_argument('--model', default='tiny')
    ap.add_argument('--check', action='store_true', help='只检查本地是否已有权重')
    ap.add_argument('--copy-to', default='', help='把权重复制到指定目录（如包内 weights/whisper）')
    args = ap.parse_args()

    cache = default_cache_dir()
    local = find_local_model(args.model)
    if local:
        size = os.path.getsize(local) / 1024 / 1024
        digest = sha256_of(local)
        print(f'已存在：{local}（{size:.1f} MB）')
        print(f'sha256 {digest}')
        expect = KNOWN_SHA256.get(args.model)
        if expect and digest != expect:
            print(f'⚠ 与官方指纹不一致（期望 {expect}）')
    elif args.check:
        print(f'本地没有 {args.model} 权重，缓存目录 {cache}')
        return 1
    else:
        import whisper
        print(f'下载 whisper-{args.model} 到 {cache} ...')
        os.makedirs(cache, exist_ok=True)
        whisper.load_model(args.model, download_root=cache)
        local = os.path.join(cache, f'{args.model}.pt')
        print(f'完成：{local}（{os.path.getsize(local) / 1024 / 1024:.1f} MB）')

    if args.copy_to:
        os.makedirs(args.copy_to, exist_ok=True)
        dst = os.path.join(args.copy_to, f'{args.model}.pt')
        shutil.copyfile(local, dst)
        print(f'已复制到 {dst}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
