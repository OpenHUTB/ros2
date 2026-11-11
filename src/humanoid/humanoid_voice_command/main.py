"""语音指令模块的离线入口（不需要 ROS，便于先验证算法链路）。

用法::

    python main.py --list                     # 列出包内口令音频
    python main.py --wav audio/forward_1.wav  # 识别单条口令
    python main.py --demo                     # 按顺序播放全部口令，打印动作序列
    python main.py --text "往前走"            # 跳过语音识别，直接测意图解析
    python main.py --bench                    # 只测 ASR + 意图的耗时
"""

import argparse
import glob
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from humanoid_voice_command.asr import WhisperASR, default_cache_dir        # noqa: E402
from humanoid_voice_command.intent import (                               # noqa: E402
    INTENTS,
    IntentClassifier,
    intent_to_command,
    load_lexicon,
)


def speed_params(cfg_path: str):
    """从 yaml 读基准速度，缺省用内置值。"""
    params = {'v_forward': 0.30, 'v_backward': 0.20, 'w_turn': 0.60}
    if os.path.exists(cfg_path):
        try:
            import yaml
            with open(cfg_path, 'r', encoding='utf-8') as fh:
                data = yaml.safe_load(fh) or {}
            params.update({k: float(v) for k, v in (data.get('speed') or {}).items()})
        except Exception:                                  # noqa: BLE001
            pass
    return params


def build_asr(args, lexicon):
    prompt = ', '.join(w for i in INTENTS for w in lexicon.get(i, []))
    return WhisperASR(model_name=args.model, model_path=args.model_path or None,
                      download_root=default_cache_dir(),
                      language=args.language,
                      initial_prompt=prompt if args.language.startswith('zh') else None)


def cmd_str(cmd):
    return 'vx=%+.2f m/s  wz=%+.2f rad/s' % (cmd['vx'], cmd['wz'])


def main():
    ap = argparse.ArgumentParser(description='人形机器人语音指令（离线演示）')
    ap.add_argument('--list', action='store_true', help='列出包内口令音频')
    ap.add_argument('--wav', default='', help='识别单条口令音频')
    ap.add_argument('--demo', action='store_true', help='顺序播放全部口令')
    ap.add_argument('--text', default='', help='直接给文本，跳过语音识别')
    ap.add_argument('--bench', action='store_true', help='测耗时')
    ap.add_argument('--model', default='tiny')
    ap.add_argument('--model-path', default='',
                    help='本地 Whisper 权重路径；缺省用缓存或自动下载')
    ap.add_argument('--language', default='zh')
    args = ap.parse_args()

    cfg = os.path.join(HERE, 'config', 'intents.yaml')
    lexicon = load_lexicon(cfg)
    clf = IntentClassifier(weights_path=os.path.join(HERE, 'weights', 'intent_mlp.pt'),
                           lexicon=lexicon)
    params = speed_params(cfg)

    if args.list:
        files = sorted(glob.glob(os.path.join(HERE, 'audio', '*.wav')))
        for f in files:
            print(os.path.basename(f))
        print('共 %d 条口令' % len(files))
        return 0

    if args.text:
        intent, conf, source = clf.classify(args.text)
        print('文本: %s' % args.text)
        print('意图: %s  置信度 %.2f  来源 %s' % (intent, conf, source))
        print('指令: %s' % cmd_str(intent_to_command(intent, 1.0, params)))
        return 0

    asr = build_asr(args, lexicon)
    print('Whisper %s 加载耗时 %.2fs' % (args.model, asr.load_latency))

    if args.bench:
        files = sorted(glob.glob(os.path.join(HERE, 'audio', '*.wav')))
        t0 = time.time()
        for f in files:
            asr.transcribe(f)
        total = time.time() - t0
        print('%d 条口令总耗时 %.2fs，平均 %.0f ms/条（CPU）'
              % (len(files), total, total / len(files) * 1000))
        return 0

    if args.wav:
        files = [args.wav]
    elif args.demo:
        files = sorted(glob.glob(os.path.join(HERE, 'audio', '*.wav')))
    else:
        ap.print_help()
        return 1

    print('%-16s %-12s %-11s %-6s %-6s %s' % ('音频', '识别文本', '意图', '来源', '耗时', '速度指令'))
    print('-' * 84)
    for f in files:
        res = asr.transcribe(f)
        intent, conf, source = clf.classify(str(res['text']))
        cmd = intent_to_command(intent, 1.0, params)
        print('%-16s %-12s %-11s %-6s %-6.0fms %s' % (
            os.path.basename(f), res['text'], intent, source,
            float(res['latency']) * 1000, cmd_str(cmd)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
