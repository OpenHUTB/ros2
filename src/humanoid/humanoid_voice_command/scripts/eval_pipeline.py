"""端到端评估：口令音频 -> Whisper 识别 -> 意图分类 -> 速度指令。

逐条打印识别文本、意图、判定来源与耗时，最后给出准确率与混淆情况。

用法::

    python scripts/eval_pipeline.py                 # 评估 audio/ 下全部口令
    python scripts/eval_pipeline.py --wav audio/forward.wav
    python scripts/eval_pipeline.py --json out.json
"""

import argparse
import glob
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, PKG_ROOT)

from humanoid_voice_command.asr import WhisperASR, default_cache_dir        # noqa: E402
from humanoid_voice_command.intent import (                               # noqa: E402
    INTENTS,
    IntentClassifier,
    intent_to_command,
    lcs_ratio,
    load_lexicon,
    normalize_text,
)

SPEED_PARAMS = {'v_forward': 0.30, 'v_backward': 0.20, 'w_turn': 0.60}


def text_hit(text: str, truth: str, lexicon) -> bool:
    """识别文本是否真的带出了正确意图的语义（衡量 ASR 本身的质量）。"""
    norm = normalize_text(text).lower()
    for kw in lexicon.get(truth, []):
        kw_norm = normalize_text(kw).lower()
        if kw_norm and (kw_norm in norm or lcs_ratio(norm, kw_norm) >= 0.5):
            return True
    return False


def truth_from_name(path: str) -> str:
    """文件名即标注：forward_1.wav -> forward，forward_en.wav -> forward。"""
    name = os.path.splitext(os.path.basename(path))[0]
    if name.endswith('_en'):
        name = name[:-3]
    if '_' in name and name.rsplit('_', 1)[-1].isdigit():
        name = name.rsplit('_', 1)[0]
    return name


def prompt_from_lexicon(lexicon) -> str:
    words = []
    for intent in INTENTS:
        words.extend(lexicon.get(intent, []))
    return ', '.join(words)


def main():
    ap = argparse.ArgumentParser(description='语音指令端到端评估')
    ap.add_argument('--wav', default='', help='只评估指定 wav；缺省评估 audio/*.wav')
    ap.add_argument('--model', default='tiny')
    ap.add_argument('--model-path', default='',
                    help='本地 Whisper 权重路径；缺省用缓存或自动下载')
    ap.add_argument('--weights', default=os.path.join(PKG_ROOT, 'weights', 'intent_mlp.pt'))
    ap.add_argument('--json', default='')
    args = ap.parse_args()

    lexicon = load_lexicon(os.path.join(PKG_ROOT, 'config', 'intents.yaml'))
    clf = IntentClassifier(weights_path=args.weights, lexicon=lexicon)
    print('意图网络: %s' % ('已加载' if clf.model is not None else '未找到，仅用规则匹配'))

    files = [args.wav] if args.wav else sorted(
        glob.glob(os.path.join(PKG_ROOT, 'audio', '*.wav')))
    if not files:
        print('没有找到音频文件')
        return 1

    # 中英文各用一个识别器（真实部署只用一个）
    asr_zh = WhisperASR(model_name=args.model, model_path=args.model_path or None,
                        download_root=default_cache_dir(),
                        language='zh', initial_prompt=prompt_from_lexicon(lexicon))
    asr_en = WhisperASR(model_name=args.model, model_path=args.model_path or None,
                        download_root=default_cache_dir(), language='en')
    print('Whisper %s 加载耗时 %.2f s' % (args.model, asr_zh.load_latency))
    print('-' * 92)
    print('%-16s %-12s %-10s %-6s %-6s %-8s %s' %
          ('音频', '识别文本', '意图', '来源', '置信', '耗时', '速度指令'))
    print('-' * 92)

    rows, ok, asr_ok = [], 0, 0
    total_latency = 0.0
    for path in files:
        truth = truth_from_name(path)
        asr = asr_en if os.path.basename(path).endswith('_en.wav') else asr_zh
        res = asr.transcribe(path)
        intent, conf, source = clf.classify(res['text'])
        cmd = intent_to_command(intent, 1.0, SPEED_PARAMS)
        hit = intent == truth
        ok += int(hit)
        asr_ok += int(text_hit(res['text'], truth, lexicon))
        total_latency += float(res['latency'])
        print('%-16s %-12s %-10s %-6s %-6.2f %-8s vx=%+.2f wz=%+.2f %s' % (
            os.path.basename(path), res['text'], intent, source, conf,
            '%.2fs' % res['latency'], cmd['vx'], cmd['wz'], '✓' if hit else '✗'))
        rows.append({
            'wav': os.path.basename(path),
            'truth': truth,
            'text': res['text'],
            'raw_text': res['raw_text'],
            'intent': intent,
            'source': source,
            'confidence': round(conf, 4),
            'latency': round(float(res['latency']), 4),
            'duration': round(float(res['duration']), 3),
            'cmd': cmd,
            'correct': hit,
        })

    n = len(files)
    print('-' * 92)
    print('意图准确率 %d/%d = %.1f%%   文本命中率 %d/%d = %.1f%%   平均推理 %.2f s/条'
          % (ok, n, 100.0 * ok / n, asr_ok, n, 100.0 * asr_ok / n, total_latency / n))

    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump({'rows': rows, 'intent_acc': ok / n, 'asr_hit': asr_ok / n}, fh,
                      ensure_ascii=False, indent=2)
        print('已写出 %s' % args.json)
    return 0 if ok == n else 2


if __name__ == '__main__':
    t0 = time.time()
    sys.exit(main())
