"""把 eval_pipeline 的 JSON 结果画成文档用的指标图。

上图：每条口令的识别耗时（按判定来源着色）
下图：意图置信度 + 无关语音的拒识情况

用法::

    python scripts/eval_pipeline.py --json eval.json
    python scripts/plot_eval.py eval.json --out ../../../../docs/img/humanoid/xxx.png
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, PKG_ROOT)

import matplotlib                                            # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt                              # noqa: E402

from humanoid_voice_command.intent import (                  # noqa: E402
    IntentClassifier,
    load_lexicon,
)

FONTS = ['C:/Windows/Fonts/msyh.ttc', 'C:/Windows/Fonts/simhei.ttf',
         '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
         '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc']

# 与口令无关的句子，用于测「误触发」（应当全部判 unknown）
OFF_TOPIC = [
    '你好', '谢谢', '早上好', '再见', '今天天气不错', '现在几点了', '你叫什么名字',
    '给我讲个笑话', '明天会下雨吗', '我有点累了', '帮我把窗户关上', '我想喝点水',
    '这个房间太亮了', '放首歌听听', '声音太大了', '把温度调高一些', '还剩多少电量',
    '你在做什么', '这里是哪里', '请稍等一下', '我需要帮助', '重启一下系统',
    '打开电视', '关闭空调', '播放音乐', '调整音量', '设置闹钟', '查询天气',
    '切换频道', '调亮一点灯光', '把窗帘拉上', '看看电量还剩多少',
]


def setup_font():
    from matplotlib import font_manager
    for path in FONTS:
        if os.path.exists(path):
            font_manager.fontManager.addfont(path)
            name = font_manager.FontProperties(fname=path).get_name()
            plt.rcParams['font.family'] = name
            break
    plt.rcParams['axes.unicode_minus'] = False


def main():
    ap = argparse.ArgumentParser(description='绘制语音指令评估指标图')
    ap.add_argument('json', help='eval_pipeline.py 输出的 JSON')
    ap.add_argument('--out', default=os.path.join(
        PKG_ROOT, '..', '..', '..', 'docs', 'img', 'humanoid',
        'humanoid_voice_command_metrics.png'))
    args = ap.parse_args()

    setup_font()
    with open(args.json, 'r', encoding='utf-8') as fh:
        data = json.load(fh)
    rows = data['rows']

    lexicon = load_lexicon(os.path.join(PKG_ROOT, 'config', 'intents.yaml'))
    clf = IntentClassifier(weights_path=os.path.join(PKG_ROOT, 'weights', 'intent_mlp.pt'),
                           lexicon=lexicon)
    rejects = [clf.classify(t)[0] for t in OFF_TOPIC]
    n_reject = sum(1 for r in rejects if r == 'unknown')

    fig, axes = plt.subplots(2, 1, figsize=(10, 6.4), gridspec_kw={'height_ratios': [1.25, 1]})

    # ---- 上图：识别耗时 ----
    ax = axes[0]
    names = [r['wav'].replace('.wav', '') for r in rows]
    lats = [r['latency'] * 1000 for r in rows]
    colors = ['#2f7fd1' if r['source'] == 'rule' else '#e0872a' for r in rows]
    ax.bar(range(len(rows)), lats, color=colors, width=0.68)
    mean_lat = sum(lats) / len(lats)
    ax.axhline(mean_lat, color='#c0392b', linestyle='--', linewidth=1.2,
               label='平均 %.0f ms' % mean_lat)
    ax.set_xticks(range(len(rows)))
    ax.set_xticklabels(names, rotation=55, ha='right', fontsize=8)
    ax.set_ylabel('Whisper tiny 推理耗时 (ms)')
    ax.set_title('CPU 单条口令识别耗时（%d 条，全部正确）' % len(rows), fontsize=11)
    ax.legend(fontsize=8, loc='upper left')
    ax.grid(axis='y', alpha=0.25)
    # 首条含模型预热，单独标注
    if lats:
        ax.annotate('首条含预热', xy=(0, lats[0]), xytext=(1.2, lats[0] * 0.92),
                    fontsize=8, color='#666',
                    arrowprops=dict(arrowstyle='->', color='#999', lw=1))
    ax.set_ylim(0, max(lats) * 1.22)

    # ---- 下图：置信度 + 拒识 ----
    ax = axes[1]
    confs = [r['confidence'] for r in rows]
    ax.bar(range(len(rows)), confs, color=colors, width=0.68)
    ax.axhline(0.45, color='#c0392b', linestyle=':', linewidth=1.2,
               label='网络置信度阈值 0.45')
    ax.axhline(0.60, color='#7f8c8d', linestyle=':', linewidth=1.2,
               label='规则相似度阈值 0.60')
    ax.set_xticks(range(len(rows)))
    ax.set_xticklabels(names, rotation=55, ha='right', fontsize=8)
    ax.set_ylabel('判定置信度')
    ax.set_ylim(0, 1.15)
    ax.set_title(
        '意图判定置信度（蓝=规则命中，橙=网络兜底）；'
        '无关语音 %d/%d 判为 unknown' % (n_reject, len(OFF_TOPIC)), fontsize=11)
    ax.legend(fontsize=8, loc='lower left', ncol=2)
    ax.grid(axis='y', alpha=0.25)

    # 标注网络兜底的那一条（真实 ASR 错字案例）
    for i, r in enumerate(rows):
        if r['source'] == 'net':
            ax.annotate('"%s"→%s' % (r['text'], r['intent']),
                        xy=(i, r['confidence']), xytext=(i - 2.5, r['confidence'] + 0.18),
                        fontsize=8, color='#b8590f',
                        arrowprops=dict(arrowstyle='->', color='#b8590f', lw=1))

    fig.tight_layout()
    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=150)
    print('已生成 %s' % out)
    print('无关语音拒识 %d/%d' % (n_reject, len(OFF_TOPIC)))
    print('平均识别耗时 %.0f ms' % mean_lat)
    return 0


if __name__ == '__main__':
    sys.exit(main())
