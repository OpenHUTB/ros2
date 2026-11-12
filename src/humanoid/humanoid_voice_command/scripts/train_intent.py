"""训练意图分类网络（字符 n-gram 哈希特征 + 两层 MLP）。

数据完全由口令模板自动生成，不需要人工标注：
每条模板会派生若干条「模拟语音识别错误」的样本（形近字/同音字替换、
随机漏字、加语气词），使网络能在规则匹配失败时仍然判对意图。

用法::

    python scripts/train_intent.py                 # 训练并写入 weights/intent_mlp.pt
    python scripts/train_intent.py --epochs 400
"""

import argparse
import os
import random
import shutil
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, PKG_ROOT)

import torch                                             # noqa: E402
import torch.nn as nn                                    # noqa: E402

from humanoid_voice_command.intent import (              # noqa: E402
    DEFAULT_LEXICON,
    FEATURE_DIM,
    INTENTS,
    IntentMLP,
    hashed_features,
    normalize_text,
)

# 每个意图的自然说法（含词表里的关键词，再补一些口语变体）
TEMPLATES = {
    'forward': ['前进', '往前走', '向前走', '往前', '向前', '前行', '往前走一点',
                '请往前走', '机器人前进', '一直向前走', '往前走吧', '向前移动'],
    'backward': ['后退', '往后退', '向后退', '退后', '往后', '退一点',
                 '请后退', '机器人后退', '往后退一点', '向后移动'],
    'turn_left': ['左转', '向左转', '往左转', '左边转', '左转弯', '往左边转',
                  '请左转', '机器人左转', '向左转一点'],
    'turn_right': ['右转', '向右转', '往右转', '右边转', '右转弯', '往右边转',
                   '请右转', '机器人右转', '向右转一点'],
    'stop': ['停止', '停下', '停住', '别动', '站住', '停下别动', '停下来',
             '马上停止', '立即停止', '不要动'],
    'wave': ['挥手', '挥挥手', '打个招呼', '招手', '挥一下手', '跟我挥手',
             '机器人挥手', '挥个手'],
    'faster': ['快一点', '快点', '加速', '走快点', '快些', '再快点',
               '速度快一点', '加快速度'],
    'slower': ['慢一点', '慢点', '减速', '走慢点', '慢些', '再慢点',
               '速度慢一点', '放慢速度'],
}

# 形近字 / 同音字表：模拟 Whisper tiny 的中文识别错误
CONFUSABLE = {
    '前': '钱剪', '进': '近劲', '退': '腿推', '左': '縮做昨', '右': '又',
    '转': '专赚', '停': '亭挺', '止': '只纸', '挥': '辉灰', '手': '首收',
    '快': '块筷', '慢': '漫满', '点': '店电', '走': '奏', '加': '家',
    '速': '术', '减': '简', '后': '候', '来': '莱', '下': '夏',
}

PREFIXES = ['', '', '', '请', '帮我', '机器人', '现在']
SUFFIXES = ['', '', '', '吧', '啊', '好吗']

# 与口令无关的语音（负样本），标签是 unknown。
# 没有这一类，网络会把任何输入都硬塞进某个动作，机器人听到「打开电视」也会走。
NEG_VERBS = ['打开', '关闭', '播放', '暂停', '查询', '调整', '调节', '设置', '切换']
NEG_OBJECTS = ['电视', '音乐', '空调', '灯光', '窗帘', '音量', '时间', '天气',
               '温度', '亮度', '频道', '闹钟']
NEG_SENTENCES = [
    '你好', '谢谢', '早上好', '再见', '今天天气不错', '现在几点了',
    '你叫什么名字', '给我讲个笑话', '明天会下雨吗', '我有点累了',
    '帮我把窗户关上', '我想喝点水', '这个房间太亮了', '放首歌听听',
    '声音太大了', '把温度调高一些', '还剩多少电量', '你在做什么',
    '这里是哪里', '请稍等一下', '我需要帮助', '记录一下当前的位置',
    '看看前面有没有障碍物', '给我看看传感器数据', '重启一下系统',
    '刚才那个命令是什么意思', '你听懂了吗', '再说一遍', '我听不清楚',
]


def build_negatives(rng: random.Random):
    """生成与口令无关的句子。"""
    out = set(NEG_SENTENCES)
    for _ in range(60):
        verb = rng.choice(NEG_VERBS)
        obj = rng.choice(NEG_OBJECTS)
        out.add(verb + obj)
        out.add(rng.choice(['请', '帮我', '麻烦']) + verb + '一下' + obj)
    for _ in range(20):
        out.add(rng.choice(NEG_SENTENCES) + rng.choice(['吧', '啊', '呢']))
    return list(out)


def corrupt(text: str, rng: random.Random) -> str:
    """随机制造一种 ASR 风格的错误。"""
    chars = list(text)
    if not chars:
        return text
    mode = rng.random()
    if mode < 0.55:
        # 形近/同音字替换
        idxs = [i for i, c in enumerate(chars) if c in CONFUSABLE]
        if idxs:
            i = rng.choice(idxs)
            chars[i] = rng.choice(CONFUSABLE[chars[i]])
    elif mode < 0.8:
        # 漏字
        if len(chars) > 2:
            del chars[rng.randrange(len(chars))]
    else:
        # 多字（重复某个字）
        i = rng.randrange(len(chars))
        chars.insert(i, chars[i])
    return ''.join(chars)


def build_dataset(seed: int = 0):
    """生成 (特征, 标签) 训练集。标签 len(INTENTS) 表示 unknown。"""
    rng = random.Random(seed)
    feats, labels = [], []
    for label, intent in enumerate(INTENTS):
        for base in TEMPLATES[intent]:
            variants = {base}
            for _ in range(12):
                variants.add(corrupt(base, rng))
            for _ in range(6):
                variants.add(rng.choice(PREFIXES) + corrupt(base, rng) + rng.choice(SUFFIXES))
            for text in variants:
                norm = normalize_text(text)
                if not norm:
                    continue
                feats.append(hashed_features(norm))
                labels.append(label)

    # 负样本：unknown 类
    unknown_label = len(INTENTS)
    for text in build_negatives(rng):
        for _ in range(3):
            variant = corrupt(text, rng) if rng.random() < 0.5 else text
            norm = normalize_text(variant)
            if not norm:
                continue
            feats.append(hashed_features(norm))
            labels.append(unknown_label)
    return torch.stack(feats), torch.tensor(labels, dtype=torch.long)


def train(epochs: int = 250, lr: float = 3e-3, seed: int = 0, out: str = ''):
    torch.manual_seed(seed)
    x, y = build_dataset(seed)
    n = x.shape[0]
    perm = torch.randperm(n)
    x, y = x[perm], y[perm]
    split = int(n * 0.85)
    x_tr, y_tr = x[:split], y[:split]
    x_va, y_va = x[split:], y[split:]

    model = IntentMLP(in_dim=FEATURE_DIM)
    opt = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
    loss_fn = nn.CrossEntropyLoss()

    best_acc, best_state = 0.0, None
    t0 = time.time()
    for epoch in range(1, epochs + 1):
        model.train()
        opt.zero_grad()
        loss = loss_fn(model(x_tr), y_tr)
        loss.backward()
        opt.step()
        if epoch % 10 == 0 or epoch == epochs:
            model.eval()
            with torch.no_grad():
                acc = float((model(x_va).argmax(dim=-1) == y_va).float().mean())
            if acc > best_acc:
                best_acc = acc
                best_state = {k: v.clone() for k, v in model.state_dict().items()}
            if epoch % 50 == 0 or epoch == epochs:
                print('epoch %3d  loss %.4f  val_acc %.4f' % (epoch, float(loss), acc))
    train_time = time.time() - t0

    if best_state is not None:
        model.load_state_dict(best_state)
    model.eval()
    # 用 trace 而不是 script：MLP 无控制流，trace 足够且不触发 jit.script 的弃用警告
    example = torch.zeros(1, FEATURE_DIM, dtype=torch.float32)
    scripted = torch.jit.trace(model, example)

    out = out or os.path.join(PKG_ROOT, 'weights', 'intent_mlp.pt')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    # torch 的 C++ 侧不接受宽字符路径，也不接受 BytesIO，先存到 ASCII 临时目录再搬运
    with tempfile.TemporaryDirectory() as tmpdir:
        tmppath = os.path.join(tmpdir, 'intent_mlp.pt')
        scripted.save(tmppath)
        shutil.copyfile(tmppath, out)
    size = os.path.getsize(out)
    print('样本 %d 条（训练 %d / 验证 %d）' % (n, split, n - split))
    print('最佳验证准确率 %.4f  训练耗时 %.1f s' % (best_acc, train_time))
    print('已保存 TorchScript: %s (%d bytes)' % (out, size))
    return out


def main():
    ap = argparse.ArgumentParser(description='训练语音意图分类网络')
    ap.add_argument('--epochs', type=int, default=250)
    ap.add_argument('--lr', type=float, default=3e-3)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--out', default='')
    args = ap.parse_args()
    print('意图类别 (%d): %s' % (len(INTENTS), ', '.join(INTENTS)))
    print('词表条目: %d' % sum(len(v) for v in DEFAULT_LEXICON.values()))
    train(epochs=args.epochs, lr=args.lr, seed=args.seed, out=args.out)


if __name__ == '__main__':
    main()
