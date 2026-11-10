"""口令文本 -> 运动意图。

两级判定，先规则后网络：

1. **规则匹配**：把识别文本与意图词表逐条算最长公共子序列（LCS）相似度，
   命中子串或相似度超过阈值即判定。可解释、零延迟，能覆盖绝大多数识别正确的情况。
2. **意图网络**：字符 1/2/3-gram 哈希特征 + 两层 MLP，用于兜底语音识别给出的
   形近/同音错字（例如 tiny 模型把「左转」识别成「縮转」）。

两级都不确定时输出 ``unknown``，上层据此不下发速度指令，避免误动作。
"""

import io
import os
import zlib
from typing import Dict, List, Optional, Sequence, Tuple

import torch
import torch.nn as nn

INTENTS: List[str] = [
    'forward',
    'backward',
    'turn_left',
    'turn_right',
    'stop',
    'wave',
    'faster',
    'slower',
]
UNKNOWN = 'unknown'
# 网络多出一个 unknown 类，用于吃掉「与口令无关」的语音（例如「打开电视」），
# 避免无关语句被强行判成某个动作而让机器人乱动。
NUM_CLASSES = len(INTENTS) + 1

# 内置默认词表（config/intents.yaml 缺失时使用）
DEFAULT_LEXICON: Dict[str, List[str]] = {
    'forward': ['前进', '向前', '往前', '前行', '向前走', '往前走', '前进一点', 'move forward'],
    'backward': ['后退', '向后', '往后', '退后', '向后退', '往后退', 'move backward'],
    'turn_left': ['左转', '向左转', '往左转', '左边转', 'turn left'],
    'turn_right': ['右转', '向右转', '往右转', '右边转', 'turn right'],
    'stop': ['停止', '停下', '停住', '别动', '站住', '停下别动', 'stop'],
    'wave': ['挥手', '打个招呼', '招手', '挥挥手', 'wave'],
    'faster': ['快一点', '快点', '加速', '走快点', '快些', 'faster'],
    'slower': ['慢一点', '慢点', '减速', '走慢点', '慢些', 'slower'],
}

# 规则判定阈值：命中子串直接 1.0，LCS 相似度需高于此值
RULE_THRESHOLD = 0.6
# 与词表的最高相似度低于此值时，直接判 unknown，不交给网络（挡住无关语音）
NET_MIN_OVERLAP = 0.30
# 网络判定的最低置信度，低于此值视为 unknown
NET_THRESHOLD = 0.45

_PUNCT = '，。！？、；：""''（）《》【】,.!?;:"\'()<>[] \t\r\n'


def normalize_text(text: str) -> str:
    """去掉标点空白，并把 Whisper 短音频偶发的重复片段折叠成一份。"""
    clean = ''.join(ch for ch in text if ch not in _PUNCT)
    if not clean:
        return ''
    # 折叠 "停止停止停止" / "左转左转" 这类整段重复
    for size in range(1, len(clean) // 2 + 1):
        if len(clean) % size:
            continue
        unit = clean[:size]
        if unit * (len(clean) // size) == clean:
            return unit
    return clean


def lcs_ratio(a: str, b: str) -> float:
    """最长公共子序列相似度，取值 [0, 1]，对错字/漏字比对编辑距离更宽容。"""
    if not a or not b:
        return 0.0
    prev = [0] * (len(b) + 1)
    for ca in a:
        cur = [0] * (len(b) + 1)
        for j, cb in enumerate(b, start=1):
            cur[j] = prev[j - 1] + 1 if ca == cb else max(prev[j], cur[j - 1])
        prev = cur
    return 2.0 * prev[-1] / (len(a) + len(b))


def rule_match(text: str, lexicon: Dict[str, List[str]],
               threshold: float = RULE_THRESHOLD) -> Tuple[Optional[str], float]:
    """返回 (意图, 得分)；无命中时返回 (None, 0.0)。"""
    norm = normalize_text(text).lower()
    if not norm:
        return None, 0.0
    best_intent, best_score = None, 0.0
    for intent, keywords in lexicon.items():
        for kw in keywords:
            kw_norm = normalize_text(kw).lower()
            if not kw_norm:
                continue
            if kw_norm in norm or norm in kw_norm:
                return intent, 1.0
            score = lcs_ratio(norm, kw_norm)
            if score > best_score:
                best_intent, best_score = intent, score
    if best_score >= threshold:
        return best_intent, best_score
    return None, best_score


# ----------------------------------------------------------------------
# 字符 n-gram 哈希特征
# ----------------------------------------------------------------------
FEATURE_DIM = 512


def hashed_features(text: str, dim: int = FEATURE_DIM) -> torch.Tensor:
    """字符 1/2/3-gram 的 crc32 哈希词袋，L2 归一化。

    用 crc32 而不是内置 hash()，因为后者受 PYTHONHASHSEED 影响，
    训练与推理会得到不同的特征索引。
    """
    norm = normalize_text(text).lower()
    vec = torch.zeros(dim, dtype=torch.float32)
    if not norm:
        return vec
    for n in (1, 2, 3):
        for i in range(len(norm) - n + 1):
            gram = norm[i:i + n]
            idx = zlib.crc32(gram.encode('utf-8')) % dim
            vec[idx] += 1.0
    norm_val = float(vec.norm())
    if norm_val > 0:
        vec /= norm_val
    return vec


class IntentMLP(nn.Module):
    """字符 n-gram 特征 -> 意图类别。"""

    def __init__(self, in_dim: int = FEATURE_DIM, hidden: int = 128,
                 num_classes: int = NUM_CLASSES):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, hidden),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(hidden, hidden // 2),
            nn.ReLU(),
            nn.Linear(hidden // 2, num_classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


def load_lexicon(path: Optional[str] = None) -> Dict[str, List[str]]:
    """从 YAML 读取词表，失败时回退到内置默认值。"""
    if not path or not os.path.exists(path):
        return dict(DEFAULT_LEXICON)
    try:
        import yaml
        with open(path, 'r', encoding='utf-8') as fh:
            data = yaml.safe_load(fh) or {}
        lexicon = data.get('lexicon') or {}
        merged = dict(DEFAULT_LEXICON)
        for intent, words in lexicon.items():
            if intent in merged and isinstance(words, (list, tuple)):
                merged[intent] = [str(w) for w in words]
        return merged
    except Exception:                                    # noqa: BLE001
        return dict(DEFAULT_LEXICON)


class IntentClassifier:
    """规则 + 网络的组合意图分类器。

    意图网络以 TorchScript 形式保存（``weights/intent_mlp.pt``），推理时
    不需要重新定义网络结构，和仓库内其它模块的权重格式保持一致。
    """

    def __init__(self, weights_path: Optional[str] = None,
                 lexicon: Optional[Dict[str, List[str]]] = None):
        self.lexicon = lexicon or dict(DEFAULT_LEXICON)
        self.model = None
        self.weights_path = weights_path
        if weights_path and os.path.exists(weights_path):
            # torch 的 C++ 侧不接受宽字符路径，用 BytesIO 绕开（中文路径兜底）
            with open(weights_path, 'rb') as fh:
                self.model = torch.jit.load(io.BytesIO(fh.read()), map_location='cpu')
            self.model.eval()

    # -- 单条 --------------------------------------------------------
    def classify(self, text: str) -> Tuple[str, float, str]:
        """返回 (意图, 置信度, 判定来源 rule/net/unknown)。"""
        intent, score = rule_match(text, self.lexicon)
        if intent is not None:
            return intent, score, 'rule'
        if score < NET_MIN_OVERLAP:
            # 与所有关键词都几乎不重合，说明说的根本不是口令
            return UNKNOWN, score, 'unknown'
        if self.model is not None:
            with torch.no_grad():
                logits = self.model(hashed_features(text).unsqueeze(0))[0]
                probs = torch.softmax(logits, dim=-1)
            conf, idx = float(probs.max()), int(probs.argmax())
            if idx >= len(INTENTS):
                return UNKNOWN, conf, 'net'
            if conf >= NET_THRESHOLD:
                return INTENTS[idx], conf, 'net'
            return UNKNOWN, conf, 'net'
        return UNKNOWN, score, 'unknown'

    # -- 批量 --------------------------------------------------------
    def classify_batch(self, texts: Sequence[str]) -> List[Tuple[str, float, str]]:
        return [self.classify(t) for t in texts]


def intent_to_command(intent: str, level: float, params: Dict[str, float]) -> Dict[str, float]:
    """意图 + 速度档位 -> 速度指令 (vx, wz)。

    ``params`` 至少包含 ``v_forward`` / ``v_backward`` / ``w_turn`` 三个基准量。
    faster / slower 由调用方先调整 level，再重新映射。
    """
    v_fwd = params.get('v_forward', 0.30) * level
    v_bwd = params.get('v_backward', 0.20) * level
    w_turn = params.get('w_turn', 0.60) * level
    table = {
        'forward': (v_fwd, 0.0),
        'backward': (-v_bwd, 0.0),
        'turn_left': (0.0, w_turn),
        'turn_right': (0.0, -w_turn),
        'stop': (0.0, 0.0),
        'wave': (0.0, 0.0),
        'faster': (0.0, 0.0),
        'slower': (0.0, 0.0),
    }
    vx, wz = table.get(intent, (0.0, 0.0))
    return {'vx': vx, 'wz': wz}
